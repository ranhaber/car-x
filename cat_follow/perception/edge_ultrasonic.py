"""Event-driven HC-SR04 reader for the Radxa ROCK 4D.

The stock ``robot_hat.Ultrasonic`` implementation repeatedly reads the ECHO
GPIO while it is low/high.  That consumes most of one CPU core.  This module
requests both-edge events from libgpiod instead, so the measurement thread
sleeps in the kernel and uses the kernel event timestamps to calculate pulse
width.

ROCK 4D header mapping:

* D2 / physical pin 13 / GPIO2_C0: ``gpiochip2`` line 16 (TRIG)
* D3 / physical pin 15 / GPIO1_C5: ``gpiochip1`` line 21 (ECHO)
"""

from __future__ import annotations

import os
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

from cat_follow.logger import get_logger

log = get_logger("perception.ultrasonic")

DEFAULT_TRIG_CHIP = "gpiochip2"
DEFAULT_TRIG_LINE = 16
DEFAULT_ECHO_CHIP = "gpiochip1"
DEFAULT_ECHO_LINE = 21
DEFAULT_CPU_CORE = 3
DEFAULT_RT_PRIORITY = 70
DEFAULT_PING_INTERVAL_S = 0.060
DEFAULT_ECHO_TIMEOUT_S = 0.040
DEFAULT_STALE_AFTER_S = 0.250
DEFAULT_MIN_DISTANCE_CM = 1.0
DEFAULT_MAX_DISTANCE_CM = 500.0


@dataclass(frozen=True)
class RangeSample:
    """Latest completed ultrasonic measurement."""

    distance_cm: Optional[float]
    measured_at: float
    valid: bool


class EdgeTimedUltrasonic:
    """Dedicated libgpiod-v1 HC-SR04 measurement worker.

    The worker owns both GPIO lines, pins itself to one CPU, and optionally
    enters ``SCHED_FIFO``.  Consumers call :meth:`latest_distance_cm`, which is
    nonblocking and never performs hardware I/O.
    """

    def __init__(
        self,
        *,
        trig_chip: str = DEFAULT_TRIG_CHIP,
        trig_line: int = DEFAULT_TRIG_LINE,
        echo_chip: str = DEFAULT_ECHO_CHIP,
        echo_line: int = DEFAULT_ECHO_LINE,
        cpu_core: int = DEFAULT_CPU_CORE,
        rt_priority: int = DEFAULT_RT_PRIORITY,
        ping_interval_s: float = DEFAULT_PING_INTERVAL_S,
        echo_timeout_s: float = DEFAULT_ECHO_TIMEOUT_S,
        stale_after_s: float = DEFAULT_STALE_AFTER_S,
        min_distance_cm: float = DEFAULT_MIN_DISTANCE_CM,
        max_distance_cm: float = DEFAULT_MAX_DISTANCE_CM,
        require_realtime: bool = True,
        gpiod_module: Optional[Any] = None,
    ) -> None:
        if cpu_core < 0:
            raise ValueError("cpu_core must be non-negative")
        if not 1 <= rt_priority <= 99:
            raise ValueError("rt_priority must be between 1 and 99")
        if not all(
            math.isfinite(float(value))
            for value in (
                ping_interval_s,
                echo_timeout_s,
                stale_after_s,
                min_distance_cm,
                max_distance_cm,
            )
        ):
            raise ValueError("ultrasonic timing and distance values must be finite")
        if ping_interval_s <= 0:
            raise ValueError("ping_interval_s must be positive")
        if echo_timeout_s <= 0:
            raise ValueError("echo_timeout_s must be positive")
        if stale_after_s <= 0:
            raise ValueError("stale_after_s must be positive")
        if min_distance_cm <= 0 or max_distance_cm <= min_distance_cm:
            raise ValueError("invalid distance limits")

        self._trig_chip_name = trig_chip
        self._trig_line_offset = int(trig_line)
        self._echo_chip_name = echo_chip
        self._echo_line_offset = int(echo_line)
        self._cpu_core = int(cpu_core)
        self._rt_priority = int(rt_priority)
        self._ping_interval_s = float(ping_interval_s)
        self._echo_timeout_s = float(echo_timeout_s)
        self._stale_after_s = float(stale_after_s)
        self._min_distance_cm = float(min_distance_cm)
        self._max_distance_cm = float(max_distance_cm)
        self._require_realtime = bool(require_realtime)
        self._gpiod = gpiod_module

        self._stop = threading.Event()
        self._ready = threading.Event()
        self._tid_ready = threading.Event()
        self._worker_tid: Optional[int] = None
        self._lock = threading.Lock()
        self._sample = RangeSample(None, 0.0, False)
        self._startup_error: Optional[BaseException] = None
        self._runtime_error: Optional[BaseException] = None
        self._thread: Optional[threading.Thread] = None

    @classmethod
    def from_env(cls) -> "EdgeTimedUltrasonic":
        """Build the production ROCK 4D reader from environment settings."""

        return cls(
            cpu_core=_env_int(
                "CAT_FOLLOW_ULTRASONIC_CPU_CORE",
                DEFAULT_CPU_CORE,
            ),
            rt_priority=_env_int(
                "CAT_FOLLOW_ULTRASONIC_RT_PRIORITY",
                DEFAULT_RT_PRIORITY,
            ),
            ping_interval_s=_env_float(
                "CAT_FOLLOW_ULTRASONIC_PING_INTERVAL_S",
                DEFAULT_PING_INTERVAL_S,
            ),
            echo_timeout_s=_env_float(
                "CAT_FOLLOW_ULTRASONIC_ECHO_TIMEOUT_S",
                DEFAULT_ECHO_TIMEOUT_S,
            ),
            stale_after_s=_env_float(
                "CAT_FOLLOW_ULTRASONIC_STALE_AFTER_S",
                DEFAULT_STALE_AFTER_S,
            ),
            require_realtime=_env_bool(
                "CAT_FOLLOW_ULTRASONIC_REQUIRE_REALTIME", True
            ),
        )

    def _current_thread_tid(self) -> int:
        if hasattr(os, "gettid"):
            return int(os.gettid())
        return int(threading.get_native_id())

    def start(self, timeout: float = 5.0) -> None:
        """Start and synchronously validate affinity, scheduling, and GPIO."""

        if self._thread is not None:
            return
        self._stop.clear()
        self._ready.clear()
        self._tid_ready.clear()
        self._worker_tid = None
        self._startup_error = None
        self._runtime_error = None
        self._thread = threading.Thread(
            target=self._run,
            name="CatFollow-UltrasonicIRQ",
            daemon=True,
        )
        self._thread.start()

        if not self._tid_ready.wait(timeout):
            self.stop()
            raise TimeoutError("ultrasonic GPIO worker did not publish a TID")
        if self._worker_tid is None:
            self.stop()
            raise RuntimeError("ultrasonic GPIO worker TID was not captured")
        self._apply_thread_policy(self._worker_tid)

        if not self._ready.wait(timeout):
            self.stop()
            raise TimeoutError("ultrasonic GPIO worker did not become ready")
        if self._startup_error is not None:
            error = self._startup_error
            self.stop()
            raise RuntimeError(
                "ultrasonic GPIO worker failed during startup"
            ) from error

    def stop(self, timeout: float = 1.0) -> None:
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=timeout)

    def latest_distance_cm(self) -> Optional[float]:
        """Return the cached distance without GPIO access."""

        with self._lock:
            sample = self._sample
        if not sample.valid:
            return None
        if time.monotonic() - sample.measured_at > self._stale_after_s:
            return None
        return sample.distance_cm

    @property
    def runtime_error(self) -> Optional[BaseException]:
        return self._runtime_error

    def _publish(self, distance_cm: Optional[float]) -> None:
        with self._lock:
            self._sample = RangeSample(
                distance_cm=distance_cm,
                measured_at=time.monotonic(),
                valid=distance_cm is not None,
            )

    def _apply_thread_policy(self, tid: int) -> None:
        """Apply affinity/RT policy from the main thread.

        systemd grants ``CAP_SYS_NICE`` to the service entrypoint. Applying
        ``SCHED_FIFO`` to the worker TID from that thread is more reliable than
        expecting ambient capabilities inside the freshly spawned worker.
        """

        os.sched_setaffinity(tid, {self._cpu_core})
        if not self._require_realtime:
            return
        try:
            os.sched_setscheduler(
                tid,
                os.SCHED_FIFO,
                os.sched_param(self._rt_priority),
            )
        except (AttributeError, PermissionError, OSError) as exc:
            if self._require_realtime:
                raise RuntimeError(
                    "SCHED_FIFO setup failed; check CAP_SYS_NICE and "
                    "LimitRTPRIO"
                ) from exc
            log.warning(
                "Ultrasonic thread pinned to CPU %d without SCHED_FIFO: %r",
                self._cpu_core,
                exc,
            )

    def _load_gpiod(self) -> Any:
        if self._gpiod is None:
            import gpiod  # type: ignore[import-not-found]

            self._gpiod = gpiod
        return self._gpiod

    def _wait_event(self, line: Any, deadline: float) -> Optional[Any]:
        while not self._stop.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            sec = int(remaining)
            nsec = int((remaining - sec) * 1_000_000_000)
            if line.event_wait(sec=sec, nsec=nsec):
                return line.event_read()
        return None

    def _drain_events(self, line: Any) -> None:
        while line.event_wait(sec=0, nsec=0):
            line.event_read()

    def _measure(self, trig: Any, echo: Any, gpiod: Any) -> Optional[float]:
        self._drain_events(echo)

        trig.set_value(1)
        time.sleep(0.000010)
        trig.set_value(0)

        deadline = time.monotonic() + self._echo_timeout_s
        rising_ns: Optional[int] = None

        while not self._stop.is_set():
            event = self._wait_event(echo, deadline)
            if event is None:
                return None

            event_ns = event.sec * 1_000_000_000 + event.nsec
            if event.type == gpiod.LineEvent.RISING_EDGE:
                rising_ns = event_ns
            elif (
                event.type == gpiod.LineEvent.FALLING_EDGE
                and rising_ns is not None
            ):
                return self._distance_from_pulse_ns(event_ns - rising_ns)
        return None

    def _distance_from_pulse_ns(self, pulse_ns: int) -> Optional[float]:
        if pulse_ns <= 0:
            return None
        # HC-SR04 datasheet: pulse duration in microseconds / 58 = cm.
        distance_cm = (pulse_ns / 1_000.0) / 58.0
        if not self._min_distance_cm <= distance_cm <= self._max_distance_cm:
            return None
        return distance_cm

    def _run(self) -> None:
        trig_chip = echo_chip = trig = echo = None
        initialized = False
        try:
            self._worker_tid = self._current_thread_tid()
            self._tid_ready.set()
            gpiod = self._load_gpiod()

            trig_chip = gpiod.Chip(self._trig_chip_name)
            echo_chip = gpiod.Chip(self._echo_chip_name)
            trig = trig_chip.get_line(self._trig_line_offset)
            echo = echo_chip.get_line(self._echo_line_offset)

            trig.request(
                consumer="cat-follow-ultrasonic",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0],
            )
            echo.request(
                consumer="cat-follow-ultrasonic",
                type=gpiod.LINE_REQ_EV_BOTH_EDGES,
                flags=getattr(gpiod, "LINE_REQ_FLAG_BIAS_PULL_DOWN", 0),
            )

            initialized = True
            self._ready.set()
            log.info(
                "Ultrasonic edge worker ready: CPU=%d SCHED_FIFO=%d "
                "TRIG=%s:%d ECHO=%s:%d",
                self._cpu_core,
                self._rt_priority,
                self._trig_chip_name,
                self._trig_line_offset,
                self._echo_chip_name,
                self._echo_line_offset,
            )

            next_ping = time.monotonic()
            while not self._stop.is_set():
                self._publish(self._measure(trig, echo, gpiod))
                next_ping += self._ping_interval_s
                # Reset after a major overrun rather than spinning to catch up.
                now = time.monotonic()
                if next_ping < now:
                    next_ping = now
                self._stop.wait(max(0.0, next_ping - now))
        except BaseException as exc:  # keep startup failures observable
            if initialized:
                self._runtime_error = exc
                self._publish(None)
                log.exception("Ultrasonic edge worker stopped unexpectedly")
            else:
                self._startup_error = exc
                self._tid_ready.set()
                self._ready.set()
        finally:
            for line in (echo, trig):
                if line is not None:
                    try:
                        line.release()
                    except Exception:  # noqa: BLE001
                        pass
            for chip in (echo_chip, trig_chip):
                if chip is not None:
                    try:
                        chip.close()
                    except Exception:  # noqa: BLE001
                        pass


def release_legacy_ultrasonic(picarx_instance: Any) -> None:
    """Release D2/D3 after ``Picarx`` eagerly constructs its polling reader."""

    ultrasonic = getattr(picarx_instance, "ultrasonic", None)
    if ultrasonic is None:
        return
    ultrasonic.close()
    log.info("Released legacy polling ultrasonic GPIO ownership")


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off"}


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be an integer") from exc


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a number") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value
