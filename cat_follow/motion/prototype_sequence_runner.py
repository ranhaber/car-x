"""Prototype-only background runner for Movement sequences.

When the contract runtime (``CommsManager`` + ``ControlLoop``) is not
available, sequences execute on a daemon thread with direct Picarx access
while holding the web UI ``hardware_lock``.

Heartbeats from the web UI are enforced here via
:meth:`MotionSequenceExecutor.advance` during interruptible waits so the
dead-man timeout works in prototype mode the same way it does under the
50 Hz DecisionEngine tick.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional

from cat_follow.logger import get_logger
from cat_follow.motion.action_plan import (
    DriveAction,
    SteerAction,
    StopAction,
    ValidatedAction,
    WaitAction,
)
from cat_follow.motion.sequence_executor import MotionSequenceExecutor

_log = get_logger("motion.prototype_sequence_runner")

# Slice length for interruptible waits; short enough to honor the default
# 500 ms heartbeat timeout with margin.
_WAIT_SLICE_S = 0.05


def _now_monotonic_ms() -> int:
    # Local helper avoids importing runtime.shared_state (circular via control).
    return int(time.monotonic_ns() // 1_000_000)


class PrototypeSequenceRunner:
    """Runs validated plans on a background thread using direct Picarx calls."""

    def __init__(
        self,
        *,
        picarx: Any,
        hardware_lock: threading.Lock,
        executor: MotionSequenceExecutor,
        on_finished: Optional[Callable[[], None]] = None,
    ) -> None:
        self._px = picarx
        self._lock = hardware_lock
        self._executor = executor
        self._on_finished = on_finished
        self._thread: Optional[threading.Thread] = None

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, actions: list[ValidatedAction], *, now_ms: int) -> tuple[bool, str]:
        if self._px is None:
            return False, "picarx not available"
        if self.is_running:
            return False, "prototype runner already active"
        ok, msg = self._executor.start(actions, now_ms=now_ms)
        if not ok:
            return ok, msg
        if not self._lock.acquire(blocking=False):
            self._executor.stop("hardware_busy")
            return False, "hardware busy"

        def _run() -> None:
            completed = False
            try:
                completed = self._run_plan(actions)
            except Exception:  # noqa: BLE001
                _log.exception("prototype sequence failed; stopping hardware")
                self._executor.stop("prototype_exception")
            finally:
                self._safe_stop_hardware()
                if completed and self._executor.is_running:
                    self._executor.complete()
                elif self._executor.is_running:
                    # Unexpected early exit without completion or stop().
                    self._executor.stop("prototype_interrupted")
                self._lock.release()
                if self._on_finished is not None:
                    try:
                        self._on_finished()
                    except Exception:  # noqa: BLE001
                        _log.exception("prototype on_finished callback failed")

        self._thread = threading.Thread(target=_run, name="CatFollow-MoveSeq", daemon=True)
        self._thread.start()
        return True, "started"

    def stop(self, reason: str = "operator_stop") -> None:
        self._executor.stop(reason)
        self._safe_stop_hardware()

    def _safe_stop_hardware(self) -> None:
        try:
            self._px.stop()
        except Exception as exc:  # noqa: BLE001
            _log.warning("prototype stop() failed: %s", exc)
        try:
            self._px.set_dir_servo_angle(0)
        except Exception as exc:  # noqa: BLE001
            _log.warning("prototype center steering failed: %s", exc)

    def _wait_interruptible(self, duration_s: float) -> bool:
        """Wait up to ``duration_s`` while enforcing heartbeat / abort.

        Returns True if the full duration elapsed while still running,
        False if the sequence was aborted (heartbeat timeout, operator stop).
        """

        deadline = time.monotonic() + max(0.0, float(duration_s))
        while True:
            now_ms = _now_monotonic_ms()
            self._executor.advance(now_ms)
            if not self._executor.is_running:
                self._safe_stop_hardware()
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return True
            time.sleep(min(_WAIT_SLICE_S, remaining))

    def _run_plan(self, actions: list[ValidatedAction]) -> bool:
        """Execute actions.  Returns True if the full plan completed."""

        for action in actions:
            if not self._executor.is_running:
                return False
            if isinstance(action, WaitAction):
                if not self._wait_interruptible(action.duration_s):
                    return False
                continue
            if isinstance(action, StopAction):
                self._px.stop()
                if not self._wait_interruptible(action.duration_s):
                    return False
                self._px.set_dir_servo_angle(0)
                continue
            if isinstance(action, DriveAction):
                self._px.set_dir_servo_angle(0)
                if action.direction == "forward":
                    self._px.forward(action.speed_pct)
                else:
                    self._px.backward(action.speed_pct)
                if not self._wait_interruptible(action.duration_s):
                    return False
                self._px.stop()
                continue
            if isinstance(action, SteerAction):
                self._px.set_dir_servo_angle(int(action.angle_deg))
                self._px.forward(action.speed_pct)
                if not self._wait_interruptible(action.duration_s):
                    return False
                self._px.stop()
                self._px.set_dir_servo_angle(0)
        return True
