"""Range adapter: prototype range sensor -> ``SharedState.range``.

Polls a distance source (typically
``cat_follow.range_sensor.get_distance_cm``) and publishes a contract-form
:class:`RangeState` into the contract ``SharedState.range`` group.

Severity model
--------------
For Milestone 3 V1 we map distance into a normalized
``obstacle_severity`` in ``[0.0, 1.0]``:

- ``distance >= obstacle_detected_cm`` -> ``severity = 0.0``
- ``distance <= obstacle_critical_cm`` -> ``severity = 1.0``
- linear ramp between

The corresponding flags follow naturally:

- ``obstacle_detected = distance < obstacle_detected_cm``
- ``obstacle_critical = distance < obstacle_critical_cm``

Failure handling
----------------
When the underlying sensor returns ``None`` (timeout, out of range,
hardware error), the adapter publishes a state with ``confidence = 0.0``
and ``distance_cm = None``.  ``DecisionEngine``'s obstacle-veto rules
already short-circuit when ``distance_cm`` is missing, so a sensor failure
will not trigger a spurious failsafe.

Telemetry
---------
Every ``update()`` emits a ``range_update`` debug-severity event so the
JSONL telemetry stream captures distance trends for replay.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from cat_follow.control.decision_engine import OBSTACLE_TOO_CLOSE_CM
from cat_follow.control.types import (
    RangeBackend,
    RangeState,
    TelemetryEventType,
    TelemetrySeverity,
)
from cat_follow.runtime.shared_state import SharedState, now_monotonic_ms
from cat_follow.telemetry.async_logger import AsyncLogger


# Default poll rate.  HC-SR04 throttles internally to ~60 ms between pings,
# so polling faster than 16 Hz returns cached values from the prototype.
DEFAULT_POLL_RATE_HZ = 20.0

# Default distance below which we report an obstacle.  The prototype
# ``calibration/steering_limits.json`` uses 50 cm as a reasonable proximity
# threshold; we follow that here and let callers override.
DEFAULT_OBSTACLE_DETECTED_CM = 50.0


class RangeAdapter:
    """Bridge from a prototype distance source to contract ``RangeState``."""

    def __init__(
        self,
        contract_shared_state: SharedState,
        read_distance: Callable[[], Optional[float]],
        *,
        backend: RangeBackend = RangeBackend.ULTRASONIC,
        obstacle_detected_cm: float = DEFAULT_OBSTACLE_DETECTED_CM,
        obstacle_critical_cm: float = OBSTACLE_TOO_CLOSE_CM,
        poll_rate_hz: float = DEFAULT_POLL_RATE_HZ,
        logger: Optional[AsyncLogger] = None,
        thread_name: str = "CatFollow-RangeAdapter",
        source: str = "RangeAdapter",
    ) -> None:
        if obstacle_detected_cm <= obstacle_critical_cm:
            raise ValueError(
                "obstacle_detected_cm must be greater than obstacle_critical_cm"
            )
        if obstacle_critical_cm <= 0:
            raise ValueError("obstacle_critical_cm must be positive")
        if poll_rate_hz <= 0:
            raise ValueError("poll_rate_hz must be positive")

        self._contract_ss = contract_shared_state
        self._read_distance = read_distance
        self._backend = backend
        self._obstacle_detected_cm = float(obstacle_detected_cm)
        self._obstacle_critical_cm = float(obstacle_critical_cm)
        self._poll_rate_hz = float(poll_rate_hz)
        self._logger = logger
        self._thread_name = thread_name
        self._source = source

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ── lifecycle ───────────────────────────────────────────────────

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name=self._thread_name,
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=timeout)

    # ── single-step ─────────────────────────────────────────────────

    def update(self) -> RangeState:
        try:
            distance = self._read_distance()
        except Exception:
            distance = None

        now = now_monotonic_ms()

        if distance is None:
            new_state = RangeState(
                timestamp_ms=int(time.time() * 1000),
                received_ms=now,
                fresh=True,
                authority=self._source,
                backend=self._backend,
                distance_cm=None,
                confidence=0.0,
                obstacle_detected=False,
                obstacle_critical=False,
                obstacle_severity=0.0,
                zone=None,
            )
        else:
            distance_cm = float(distance)
            obstacle_detected = distance_cm < self._obstacle_detected_cm
            obstacle_critical = distance_cm < self._obstacle_critical_cm
            severity = self._compute_severity(distance_cm)
            new_state = RangeState(
                timestamp_ms=int(time.time() * 1000),
                received_ms=now,
                fresh=True,
                authority=self._source,
                backend=self._backend,
                distance_cm=distance_cm,
                confidence=1.0,
                obstacle_detected=obstacle_detected,
                obstacle_critical=obstacle_critical,
                obstacle_severity=severity,
                zone=None,
            )

        self._contract_ss.update_range(new_state)
        self._log_update(new_state)
        return new_state

    # ── internals ───────────────────────────────────────────────────

    def _run(self) -> None:
        period_s = 1.0 / max(self._poll_rate_hz, 1e-3)
        while not self._stop.is_set():
            try:
                self.update()
            except Exception:
                self._log_thread_exception()
            self._stop.wait(period_s)

    def _compute_severity(self, distance_cm: float) -> float:
        if distance_cm >= self._obstacle_detected_cm:
            return 0.0
        if distance_cm <= self._obstacle_critical_cm:
            return 1.0
        span = self._obstacle_detected_cm - self._obstacle_critical_cm
        if span <= 0:
            return 1.0
        return (self._obstacle_detected_cm - distance_cm) / span

    def _log_update(self, state: RangeState) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.RANGE_UPDATE,
            severity=TelemetrySeverity.DEBUG,
            source=self._source,
            state=None,
            data={
                "backend": state.backend.value,
                "distance_cm": state.distance_cm,
                "confidence": state.confidence,
                "obstacle_detected": state.obstacle_detected,
                "obstacle_critical": state.obstacle_critical,
                "obstacle_severity": state.obstacle_severity,
            },
        )

    def _log_thread_exception(self) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.THREAD_HEALTH,
            severity=TelemetrySeverity.ERROR,
            source=self._source,
            state=None,
            data={"event": "range_adapter_exception"},
        )


__all__ = [
    "DEFAULT_OBSTACLE_DETECTED_CM",
    "DEFAULT_POLL_RATE_HZ",
    "RangeAdapter",
]
