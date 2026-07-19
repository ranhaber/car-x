"""Single hardware-output boundary for the contract-driven runtime.

`MotorInterface` is the only module that should send actuator commands.
Lower-level ``MotorBackend`` strategies translate normalized commands into
hardware-specific signals.  Milestone 2 ships a no-op backend used for
end-to-end wiring tests; Milestone 3 will add a real PiCar-X backend.

Logging policy
--------------
Per Milestone 2 design: log motor commands only when the (speed, steering,
brake) tuple differs from the last applied tuple.  This keeps telemetry at
50 Hz from drowning the queue while still capturing every meaningful change
including transitions to/from braking.

Emergency stops are always logged at ``critical`` severity so failsafe
events survive the queue's drop policy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

from cat_follow.control.types import (
    DecisionOutput,
    FsmState,
    TelemetryEventType,
    TelemetrySeverity,
)
from cat_follow.telemetry.async_logger import AsyncLogger


def _clamp(value: float, low: float, high: float) -> float:
    if value < low:
        return low
    if value > high:
        return high
    return value


@dataclass(frozen=True)
class MotorCommand:
    """Normalized actuator command after clamping."""

    speed: float
    steering: float
    brake: bool


class MotorBackend(Protocol):
    """Contract a motor backend must satisfy."""

    def apply(self, *, speed: float, steering: float, brake: bool) -> None:
        ...

    def emergency_stop(self) -> None:
        ...


class NoOpMotorBackend:
    """Records calls without touching hardware.  Used for tests and for
    Milestone 2 end-to-end wiring before the real PiCar-X backend lands.
    """

    def __init__(self) -> None:
        self.applied: list = []
        self.emergency_stops: int = 0

    def apply(self, *, speed: float, steering: float, brake: bool) -> None:
        self.applied.append(MotorCommand(speed=speed, steering=steering, brake=brake))

    def emergency_stop(self) -> None:
        self.emergency_stops += 1


class MotorInterface:
    """Public boundary that owns clamping, change-logging, and dispatch.

    Producers (typically :class:`ControlLoop`) call :py:meth:`apply` once per
    control tick with the latest :class:`DecisionOutput`.
    """

    def __init__(
        self,
        backend: MotorBackend,
        logger: Optional[AsyncLogger] = None,
        source: str = "MotorInterface",
    ) -> None:
        self._backend = backend
        self._logger = logger
        self._source = source
        self._last_command: Optional[Tuple[float, float, bool]] = None

    def apply(self, decision: DecisionOutput) -> MotorCommand:
        speed = _clamp(decision.speed, -1.0, 1.0)
        steering = _clamp(decision.steering, -1.0, 1.0)
        brake = bool(decision.brake)
        command = MotorCommand(speed=speed, steering=steering, brake=brake)
        tup = (speed, steering, brake)

        if tup != self._last_command:
            self._last_command = tup
            self._log_change(decision, command)

        self._backend.apply(speed=speed, steering=steering, brake=brake)
        return command

    def emergency_stop(self, *, reason: str = "emergency_stop") -> None:
        if self._logger is not None:
            self._logger.log(
                event_type=TelemetryEventType.MOTOR_COMMAND,
                severity=TelemetrySeverity.CRITICAL,
                source=self._source,
                state=None,
                data={
                    "emergency_stop": True,
                    "reason": reason,
                },
            )
        self._backend.emergency_stop()
        self._last_command = (0.0, 0.0, True)

    # ── helpers ─────────────────────────────────────────────────────

    def _log_change(self, decision: DecisionOutput, command: MotorCommand) -> None:
        if self._logger is None:
            return
        severity = (
            TelemetrySeverity.INFO if command.brake else TelemetrySeverity.DEBUG
        )
        state: Optional[FsmState] = decision.requested_state
        self._logger.log(
            event_type=TelemetryEventType.MOTOR_COMMAND,
            severity=severity,
            source=self._source,
            state=state,
            data={
                "speed": command.speed,
                "steering": command.steering,
                "brake": command.brake,
                "reason": decision.reason.value,
            },
        )


__all__ = [
    "MotorBackend",
    "MotorCommand",
    "MotorInterface",
    "NoOpMotorBackend",
]
