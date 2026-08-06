"""Single hardware-output boundary for the contract-driven runtime.

`MotorInterface` is the only module that should send actuator commands.
Lower-level ``MotorBackend`` strategies translate normalized commands into
hardware-specific signals.  Milestone 2 ships a no-op backend used for
end-to-end wiring tests; Milestone 3 will add a real PiCar-X backend.

Logging policy
--------------
Per Milestone 2 design: log motor commands only when the (speed, steering,
brake) tuple differs from the last applied tuple.  Pan-only changes are also
logged so look/drive telemetry stays reconstructable.  This keeps telemetry at
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


@dataclass(frozen=True)
class LookActuatorCommand:
    """Camera pan command after clamping to hardware limits."""

    pan_deg: float
    mode: str
    reason: str


class MotorBackend(Protocol):
    """Contract a motor backend must satisfy.

    ``apply_look`` is required: look/drive pan is part of the actuator
    contract, not an optional extension.
    """

    def apply(self, *, speed: float, steering: float, brake: bool) -> None:
        ...

    def apply_look(self, *, pan_deg: float) -> None:
        ...

    def emergency_stop(self) -> None:
        ...


class NoOpMotorBackend:
    """Records calls without touching hardware.  Used for tests and for
    Milestone 2 end-to-end wiring before the real PiCar-X backend lands.
    """

    def __init__(self, *, pan_forward_deg: float = 0.0) -> None:
        self.applied: list = []
        self.look_applied: list = []
        self.emergency_stops: int = 0
        self._pan_forward_deg = float(pan_forward_deg)

    def apply(self, *, speed: float, steering: float, brake: bool) -> None:
        self.applied.append(MotorCommand(speed=speed, steering=steering, brake=brake))

    def apply_look(self, *, pan_deg: float) -> None:
        self.look_applied.append(float(pan_deg))

    def emergency_stop(self) -> None:
        self.emergency_stops += 1
        self.look_applied.append(self._pan_forward_deg)


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
        *,
        pan_min_deg: float = -90.0,
        pan_max_deg: float = 90.0,
        pan_forward_deg: float = 0.0,
    ) -> None:
        self._backend = backend
        self._logger = logger
        self._source = source
        self._last_command: Optional[Tuple[float, float, bool]] = None
        self._last_pan: Optional[float] = None
        self._pan_min_deg = float(pan_min_deg)
        self._pan_max_deg = float(pan_max_deg)
        self._pan_forward_deg = float(pan_forward_deg)

    def apply(self, decision: DecisionOutput) -> MotorCommand:
        speed = _clamp(decision.speed, -1.0, 1.0)
        steering = _clamp(decision.steering, -1.0, 1.0)
        brake = bool(decision.brake)
        command = MotorCommand(speed=speed, steering=steering, brake=brake)
        tup = (speed, steering, brake)

        chassis_changed = tup != self._last_command
        if chassis_changed:
            self._last_command = tup

        self._backend.apply(speed=speed, steering=steering, brake=brake)
        look_cmd = self.apply_look(decision)
        if chassis_changed:
            self._log_change(decision, command)
        elif look_cmd is not None:
            self._log_look(decision, look_cmd)
        return command

    def apply_look(
        self, decision: DecisionOutput
    ) -> Optional[LookActuatorCommand]:
        pan = _clamp(decision.look.pan_deg, self._pan_min_deg, self._pan_max_deg)
        cmd = LookActuatorCommand(
            pan_deg=pan,
            mode=decision.look.mode.value,
            reason=decision.look.reason,
        )
        if self._last_pan is not None and abs(pan - self._last_pan) <= 1e-3:
            return None
        self._backend.apply_look(pan_deg=pan)
        self._last_pan = pan
        return cmd

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
                    "pan_forward_deg": self._pan_forward_deg,
                },
            )
        self._backend.emergency_stop()
        self._last_command = (0.0, 0.0, True)
        # Must match backend hardware pan after e-stop (calibrated forward).
        self._last_pan = self._pan_forward_deg

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
                "look_drive_mode": decision.look.mode.value,
                "pan_deg": decision.look.pan_deg,
                "look_reason": decision.look.reason,
                "pixel_error_px": decision.look.pixel_error_px,
                "camera_request": decision.look.camera_request,
            },
        )

    def _log_look(
        self, decision: DecisionOutput, command: LookActuatorCommand
    ) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.MOTOR_COMMAND,
            severity=TelemetrySeverity.DEBUG,
            source=self._source,
            state=decision.requested_state,
            data={
                "look_only": True,
                "pan_deg": command.pan_deg,
                "look_drive_mode": command.mode,
                "look_reason": command.reason,
                "pixel_error_px": decision.look.pixel_error_px,
                "camera_request": decision.look.camera_request,
            },
        )


__all__ = [
    "LookActuatorCommand",
    "MotorBackend",
    "MotorCommand",
    "MotorInterface",
    "NoOpMotorBackend",
]
