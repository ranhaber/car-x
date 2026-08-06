"""PiCar-X implementation of the :class:`MotorBackend` protocol.

Adapts normalized control commands (``speed`` and ``steering`` in
``[-1.0, 1.0]``) to the SunFounder PiCar-X SDK methods
(``forward``/``backward``/``stop``/``set_dir_servo_angle``).

Design notes
------------
- The backend talks to the SDK directly.  It does not use the prototype
  ``cat_follow/motion/driver.py`` shim so we don't drag along its module-
  level state.
- Calibration is constructor-injected so tests can override max steering
  angle and speed cap without touching hardware.
- The backend is duck-typed: any object exposing ``forward``, ``backward``,
  ``stop``, and ``set_dir_servo_angle`` is acceptable.  This keeps the
  ``picarx`` import out of the backend itself, so unit tests stay
  hardware-free.
"""

from __future__ import annotations

import warnings
from typing import Optional, Protocol


# Default calibration values that match the prototype.  ``Picarx.DIR_MAX``
# is 30 degrees but the existing prototype caps daily-use steering at 25,
# so we follow that.
DEFAULT_MAX_STEER_DEG = 25.0
DEFAULT_MAX_SPEED_PCT = 100


class _PicarxLike(Protocol):
    """Subset of the ``Picarx`` API used by this backend."""

    def forward(self, speed: int) -> None: ...

    def backward(self, speed: int) -> None: ...

    def stop(self) -> None: ...

    def set_dir_servo_angle(self, angle_deg: float) -> None: ...

    def set_cam_pan_angle(self, angle_deg: float) -> None: ...


class PiCarXBackend:
    """Drives a real (or stub) PiCar-X via the SunFounder SDK."""

    def __init__(
        self,
        picarx: _PicarxLike,
        max_steer_deg: float = DEFAULT_MAX_STEER_DEG,
        max_speed_pct: int = DEFAULT_MAX_SPEED_PCT,
        *,
        pan_forward_deg: float = 0.0,
    ) -> None:
        if max_steer_deg <= 0:
            raise ValueError("max_steer_deg must be positive")
        if max_speed_pct <= 0:
            raise ValueError("max_speed_pct must be positive")
        self._px = picarx
        self._max_steer_deg = float(max_steer_deg)
        self._max_speed_pct = int(max_speed_pct)
        self._pan_forward_deg = float(pan_forward_deg)
        self._last_steering: Optional[float] = None
        self._last_drive: Optional[str] = None  # "forward" / "backward" / "stop"
        self._last_pan: Optional[float] = None
        self._warned_missing_pan = False

    # ── MotorBackend protocol ───────────────────────────────────────

    def apply(self, *, speed: float, steering: float, brake: bool) -> None:
        # Steering is updated unconditionally so the wheels track even when
        # the car is stopped.
        steer_deg = self._scale_steering(steering)
        if steer_deg != self._last_steering:
            self._px.set_dir_servo_angle(steer_deg)
            self._last_steering = steer_deg

        if brake or speed == 0.0:
            if self._last_drive != "stop":
                self._px.stop()
                self._last_drive = "stop"
            return

        if speed > 0:
            self._px.forward(self._scale_speed(speed))
            self._last_drive = "forward"
        else:
            self._px.backward(self._scale_speed(-speed))
            self._last_drive = "backward"

    def apply_look(self, *, pan_deg: float) -> None:
        if self._last_pan is not None and abs(pan_deg - self._last_pan) < 1e-3:
            return
        setter = getattr(self._px, "set_cam_pan_angle", None)
        if not callable(setter):
            if not self._warned_missing_pan:
                warnings.warn(
                    "PiCarXBackend: set_cam_pan_angle missing; pan commands ignored",
                    RuntimeWarning,
                    stacklevel=2,
                )
                self._warned_missing_pan = True
            # Still advance the software cache so MotorInterface dedup and
            # e-stop stay coherent on stubs without a pan servo API.
            self._last_pan = float(pan_deg)
            return
        setter(float(pan_deg))
        self._last_pan = float(pan_deg)

    def emergency_stop(self) -> None:
        # Stop the drivetrain twice (matching SDK convention) and center the
        # wheels so the next command starts from a known steering state.
        self._px.stop()
        self._px.set_dir_servo_angle(0.0)
        self._last_drive = "stop"
        self._last_steering = 0.0
        setter = getattr(self._px, "set_cam_pan_angle", None)
        if not callable(setter):
            if not self._warned_missing_pan:
                warnings.warn(
                    "PiCarXBackend: set_cam_pan_angle missing; "
                    "e-stop pan reset skipped",
                    RuntimeWarning,
                    stacklevel=2,
                )
                self._warned_missing_pan = True
            # Keep cache coherent with MotorInterface (which records forward).
            self._last_pan = self._pan_forward_deg
            return
        setter(self._pan_forward_deg)
        self._last_pan = self._pan_forward_deg

    # ── helpers ─────────────────────────────────────────────────────

    def _scale_steering(self, normalized: float) -> float:
        # Caller (MotorInterface) already clamps to [-1.0, 1.0] but apply
        # a defensive clamp so this class is safe in isolation too.
        if normalized > 1.0:
            normalized = 1.0
        elif normalized < -1.0:
            normalized = -1.0
        return normalized * self._max_steer_deg

    def _scale_speed(self, normalized_magnitude: float) -> int:
        if normalized_magnitude < 0.0:
            normalized_magnitude = 0.0
        elif normalized_magnitude > 1.0:
            normalized_magnitude = 1.0
        return int(round(normalized_magnitude * self._max_speed_pct))


__all__ = [
    "DEFAULT_MAX_SPEED_PCT",
    "DEFAULT_MAX_STEER_DEG",
    "PiCarXBackend",
]
