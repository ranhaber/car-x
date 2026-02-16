"""
Thin wrapper over picar-x: stop(), forward(speed), backward(speed), set_steer(angle).
Uses calibration for steering clamp. Stub mode: no real hardware if px is None.
"""
from typing import Optional

# Optional: from picarx import Picarx
_px = None  # set by main or test to real Picarx() for hardware


def set_car(car) -> None:
    """Inject picar-x instance. Call once at startup."""
    global _px
    _px = car


def stop() -> None:
    if _px is not None:
        _px.stop()
    # else stub: no-op


def forward(speed: int) -> None:
    speed = max(0, min(100, int(speed)))
    if _px is not None:
        _px.forward(speed)
    # else stub


def backward(speed: int) -> None:
    speed = max(0, min(100, int(speed)))
    if _px is not None:
        _px.backward(speed)
    # else stub


def set_steer(angle_deg: float) -> None:
    """Set steering angle in degrees (e.g. -25 to +25). Clamp is applied in limits."""
    if _px is not None:
        _px.set_dir_servo_angle(angle_deg)
    # else stub


def get_steer_limits(calibration) -> tuple:
    """Returns (min_deg, max_deg) from calibration."""
    if calibration is None:
        return (-25.0, 25.0)
    m = calibration.get_max_steer_angle_deg()
    return (-m, m)
