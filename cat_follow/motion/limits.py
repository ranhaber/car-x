"""Apply calibration limits: clamp steer and speed."""
from typing import Optional


def clamp_steer(angle_deg: float, calibration) -> float:
    """Clamp steering to ± max_steer_angle_deg."""
    if calibration is None:
        return max(-30, min(30, angle_deg))
    m = calibration.get_max_steer_angle_deg()
    return max(-m, min(m, angle_deg))


def clamp_speed(speed: int, max_speed: int = 100) -> int:
    """Clamp speed to 0..max_speed."""
    return max(0, min(max_speed, int(speed)))
