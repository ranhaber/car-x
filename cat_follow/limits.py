"""
Limits for motion: clamp steering and speed based on calibration.
"""

def clamp_steer(steer_angle: float, calibration) -> float:
    """
    Clamp steering angle to calibration limits (or +/- 30 default).
    """
    max_angle = 30.0
    if calibration is not None:
        max_angle = calibration.get_max_steer_angle_deg()
    return max(-max_angle, min(max_angle, steer_angle))

def clamp_speed(speed: float) -> float:
    """
    Clamp speed to 0-100.
    """
    return max(0.0, min(100.0, speed))