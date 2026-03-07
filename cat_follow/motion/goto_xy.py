"""
Motion logic to drive toward a target (x, y).
Uses odometry (current x, y, heading) and calculates steering/speed.
"""

import math
from typing import Tuple

from . import limits


def compute_goto(
    current_x: float,
    current_y: float,
    current_heading: float,
    target_x: float,
    target_y: float,
    calib
) -> Tuple[float, float, bool]:
    """
    Calculate steering and speed to drive toward target.

    Args:
        current_x, current_y: Current position (cm).
        current_heading: Current heading (degrees).
        target_x, target_y: Target position (cm).
        calib: Calibration object for limits.

    Returns:
        (steer_angle, speed, arrived)
        steer_angle: degrees (negative=left, positive=right).
        speed: motor speed value (0-100).
        arrived: True if within threshold distance.
    """
    dx = target_x - current_x
    dy = target_y - current_y
    dist = math.sqrt(dx * dx + dy * dy)

    # Arrival threshold (e.g. 10 cm)
    if dist < 10.0:
        return 0.0, 0.0, True

    # Calculate desired heading
    desired_heading = math.degrees(math.atan2(dy, dx))

    # Calculate heading error (shortest path)
    error = desired_heading - current_heading
    while error > 180: error -= 360
    while error < -180: error += 360

    steer = limits.clamp_steer(error, calib)

    # Speed control: slow down if turning sharply or close to target
    base_speed = 30
    if abs(error) > 20:
        speed = 20
    elif dist < 20:
        speed = 20
    else:
        speed = base_speed

    return steer, limits.clamp_speed(speed), False