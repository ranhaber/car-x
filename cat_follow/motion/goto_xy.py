"""
Motion logic to drive toward a target (x, y).
Uses odometry (current x, y, heading) and calculates steering/speed.
"""

import math
from typing import Tuple

from . import limits

ARRIVAL_THRESHOLD_CM = 10.0
GOTO_ARRIVAL_CM = ARRIVAL_THRESHOLD_CM
KP = 1.0
CRUISE_SPEED = 30
SLOW_SPEED = 20
SLOW_ERROR_DEG = 20.0


def compute_bearing_deg(
    current_x: float,
    current_y: float,
    target_x: float,
    target_y: float,
) -> float:
    """Return the bearing from the current point to the target in degrees."""
    return math.degrees(math.atan2(target_y - current_y, target_x - current_x))


def normalize_angle(angle_deg: float) -> float:
    """Normalize an angle to the inclusive-low range [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def compute_heading_error(
    desired_heading: float,
    current_heading: float,
) -> float:
    """Return the shortest signed turn from current to desired heading."""
    return normalize_angle(desired_heading - current_heading)


def compute_distance(
    current_x: float,
    current_y: float,
    target_x: float,
    target_y: float,
) -> float:
    """Return Euclidean distance between current and target coordinates."""
    return math.hypot(target_x - current_x, target_y - current_y)


def compute_goto(
    current_x: float,
    current_y: float,
    current_heading: float,
    target_x: float,
    target_y: float,
    calib=None,
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
    dist = compute_distance(current_x, current_y, target_x, target_y)

    if dist < ARRIVAL_THRESHOLD_CM:
        return 0.0, 0.0, True

    desired_heading = compute_bearing_deg(
        current_x,
        current_y,
        target_x,
        target_y,
    )
    error = compute_heading_error(desired_heading, current_heading)

    steer = limits.clamp_steer(error * KP, calib)

    # Speed control: slow down if turning sharply or close to target
    if abs(error) > SLOW_ERROR_DEG:
        speed = SLOW_SPEED
    elif dist < 20:
        speed = SLOW_SPEED
    else:
        speed = CRUISE_SPEED

    return steer, limits.clamp_speed(speed), False