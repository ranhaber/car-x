"""
Go-to-XY: proportional heading controller (Option A).

Given the car's current position and heading (from odometry) and a target
(tx, ty), compute the steering angle and speed to drive toward the target.

Future upgrade: replace with Pure Pursuit for multi-waypoint paths.

All distances in **centimeters**, angles in **degrees**.
"""

import math
from typing import Tuple

from cat_follow.logger import get_logger
from cat_follow.motion.limits import clamp_steer, clamp_speed

log = get_logger("goto_xy")

# ---------------------------------------------------------------------------
# Tuning parameters (can be moved to calibration later)
# ---------------------------------------------------------------------------
KP: float = 1.5                    # Proportional gain for heading error -> steer
ARRIVAL_THRESHOLD_CM: float = 10.0  # "close enough" distance to target
SLOW_ERROR_DEG: float = 45.0       # If heading error > this, use slow speed
CRUISE_SPEED: int = 40             # Normal forward speed (picar-x 0-100)
SLOW_SPEED: int = 20               # Speed when heading is far off


def compute_bearing_deg(x: float, y: float, tx: float, ty: float) -> float:
    """Return bearing from (x, y) to (tx, ty) in degrees [-180, 180)."""
    dx = tx - x
    dy = ty - y
    return math.degrees(math.atan2(dy, dx))


def normalize_angle(angle_deg: float) -> float:
    """Normalize angle to [-180, 180)."""
    a = angle_deg % 360
    if a >= 180:
        a -= 360
    return a


def compute_heading_error(desired_deg: float, current_deg: float) -> float:
    """Signed heading error in [-180, 180). Positive = need to turn left."""
    return normalize_angle(desired_deg - current_deg)


def compute_distance(x: float, y: float, tx: float, ty: float) -> float:
    """Euclidean distance in cm."""
    return math.sqrt((tx - x) ** 2 + (ty - y) ** 2)


def compute_goto(
    x: float,
    y: float,
    heading_deg: float,
    tx: float,
    ty: float,
    calibration=None,
) -> Tuple[float, int, bool]:
    """Compute (steer_deg, speed, arrived) for one tick.

    Parameters
    ----------
    x, y : float
        Current position in cm (from odometry).
    heading_deg : float
        Current heading in degrees (from odometry).
    tx, ty : float
        Target position in cm.
    calibration : Calibration, optional
        For steering limits.  If None, uses default ±25 deg.

    Returns
    -------
    steer_deg : float
        Steering angle to command (clamped to calibration limits).
    speed : int
        Forward speed to command (0 if arrived).
    arrived : bool
        True if car is within ARRIVAL_THRESHOLD_CM of target.
    """
    distance = compute_distance(x, y, tx, ty)

    if distance < ARRIVAL_THRESHOLD_CM:
        return 0.0, 0, True

    bearing = compute_bearing_deg(x, y, tx, ty)
    error = compute_heading_error(bearing, heading_deg)

    # Proportional steering
    steer = KP * error
    steer = clamp_steer(steer, calibration)

    # Speed: slow down when heading is far off
    if abs(error) > SLOW_ERROR_DEG:
        speed = SLOW_SPEED
    else:
        speed = CRUISE_SPEED

    speed = clamp_speed(speed)

    return steer, speed, False
