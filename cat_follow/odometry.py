"""
Odometry: dead-reckoning position (x, y) and heading from
commanded speed, steering angle, and elapsed time.

Uses a **bicycle model**:
  - The car has a wheelbase L (front axle to rear axle).
  - Steering angle delta turns the front wheel.
  - At each time step dt:
      turn_radius  = L / tan(delta)          (if delta != 0)
      d_heading    = (v * dt) / turn_radius  (radians)
      dx, dy from arc or straight line.

Units:
  - Position (x, y) in **centimeters** from origin.
  - Heading in **degrees** (0 = forward at reset, CCW positive).
  - Speed in picar-x units (0-100); converted to cm/s via calibration.

Call ``reset()`` at startup to set origin (0, 0, 0).
Call ``update(dt, speed, steer_deg, calib)`` every main-loop tick.
"""

import math
from typing import Tuple, Optional

from cat_follow.logger import get_logger

log = get_logger("odometry")

# ---------------------------------------------------------------------------
# State (module-level; single car)
# ---------------------------------------------------------------------------
_x: float = 0.0          # cm
_y: float = 0.0          # cm
_heading_deg: float = 0.0  # degrees, CCW positive

# PiCar-X wheelbase (front axle to rear axle) in cm.
# Measured from SunFounder docs / physical car (~11.4 cm).
WHEELBASE_CM: float = 11.4

# Minimum absolute steering angle (degrees) below which we treat the
# car as going straight (avoids division by near-zero tan).
_STRAIGHT_THRESHOLD_DEG: float = 0.5


def reset(x: float = 0.0, y: float = 0.0, heading_deg: float = 0.0) -> None:
    """Set the origin. Call once at startup or when re-homing."""
    global _x, _y, _heading_deg
    _x, _y, _heading_deg = float(x), float(y), float(heading_deg)
    log.info("Odometry reset to (%.2f, %.2f) heading=%.1f deg", _x, _y, _heading_deg)


def update(
    dt_sec: float,
    speed: float,
    steer_deg: float,
    cm_per_sec: Optional[float] = None,
) -> None:
    """Integrate one time step.

    Parameters
    ----------
    dt_sec : float
        Elapsed time since last call (seconds).
    speed : float
        Commanded speed in picar-x units (0-100).  Sign indicates
        direction: positive = forward, negative = backward.
    steer_deg : float
        Current steering angle in degrees (positive = left).
    cm_per_sec : float, optional
        If provided, this overrides the speed-to-velocity conversion.
        Pass ``calib.get_cm_per_sec(abs(speed))`` from calibration.
        If None, a rough default of ``speed * 0.5`` cm/s is used.
    """
    global _x, _y, _heading_deg

    if dt_sec <= 0 or speed == 0:
        return

    # Convert speed to velocity in cm/s
    if cm_per_sec is not None:
        v = cm_per_sec
    else:
        v = abs(speed) * 0.5  # rough fallback

    # Apply sign: negative speed = backward
    if speed < 0:
        v = -v

    # Distance traveled this tick
    distance = v * dt_sec  # cm

    heading_rad = math.radians(_heading_deg)

    if abs(steer_deg) < _STRAIGHT_THRESHOLD_DEG:
        # Straight line
        _x += distance * math.cos(heading_rad)
        _y += distance * math.sin(heading_rad)
    else:
        # Bicycle model: arc
        steer_rad = math.radians(steer_deg)
        turn_radius = WHEELBASE_CM / math.tan(steer_rad)  # cm, signed

        d_heading = distance / turn_radius  # radians, signed

        # New heading
        new_heading_rad = heading_rad + d_heading

        # Arc displacement (center of rear axle traces the arc)
        _x += turn_radius * (math.sin(new_heading_rad) - math.sin(heading_rad))
        _y += turn_radius * (-math.cos(new_heading_rad) + math.cos(heading_rad))

        _heading_deg = math.degrees(new_heading_rad)

    # Normalize heading to [-180, 180)
    _heading_deg = (_heading_deg + 180) % 360 - 180


def get_position() -> Tuple[float, float]:
    """Return current (x, y) in centimeters from origin."""
    return (_x, _y)


def get_heading_deg() -> float:
    """Return current heading in degrees (CCW positive, [-180, 180))."""
    return _heading_deg
