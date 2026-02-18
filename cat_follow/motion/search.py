"""
Search arc: alternate left/right steering at low speed so the car scans for the cat.
Used in SEARCH and LOST_SEARCH states.
"""
from . import limits

# Alternate direction every this many seconds
ARC_DURATION_SEC = 2.0
# Low speed while searching (0–100)
SEARCH_SPEED = 20


def compute_search_tick(cycle_sec: float, calibration) -> tuple:
    """
    Compute steer and speed for the current moment in the search arc.

    Alternates: first ARC_DURATION_SEC steer left (positive), then right (negative), etc.
    Steering is clamped to calibration limits.

    Args:
        cycle_sec: Seconds since search started (or any monotonic cycle time).
        calibration: Calibration with get_max_steer_angle_deg(); can be None for ±30.

    Returns:
        (steer_deg, speed): steer in degrees (positive = left), speed 0–100.
    """
    phase = cycle_sec / ARC_DURATION_SEC
    # 0–1: left, 1–2: right, 2–3: left, ...
    direction = 1.0 if (int(phase) % 2 == 0) else -1.0
    max_steer = 30.0 if calibration is None else calibration.get_max_steer_angle_deg()
    steer = direction * max_steer
    steer = limits.clamp_steer(steer, calibration)
    speed = limits.clamp_speed(SEARCH_SPEED)
    return (steer, speed)


def compute_full_circle_tick(calibration) -> tuple:
    """
    Steer and speed for a full-circle search: turn one direction (left) at max steer.
    Used in SEARCH/LOST_SEARCH until 360° accumulated; then stop.

    Returns:
        (steer_deg, speed): steer = max left (positive), speed = SEARCH_SPEED.
    """
    max_steer = 30.0 if calibration is None else calibration.get_max_steer_angle_deg()
    steer = limits.clamp_steer(max_steer, calibration)
    speed = limits.clamp_speed(SEARCH_SPEED)
    return (steer, speed)
