"""
Distance (range) sensor: ultrasonic as source of truth when hardware is present.

When running on Pi with Picarx, call set_car(px) so get_distance_cm() uses
px.get_distance(). When no car (stub) or invalid read, returns None.

Read interval: we throttle to MIN_READ_INTERVAL_SEC (60 ms) between hardware
pings to avoid interference (HC-SR04 typically needs ~60 ms between readings).
Within that interval we return the last cached value.
"""

import time
from typing import Optional

# Same car instance as motion.driver when on hardware
_car = None
_last_distance_cm: Optional[float] = None
_last_read_time: float = 0.0

# Minimum seconds between hardware reads (HC-SR04 often needs ~60 ms)
MIN_READ_INTERVAL_SEC = 0.06

# Valid range: ultrasonic typically 2–400 cm; -1 = error from robot_hat
MIN_CM = 1.0
MAX_CM = 500.0


def set_car(car) -> None:
    """Inject Picarx (or any object with get_distance() returning cm). Call once at startup."""
    global _car
    _car = car


def get_distance_cm() -> Optional[float]:
    """
    Return distance in cm from ultrasonic, or None if no sensor or invalid read.
    Throttled to MIN_READ_INTERVAL_SEC between hardware pings; returns cached value otherwise.
    """
    global _last_distance_cm, _last_read_time
    now = time.monotonic()
    if _car is not None and (now - _last_read_time) < MIN_READ_INTERVAL_SEC and _last_distance_cm is not None:
        return _last_distance_cm
    if _car is None:
        _last_distance_cm = None
        return None
    try:
        d = _car.get_distance()
    except Exception:
        _last_distance_cm = None
        _last_read_time = now
        return None
    _last_read_time = now
    if d is None or not isinstance(d, (int, float)):
        _last_distance_cm = None
        return None
    if d < 0:
        _last_distance_cm = None
        return None  # timeout/error
    if d < MIN_CM or d > MAX_CM:
        _last_distance_cm = None
        return None  # out of range
    _last_distance_cm = float(d)
    return _last_distance_cm


def get_last_distance_cm() -> Optional[float]:
    """Last valid distance (cm) from ultrasonic, for display. No hardware read."""
    return _last_distance_cm
