"""
Odometry: (x, y) and heading from time + speed + steering.
Stub: returns (0,0) and 0. Replace with time-based dead reckoning using picar-x.
"""
from typing import Tuple

_x, _y, _heading = 0.0, 0.0, 0.0


def reset(x: float = 0, y: float = 0, heading_deg: float = 0) -> None:
    global _x, _y, _heading
    _x, _y, _heading = x, y, heading_deg


def update(dt_sec: float, speed: float, steer_deg: float) -> None:
    """Stub: no real integration. Replace with proper dead reckoning."""
    global _x, _y, _heading
    # Placeholder: move in heading direction
    import math
    rad = math.radians(_heading)
    _x += speed * 0.01 * math.cos(rad) * dt_sec * 50  # rough scale
    _y += speed * 0.01 * math.sin(rad) * dt_sec * 50
    _heading += steer_deg * dt_sec * 0.5
    _heading = _heading % 360
    if _heading > 180:
        _heading -= 360


def get_position() -> Tuple[float, float]:
    return (_x, _y)


def get_heading_deg() -> float:
    return _heading
