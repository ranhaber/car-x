"""
Modular car location: pluggable providers for current position and heading.

Usage:
    from cat_follow.location import get_position, get_heading_deg, update, reset, set_provider

    set_provider(OdometryProvider())  # or EncoderProvider(), IMUProvider(), etc.
    reset(0, 0, 0)
    ...
    x, y = get_position()
    heading = get_heading_deg()
    update(dt_sec, speed, steer_deg, cm_per_sec)
"""

from typing import Tuple

from .providers import LocationProvider, OdometryProvider

# Default provider (set at startup by main_loop)
_provider: LocationProvider = OdometryProvider()


def set_provider(provider: LocationProvider) -> None:
    """Set the active location provider (e.g. odometry, encoders, IMU)."""
    global _provider
    _provider = provider


def get_position() -> Tuple[float, float]:
    """Return (x_cm, y_cm) from the current provider."""
    return _provider.get_position()


def get_heading_deg() -> float:
    """Return heading in degrees from the current provider."""
    return _provider.get_heading_deg()


def update(
    dt_sec: float,
    speed: int,
    steer_deg: float,
    cm_per_sec: float,
) -> None:
    """Update the provider after a motion tick (no-op for non–dead-reckoning)."""
    _provider.update(dt_sec, speed, steer_deg, cm_per_sec)


def reset(x_cm: float, y_cm: float, heading_deg: float) -> None:
    """Reset position and heading on the current provider."""
    _provider.reset(x_cm, y_cm, heading_deg)


__all__ = [
    "LocationProvider",
    "OdometryProvider",
    "set_provider",
    "get_position",
    "get_heading_deg",
    "update",
    "reset",
]
