"""
Location providers: pluggable sources for the car's current position and heading.

Implementations:
- OdometryProvider: bicycle-model dead reckoning (current default).
Future: EncoderProvider, IMUProvider, VisualOdometryProvider, etc.
"""

from typing import Tuple


class LocationProvider:
    """
    Interface for car location. All coordinates in cm, heading in degrees.
    """

    def get_position(self) -> Tuple[float, float]:
        """Return (x_cm, y_cm) in world or odometry frame."""
        raise NotImplementedError

    def get_heading_deg(self) -> float:
        """Return heading in degrees (e.g. 0 = east, 90 = north)."""
        raise NotImplementedError

    def update(
        self,
        dt_sec: float,
        speed: int,
        steer_deg: float,
        cm_per_sec: float,
    ) -> None:
        """
        Update state after a motion tick (for dead-reckoning providers).
        No-op for providers that don't integrate (e.g. external GPS).
        """
        pass

    def reset(self, x_cm: float, y_cm: float, heading_deg: float) -> None:
        """Reset position and heading (e.g. at startup or re-localization)."""
        raise NotImplementedError


class OdometryProvider(LocationProvider):
    """Uses the bicycle-model odometry module (dead reckoning from speed + steer)."""

    def __init__(self):
        from cat_follow import odometry
        self._odometry = odometry

    def get_position(self) -> Tuple[float, float]:
        return self._odometry.get_position()

    def get_heading_deg(self) -> float:
        return self._odometry.get_heading_deg()

    def update(
        self,
        dt_sec: float,
        speed: int,
        steer_deg: float,
        cm_per_sec: float,
    ) -> None:
        self._odometry.update(dt_sec, speed, steer_deg, cm_per_sec)

    def reset(self, x_cm: float, y_cm: float, heading_deg: float) -> None:
        self._odometry.reset(x_cm, y_cm, heading_deg)
