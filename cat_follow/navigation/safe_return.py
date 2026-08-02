"""Centralized safe-return viability predicate for RETURN_HOME routing."""

from __future__ import annotations

from typing import Optional, Tuple

from cat_follow.control.types import (
    GeofenceState,
    HomeState,
    MissionState,
    NavigationState,
)


def home_is_valid(home: HomeState) -> bool:
    """Return True when a durable or legacy in-memory home may be used."""

    return bool(home.valid or home.set)


def home_has_map_pose(home: HomeState) -> bool:
    """Return True when ``x_m``/``y_m`` come from a versioned home record.

    Every producer that writes a home bumps ``home_version`` and fills the
    meter fields, so ``(0.0, 0.0)`` there is the map origin rather than an
    unset value.  Hand-built legacy states keep the centimeter conversion.
    """

    return int(home.home_version) > 0


def home_map_pose_m(home: HomeState) -> Tuple[float, float]:
    """Return home as map-frame meters, preferring the durable meter fields."""

    if home_has_map_pose(home):
        return float(home.x_m), float(home.y_m)
    return float(home.x) / 100.0, float(home.y) / 100.0


def frozen_home_pose(
    mission: MissionState, home: HomeState
) -> Optional[tuple[float, float, float, str, float, float]]:
    """Return frozen or durable home as (x, y, yaw, frame, x_m, y_m)."""

    if mission.home_version_frozen is not None:
        if (
            mission.frozen_home_x is None
            or mission.frozen_home_y is None
            or mission.frozen_home_x_m is None
            or mission.frozen_home_y_m is None
        ):
            return None
        return (
            float(mission.frozen_home_x),
            float(mission.frozen_home_y),
            float(mission.frozen_home_yaw_rad),
            str(mission.frozen_home_frame_id or "yard"),
            float(mission.frozen_home_x_m),
            float(mission.frozen_home_y_m),
        )
    if not home_is_valid(home):
        return None
    x_m, y_m = home_map_pose_m(home)
    return (
        float(home.x),
        float(home.y),
        float(home.yaw_rad),
        str(home.frame_id or "yard"),
        float(x_m),
        float(y_m),
    )


def safe_return_possible(
    *,
    home: HomeState,
    mission: MissionState,
    range_healthy: bool,
    lidar_healthy: bool,
    geofence: Optional[GeofenceState] = None,
    navigation: Optional[NavigationState] = None,
    require_navigation_healthy: bool = False,
) -> Tuple[bool, str]:
    """Return ``(ok, reason)`` for executing RETURN_HOME safely.

    When a mission has frozen a home version, that freeze must resolve to a
    usable pose.  Unconfigured geofence is treated as non-blocking so host
    tests without a polygon file remain valid; configured geofence must be
    observable and unbreached.
    """

    if mission.home_version_frozen is not None:
        if frozen_home_pose(mission, home) is None:
            return False, "frozen_home_invalid"
    elif not home_is_valid(home):
        return False, "home_missing"

    if not range_healthy or not lidar_healthy:
        return False, "sensors_unhealthy"

    if geofence is not None and geofence.configured:
        if geofence.breach_confirmed:
            return False, "geofence_breach"
        if not geofence.localization_valid_for_containment:
            return False, "geofence_unobservable"

    if (
        require_navigation_healthy
        and navigation is not None
        and not navigation.healthy
    ):
        return False, "navigation_unhealthy"

    return True, "ok"
