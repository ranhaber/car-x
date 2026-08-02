"""Startup readiness helpers for durable home + overhead localization seed."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional

from cat_follow.control.types import (
    HomeState,
    OverheadState,
    SystemState,
)
from cat_follow.home.store import HomePersistError, HomeStore
from cat_follow.navigation.geofence import GeofencePolygon, load_geofence_polygon
from cat_follow.runtime.shared_state import SharedState, now_monotonic_ms


@dataclass(frozen=True)
class StartupArtifacts:
    home: Optional[HomeState]
    geofence: Optional[GeofencePolygon]
    home_load_error: Optional[str]
    geofence_load_error: Optional[str]
    seed_applied: bool
    ready: bool
    degraded_reason: Optional[str]


def load_startup_artifacts(
    *,
    home_store: HomeStore,
    geofence_path: Optional[str],
    require_home: bool = False,
    require_geofence: bool = False,
) -> StartupArtifacts:
    """Load durable home and optional geofence configuration at boot."""

    home = None
    home_error = None
    try:
        home = home_store.load()
    except HomePersistError as exc:
        home_error = str(exc)

    geofence = None
    geofence_error = None
    if geofence_path:
        try:
            geofence = load_geofence_polygon(geofence_path)
        except (OSError, ValueError, TypeError) as exc:
            geofence_error = str(exc)

    degraded = home_error or geofence_error
    if require_home and home is None and home_error is None:
        degraded = degraded or "home_missing"
    if require_geofence and geofence is None and geofence_error is None:
        degraded = degraded or "geofence_missing"
    ready = degraded is None
    return StartupArtifacts(
        home=home,
        geofence=geofence,
        home_load_error=home_error,
        geofence_load_error=geofence_error,
        seed_applied=False,
        ready=ready,
        degraded_reason=degraded,
    )


def apply_startup_to_shared_state(
    shared_state: SharedState,
    artifacts: StartupArtifacts,
) -> None:
    """Publish loaded home and startup readiness into SharedState."""

    if artifacts.home is not None:
        shared_state.update_home(artifacts.home)
    now_ms = now_monotonic_ms()
    current = shared_state.get_system()
    shared_state.update_system(
        replace(
            current,
            received_ms=now_ms,
            startup_ready=artifacts.ready,
            startup_seed_applied=artifacts.seed_applied,
            startup_degraded_reason=artifacts.degraded_reason,
        )
    )


def maybe_seed_from_overhead(
    *,
    overhead: OverheadState,
    system: SystemState,
    min_confidence: float = 1.0,
    max_age_ms: int = 2000,
    now_ms: Optional[int] = None,
) -> tuple[bool, Optional[tuple[float, float, float]], Optional[str]]:
    """Validate a one-shot overhead car pose seed candidate.

    Returns ``(accepted, (x_cm, y_cm, heading_rad)|None, reason)``.
    Actual TF/Nav2 initial-pose injection remains a ROS transport concern;
    this helper only qualifies the observation.
    """

    if system.startup_seed_applied:
        return False, None, "already_seeded"
    now = now_ms if now_ms is not None else now_monotonic_ms()
    if overhead.received_ms <= 0 or now - overhead.received_ms > max_age_ms:
        return False, None, "overhead_stale"
    if overhead.car.confidence < min_confidence:
        return False, None, "car_confidence_low"
    return True, (overhead.car.x, overhead.car.y, overhead.car.heading), None
