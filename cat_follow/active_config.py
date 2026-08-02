"""Snapshot of configuration values currently wired into runtime behavior."""

from __future__ import annotations

from typing import Any

from cat_follow.control.decision_engine import (
    LIDAR_STALE_MS,
    NAVIGATION_STALE_MS,
    OVERHEAD_STALE_WARNING_MS,
    RANGE_STALE_MS,
    VISION_STALE_MS,
)
from cat_follow.safety_config import SafetyConfig
from cat_follow.target_config import TargetRuntimeConfig


def active_runtime_config_dict(
    safety_config: SafetyConfig,
    target_config: TargetRuntimeConfig | None = None,
) -> dict[str, Any]:
    """Return a stable status/telemetry payload for wired control constants."""

    target = target_config or TargetRuntimeConfig()
    return {
        "schema": "active-runtime-config-v1",
        "applied_to_behavior": True,
        "values": {
            "range_adapter_obstacle_critical_cm": (
                safety_config.obstacle_too_close_cm
            ),
            "range_adapter_obstacle_detected_cm": (
                safety_config.obstacle_detected_cm
            ),
            "sensor_recovery_sec": target.sensor_recovery_sec,
            "search_entry_distance_cm": target.search_entry_distance_cm,
            "search_speed_cap_mps": target.search_speed_cap_mps,
            "search_lock_observations": target.search_lock_observations,
            "search_interval_sec": target.search_interval_sec,
            "local_track_stale_ms": target.local_track_stale_ms,
            "overhead_invalid_max_sec": target.overhead_invalid_max_sec,
            "handoff_wait_sec": target.handoff_wait_sec,
            "nav_moving_goal_max_hz": target.nav_moving_goal_max_hz,
            "nav_moving_goal_min_displacement_cm": (
                target.nav_moving_goal_min_displacement_cm
            ),
            "nav_completion_xy_cm": target.nav_completion_xy_cm,
            "nav_completion_yaw_rad": target.nav_completion_yaw_rad,
            "nav_completion_dwell_sec": target.nav_completion_dwell_sec,
            "home_file": target.home_file,
            "geofence_file": target.geofence_file,
            "startup_overhead_max_age_ms": target.startup_overhead_max_age_ms,
            "brake_reverse_trigger_cm": target.brake_reverse_trigger_cm,
            "brake_reverse_settle_ms": target.brake_reverse_settle_ms,
            "brake_reverse_duration_sec": target.brake_reverse_duration_sec,
            "brake_reverse_normalized": target.brake_reverse_normalized,
            "brake_reverse_max_attempts": target.brake_reverse_max_attempts,
            "brake_reverse_reset_cm": target.brake_reverse_reset_cm,
            "brake_reverse_reset_sec": target.brake_reverse_reset_sec,
            "overhead_stale_warning_ms": OVERHEAD_STALE_WARNING_MS,
            "vision_stale_ms": VISION_STALE_MS,
            "range_stale_ms": RANGE_STALE_MS,
            "lidar_stale_ms": LIDAR_STALE_MS,
            "navigation_stale_ms": NAVIGATION_STALE_MS,
        },
    }


__all__ = ["active_runtime_config_dict"]
