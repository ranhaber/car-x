"""Validated configuration for the approved target redesign.

The values are loaded and validated as one target schema.  Telemetry reports
which fields have become active during the staged migration; fields not listed
in ``applied_fields`` remain observational.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import os
from typing import Any


_PREFIX = "CAT_FOLLOW_"

# Bounded recording retention defaults (8 GiB retained, 1 GiB kept free).
DEFAULT_RECORDING_QUOTA_BYTES = 8 * 1024**3
DEFAULT_RECORDING_MIN_FREE_BYTES = 1024**3


def _raw(name: str) -> str | None:
    value = os.getenv(f"{_PREFIX}{name}")
    if value is None:
        return None
    value = value.strip()
    return value or None


def _float(name: str, default: float, *, minimum: float = 0.0) -> float:
    raw = _raw(name)
    value = default if raw is None else float(raw)
    if not math.isfinite(value) or value < minimum:
        raise ValueError(f"{_PREFIX}{name} must be finite and >= {minimum:g}")
    return value


def _int(name: str, default: int, *, minimum: int = 0) -> int:
    raw = _raw(name)
    value = default if raw is None else int(raw)
    if value < minimum:
        raise ValueError(f"{_PREFIX}{name} must be >= {minimum}")
    return value


def _bool(name: str, default: bool) -> bool:
    raw = _raw(name)
    if raw is None:
        return default
    normalized = raw.lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{_PREFIX}{name} must be a boolean")


def _optional_float(name: str) -> float | None:
    raw = _raw(name)
    if raw is None:
        return None
    value = float(raw)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{_PREFIX}{name} must be finite and >= 0")
    return value


def _optional_int(name: str) -> int | None:
    raw = _raw(name)
    if raw is None:
        return None
    value = int(raw)
    if value < 0:
        raise ValueError(f"{_PREFIX}{name} must be >= 0")
    return value


def _optional_str(name: str) -> str | None:
    return _raw(name)


@dataclass(frozen=True)
class TargetRuntimeConfig:
    """Canonical redesign knobs activated incrementally by migration slice."""

    search_entry_distance_cm: float = 200.0
    search_speed_cap_mps: float = 0.10
    search_lock_observations: int = 3
    search_interval_sec: float = 10.0
    local_track_stale_ms: int = 350
    overhead_invalid_max_sec: float = 10.0
    sensor_recovery_sec: float = 2.0
    handoff_wait_sec: float = 10.0
    recording_postroll_sec: float = 10.0
    nav_moving_goal_max_hz: float = 2.0
    nav_moving_goal_min_displacement_cm: float = 25.0
    nav_completion_xy_cm: float = 20.0
    nav_completion_yaw_rad: float = 0.3
    nav_completion_dwell_sec: float = 1.0
    brake_reverse_trigger_cm: float = 15.0
    brake_reverse_settle_ms: int = 100
    brake_reverse_duration_sec: float = 0.5
    brake_reverse_normalized: float = -0.30
    brake_reverse_max_attempts: int = 3
    brake_reverse_reset_cm: float = 20.0
    brake_reverse_reset_sec: float = 2.0
    nav_ultrasonic_costmap: bool = True
    nav2_backup_enabled: bool = False
    home_file: str | None = None
    geofence_file: str | None = None
    startup_overhead_max_age_ms: int = 2000
    startup_require_home: bool = False
    startup_require_geofence: bool = False
    overhead_min_confidence: float | None = None
    association_bearing_gate_rad: float | None = None
    recording_dir: str | None = None
    recording_segment_sec: float = 60.0
    # Recording retention must be bounded by default: with no quota and no
    # reserve the SD card fills up and never recovers on its own.
    recording_quota_bytes: int | None = DEFAULT_RECORDING_QUOTA_BYTES
    recording_min_free_bytes: int | None = DEFAULT_RECORDING_MIN_FREE_BYTES
    thermal_critical_return_speed_cap_mps: float = 0.08
    thermal_critical_unsafe: bool = False

    def __post_init__(self) -> None:
        if self.nav_moving_goal_max_hz <= 0.0:
            raise ValueError("nav_moving_goal_max_hz must be > 0")
        if not -1.0 <= self.brake_reverse_normalized < 0.0:
            raise ValueError("brake_reverse_normalized must be in [-1, 0)")
        if self.brake_reverse_reset_cm <= self.brake_reverse_trigger_cm:
            raise ValueError(
                "brake_reverse_reset_cm must exceed brake_reverse_trigger_cm"
            )
        if self.nav2_backup_enabled:
            raise ValueError("CAT_FOLLOW_NAV2_BACKUP_ENABLED must remain disabled")
        if (
            self.overhead_min_confidence is not None
            and self.overhead_min_confidence > 1.0
        ):
            raise ValueError("overhead_min_confidence must be within [0, 1]")
        if self.thermal_critical_return_speed_cap_mps <= 0.0:
            raise ValueError(
                "thermal_critical_return_speed_cap_mps must be > 0"
            )

    def telemetry_dict(self) -> dict[str, Any]:
        """Return a stable status/telemetry payload with migration metadata."""

        values = asdict(self)
        required = (
            "overhead_min_confidence",
            "association_bearing_gate_rad",
            "recording_quota_bytes",
            "recording_min_free_bytes",
        )
        return {
            "schema": "target-runtime-config-v1",
            "applied_to_behavior": False,
            "applied_fields": [
                "search_entry_distance_cm",
                "search_speed_cap_mps",
                "search_lock_observations",
                "search_interval_sec",
                "local_track_stale_ms",
                "overhead_invalid_max_sec",
                "sensor_recovery_sec",
                "handoff_wait_sec",
                "nav_moving_goal_max_hz",
                "nav_moving_goal_min_displacement_cm",
                "nav_completion_xy_cm",
                "nav_completion_yaw_rad",
                "nav_completion_dwell_sec",
                "brake_reverse_trigger_cm",
                "brake_reverse_settle_ms",
                "brake_reverse_duration_sec",
                "brake_reverse_normalized",
                "brake_reverse_max_attempts",
                "brake_reverse_reset_cm",
                "brake_reverse_reset_sec",
                "home_file",
                "geofence_file",
                "startup_overhead_max_age_ms",
                "recording_postroll_sec",
                "recording_dir",
                "recording_segment_sec",
                "recording_quota_bytes",
                "recording_min_free_bytes",
                "nav_ultrasonic_costmap",
                "thermal_critical_return_speed_cap_mps",
            ],
            "values": values,
            "missing_deployment_required": [
                name for name in required if values[name] is None
            ],
        }


def load_target_runtime_config() -> TargetRuntimeConfig:
    """Load and validate canonical target knobs from ``CAT_FOLLOW_*`` env."""

    return TargetRuntimeConfig(
        search_entry_distance_cm=_float("SEARCH_ENTRY_DISTANCE_CM", 200.0),
        search_speed_cap_mps=_float("SEARCH_SPEED_CAP_MPS", 0.10),
        search_lock_observations=_int("SEARCH_LOCK_OBSERVATIONS", 3, minimum=1),
        search_interval_sec=_float("SEARCH_INTERVAL_SEC", 10.0),
        local_track_stale_ms=_int("LOCAL_TRACK_STALE_MS", 350, minimum=1),
        overhead_invalid_max_sec=_float("OVERHEAD_INVALID_MAX_SEC", 10.0),
        sensor_recovery_sec=_float("SENSOR_RECOVERY_SEC", 2.0),
        handoff_wait_sec=_float("HANDOFF_WAIT_SEC", 10.0),
        recording_postroll_sec=_float("RECORDING_POSTROLL_SEC", 10.0),
        nav_moving_goal_max_hz=_float("NAV_MOVING_GOAL_MAX_HZ", 2.0),
        nav_moving_goal_min_displacement_cm=_float(
            "NAV_MOVING_GOAL_MIN_DISPLACEMENT_CM", 25.0
        ),
        nav_completion_xy_cm=_float("NAV_COMPLETION_XY_CM", 20.0),
        nav_completion_yaw_rad=_float("NAV_COMPLETION_YAW_RAD", 0.3),
        nav_completion_dwell_sec=_float("NAV_COMPLETION_DWELL_SEC", 1.0),
        brake_reverse_trigger_cm=_float("BRAKE_REVERSE_TRIGGER_CM", 15.0),
        brake_reverse_settle_ms=_int("BRAKE_REVERSE_SETTLE_MS", 100),
        brake_reverse_duration_sec=_float("BRAKE_REVERSE_DURATION_SEC", 0.5),
        brake_reverse_normalized=_float(
            "BRAKE_REVERSE_NORMALIZED", -0.30, minimum=-1.0
        ),
        brake_reverse_max_attempts=_int(
            "BRAKE_REVERSE_MAX_ATTEMPTS", 3, minimum=1
        ),
        brake_reverse_reset_cm=_float("BRAKE_REVERSE_RESET_CM", 20.0),
        brake_reverse_reset_sec=_float("BRAKE_REVERSE_RESET_SEC", 2.0),
        nav_ultrasonic_costmap=_bool("NAV_ULTRASONIC_COSTMAP", True),
        nav2_backup_enabled=_bool("NAV2_BACKUP_ENABLED", False),
        home_file=_optional_str("HOME_FILE"),
        geofence_file=_optional_str("GEOFENCE_FILE"),
        startup_overhead_max_age_ms=_int(
            "STARTUP_OVERHEAD_MAX_AGE_MS", 2000, minimum=1
        ),
        startup_require_home=_bool("STARTUP_REQUIRE_HOME", False),
        startup_require_geofence=_bool("STARTUP_REQUIRE_GEOFENCE", False),
        overhead_min_confidence=_optional_float("OVERHEAD_MIN_CONFIDENCE"),
        association_bearing_gate_rad=_optional_float(
            "ASSOCIATION_BEARING_GATE_RAD"
        ),
        recording_dir=_optional_str("RECORDING_DIR"),
        recording_segment_sec=_float("RECORDING_SEGMENT_SEC", 60.0, minimum=1.0),
        recording_quota_bytes=_int(
            "RECORDING_QUOTA_BYTES", DEFAULT_RECORDING_QUOTA_BYTES, minimum=1
        ),
        recording_min_free_bytes=_int(
            "RECORDING_MIN_FREE_BYTES",
            DEFAULT_RECORDING_MIN_FREE_BYTES,
            minimum=1,
        ),
        thermal_critical_return_speed_cap_mps=_float(
            "THERMAL_CRITICAL_RETURN_SPEED_CAP_MPS", 0.08
        ),
        thermal_critical_unsafe=_bool("THERMAL_CRITICAL_UNSAFE", False),
    )


__all__ = [
    "DEFAULT_RECORDING_MIN_FREE_BYTES",
    "DEFAULT_RECORDING_QUOTA_BYTES",
    "TargetRuntimeConfig",
    "load_target_runtime_config",
]
