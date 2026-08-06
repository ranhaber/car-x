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
    # Look/drive (see Look_Drive_Path_Design.md)
    look_n_enter_px: float = 40.0
    look_n_exit_px: float = 80.0
    look_center_deadband_px: float = 8.0
    look_pan_slew_deg_s: float = 90.0
    look_pan_forward_deadband_deg: float = 2.0
    look_pan_reset_timeout_ms: int = 800
    look_mode_dwell_ms: int = 400
    look_control_period_ms: int = 20
    look_frame_half_width_px: float = 320.0
    look_px_per_deg: float = 4.0
    look_path_turn_threshold: float = 0.35
    look_pan_forward_deg: float = 0.0
    # Steering envelope
    envelope_provider: str = "costmap_sweep"  # costmap_sweep | point
    envelope_lookahead_m: float = 0.6
    envelope_sample_count: int = 21
    envelope_stale_ttl_ms: int = 500
    envelope_max_half_width: float = 0.85
    envelope_lethal_cost: int = 50
    envelope_wheelbase_m: float = 0.15
    envelope_footprint_length_m: float = 0.25
    envelope_footprint_width_m: float = 0.18

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
        if self.look_n_exit_px <= self.look_n_enter_px:
            raise ValueError("look_n_exit_px must exceed look_n_enter_px")
        if self.look_pan_slew_deg_s <= 0.0:
            raise ValueError("look_pan_slew_deg_s must be > 0")
        if self.look_px_per_deg <= 0.0:
            raise ValueError("look_px_per_deg must be > 0")
        if self.envelope_provider not in {"costmap_sweep", "point"}:
            raise ValueError(
                "envelope_provider must be 'costmap_sweep' or 'point'"
            )
        if self.envelope_sample_count < 3 or self.envelope_sample_count % 2 == 0:
            raise ValueError("envelope_sample_count must be odd and >= 3")
        if self.envelope_lookahead_m <= 0.0:
            raise ValueError("envelope_lookahead_m must be > 0")
        if not 0.0 < self.envelope_max_half_width <= 1.0:
            raise ValueError("envelope_max_half_width must be in (0, 1]")

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
                "look_n_enter_px",
                "look_n_exit_px",
                "look_pan_slew_deg_s",
                "look_pan_forward_deadband_deg",
                "look_pan_reset_timeout_ms",
                "look_mode_dwell_ms",
                "envelope_provider",
                "envelope_lookahead_m",
                "envelope_sample_count",
                "envelope_stale_ttl_ms",
                "envelope_max_half_width",
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
        look_n_enter_px=_float("LOOK_N_ENTER_PX", 40.0),
        look_n_exit_px=_float("LOOK_N_EXIT_PX", 80.0),
        look_center_deadband_px=_float("LOOK_CENTER_DEADBAND_PX", 8.0),
        look_pan_slew_deg_s=_float("LOOK_PAN_SLEW_DEG_S", 90.0),
        look_pan_forward_deadband_deg=_float(
            "LOOK_PAN_FORWARD_DEADBAND_DEG", 2.0
        ),
        look_pan_reset_timeout_ms=_int("LOOK_PAN_RESET_TIMEOUT_MS", 800, minimum=1),
        look_mode_dwell_ms=_int("LOOK_MODE_DWELL_MS", 400, minimum=0),
        look_control_period_ms=_int("LOOK_CONTROL_PERIOD_MS", 20, minimum=1),
        look_frame_half_width_px=_float("LOOK_FRAME_HALF_WIDTH_PX", 320.0),
        look_px_per_deg=_float("LOOK_PX_PER_DEG", 4.0),
        look_path_turn_threshold=_float("LOOK_PATH_TURN_THRESHOLD", 0.35),
        look_pan_forward_deg=_float(
            "LOOK_PAN_FORWARD_DEG", 0.0, minimum=-90.0
        ),
        envelope_provider=(
            _raw("ENVELOPE_PROVIDER") or "costmap_sweep"
        ).strip().lower(),
        envelope_lookahead_m=_float("ENVELOPE_LOOKAHEAD_M", 0.6),
        envelope_sample_count=_int("ENVELOPE_SAMPLE_COUNT", 21, minimum=3),
        envelope_stale_ttl_ms=_int("ENVELOPE_STALE_TTL_MS", 500, minimum=1),
        envelope_max_half_width=_float("ENVELOPE_MAX_HALF_WIDTH", 0.85),
        envelope_lethal_cost=_int("ENVELOPE_LETHAL_COST", 50, minimum=1),
        envelope_wheelbase_m=_float("ENVELOPE_WHEELBASE_M", 0.15),
        envelope_footprint_length_m=_float("ENVELOPE_FOOTPRINT_LENGTH_M", 0.25),
        envelope_footprint_width_m=_float("ENVELOPE_FOOTPRINT_WIDTH_M", 0.18),
    )


__all__ = [
    "DEFAULT_RECORDING_MIN_FREE_BYTES",
    "DEFAULT_RECORDING_QUOTA_BYTES",
    "TargetRuntimeConfig",
    "load_target_runtime_config",
]
