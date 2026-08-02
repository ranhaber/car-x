"""Tests for observational canonical target configuration."""

import pytest

from cat_follow.target_config import (
    DEFAULT_RECORDING_MIN_FREE_BYTES,
    DEFAULT_RECORDING_QUOTA_BYTES,
    load_target_runtime_config,
)


_TARGET_ENV_NAMES = (
    "SEARCH_ENTRY_DISTANCE_CM",
    "SEARCH_SPEED_CAP_MPS",
    "SEARCH_LOCK_OBSERVATIONS",
    "SEARCH_INTERVAL_SEC",
    "LOCAL_TRACK_STALE_MS",
    "OVERHEAD_INVALID_MAX_SEC",
    "SENSOR_RECOVERY_SEC",
    "HANDOFF_WAIT_SEC",
    "RECORDING_POSTROLL_SEC",
    "NAV_MOVING_GOAL_MAX_HZ",
    "NAV_MOVING_GOAL_MIN_DISPLACEMENT_CM",
    "NAV_COMPLETION_XY_CM",
    "NAV_COMPLETION_YAW_RAD",
    "NAV_COMPLETION_DWELL_SEC",
    "BRAKE_REVERSE_TRIGGER_CM",
    "BRAKE_REVERSE_SETTLE_MS",
    "BRAKE_REVERSE_DURATION_SEC",
    "BRAKE_REVERSE_NORMALIZED",
    "BRAKE_REVERSE_MAX_ATTEMPTS",
    "BRAKE_REVERSE_RESET_CM",
    "BRAKE_REVERSE_RESET_SEC",
    "NAV_ULTRASONIC_COSTMAP",
    "NAV2_BACKUP_ENABLED",
    "HOME_FILE",
    "GEOFENCE_FILE",
    "STARTUP_OVERHEAD_MAX_AGE_MS",
    "STARTUP_REQUIRE_HOME",
    "STARTUP_REQUIRE_GEOFENCE",
    "OVERHEAD_MIN_CONFIDENCE",
    "ASSOCIATION_BEARING_GATE_RAD",
    "RECORDING_DIR",
    "RECORDING_SEGMENT_SEC",
    "RECORDING_QUOTA_BYTES",
    "RECORDING_MIN_FREE_BYTES",
    "THERMAL_CRITICAL_RETURN_SPEED_CAP_MPS",
    "THERMAL_CRITICAL_UNSAFE",
)


def _clear_target_env(monkeypatch):
    for name in _TARGET_ENV_NAMES:
        monkeypatch.delenv(f"CAT_FOLLOW_{name}", raising=False)


def test_target_config_reports_partially_applied_canonical_defaults(monkeypatch):
    _clear_target_env(monkeypatch)

    cfg = load_target_runtime_config()
    telemetry = cfg.telemetry_dict()

    assert cfg.brake_reverse_trigger_cm == 15.0
    assert cfg.sensor_recovery_sec == 2.0
    assert cfg.overhead_invalid_max_sec == 10.0
    assert cfg.nav_moving_goal_max_hz == 2.0
    assert cfg.nav_moving_goal_min_displacement_cm == 25.0
    assert telemetry["applied_to_behavior"] is False
    assert "sensor_recovery_sec" in telemetry["applied_fields"]
    assert "brake_reverse_trigger_cm" in telemetry["applied_fields"]
    assert "search_entry_distance_cm" in telemetry["applied_fields"]
    assert "search_lock_observations" in telemetry["applied_fields"]
    assert "overhead_invalid_max_sec" in telemetry["applied_fields"]
    assert "handoff_wait_sec" in telemetry["applied_fields"]
    assert "nav_moving_goal_max_hz" in telemetry["applied_fields"]
    assert "nav_moving_goal_min_displacement_cm" in telemetry["applied_fields"]
    assert "nav_completion_xy_cm" in telemetry["applied_fields"]
    assert "nav_completion_yaw_rad" in telemetry["applied_fields"]
    assert "nav_completion_dwell_sec" in telemetry["applied_fields"]
    assert "home_file" in telemetry["applied_fields"]
    assert "geofence_file" in telemetry["applied_fields"]
    assert "startup_overhead_max_age_ms" in telemetry["applied_fields"]
    assert "recording_postroll_sec" in telemetry["applied_fields"]
    assert "recording_dir" in telemetry["applied_fields"]
    assert "recording_segment_sec" in telemetry["applied_fields"]
    assert "nav_ultrasonic_costmap" in telemetry["applied_fields"]
    assert "thermal_critical_return_speed_cap_mps" in telemetry["applied_fields"]
    # Recording retention ships bounded defaults so an unconfigured deployment
    # still reclaims disk instead of filling the card.
    assert cfg.recording_quota_bytes == DEFAULT_RECORDING_QUOTA_BYTES
    assert cfg.recording_min_free_bytes == DEFAULT_RECORDING_MIN_FREE_BYTES
    assert set(telemetry["missing_deployment_required"]) == {
        "overhead_min_confidence",
        "association_bearing_gate_rad",
    }


def test_target_config_env_overrides(monkeypatch):
    _clear_target_env(monkeypatch)
    monkeypatch.setenv("CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM", "17.5")
    monkeypatch.setenv("CAT_FOLLOW_BRAKE_REVERSE_RESET_CM", "23")
    monkeypatch.setenv("CAT_FOLLOW_NAV_MOVING_GOAL_MAX_HZ", "1.5")
    monkeypatch.setenv("CAT_FOLLOW_RECORDING_QUOTA_BYTES", "1000000")

    cfg = load_target_runtime_config()

    assert cfg.brake_reverse_trigger_cm == 17.5
    assert cfg.brake_reverse_reset_cm == 23.0
    assert cfg.nav_moving_goal_max_hz == 1.5
    assert cfg.recording_quota_bytes == 1_000_000


def test_target_config_rejects_unsafe_relationships(monkeypatch):
    _clear_target_env(monkeypatch)
    monkeypatch.setenv("CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM", "25")
    monkeypatch.setenv("CAT_FOLLOW_BRAKE_REVERSE_RESET_CM", "20")

    with pytest.raises(ValueError, match="reset_cm must exceed"):
        load_target_runtime_config()


def test_target_config_keeps_nav2_backup_disabled(monkeypatch):
    _clear_target_env(monkeypatch)
    monkeypatch.setenv("CAT_FOLLOW_NAV2_BACKUP_ENABLED", "1")

    with pytest.raises(ValueError, match="must remain disabled"):
        load_target_runtime_config()
