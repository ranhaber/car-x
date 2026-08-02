"""Tests for active runtime configuration snapshots."""

from __future__ import annotations

from cat_follow.active_config import active_runtime_config_dict
from cat_follow.safety_config import SafetyConfig


def test_active_runtime_config_defaults():
    payload = active_runtime_config_dict(SafetyConfig())
    assert payload["schema"] == "active-runtime-config-v1"
    assert payload["applied_to_behavior"] is True
    values = payload["values"]
    assert values["range_adapter_obstacle_critical_cm"] == 10.0
    assert values["range_adapter_obstacle_detected_cm"] == 50.0
    assert values["sensor_recovery_sec"] == 2.0
    assert values["brake_reverse_trigger_cm"] == 15.0
    assert values["brake_reverse_normalized"] == -0.30
    assert values["brake_reverse_max_attempts"] == 3
    assert values["search_entry_distance_cm"] == 200.0
    assert values["search_speed_cap_mps"] == 0.10
    assert values["search_lock_observations"] == 3
    assert values["overhead_invalid_max_sec"] == 10.0
    assert values["handoff_wait_sec"] == 10.0
    assert values["overhead_stale_warning_ms"] == 300
    assert values["vision_stale_ms"] == 350
    assert values["range_stale_ms"] == 500
    assert values["lidar_stale_ms"] == 500
    assert values["navigation_stale_ms"] == 500
