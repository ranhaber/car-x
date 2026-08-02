"""NAV-15 ultrasonic Range message + dual-sensor health contracts."""

import math

import pytest

from cat_follow.control.types import RangeBackend, RangeState
from cat_follow.navigation.dual_sensor_health import build_dual_sensor_health
from cat_follow.navigation.ultrasonic_range import (
    ULTRASONIC_COSTMAP_STALE_MS,
    ULTRASONIC_FRAME_ID,
    ULTRASONIC_RANGE_TOPIC,
    UltrasonicRangeValidationError,
    build_ultrasonic_range_message,
    maybe_build_from_range_state,
    ultrasonic_reading_is_fresh,
    validate_ultrasonic_range_dict,
)


def test_nav_15_builds_validated_range_message():
    msg = build_ultrasonic_range_message(118.0, stamp_sec=12.5)
    assert msg.topic == ULTRASONIC_RANGE_TOPIC
    assert msg.frame_id == ULTRASONIC_FRAME_ID
    assert msg.radiation_type == 0
    assert msg.range == pytest.approx(1.18)
    assert msg.field_of_view == pytest.approx(math.radians(15.0))
    assert msg.costmap_layer_enabled is True
    validate_ultrasonic_range_dict(msg.to_dict())


def test_nav_15_rejects_non_finite_or_out_of_envelope():
    with pytest.raises(UltrasonicRangeValidationError):
        build_ultrasonic_range_message(float("nan"), stamp_sec=1.0)
    with pytest.raises(UltrasonicRangeValidationError):
        build_ultrasonic_range_message(0.5, stamp_sec=1.0)  # < min 2 cm
    with pytest.raises(UltrasonicRangeValidationError):
        build_ultrasonic_range_message(500.0, stamp_sec=1.0)  # > max 4 m


def test_nav_15_costmap_disable_still_builds_direct_safety_payload():
    msg = build_ultrasonic_range_message(
        50.0, stamp_sec=1.0, costmap_layer_enabled=False
    )
    assert msg.costmap_layer_enabled is False
    assert maybe_build_from_range_state(
        50.0, stamp_sec=1.0, costmap_layer_enabled=False
    ) is not None


def test_ultrasonic_reading_is_fresh_within_ttl():
    assert ultrasonic_reading_is_fresh(900, 1000) is True
    assert ultrasonic_reading_is_fresh(
        500, 1000, stale_ttl_ms=ULTRASONIC_COSTMAP_STALE_MS
    ) is True
    assert ultrasonic_reading_is_fresh(499, 1000) is False
    assert ultrasonic_reading_is_fresh(0, 1000) is False
    assert ultrasonic_reading_is_fresh(-1, 1000) is False


def test_maybe_build_rejects_stale_shared_state_for_costmap():
    """A dead HC-SR04 must not keep feeding Nav2 a freshly stamped ghost."""

    assert (
        maybe_build_from_range_state(
            80.0,
            stamp_sec=1.0,
            costmap_layer_enabled=True,
            received_ms=400,
            now_ms=1000,
        )
        is None
    )
    fresh = maybe_build_from_range_state(
        80.0,
        stamp_sec=1.0,
        costmap_layer_enabled=True,
        received_ms=600,
        now_ms=1000,
    )
    assert fresh is not None
    assert fresh.range == pytest.approx(0.80)


def test_maybe_build_without_age_keeps_legacy_host_validation_path():
    # Host unit tests that only have a distance still validate the envelope.
    assert maybe_build_from_range_state(
        50.0, stamp_sec=1.0, costmap_layer_enabled=True
    ) is not None


def test_dual_sensor_health_hold_fields():
    ultrasonic = RangeState(
        received_ms=900,
        fresh=True,
        backend=RangeBackend.ULTRASONIC,
        distance_cm=100.0,
        confidence=1.0,
    )
    lidar = RangeState(
        received_ms=100,
        fresh=False,
        backend=RangeBackend.LIDAR_C1,
        distance_cm=None,
        confidence=0.0,
    )
    health = build_dual_sensor_health(
        ultrasonic=ultrasonic,
        lidar=lidar,
        now_ms=1000,
        ultrasonic_stale_ms=500,
        lidar_stale_ms=500,
        hold_active=True,
        hold_started_ms=800,
        hold_reason="sensor_health_hold",
        recovery_deadline_ms=2800,
        costmap_layer_enabled=True,
    )
    payload = health.to_dict()
    assert payload["required_for_motion"] is True
    assert payload["hold_active"] is True
    assert payload["ultrasonic"]["valid"] is True
    assert payload["ultrasonic"]["frame_id"] == ULTRASONIC_FRAME_ID
    assert payload["ultrasonic"]["costmap_layer_enabled"] is True
    assert payload["lidar"]["faulted"] is True
    assert payload["recovery_deadline_ms"] == 2800


def test_nav2_params_include_range_sensor_layer():
    from pathlib import Path

    text = (
        Path(__file__).resolve().parents[1]
        / "ros_ws"
        / "cat_follow_bringup"
        / "config"
        / "nav2_params.yaml"
    ).read_text(encoding="utf-8")
    assert "nav2_costmap_2d::RangeSensorLayer" in text
    assert "/ultrasonic_range" in text
    assert "range_sensor_layer" in text


def test_urdf_includes_ultrasonic_link():
    from pathlib import Path

    text = (
        Path(__file__).resolve().parents[1]
        / "ros_ws"
        / "cat_follow_bringup"
        / "urdf"
        / "picarx_lidar.urdf"
    ).read_text(encoding="utf-8")
    assert 'link name="ultrasonic_link"' in text
