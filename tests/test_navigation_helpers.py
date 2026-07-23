"""Tests for ROS-independent helpers in the navigation bridge/publisher.

These exercise the pure math (quaternion, front-sector reduction, odom unit
conversion) and the fail-closed sanitization helpers without requiring rclpy,
so they run everywhere (ROS may be absent in CI).
"""

import math

import pytest

from cat_follow.navigation.odom_publisher import (
    odometry_reading_m,
    yaw_to_quaternion,
)
from cat_follow.navigation.ros_bridge import (
    MAX_PLANNER_SPEED_MPS,
    MAX_PLANNER_YAW_RATE_RAD_S,
    _front_min_distance_cm,
    _yaw_from_quaternion,
    reduce_front_sector,
    sanitize_cmd_vel,
    sanitize_odom_pose,
)
import cat_follow.odometry as odom


def test_yaw_quaternion_round_trip():
    for yaw in (-2.0, -0.5, 0.0, 0.75, 3.0):
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        recovered = _yaw_from_quaternion(qx, qy, qz, qw)
        # Compare on the circle to avoid +/-pi wrap mismatches.
        assert abs(math.atan2(math.sin(yaw - recovered), math.cos(yaw - recovered))) < 1e-6


def test_front_min_distance_picks_forward_sector():
    # 5 beams spanning -90..+90 deg; the closest is off to the side and must
    # be ignored, leaving the forward beam.
    angle_min = -math.pi / 2
    angle_increment = math.pi / 4  # 45 deg steps -> -90,-45,0,45,90
    ranges = [0.2, 2.0, 1.5, 2.0, 0.3]  # nearest are the +/-90 side beams
    dist_cm = _front_min_distance_cm(
        ranges, angle_min, angle_increment, range_min=0.0, range_max=12.0
    )
    # Forward sector (+/-30 deg) only includes the 0-deg beam (1.5 m).
    assert dist_cm is not None
    assert abs(dist_cm - 150.0) < 1e-6


def test_front_min_distance_ignores_invalid():
    ranges = [float("inf"), float("nan"), 0.0, -1.0]
    dist_cm = _front_min_distance_cm(
        ranges, 0.0, 0.1, range_min=0.0, range_max=12.0
    )
    assert dist_cm is None


def test_front_sector_respects_range_min():
    # A single forward beam below range_min is a sensor artifact, not an
    # imminent obstacle; it must be rejected (not treated as a near collision).
    ranges = [0.05]  # 5 cm, below a 0.15 m range_min
    result = reduce_front_sector(
        ranges, 0.0, 0.1, range_min=0.15, range_max=12.0
    )
    assert result.min_distance_cm is None
    assert result.in_sector_beams == 1
    assert result.usable_beams == 0

    # The same beam is usable once it clears range_min.
    result_ok = reduce_front_sector(
        ranges, 0.0, 0.1, range_min=0.01, range_max=12.0
    )
    assert result_ok.usable_beams == 1
    assert abs(result_ok.min_distance_cm - 5.0) < 1e-6


def test_front_sector_diagnostics_counts():
    # Two beams inside the +/-30 deg sector; one valid, one beyond range_max.
    ranges = [1.0, 100.0]
    result = reduce_front_sector(
        ranges, 0.0, math.radians(10.0), range_min=0.0, range_max=12.0
    )
    assert result.total_beams == 2
    assert result.in_sector_beams == 2
    assert result.usable_beams == 1
    assert abs(result.min_distance_cm - 100.0) < 1e-6


def test_sanitize_cmd_vel_clamps_and_scales():
    pc, sl = sanitize_cmd_vel(MAX_PLANNER_SPEED_MPS, MAX_PLANNER_YAW_RATE_RAD_S)
    assert abs(sl - 1.0) < 1e-9
    assert abs(pc - 1.0) < 1e-9
    # Over-range values clamp to unit authority.
    pc2, sl2 = sanitize_cmd_vel(10.0, -10.0)
    assert sl2 == 1.0
    assert pc2 == -1.0
    # Reverse linear.x still maps to a positive (magnitude) speed limit.
    _, sl3 = sanitize_cmd_vel(-MAX_PLANNER_SPEED_MPS, 0.0)
    assert abs(sl3 - 1.0) < 1e-9


@pytest.mark.parametrize(
    "lin, ang",
    [
        (float("nan"), 0.0),
        (0.0, float("nan")),
        (float("inf"), 0.0),
        (0.0, float("-inf")),
    ],
)
def test_sanitize_cmd_vel_rejects_non_finite(lin, ang):
    # A NaN/inf must NOT be clamped into full authority; it fails closed to None.
    assert sanitize_cmd_vel(lin, ang) is None


def test_sanitize_odom_pose_valid():
    qx, qy, qz, qw = yaw_to_quaternion(math.radians(30.0))
    out = sanitize_odom_pose(1.5, -2.0, qx, qy, qz, qw)
    assert out is not None
    x, y, yaw = out
    assert abs(x - 1.5) < 1e-9
    assert abs(y + 2.0) < 1e-9
    assert abs(yaw - math.radians(30.0)) < 1e-9


@pytest.mark.parametrize(
    "args",
    [
        (float("nan"), 0.0, 0.0, 0.0, 0.0, 1.0),
        (0.0, float("inf"), 0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, float("nan"), 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),  # degenerate (zero-norm) quaternion
    ],
)
def test_sanitize_odom_pose_rejects_invalid(args):
    assert sanitize_odom_pose(*args) is None


def test_odometry_reading_converts_cm_to_m():
    odom.reset(120.0, -80.0, 90.0)
    try:
        x_m, y_m, yaw = odometry_reading_m()
        assert abs(x_m - 1.2) < 1e-9
        assert abs(y_m + 0.8) < 1e-9
        assert abs(yaw - math.radians(90.0)) < 1e-9
    finally:
        odom.reset(0.0, 0.0, 0.0)
