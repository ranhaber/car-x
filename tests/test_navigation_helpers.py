"""Tests for ROS-independent helpers in the navigation bridge/publisher.

These exercise the pure math (quaternion, front-sector reduction, odom unit
conversion) without requiring rclpy, so they run everywhere.
"""

import math

from cat_follow.navigation.odom_publisher import (
    odometry_reading_m,
    yaw_to_quaternion,
)
from cat_follow.navigation.ros_bridge import (
    _front_min_distance_cm,
    _yaw_from_quaternion,
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
    dist_cm = _front_min_distance_cm(ranges, angle_min, angle_increment, range_max=12.0)
    # Forward sector (+/-30 deg) only includes the 0-deg beam (1.5 m).
    assert dist_cm is not None
    assert abs(dist_cm - 150.0) < 1e-6


def test_front_min_distance_ignores_invalid():
    ranges = [float("inf"), float("nan"), 0.0, -1.0]
    dist_cm = _front_min_distance_cm(ranges, 0.0, 0.1, range_max=12.0)
    assert dist_cm is None


def test_odometry_reading_converts_cm_to_m():
    odom.reset(120.0, -80.0, 90.0)
    try:
        x_m, y_m, yaw = odometry_reading_m()
        assert abs(x_m - 1.2) < 1e-9
        assert abs(y_m + 0.8) < 1e-9
        assert abs(yaw - math.radians(90.0)) < 1e-9
    finally:
        odom.reset(0.0, 0.0, 0.0)
