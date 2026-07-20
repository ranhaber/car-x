"""Publish the robot TF tree from the PiCar-X URDF.

``robot_state_publisher`` reads ``urdf/picarx_lidar.urdf`` and emits the static
``base_link -> laser`` / ``base_link -> camera_link`` transforms Nav2 and
slam_toolbox need.  The dynamic ``odom -> base_link`` transform is published
separately by cat_follow's odom_publisher (``cat_follow.navigation``).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("cat_follow_bringup")
    urdf_path = os.path.join(share, "urdf", "picarx_lidar.urdf")
    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
        ]
    )
