"""RPLidar C1 scan-matching odometry using RF2O.

RF2O consumes ``/scan`` and is the production/default sole publisher of
``/odom`` and the dynamic ``odom -> base_link`` transform. The bicycle
odometry source is disabled (it would publish frozen odometry), so RF2O is
enabled by default for any configuration; the ``enabled`` argument can still
force it off for bench tests.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _lidar_odom_enabled_from_env() -> bool:
    """Mirror ``cat_follow.navigation.odom_source.lidar_odom_launch_enabled``.

    Bicycle odometry is disabled, so lidar RF2O is always the enabled default.
    """
    return True


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("cat_follow_bringup")
    params_file = os.path.join(share, "config", "lidar_odom.yaml")

    enabled = LaunchConfiguration("enabled")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enabled",
                default_value="true" if _lidar_odom_enabled_from_env() else "false",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry",
                output="screen",
                condition=IfCondition(enabled),
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
