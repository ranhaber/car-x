"""Full autonomous-navigation bringup on the ROCK 4D.

Composes: C1 lidar + robot TF + slam_toolbox localization (map->odom on the
saved yard_map) + Nav2 (composed, embedded-tuned).  Odometry (odom->base_link)
is published separately by cat_follow's odom_publisher, started via the
cat_follow runtime ``--ros-nav`` flag or its own systemd unit.

Nav2 is launched in composition mode via ``nav2_bringup``'s ``navigation``
launch to reduce CPU.  Run on the board with ROS 2 Jazzy sourced::

    ros2 launch cat_follow_bringup rock4d_nav.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("cat_follow_bringup")
    launch_dir = os.path.join(share, "launch")
    nav2_params = os.path.join(share, "config", "nav2_params.yaml")
    slam_localization = os.path.join(share, "config", "slam_localization.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "sllidar_c1.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "tf_urdf.launch.py")
                )
            ),
            Node(
                package="slam_toolbox",
                executable="localization_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_localization, {"use_sim_time": use_sim_time}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                    "use_composition": "True",
                    "autostart": "True",
                }.items(),
            ),
        ]
    )
