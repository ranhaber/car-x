"""One-time yard mapping: C1 lidar + TF + slam_toolbox online_async.

Run this once with teleop to survey the yard, then save the map::

    ros2 launch cat_follow_bringup mapping.launch.py
    # drive the car slowly around the yard, then:
    ros2 run nav2_map_server map_saver_cli -f \
      $(ros2 pkg prefix cat_follow_bringup)/share/cat_follow_bringup/maps/yard_map

Copy the resulting ``yard_map.yaml`` + ``yard_map.pgm`` back into
``ros_ws/cat_follow_bringup/maps/`` and commit them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("cat_follow_bringup")
    launch_dir = os.path.join(share, "launch")
    slam_params = os.path.join(share, "config", "slam_mapper.yaml")

    return LaunchDescription(
        [
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
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params],
            ),
        ]
    )
