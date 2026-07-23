"""One-time yard mapping (map-free): C1 lidar + TF + lidar odometry + slam_toolbox.

First-time mapping does NOT require an existing map: slam_toolbox runs in
async *mapping* mode and builds a fresh map from lidar RF2O odometry + /scan.

Run this once with teleop to survey the yard, then save the map::

    ros2 launch cat_follow_bringup mapping.launch.py
    # drive the car slowly around the yard, then:
    ros2 run nav2_map_server map_saver_cli -f \
      $(ros2 pkg prefix cat_follow_bringup)/share/cat_follow_bringup/maps/yard_map

That also serializes the slam_toolbox posegraph. Save it for localization::

    ros2 service call /slam_toolbox/serialize_map \
      slam_toolbox/srv/SerializePoseGraph "{filename: '.../maps/yard_map'}"

Copy the resulting ``yard_map.*`` files back into
``ros_ws/cat_follow_bringup/maps/`` and set CAT_FOLLOW_MAP_FILE to that basename
for ``rock4d_nav.launch.py``.

Preflight readiness: after launch, confirm the odometry chain is healthy before
driving::

    ros2 topic hz /scan                         # ~10 Hz from the C1
    ros2 topic hz /odom                          # RF2O odometry
    ros2 run tf2_ros tf2_echo odom base_link     # live odom->base_link TF

Local ``odom -> base_link`` comes from RF2O (the production/default and only
supported odometry source). The bicycle OdomPublisher is disabled, so mapping
always forces lidar RF2O regardless of CAT_FOLLOW_ODOM_SOURCE.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _preflight(context, *args, **kwargs):
    """Force/validate lidar odom mode and surface readiness guidance."""
    messages = []
    source = os.environ.get("CAT_FOLLOW_ODOM_SOURCE", "lidar").strip().lower()
    if source and source != "lidar":
        messages.append(
            LogInfo(
                msg=(
                    f"[mapping preflight] CAT_FOLLOW_ODOM_SOURCE={source!r} is "
                    "ignored; mapping forces lidar RF2O odometry (bicycle source "
                    "is disabled)."
                )
            )
        )
    messages.append(
        LogInfo(
            msg=(
                "[mapping preflight] Verify readiness before driving: "
                "'ros2 topic hz /scan' (~10 Hz), 'ros2 topic hz /odom' (RF2O), "
                "'ros2 run tf2_ros tf2_echo odom base_link' (live TF)."
            )
        )
    )
    return messages


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("cat_follow_bringup")
    launch_dir = os.path.join(share, "launch")
    slam_params = os.path.join(share, "config", "slam_mapper.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_preflight),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "lidar_odom.launch.py")
                ),
                # Mapping always needs local odometry; force RF2O on regardless
                # of CAT_FOLLOW_ODOM_SOURCE (bicycle is disabled).
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "enabled": "true",
                }.items(),
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["slam_toolbox"],
                    }
                ],
            ),
        ]
    )
