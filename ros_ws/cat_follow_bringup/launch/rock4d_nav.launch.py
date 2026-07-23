"""Full autonomous-navigation bringup on the ROCK 4D.

Composes: C1 lidar + robot TF + lidar odometry (RF2O, the production/default
odometry source) + slam_toolbox *localization* (map->odom on a saved yard map)
+ Nav2 (composed, embedded-tuned).

Localization requires a saved map.  Point it at one with the ``map_file``
launch argument or the ``CAT_FOLLOW_MAP_FILE`` environment variable (the map's
basename without extension).  This launch validates at startup that the map is
configured and that its serialized ``<basename>.posegraph`` / ``.data`` files
exist, and aborts with a clear error otherwise — normal localization must never
start against an empty/missing map.  (First-time surveying is map-free; use
``mapping.launch.py`` instead.)

Run on the board with ROS 2 Jazzy sourced::

    CAT_FOLLOW_MAP_FILE=/opt/.../maps/yard_map \
      ros2 launch cat_follow_bringup rock4d_nav.launch.py
    # or
    ros2 launch cat_follow_bringup rock4d_nav.launch.py map_file:=/abs/path/yard_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_map_file(map_file: str) -> str:
    """Validate the configured saved map, importing the shared helper if possible."""
    try:
        from cat_follow.navigation.map_config import validate_localization_map

        return validate_localization_map(map_file)
    except ImportError:
        # Fallback mirror of cat_follow.navigation.map_config so the launch is
        # self-contained even if cat_follow is not importable.
        if not map_file:
            raise ValueError(
                "no saved map configured: set CAT_FOLLOW_MAP_FILE (or the "
                "'map_file' launch argument) to the basename of a map saved "
                "during a mapping session. Run mapping.launch.py first; "
                "localization/Nav2 cannot start without a saved map."
            )
        missing = [
            path
            for path in (map_file + ".posegraph", map_file + ".data")
            if not os.path.exists(path)
        ]
        if missing:
            raise FileNotFoundError(
                "configured map is incomplete; expected serialized slam_toolbox "
                f"files are missing: {', '.join(missing)}. Re-run the mapping "
                "session or fix CAT_FOLLOW_MAP_FILE."
            )
        return map_file


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("cat_follow_bringup")
    launch_dir = os.path.join(share, "launch")
    nav2_params = os.path.join(share, "config", "nav2_params.yaml")
    slam_localization = os.path.join(share, "config", "slam_localization.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    raw_map_file = LaunchConfiguration("map_file").perform(context).strip()
    # Strip a stray serialized-map suffix so either the basename or a sidecar
    # path is accepted.
    for suffix in (".posegraph", ".data"):
        if raw_map_file.endswith(suffix):
            raw_map_file = raw_map_file[: -len(suffix)]
            break
    map_file = _validate_map_file(raw_map_file)

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    return [
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
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        Node(
            package="slam_toolbox",
            executable="localization_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                slam_localization,
                {
                    "use_sim_time": use_sim_time,
                    # Override the placeholder in slam_localization.yaml with
                    # the validated saved-map basename.
                    "map_file_name": map_file,
                },
            ],
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


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "map_file",
                default_value=os.environ.get("CAT_FOLLOW_MAP_FILE", ""),
                description=(
                    "Basename (no extension) of the saved slam_toolbox map used "
                    "for localization. Defaults to CAT_FOLLOW_MAP_FILE. Required: "
                    "localization aborts if unset or the serialized files are "
                    "missing."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
