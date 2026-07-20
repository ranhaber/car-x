"""Bring up the Slamtec RPLIDAR C1 as a ROS 2 /scan source.

Launches the ``sllidar_ros2`` driver against the stable ``/dev/rplidar``
symlink (see ``cat_follow/scripts/99-rplidar.rules``) at the C1's native
460800 baud, publishing ``sensor_msgs/LaserScan`` on ``/scan`` in the
``laser`` frame.

Verify after launch::

    ros2 topic hz /scan
    ros2 topic echo /scan --once
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/rplidar"),
            DeclareLaunchArgument("serial_baudrate", default_value="460800"),
            DeclareLaunchArgument("frame_id", default_value="laser"),
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_c1",
                output="screen",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "serial_baudrate": serial_baudrate,
                        "frame_id": frame_id,
                        "inverted": False,
                        "angle_compensate": True,
                        "scan_mode": "Standard",
                    }
                ],
            ),
        ]
    )
