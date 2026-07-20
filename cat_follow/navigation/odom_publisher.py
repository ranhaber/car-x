"""Publish cat_follow dead-reckoning odometry as ROS 2 ``nav_msgs/Odometry``.

PiCar-X has no wheel encoders, so slam_toolbox/Nav2 are fed the bicycle-model
estimate from :mod:`cat_follow.odometry` (corrected downstream by scan
matching).  This node publishes ``/odom`` and broadcasts the dynamic
``odom -> base_link`` transform; the static ``base_link -> laser/camera`` tree
comes from ``robot_state_publisher`` (see ``tf_urdf.launch.py``).

``rclpy`` and message imports are guarded so this file imports cleanly on
machines without ROS 2; :func:`main` raises a clear error there.
"""

from __future__ import annotations

import math
from typing import Callable, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster

    _HAS_ROS = True
except Exception:  # pragma: no cover - ROS absent on dev machines
    _HAS_ROS = False
    Node = object  # type: ignore

from cat_follow import odometry as odom


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    """Return the (x, y, z, w) quaternion for a planar yaw rotation."""
    half = yaw_rad * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def odometry_reading_m() -> Tuple[float, float, float]:
    """Return cat_follow odometry as (x_m, y_m, yaw_rad) in ROS units.

    cat_follow stores position in centimeters and heading in degrees (CCW),
    matching REP-103's base_link convention once converted.
    """
    x_cm, y_cm = odom.get_position()
    yaw_rad = math.radians(odom.get_heading_deg())
    return (x_cm / 100.0, y_cm / 100.0, yaw_rad)


if _HAS_ROS:

    class OdomPublisher(Node):
        """rclpy node publishing /odom + odom->base_link TF at a fixed rate."""

        def __init__(
            self,
            *,
            reader: Callable[[], Tuple[float, float, float]] = odometry_reading_m,
            rate_hz: float = 20.0,
            odom_frame: str = "odom",
            base_frame: str = "base_link",
        ) -> None:
            super().__init__("cat_follow_odom_publisher")
            self._reader = reader
            self._odom_frame = odom_frame
            self._base_frame = base_frame
            self._pub = self.create_publisher(Odometry, "odom", 10)
            self._tf = TransformBroadcaster(self)
            self.create_timer(1.0 / max(rate_hz, 1e-3), self._on_timer)

        def _on_timer(self) -> None:
            x_m, y_m, yaw = self._reader()
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            stamp = self.get_clock().now().to_msg()

            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = self._odom_frame
            msg.child_frame_id = self._base_frame
            msg.pose.pose.position.x = x_m
            msg.pose.pose.position.y = y_m
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            # High covariance on unmeasured DOFs (no encoders / IMU yet).
            msg.pose.covariance[0] = 0.05
            msg.pose.covariance[7] = 0.05
            msg.pose.covariance[35] = 0.1
            self._pub.publish(msg)

            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = x_m
            tf.transform.translation.y = y_m
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf.sendTransform(tf)


def main(args: Optional[list] = None) -> int:
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available; run this on the ROCK 4D with ROS 2 Jazzy "
            "sourced (source /opt/ros/jazzy/setup.bash)."
        )
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


__all__ = ["yaw_to_quaternion", "odometry_reading_m", "main"]
if _HAS_ROS:
    __all__.append("OdomPublisher")


if __name__ == "__main__":
    raise SystemExit(main())
