"""Publish cat_follow dead-reckoning odometry as ROS 2 ``nav_msgs/Odometry``.

PiCar-X has no wheel encoders, so this node was intended to feed slam_toolbox /
Nav2 the bicycle-model estimate from :mod:`cat_follow.odometry` (corrected
downstream by scan matching).

**Disabled source.** The contract runtime (``runtime/app.py`` +
``DecisionEngine``) never calls :func:`cat_follow.odometry.update`, so this
publisher would emit a *frozen* pose (a static ``odom -> base_link``). Feeding
a stationary odometry estimate to SLAM/Nav2 is unsafe, so constructing
:class:`OdomPublisher` (and :func:`main`) now fails closed with a clear error.
Lidar RF2O is the production owner of ``/odom`` and ``odom -> base_link`` (see
:mod:`cat_follow.navigation.odom_source`).

The pure helpers (:func:`yaw_to_quaternion`, :func:`odometry_reading_m`) remain
importable for unit tests.

``rclpy`` and message imports are guarded so this file imports cleanly on
machines without ROS 2; :func:`main` raises a clear error there.
"""

from __future__ import annotations

import math
from typing import Callable, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node

    _HAS_ROS = True
except Exception:  # pragma: no cover - ROS absent on dev machines
    _HAS_ROS = False
    Node = object  # type: ignore

from cat_follow import odometry as odom
from cat_follow.navigation.odom_source import BICYCLE_ODOM_DISABLED_MSG


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
            # Fail closed: the bicycle odometry source is disabled because the
            # contract runtime never integrates commanded motion, which would
            # make this publisher emit a frozen pose.  Refuse to construct so no
            # stationary /odom or static odom->base_link is ever published.
            raise RuntimeError(BICYCLE_ODOM_DISABLED_MSG)


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
