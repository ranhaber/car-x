"""Bridge ROS 2 navigation topics into cat_follow contract state.

This rclpy node is the *only* coupling point between the ROS 2 stack and the
cat_follow runtime.  It never commands motors (only ``DecisionEngine`` does);
it translates topics into the existing contract dataclasses:

- ``/scan``     -> lidar :class:`RangeState` (backend ``LIDAR_C1``) via
                   ``SharedState.update_lidar_range``, fused with the
                   ultrasonic ``RangeAdapter`` inside ``DecisionEngine``.
- ``/odom``     -> :class:`NavigationState.heading` / ``heading_valid``.
- ``/cmd_vel``  -> :class:`NavigationState.path_correction` (from angular.z)
                   and ``speed_limit`` (from linear.x scaled by max speed).

``rclpy`` and message imports are guarded so the module imports cleanly on
machines without ROS 2; :func:`main` and :func:`spin_in_thread` raise a clear
error there.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist

    _HAS_ROS = True
except Exception:  # pragma: no cover - ROS absent on dev machines
    _HAS_ROS = False
    Node = object  # type: ignore

from cat_follow.control.decision_engine import OBSTACLE_TOO_CLOSE_CM
from cat_follow.control.types import (
    NavigationState,
    RangeBackend,
    RangeState,
)
from cat_follow.runtime.shared_state import SharedState, now_monotonic_ms


# Front sector half-angle (radians) used to reduce /scan to a single forward
# obstacle distance for the veto.  +/-30 deg around straight ahead.
FRONT_HALF_ANGLE_RAD = math.radians(30.0)

# Speed at/above which linear.x maps to speed_limit == 1.0 (m/s).
MAX_PLANNER_SPEED_MPS = 0.30

# Max |angular.z| (rad/s) that maps to path_correction == +/-1.0.
MAX_PLANNER_YAW_RATE = 1.5


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return the planar yaw (rad) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _front_min_distance_cm(
    ranges, angle_min: float, angle_increment: float, range_max: float
) -> Optional[float]:
    """Minimum valid range (cm) within the front sector, or None."""
    best_m: Optional[float] = None
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.0 or r > range_max:
            continue
        angle = angle_min + i * angle_increment
        # Normalize to [-pi, pi] and keep only the forward sector.
        angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
        if abs(angle) > FRONT_HALF_ANGLE_RAD:
            continue
        if best_m is None or r < best_m:
            best_m = r
    return None if best_m is None else best_m * 100.0


if _HAS_ROS:

    class RosBridge(Node):
        """rclpy node writing NavigationState + lidar RangeState."""

        def __init__(
            self,
            shared_state: SharedState,
            *,
            obstacle_detected_cm: float = 50.0,
            obstacle_critical_cm: float = OBSTACLE_TOO_CLOSE_CM,
        ) -> None:
            super().__init__("cat_follow_ros_bridge")
            self._ss = shared_state
            self._obstacle_detected_cm = float(obstacle_detected_cm)
            self._obstacle_critical_cm = float(obstacle_critical_cm)

            self.create_subscription(
                LaserScan, "scan", self._on_scan, qos_profile_sensor_data
            )
            self.create_subscription(Odometry, "odom", self._on_odom, 10)
            self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

            # Cached navigation fields updated from separate topics.
            self._heading = 0.0
            self._heading_valid = False
            self._path_correction = 0.0
            self._speed_limit = 0.0

        # ── /scan -> lidar RangeState ────────────────────────────────

        def _on_scan(self, msg) -> None:  # noqa: ANN001
            dist_cm = _front_min_distance_cm(
                msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max
            )
            now = now_monotonic_ms()
            if dist_cm is None:
                state = RangeState(
                    timestamp_ms=int(time.time() * 1000),
                    received_ms=now,
                    fresh=True,
                    authority="RosBridge",
                    backend=RangeBackend.LIDAR_C1,
                    distance_cm=None,
                    confidence=0.0,
                )
            else:
                detected = dist_cm < self._obstacle_detected_cm
                critical = dist_cm < self._obstacle_critical_cm
                span = self._obstacle_detected_cm - self._obstacle_critical_cm
                if dist_cm >= self._obstacle_detected_cm:
                    severity = 0.0
                elif dist_cm <= self._obstacle_critical_cm or span <= 0:
                    severity = 1.0
                else:
                    severity = (self._obstacle_detected_cm - dist_cm) / span
                state = RangeState(
                    timestamp_ms=int(time.time() * 1000),
                    received_ms=now,
                    fresh=True,
                    authority="RosBridge",
                    backend=RangeBackend.LIDAR_C1,
                    distance_cm=dist_cm,
                    confidence=1.0,
                    obstacle_detected=detected,
                    obstacle_critical=critical,
                    obstacle_severity=severity,
                    zone="front",
                )
            self._ss.update_lidar_range(state)

        # ── /odom -> NavigationState.heading ─────────────────────────

        def _on_odom(self, msg) -> None:  # noqa: ANN001
            q = msg.pose.pose.orientation
            self._heading = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
            self._heading_valid = True
            self._publish_navigation()

        # ── /cmd_vel -> path_correction + speed_limit ────────────────

        def _on_cmd_vel(self, msg) -> None:  # noqa: ANN001
            self._path_correction = max(
                -1.0, min(1.0, msg.angular.z / MAX_PLANNER_YAW_RATE)
            )
            self._speed_limit = max(
                0.0, min(1.0, abs(msg.linear.x) / MAX_PLANNER_SPEED_MPS)
            )
            self._publish_navigation()

        def _publish_navigation(self) -> None:
            now = now_monotonic_ms()
            self._ss.update_navigation(
                NavigationState(
                    timestamp_ms=int(time.time() * 1000),
                    received_ms=now,
                    fresh=True,
                    authority="RosBridge",
                    heading=self._heading,
                    heading_valid=self._heading_valid,
                    speed_limit=self._speed_limit,
                    path_correction=self._path_correction,
                )
            )


def spin_in_thread(shared_state: SharedState) -> "threading.Thread":
    """Start rclpy on a daemon thread running the bridge + odom publisher.

    Both nodes share the app's process (and therefore the same SharedState and
    ``cat_follow.odometry`` state) under one SingleThreadedExecutor.
    """
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available; run on the ROCK 4D with ROS 2 Jazzy sourced."
        )

    def _run() -> None:
        from rclpy.executors import SingleThreadedExecutor

        rclpy.init()
        nodes = [RosBridge(shared_state)]
        try:
            from cat_follow.navigation.odom_publisher import OdomPublisher

            nodes.append(OdomPublisher())
        except Exception:  # noqa: BLE001 - odom publisher optional
            pass

        executor = SingleThreadedExecutor()
        for node in nodes:
            executor.add_node(node)
        try:
            executor.spin()
        finally:
            for node in nodes:
                node.destroy_node()
            rclpy.shutdown()

    thread = threading.Thread(target=_run, name="CatFollow-RosBridge", daemon=True)
    thread.start()
    return thread


def request_shutdown() -> None:
    """Best-effort rclpy shutdown so a spinning bridge thread returns."""
    if not _HAS_ROS:
        return
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:  # pragma: no cover - best effort
        pass


def main(args: Optional[list] = None) -> int:  # pragma: no cover - needs ROS
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available; run on the ROCK 4D with ROS 2 Jazzy sourced."
        )
    rclpy.init(args=args)
    node = RosBridge(SharedState())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


__all__ = [
    "spin_in_thread",
    "request_shutdown",
    "main",
    "_front_min_distance_cm",
    "_yaw_from_quaternion",
]
if _HAS_ROS:
    __all__.append("RosBridge")
