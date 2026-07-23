"""Bridge ROS 2 navigation topics into cat_follow contract state.

This rclpy node is the *only* coupling point between the ROS 2 stack and the
cat_follow runtime.  It never commands motors (only ``DecisionEngine`` does);
it translates topics into the existing contract dataclasses:

- ``/scan``     -> lidar :class:`RangeState` (backend ``LIDAR_C1``) via
                   ``SharedState.update_lidar_range``, fused with the
                   ultrasonic ``RangeAdapter`` inside ``DecisionEngine``.
                   Also feeds a downsampled scan overlay for the web map.
- ``/odom``     -> :class:`NavigationState.heading` / ``heading_valid``.
- ``/cmd_vel``  -> :class:`NavigationState.path_correction` (from angular.z)
                   and ``speed_limit`` (from linear.x scaled by max speed).
                   The bridge subscribes to the *smoothed* command that Nav2's
                   velocity_smoother emits (``cmd_vel_smoothed`` by default),
                   i.e. the final velocity after accel/decel limiting — not the
                   raw controller output.
- ``/map``      -> web-UI occupancy snapshot (:mod:`map_snapshot`).
- TF ``map->base_link`` (fallback ``odom->base_link``) -> web-UI robot pose.

Safety-critical work (the ``/scan`` front-sector reduction that feeds the lidar
veto) runs directly on the executor callback.  The heavier, non-safety map /
scan-overlay downsampling is handed to a background worker thread so it can
never block the single-threaded safety callback path.

``rclpy`` and message imports are guarded so the module imports cleanly on
machines without ROS 2; :func:`main` and :func:`spin_in_thread` raise a clear
error there.
"""

from __future__ import annotations

import math
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid, Odometry
    from geometry_msgs.msg import Twist

    _HAS_ROS = True
except Exception:  # pragma: no cover - ROS absent on dev machines
    _HAS_ROS = False
    Node = object  # type: ignore

from cat_follow.safety_config import DEFAULT_OBSTACLE_TOO_CLOSE_CM
from cat_follow.control.types import (
    NavigationState,
    RangeBackend,
    RangeState,
)
from cat_follow.navigation.map_snapshot import (
    publish_map_grid,
    publish_robot_pose,
    publish_scan_overlay,
)
from cat_follow.navigation.odom_source import BICYCLE_ODOM_DISABLED_MSG
from cat_follow.runtime.shared_state import SharedState, now_monotonic_ms


# Front sector half-angle (radians) used to reduce /scan to a single forward
# obstacle distance for the veto.  +/-30 deg around straight ahead.
FRONT_HALF_ANGLE_RAD = math.radians(30.0)

# Speed at/above which linear.x maps to speed_limit == 1.0 (m/s).
MAX_PLANNER_SPEED_MPS = 0.30

# Max |angular.z| (rad/s) that maps to path_correction == +/-1.0.
MAX_PLANNER_YAW_RATE_RAD_S = 1.5

# How often to sample TF for the web-UI pose (Hz).
POSE_PUBLISH_HZ = 5.0

# Age (ms) beyond which the last /cmd_vel is no longer allowed to drive.  A
# silent planner (Nav2 stops publishing) must not keep the last velocity
# authoritative just because /odom is still arriving.
CMD_VEL_STALE_MS = 500

# Minimum interval (s) between repeated rate-limited diagnostics so a persistent
# fault (empty sector, bad TF, overlay error) logs meaningfully without
# flooding the journal at the /scan rate.
DIAG_WARN_MIN_INTERVAL_S = 5.0

# Final velocity command topic the bridge consumes.  Nav2's velocity_smoother
# publishes the accel/decel-limited command here; overridable for setups that
# route the authoritative command elsewhere.
DEFAULT_CMD_VEL_TOPIC = os.environ.get(
    "CAT_FOLLOW_CMD_VEL_TOPIC", "cmd_vel_smoothed"
)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return the planar yaw (rad) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class FrontSectorResult:
    """Outcome of reducing a LaserScan to a single forward obstacle distance."""

    min_distance_cm: Optional[float]
    total_beams: int
    in_sector_beams: int
    usable_beams: int


def reduce_front_sector(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    *,
    half_angle_rad: float = FRONT_HALF_ANGLE_RAD,
) -> FrontSectorResult:
    """Reduce a scan to the nearest valid forward obstacle plus diagnostics.

    A beam is *usable* only when it is finite and within the sensor's own
    ``[range_min, range_max]`` window (per ``sensor_msgs/LaserScan``): readings
    below ``range_min`` are sensor artifacts, not real close obstacles, and must
    not be treated as an imminent collision.
    """
    best_m: Optional[float] = None
    in_sector = 0
    usable = 0
    total = 0
    lo = max(0.0, float(range_min))
    for i, r in enumerate(ranges):
        total += 1
        angle = angle_min + i * angle_increment
        # Normalize to [-pi, pi] and keep only the forward sector.
        angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
        if abs(angle) > half_angle_rad:
            continue
        in_sector += 1
        if r is None or not math.isfinite(r):
            continue
        # Reject non-positive returns and readings outside the sensor's own
        # [range_min, range_max] window (sub-range_min values are artifacts).
        if r <= 0.0 or r < lo or r > range_max:
            continue
        usable += 1
        if best_m is None or r < best_m:
            best_m = r
    return FrontSectorResult(
        min_distance_cm=None if best_m is None else best_m * 100.0,
        total_beams=total,
        in_sector_beams=in_sector,
        usable_beams=usable,
    )


def _front_min_distance_cm(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> Optional[float]:
    """Minimum valid range (cm) within the front sector, or None.

    Thin wrapper over :func:`reduce_front_sector` for callers that only need the
    distance.  ``range_min`` is honored so sub-minimum artifacts are ignored.
    """
    return reduce_front_sector(
        ranges, angle_min, angle_increment, range_min, range_max
    ).min_distance_cm


def sanitize_cmd_vel(
    linear_x: float, angular_z: float
) -> Optional[Tuple[float, float]]:
    """Map a Twist to ``(path_correction, speed_limit)`` or None if non-finite.

    Fail closed on NaN/inf: returning None means the caller must drop the
    command (and let it age out) instead of clamping a NaN into full authority.
    """
    if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
        return None
    path_correction = max(-1.0, min(1.0, angular_z / MAX_PLANNER_YAW_RATE_RAD_S))
    speed_limit = max(0.0, min(1.0, abs(linear_x) / MAX_PLANNER_SPEED_MPS))
    return path_correction, speed_limit


def sanitize_odom_pose(
    px: float,
    py: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> Optional[Tuple[float, float, float]]:
    """Return ``(x, y, yaw)`` from an odom pose, or None if invalid.

    Fail closed when any component is non-finite or the quaternion is
    degenerate (near-zero norm), so a garbage pose never becomes an
    authoritative heading.
    """
    values = (px, py, qx, qy, qz, qw)
    if not all(math.isfinite(v) for v in values):
        return None
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-6:
        return None
    return float(px), float(py), _yaw_from_quaternion(qx, qy, qz, qw)


if _HAS_ROS:

    class RosBridge(Node):
        """rclpy node writing NavigationState + lidar RangeState + map snapshot."""

        def __init__(
            self,
            shared_state: SharedState,
            *,
            obstacle_detected_cm: float = 50.0,
            obstacle_critical_cm: float = DEFAULT_OBSTACLE_TOO_CLOSE_CM,
            cmd_vel_topic: str = DEFAULT_CMD_VEL_TOPIC,
        ) -> None:
            super().__init__("cat_follow_ros_bridge")
            self._ss = shared_state
            # Two-threshold safety pair is read on the /scan callback thread and
            # updated from the web-UI config thread; guard so a reader never sees
            # a torn (detected, critical) pair.
            self._threshold_lock = threading.Lock()
            self._obstacle_detected_cm = float(obstacle_detected_cm)
            self._obstacle_critical_cm = float(obstacle_critical_cm)

            self.create_subscription(
                LaserScan, "scan", self._on_scan, qos_profile_sensor_data
            )
            self.create_subscription(Odometry, "odom", self._on_odom, 10)
            self._cmd_vel_topic = str(cmd_vel_topic)
            self.create_subscription(
                Twist, self._cmd_vel_topic, self._on_cmd_vel, 10
            )
            self.get_logger().info(
                "cmd_vel bridged from '%s' (post velocity_smoother)",
                self._cmd_vel_topic,
            )

            # Maps are latched / transient-local from slam_toolbox / map_server.
            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(OccupancyGrid, "map", self._on_map, map_qos)

            # Cached navigation fields updated from separate topics.
            self._heading = 0.0
            self._heading_valid = False
            self._path_correction = 0.0
            self._speed_limit = 0.0
            self._odom_x = 0.0
            self._odom_y = 0.0
            # Per-topic receipt times so /cmd_vel can age out independently of
            # /odom (a silent planner must not keep the last velocity alive).
            self._odom_received_ms = 0
            self._cmd_vel_received_ms = 0

            # Rate-limited diagnostics bookkeeping (shared across callback +
            # worker threads).
            self._warn_lock = threading.Lock()
            self._warn_last_s: dict[str, float] = {}

            # Off-callback worker for non-safety map / overlay downsampling so
            # the safety-critical /scan callback never blocks on it.
            self._map_queue: "queue.Queue[tuple]" = queue.Queue(maxsize=2)
            self._worker_stop = threading.Event()
            self._worker = threading.Thread(
                target=self._map_worker_loop,
                name="RosBridge-Map",
                daemon=True,
            )
            self._worker.start()

            self._tf_buffer = None
            self._tf_listener = None
            try:
                from tf2_ros import Buffer, TransformListener

                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, self)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    "tf2_ros unavailable (%s); web map pose falls back to /odom",
                    exc,
                )

            self.create_timer(1.0 / POSE_PUBLISH_HZ, self._on_pose_timer)

        # ── diagnostics ──────────────────────────────────────────────

        def _warn_rate_limited(self, key: str, fmt: str, *args) -> None:
            """Emit ``get_logger().warning`` at most once per interval per key."""
            now = time.monotonic()
            with self._warn_lock:
                last = self._warn_last_s.get(key, 0.0)
                if now - last < DIAG_WARN_MIN_INTERVAL_S:
                    return
                self._warn_last_s[key] = now
            self.get_logger().warning(fmt, *args)

        def set_safety_thresholds(self, config) -> None:
            if config.obstacle_detected_cm <= config.obstacle_too_close_cm:
                raise ValueError(
                    "obstacle_detected_cm must exceed obstacle_too_close_cm"
                )
            # Publish both values atomically so a concurrent /scan reader cannot
            # observe a mismatched (detected, critical) pair.
            with self._threshold_lock:
                self._obstacle_detected_cm = float(config.obstacle_detected_cm)
                self._obstacle_critical_cm = float(config.obstacle_too_close_cm)

        # ── background map / overlay worker ──────────────────────────

        def _map_worker_loop(self) -> None:
            while not self._worker_stop.is_set():
                try:
                    kind, payload = self._map_queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                try:
                    if kind == "map":
                        publish_map_grid(**payload)
                    elif kind == "scan":
                        publish_scan_overlay(*payload)
                except Exception as exc:  # noqa: BLE001
                    self._warn_rate_limited(
                        "map_worker",
                        "map/overlay processing failed: %s",
                        exc,
                    )

        def _enqueue_latest(self, item: tuple) -> None:
            """Enqueue keeping only the freshest work; drop old on backpressure."""
            try:
                self._map_queue.put_nowait(item)
                return
            except queue.Full:
                pass
            try:
                self._map_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._map_queue.put_nowait(item)
            except queue.Full:
                pass

        # ── /scan -> lidar RangeState + scan overlay ─────────────────

        def _on_scan(self, msg) -> None:  # noqa: ANN001
            result = reduce_front_sector(
                msg.ranges,
                msg.angle_min,
                msg.angle_increment,
                msg.range_min,
                msg.range_max,
            )
            dist_cm = result.min_distance_cm
            now = now_monotonic_ms()
            with self._threshold_lock:
                detected_cm = self._obstacle_detected_cm
                critical_cm = self._obstacle_critical_cm
            if dist_cm is None:
                # Fail closed: no usable forward reading.  Do NOT synthesize a
                # clear range; surface a meaningful, rate-limited diagnostic so
                # a persistently empty sector is visible in the journal.
                self._warn_rate_limited(
                    "front_sector_empty",
                    "front lidar sector unusable (fail-closed): "
                    "in_sector=%d usable=%d total=%d range=[%.2f, %.2f] m",
                    result.in_sector_beams,
                    result.usable_beams,
                    result.total_beams,
                    float(msg.range_min),
                    float(msg.range_max),
                )
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
                detected = dist_cm < detected_cm
                critical = dist_cm < critical_cm
                span = detected_cm - critical_cm
                if dist_cm >= detected_cm:
                    severity = 0.0
                elif dist_cm <= critical_cm or span <= 0:
                    severity = 1.0
                else:
                    severity = (detected_cm - dist_cm) / span
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
            # Overlay downsampling is cosmetic; never do it inline on the safety
            # callback.  Copy the ranges and hand off to the worker.
            self._enqueue_latest(
                (
                    "scan",
                    (
                        list(msg.ranges),
                        float(msg.angle_min),
                        float(msg.angle_increment),
                        float(msg.range_max),
                    ),
                )
            )

        # ── /map -> web occupancy snapshot ───────────────────────────

        def _on_map(self, msg) -> None:  # noqa: ANN001
            info = msg.info
            origin = info.origin
            ox = float(origin.position.x)
            oy = float(origin.position.y)
            yaw = _yaw_from_quaternion(
                origin.orientation.x,
                origin.orientation.y,
                origin.orientation.z,
                origin.orientation.w,
            )
            resolution = float(info.resolution)
            # Validate finite map geometry before publishing; a NaN origin or
            # resolution would corrupt the web overlay's coordinate mapping.
            if not all(math.isfinite(v) for v in (ox, oy, yaw, resolution)):
                self._warn_rate_limited(
                    "map_nonfinite",
                    "dropping /map with non-finite geometry "
                    "(origin=(%s, %s) yaw=%s res=%s)",
                    ox,
                    oy,
                    yaw,
                    resolution,
                )
                return
            # Copy grid data and downsample off the callback thread.
            self._enqueue_latest(
                (
                    "map",
                    {
                        "data": list(msg.data),
                        "width": int(info.width),
                        "height": int(info.height),
                        "resolution_m": resolution,
                        "origin_x": ox,
                        "origin_y": oy,
                        "origin_yaw": yaw,
                        "source": "ros_/map",
                    },
                )
            )

        # ── /odom -> NavigationState.heading (+ pose fallback) ───────

        def _on_odom(self, msg) -> None:  # noqa: ANN001
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            sanitized = sanitize_odom_pose(p.x, p.y, q.x, q.y, q.z, q.w)
            if sanitized is None:
                # Fail closed: ignore the sample and let odom age out rather
                # than adopting a non-finite heading/pose.
                self._warn_rate_limited(
                    "odom_nonfinite",
                    "dropping /odom with non-finite pose/quaternion",
                )
                return
            self._odom_x, self._odom_y, self._heading = sanitized
            self._heading_valid = True
            self._odom_received_ms = now_monotonic_ms()
            self._publish_navigation()

        # ── /cmd_vel -> path_correction + speed_limit ────────────────

        def _on_cmd_vel(self, msg) -> None:  # noqa: ANN001
            sanitized = sanitize_cmd_vel(msg.linear.x, msg.angular.z)
            if sanitized is None:
                # Fail closed: never clamp a NaN into full authority.  Drop the
                # command so it ages out to a safe stop.
                self._warn_rate_limited(
                    "cmd_vel_nonfinite",
                    "dropping non-finite /cmd_vel (linear.x/angular.z)",
                )
                return
            self._path_correction, self._speed_limit = sanitized
            self._cmd_vel_received_ms = now_monotonic_ms()
            self._publish_navigation()

        def _publish_navigation(self) -> None:
            now = now_monotonic_ms()

            # Age /cmd_vel independently: if the planner has gone silent, drop
            # the drive terms to zero so a continuing /odom stream cannot keep
            # the stale velocity authoritative.
            cmd_vel_fresh = (
                self._cmd_vel_received_ms > 0
                and (now - self._cmd_vel_received_ms) <= CMD_VEL_STALE_MS
            )
            speed_limit = self._speed_limit if cmd_vel_fresh else 0.0
            path_correction = self._path_correction if cmd_vel_fresh else 0.0

            # The NavigationState is only "fresh" for driving when BOTH odom and
            # cmd_vel are within TTL.  Encoding this as the min receipt time lets
            # the DecisionEngine's age check fail closed if either input stalls.
            if self._odom_received_ms > 0 and self._cmd_vel_received_ms > 0:
                drive_received_ms = min(
                    self._odom_received_ms, self._cmd_vel_received_ms
                )
            else:
                drive_received_ms = 0

            self._ss.update_navigation(
                NavigationState(
                    timestamp_ms=int(time.time() * 1000),
                    received_ms=drive_received_ms,
                    fresh=cmd_vel_fresh,
                    authority="RosBridge",
                    heading=self._heading,
                    heading_valid=self._heading_valid,
                    speed_limit=speed_limit,
                    path_correction=path_correction,
                )
            )

        def _on_pose_timer(self) -> None:
            """Publish map-frame pose for the web UI (TF preferred)."""
            if self._tf_buffer is not None:
                try:
                    tf = self._tf_buffer.lookup_transform(
                        "map", "base_link", rclpy.time.Time()
                    )
                    t = tf.transform.translation
                    q = tf.transform.rotation
                    yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
                    # Validate finite TF before publishing a map-frame pose.
                    if all(math.isfinite(v) for v in (t.x, t.y, yaw)):
                        publish_robot_pose(
                            x=float(t.x),
                            y=float(t.y),
                            yaw=yaw,
                            frame="map",
                        )
                        return
                    self._warn_rate_limited(
                        "tf_nonfinite",
                        "map->base_link TF has non-finite values; skipping pose",
                    )
                    return
                except Exception:  # noqa: BLE001 - TF not ready yet
                    pass
            if self._heading_valid and all(
                math.isfinite(v) for v in (self._odom_x, self._odom_y, self._heading)
            ):
                publish_robot_pose(
                    x=self._odom_x,
                    y=self._odom_y,
                    yaw=self._heading,
                    frame="odom",
                )

        def destroy_node(self):  # noqa: ANN201
            # Stop and join the worker so no map processing thread leaks.
            self._worker_stop.set()
            worker = getattr(self, "_worker", None)
            if worker is not None and worker.is_alive():
                worker.join(timeout=1.0)
            return super().destroy_node()


def spin_in_thread(
    shared_state: SharedState,
    *,
    start_bicycle_odom: bool = False,
    safety_config=None,
    bridge_holder: dict | None = None,
) -> "threading.Thread":
    """Start rclpy on a daemon thread running the bridge (+ optional odom).

    ``start_bicycle_odom`` is rejected: the bicycle odometry source is disabled
    because the contract runtime never integrates commanded motion, so the
    publisher would emit a frozen ``/odom``.  Lidar RF2O must own ``/odom`` and
    ``odom -> base_link`` (see :mod:`cat_follow.navigation.odom_source`).
    """
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available; run on the ROCK 4D with ROS 2 Jazzy sourced."
        )
    if start_bicycle_odom:
        raise RuntimeError(BICYCLE_ODOM_DISABLED_MSG)

    def _run() -> None:
        from rclpy.executors import SingleThreadedExecutor

        rclpy.init()
        bridge_kwargs = {}
        if safety_config is not None:
            bridge_kwargs = {
                "obstacle_detected_cm": safety_config.obstacle_detected_cm,
                "obstacle_critical_cm": safety_config.obstacle_too_close_cm,
            }
        bridge = RosBridge(shared_state, **bridge_kwargs)
        if bridge_holder is not None:
            bridge_holder["node"] = bridge
        nodes = [bridge]

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
    "reduce_front_sector",
    "FrontSectorResult",
    "sanitize_cmd_vel",
    "sanitize_odom_pose",
    "_front_min_distance_cm",
    "_yaw_from_quaternion",
]
if _HAS_ROS:
    __all__.append("RosBridge")
