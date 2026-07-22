"""Thread-safe occupancy-map snapshot for the web UI.

ROS publishes ``nav_msgs/OccupancyGrid`` on ``/map`` and the robot pose in the
``map`` frame (via TF).  This module stores a *downsampled* copy so the Flask
``/api/map`` endpoint can poll at ~1–2 Hz without shipping megabyte grids to
the browser.

Cell encoding for the wire format (uint8, base64):
  0   = free
  128 = unknown
  255 = occupied
"""

from __future__ import annotations

import base64
import math
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple


# Target max dimension for the UI canvas grid (cells on the long edge).
DEFAULT_MAX_DIM = 200

# Max scan endpoints retained for the ray overlay.
DEFAULT_MAX_SCAN_POINTS = 72

# Freshness TTLs (ms).  Freshness is recomputed at read time from the relevant
# ``received_ms`` so a stalled ROS bridge / dead SLAM node stops looking live
# instead of leaving the last sample authoritative forever.  The map is latched
# (transient-local) and updates slowly, so it gets a much longer TTL than the
# fast pose/scan streams.
POSE_STALE_MS = 1500
SCAN_STALE_MS = 1500
MAP_STALE_MS = 30000


@dataclass(frozen=True)
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    frame: str = "map"
    fresh: bool = False
    received_ms: int = 0


@dataclass(frozen=True)
class ScanPoint:
    angle: float  # rad, in base_link / laser frame (sensor convention)
    range_m: float


@dataclass(frozen=True)
class MapSnapshot:
    available: bool = False
    width: int = 0
    height: int = 0
    resolution_m: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0
    cells_b64: str = ""
    pose: RobotPose = field(default_factory=RobotPose)
    scan: Tuple[ScanPoint, ...] = ()
    scan_received_ms: int = 0
    source: str = "none"
    received_ms: int = 0
    note: str = ""


_lock = threading.Lock()
_current = MapSnapshot(note="No map yet (waiting for ROS /map)")


def _now_ms() -> int:
    return int(time.monotonic() * 1000)


def occupancy_to_uint8(value: int) -> int:
    """Map OccupancyGrid cell value to UI uint8."""
    if value < 0:
        return 128
    if value >= 65:  # common occupied threshold
        return 255
    if value == 0:
        return 0
    # Soft occupied / free gradient
    return int(max(0, min(255, value * 255 // 100)))


def downsample_occupancy(
    data: Sequence[int],
    width: int,
    height: int,
    *,
    max_dim: int = DEFAULT_MAX_DIM,
) -> Tuple[bytes, int, int, float]:
    """Downsample a row-major occupancy grid.

    Returns ``(cells_bytes, out_w, out_h, scale)`` where ``scale`` is
    ``original_cells / output_cells`` along each axis (uniform).
    """
    if width <= 0 or height <= 0 or not data:
        return b"", 0, 0, 1.0

    long_edge = max(width, height)
    if long_edge <= max_dim:
        scale = 1
        out_w, out_h = width, height
    else:
        scale = int(math.ceil(long_edge / float(max_dim)))
        out_w = max(1, (width + scale - 1) // scale)
        out_h = max(1, (height + scale - 1) // scale)

    out = bytearray(out_w * out_h)
    for oy in range(out_h):
        for ox in range(out_w):
            # Prefer occupied > unknown > free within the block.
            block_occ = False
            block_unk = False
            for dy in range(scale):
                sy = oy * scale + dy
                if sy >= height:
                    break
                row = sy * width
                for dx in range(scale):
                    sx = ox * scale + dx
                    if sx >= width:
                        break
                    v = int(data[row + sx])
                    if v >= 65:
                        block_occ = True
                    elif v < 0:
                        block_unk = True
            if block_occ:
                out[oy * out_w + ox] = 255
            elif block_unk:
                out[oy * out_w + ox] = 128
            else:
                out[oy * out_w + ox] = 0
    return bytes(out), out_w, out_h, float(scale)


def encode_cells_b64(cells: bytes) -> str:
    return base64.b64encode(cells).decode("ascii")


def downsample_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_max: float,
    *,
    max_points: int = DEFAULT_MAX_SCAN_POINTS,
) -> Tuple[ScanPoint, ...]:
    """Keep evenly spaced finite scan endpoints for a lightweight overlay."""
    n = len(ranges)
    if n == 0:
        return ()
    step = max(1, n // max_points)
    points: List[ScanPoint] = []
    for i in range(0, n, step):
        r = ranges[i]
        if r is None or not math.isfinite(r) or r <= 0.0 or r > range_max:
            continue
        points.append(
            ScanPoint(
                angle=float(angle_min + i * angle_increment),
                range_m=float(r),
            )
        )
    return tuple(points)


def publish_map_grid(
    *,
    data: Sequence[int],
    width: int,
    height: int,
    resolution_m: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float = 0.0,
    source: str = "ros_/map",
    max_dim: int = DEFAULT_MAX_DIM,
) -> None:
    """Downsample and publish a new occupancy grid; keep last pose/scan."""
    global _current
    cells, out_w, out_h, scale = downsample_occupancy(
        data, width, height, max_dim=max_dim
    )
    # Origin stays the same; resolution grows by the integer scale factor.
    out_res = float(resolution_m) * scale
    with _lock:
        prev = _current
        _current = MapSnapshot(
            available=out_w > 0 and out_h > 0,
            width=out_w,
            height=out_h,
            resolution_m=out_res,
            origin_x=float(origin_x),
            origin_y=float(origin_y),
            origin_yaw=float(origin_yaw),
            cells_b64=encode_cells_b64(cells) if cells else "",
            pose=prev.pose,
            scan=prev.scan,
            scan_received_ms=prev.scan_received_ms,
            source=source,
            received_ms=_now_ms(),
            note="",
        )


def publish_robot_pose(
    *,
    x: float,
    y: float,
    yaw: float,
    frame: str = "map",
) -> None:
    global _current
    with _lock:
        prev = _current
        _current = MapSnapshot(
            available=prev.available,
            width=prev.width,
            height=prev.height,
            resolution_m=prev.resolution_m,
            origin_x=prev.origin_x,
            origin_y=prev.origin_y,
            origin_yaw=prev.origin_yaw,
            cells_b64=prev.cells_b64,
            pose=RobotPose(
                x=float(x),
                y=float(y),
                yaw=float(yaw),
                frame=str(frame),
                fresh=True,
                received_ms=_now_ms(),
            ),
            scan=prev.scan,
            scan_received_ms=prev.scan_received_ms,
            source=prev.source,
            received_ms=prev.received_ms,
            note=prev.note,
        )


def publish_scan_overlay(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_max: float,
) -> None:
    global _current
    points = downsample_scan(
        ranges, angle_min, angle_increment, range_max
    )
    with _lock:
        prev = _current
        _current = MapSnapshot(
            available=prev.available,
            width=prev.width,
            height=prev.height,
            resolution_m=prev.resolution_m,
            origin_x=prev.origin_x,
            origin_y=prev.origin_y,
            origin_yaw=prev.origin_yaw,
            cells_b64=prev.cells_b64,
            pose=prev.pose,
            scan=points,
            scan_received_ms=_now_ms(),
            source=prev.source,
            received_ms=prev.received_ms,
            note=prev.note,
        )


def get_map_snapshot() -> MapSnapshot:
    with _lock:
        return _current


def map_snapshot_dict() -> Dict[str, Any]:
    snap = get_map_snapshot()
    d = asdict(snap)
    # asdict turns ScanPoint tuples into list[dict] already.

    # Recompute freshness at read time from received_ms + TTL so a stalled ROS
    # bridge or dead SLAM node stops looking live.  Consumers (web UI) should
    # trust these computed flags rather than the sticky stored `fresh` fields.
    now = _now_ms()
    map_fresh = bool(snap.available) and (now - snap.received_ms) <= MAP_STALE_MS
    pose_fresh = (
        snap.pose.received_ms > 0
        and (now - snap.pose.received_ms) <= POSE_STALE_MS
    )
    scan_fresh = (
        snap.scan_received_ms > 0
        and (now - snap.scan_received_ms) <= SCAN_STALE_MS
    )
    # A pose is only safe to overlay on the map grid when it is both fresh and
    # actually expressed in the map frame.  On TF failure the bridge falls back
    # to the odom frame, which drifts relative to the map; drawing that pose
    # (and its scan rays) over the map grid would be misleading.
    pose_on_map = pose_fresh and snap.pose.frame == "map"

    d["map_fresh"] = map_fresh
    d["pose_fresh"] = pose_fresh
    d["scan_fresh"] = scan_fresh
    d["pose_on_map"] = pose_on_map
    # Override the sticky stored flag with the age-based result.
    d["pose"]["fresh"] = pose_fresh
    return d


def reset_map_snapshot_for_tests() -> None:
    """Clear published state (unit tests only)."""
    global _current
    with _lock:
        _current = MapSnapshot(note="No map yet (waiting for ROS /map)")
