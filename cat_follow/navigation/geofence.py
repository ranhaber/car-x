"""Car geofence containment helpers (point-in-polygon, not cat perimeter)."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from typing import Optional, Sequence, Tuple

from cat_follow.control.types import GeofenceState
from cat_follow.runtime.shared_state import now_monotonic_ms

Point = Tuple[float, float]


@dataclass(frozen=True)
class GeofencePolygon:
    geofence_id: str
    vertices_m: Tuple[Point, ...]
    frame_id: str = "map"

    def __post_init__(self) -> None:
        if len(self.vertices_m) < 3:
            raise ValueError("geofence polygon requires at least 3 vertices")


def load_geofence_polygon(path: str) -> GeofencePolygon:
    with open(path, "r", encoding="utf-8") as handle:
        raw = json.load(handle)
    if not isinstance(raw, dict):
        raise ValueError("geofence file must be an object")
    geofence_id = str(raw.get("car_geofence_id") or raw.get("id") or "")
    if not geofence_id:
        raise ValueError("geofence file missing car_geofence_id")
    frame_id = str(raw.get("frame_id", "map"))
    vertices_raw = raw.get("vertices_m") or raw.get("vertices")
    if not isinstance(vertices_raw, list) or len(vertices_raw) < 3:
        raise ValueError("geofence vertices_m must contain at least 3 points")
    vertices: list[Point] = []
    unit = str(raw.get("unit", "m")).lower()
    scale = 0.01 if unit in {"cm", "centimeter", "centimeters"} else 1.0
    for item in vertices_raw:
        if not isinstance(item, (list, tuple)) or len(item) < 2:
            raise ValueError("geofence vertex must be [x, y]")
        vertices.append((float(item[0]) * scale, float(item[1]) * scale))
    return GeofencePolygon(
        geofence_id=geofence_id,
        vertices_m=tuple(vertices),
        frame_id=frame_id,
    )


def point_in_polygon(x: float, y: float, vertices: Sequence[Point]) -> bool:
    """Ray-casting containment test (boundary counts as inside)."""

    inside = False
    n = len(vertices)
    j = n - 1
    for i in range(n):
        xi, yi = vertices[i]
        xj, yj = vertices[j]
        if (yi > y) != (yj > y):
            denom = yj - yi
            if abs(denom) > 1e-12:
                x_cross = (xj - xi) * (y - yi) / denom + xi
                if x <= x_cross + 1e-12:
                    inside = not inside
        # Boundary segment check.
        if _on_segment(x, y, (xi, yi), (xj, yj)):
            return True
        j = i
    return inside


def distance_to_boundary_cm(
    x: float, y: float, vertices: Sequence[Point]
) -> float:
    """Signed distance in cm: positive inside, negative outside."""

    min_dist_m = float("inf")
    n = len(vertices)
    for i in range(n):
        a = vertices[i]
        b = vertices[(i + 1) % n]
        min_dist_m = min(min_dist_m, _point_to_segment_distance(x, y, a, b))
    inside = point_in_polygon(x, y, vertices)
    dist_cm = min_dist_m * 100.0
    return dist_cm if inside else -dist_cm


def evaluate_geofence(
    polygon: Optional[GeofencePolygon],
    *,
    pose_x_m: float,
    pose_y_m: float,
    pose_received_ms: int,
    now_ms: int,
    pose_stale_ms: int = 500,
    previous: Optional[GeofenceState] = None,
) -> GeofenceState:
    """Publish containment status for the car ``base_link`` center."""

    received = now_ms if now_ms > 0 else now_monotonic_ms()
    if polygon is None:
        return GeofenceState(
            received_ms=received,
            configured=False,
            localization_valid_for_containment=False,
            car_inside=True,
            car_distance_to_boundary_cm=0.0,
        )

    localization_valid = (
        pose_received_ms > 0 and received - pose_received_ms <= pose_stale_ms
    )
    if not localization_valid:
        prev_breach = bool(previous.breach_confirmed) if previous else False
        return GeofenceState(
            received_ms=received,
            car_geofence_id=polygon.geofence_id,
            configured=True,
            car_inside=True if previous is None else previous.car_inside,
            car_distance_to_boundary_cm=(
                previous.car_distance_to_boundary_cm if previous else 0.0
            ),
            localization_valid_for_containment=False,
            breach_confirmed=prev_breach,
            breach_at_ms=previous.breach_at_ms if previous else None,
        )

    inside = point_in_polygon(pose_x_m, pose_y_m, polygon.vertices_m)
    distance = distance_to_boundary_cm(
        pose_x_m, pose_y_m, polygon.vertices_m
    )
    breach = not inside
    breach_at = None
    if breach:
        if previous is not None and previous.breach_confirmed:
            breach_at = previous.breach_at_ms
        else:
            breach_at = received
    return GeofenceState(
        received_ms=received,
        car_geofence_id=polygon.geofence_id,
        configured=True,
        car_inside=inside,
        car_distance_to_boundary_cm=distance,
        localization_valid_for_containment=True,
        breach_confirmed=breach,
        breach_at_ms=breach_at,
    )


def default_geofence_path() -> Optional[str]:
    override = os.getenv("CAT_FOLLOW_GEOFENCE_FILE")
    if override:
        return override
    return None


def _on_segment(
    x: float, y: float, a: Point, b: Point, *, eps: float = 1e-9
) -> bool:
    ax, ay = a
    bx, by = b
    cross = (x - ax) * (by - ay) - (y - ay) * (bx - ax)
    if abs(cross) > eps:
        return False
    return (
        min(ax, bx) - eps <= x <= max(ax, bx) + eps
        and min(ay, by) - eps <= y <= max(ay, by) + eps
    )


def _point_to_segment_distance(x: float, y: float, a: Point, b: Point) -> float:
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(x - ax, y - ay)
    t = ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(x - proj_x, y - proj_y)
