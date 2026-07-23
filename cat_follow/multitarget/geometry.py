"""Pure geometry helpers for multi-target tracking."""

from __future__ import annotations

import math


def dist(p: tuple[float, float], q: tuple[float, float]) -> float:
    """Return Euclidean distance between two points."""
    return math.hypot(p[0] - q[0], p[1] - q[1])


def rect_intersection_area(a, b) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    width = min(ax + aw, bx + bw) - max(ax, bx)
    height = min(ay + ah, by + bh) - max(ay, by)
    return float(max(0.0, width) * max(0.0, height))


def rect_iou(a, b) -> float:
    """Return intersection-over-union for two ``(x, y, w, h)`` boxes."""
    intersection = rect_intersection_area(a, b)
    union = a[2] * a[3] + b[2] * b[3] - intersection
    return intersection / union if union > 0 else 0.0


def point_in_polygon(point, polygon) -> bool:
    """Ray-casting point-in-polygon test."""
    x, y = point
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-9) + xi
        ):
            inside = not inside
        j = i
    return inside
