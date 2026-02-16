"""
Load calibration from JSON files. Exposes speed->cm/s, max steer, optional bbox->distance.
Calibration dir is next to this file.
"""
import json
import os
from typing import Dict, Optional

_CALIB_DIR = os.path.dirname(os.path.abspath(__file__))


def _load_json(name: str) -> dict:
    path = os.path.join(_CALIB_DIR, name)
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


class Calibration:
    """Single place for all calibration. Uses JSON files in calibration/."""

    def __init__(self, calib_dir: Optional[str] = None):
        self._dir = calib_dir or _CALIB_DIR
        self._speed = _load_json("speed_time_distance.json")
        self._steering = _load_json("steering_limits.json")
        self._bbox_dist = _load_json("bbox_distance.json")

    def get_cm_per_sec(self, speed: int) -> float:
        """Speed (0-100) -> cm per second. Linear interpolation if between keys."""
        table = self._speed.get("speed_to_cm_per_sec") or {}
        # keys may be strings in JSON
        by_int = {int(k): float(v) for k, v in table.items()}
        if not by_int:
            return max(1.0, speed * 0.4)  # fallback
        speeds = sorted(by_int.keys())
        if speed <= speeds[0]:
            return by_int[speeds[0]]
        if speed >= speeds[-1]:
            return by_int[speeds[-1]]
        for i in range(len(speeds) - 1):
            if speeds[i] <= speed <= speeds[i + 1]:
                a, b = speeds[i], speeds[i + 1]
                return by_int[a] + (speed - a) / (b - a) * (by_int[b] - by_int[a])
        return by_int[speeds[-1]]

    def get_max_steer_angle_deg(self) -> float:
        """Max steering angle (symmetric), degrees. Clamp steer to ± this."""
        return float(self._steering.get("max_steer_angle_deg", 25.0))

    def get_min_turn_radius_cm(self) -> float:
        """Min turn radius in cm (max curvature)."""
        return float(self._steering.get("min_turn_radius_cm", 40.0))

    def get_distance_cm_from_bbox_area(self, bbox_area_px: float) -> Optional[float]:
        """Optional: bbox area (pixels²) -> distance in cm. Returns None if not calibrated."""
        # Expect format like {"area_to_cm": [[area1, cm1], [area2, cm2], ...]} or formula
        table = self._bbox_dist.get("area_to_cm")
        if not table or not isinstance(table, list):
            return None
        # Simple: find nearest pair and interpolate
        if len(table) < 2:
            return float(table[0][1]) if table else None
        sorted_pairs = sorted(table, key=lambda p: p[0])
        if bbox_area_px <= sorted_pairs[0][0]:
            return float(sorted_pairs[0][1])
        if bbox_area_px >= sorted_pairs[-1][0]:
            return float(sorted_pairs[-1][1])
        for i in range(len(sorted_pairs) - 1):
            a1, d1 = sorted_pairs[i]
            a2, d2 = sorted_pairs[i + 1]
            if a1 <= bbox_area_px <= a2:
                return d1 + (bbox_area_px - a1) / (a2 - a1) * (d2 - d1)
        return float(sorted_pairs[-1][1])

    def get_target_distance_cm(self) -> float:
        """Target distance to cat in TRACK (e.g. 15 cm)."""
        return float(self._bbox_dist.get("target_distance_cm", 15.0))
