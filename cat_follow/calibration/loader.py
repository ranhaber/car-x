"""
Load calibration from JSON files. Exposes speed->cm/s, max steer, target approach distance.

Calibration is loaded once at startup and updated only when the user saves from the Web UI
(or explicitly reloads); it is not constantly reloaded during operation.
"""

import json
import os
import threading
from typing import Dict, Optional, Tuple

from cat_follow.logger import get_logger

_CALIB_DIR = os.path.dirname(os.path.abspath(__file__))
_log = get_logger("calibration")

# Bbox area calibration is for this resolution only. Do not change without re-calibrating.
CALIBRATION_IMAGE_SIZE: Tuple[int, int] = (640, 480)


def _load_json(name: str) -> dict:
    path = os.path.join(_CALIB_DIR, name)
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def _save_json(name: str, data: dict, calib_dir: str):
    """Save dictionary to a JSON file."""
    path = os.path.join(calib_dir, name)
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except Exception as e:
        _log.warning("Error saving calibration file %s: %s", name, e)


class Calibration:
    """Single place for all calibration. Uses JSON files in calibration/.

    Loaded once at startup; updated only when the user saves from the Web UI (or reload()).
    Not constantly reloaded during operation. Thread-safe: all get/set/reload/save under _lock.
    """

    def __init__(self, calib_dir: Optional[str] = None):
        self._dir = calib_dir or _CALIB_DIR
        self._lock = threading.Lock()
        self.reload()

    def reload(self):
        """Re-read all calibration files from disk."""
        with self._lock:
            self._speed = _load_json("speed_time_distance.json")
            self._steering = _load_json("steering_limits.json")

    def save(self):
        """Save current calibration values back to JSON files."""
        with self._lock:
            _save_json("speed_time_distance.json", self._speed, self._dir)
            _save_json("steering_limits.json", self._steering, self._dir)

    def get_cm_per_sec(self, speed: int) -> float:
        """Speed (0-100) -> cm per second. Linear interpolation if between keys."""
        with self._lock:
            table = self._speed.get("speed_to_cm_per_sec") or {}
            by_int = {int(k): float(v) for k, v in table.items()}
            if not by_int:
                return max(1.0, speed * 0.4)
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
        with self._lock:
            return float(self._steering.get("max_steer_angle_deg", 25.0))

    def get_min_turn_radii_cm(self) -> Tuple[float, float]:
        """Min turn radii in cm (left, right) for max curvature."""
        with self._lock:
            radii = self._steering.get("min_turn_radius_cm", {})
            if isinstance(radii, (int, float)):
                return (float(radii), float(radii))
            left = float(radii.get("left", 40.0))
            right = float(radii.get("right", 40.0))
            return (left, right)

    def get_target_distance_cm(self) -> float:
        """Closest approach distance (cm) for obstacle/approach logic. From steering_limits.json."""
        with self._lock:
            return float(self._steering.get("target_distance_cm", 15.0))

    # --- Setters for Web UI ---

    def get_all_calibration_data(self) -> dict:
        """Return all calibration data as a dictionary for the UI."""
        with self._lock:
            return {
                "speed": self._speed,
                "steering": self._steering,
            }

    def set_all_calibration_data(self, data: dict):
        """Update calibration from a dictionary (from UI)."""
        with self._lock:
            if "speed" in data and isinstance(data["speed"], dict):
                self._speed = data["speed"]
            if "steering" in data and isinstance(data["steering"], dict):
                self._steering = data["steering"]
