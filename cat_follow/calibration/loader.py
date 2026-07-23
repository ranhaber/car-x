"""
Load calibration from JSON files. Exposes speed->cm/s, max steer, target approach distance.

Calibration is loaded once at startup and updated only when the user saves from the Web UI
(or explicitly reloads); it is not constantly reloaded during operation.
"""

from __future__ import annotations

import copy
import json
import math
import os
import threading
from typing import Any, Dict, List, Optional, Tuple

from cat_follow.logger import get_logger

_CALIB_DIR = os.path.dirname(os.path.abspath(__file__))
_log = get_logger("calibration")

# Bbox area calibration is for this resolution only. Do not change without re-calibrating.
CALIBRATION_IMAGE_SIZE: Tuple[int, int] = (640, 480)

_MIN_MAX_STEER_DEG = 1.0
_MAX_MAX_STEER_DEG = 45.0
_MIN_TARGET_DISTANCE_CM = 1.0
_MAX_TARGET_DISTANCE_CM = 300.0
_MIN_TURN_RADIUS_CM = 1.0
_MAX_TURN_RADIUS_CM = 1000.0
_MIN_SPEED_KEY = 0
_MAX_SPEED_KEY = 100
_MIN_CM_PER_SEC = 0.0
_MAX_CM_PER_SEC = 500.0


def _load_json(name: str, calib_dir: str) -> dict:
    path = os.path.join(calib_dir, name)
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


def _is_finite_number(value: Any) -> bool:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number)


def _deep_merge_dict(base: dict, patch: dict) -> dict:
    """Return a deep merge of ``patch`` onto ``base`` without mutating inputs."""

    out = copy.deepcopy(base)
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge_dict(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def validate_calibration_candidate(data: dict) -> List[str]:
    """Validate a full calibration document before mutation/persistence."""

    errors: List[str] = []
    if not isinstance(data, dict):
        return ["calibration payload must be an object"]

    speed = data.get("speed", {})
    steering = data.get("steering", {})
    if speed is None:
        speed = {}
    if steering is None:
        steering = {}
    if not isinstance(speed, dict):
        errors.append("speed must be an object")
        speed = {}
    if not isinstance(steering, dict):
        errors.append("steering must be an object")
        steering = {}

    table = speed.get("speed_to_cm_per_sec")
    if table is not None:
        if not isinstance(table, dict):
            errors.append("speed.speed_to_cm_per_sec must be an object")
        else:
            for key, value in table.items():
                try:
                    speed_key = int(key)
                except (TypeError, ValueError):
                    errors.append(f"speed key {key!r} must be an integer 0-100")
                    continue
                if not (_MIN_SPEED_KEY <= speed_key <= _MAX_SPEED_KEY):
                    errors.append(f"speed key {speed_key} must be 0-100")
                if not _is_finite_number(value):
                    errors.append(f"speed[{speed_key}] must be a finite number")
                    continue
                cm_s = float(value)
                if not (_MIN_CM_PER_SEC <= cm_s <= _MAX_CM_PER_SEC):
                    errors.append(
                        f"speed[{speed_key}] cm/s must be "
                        f"{_MIN_CM_PER_SEC:g}-{_MAX_CM_PER_SEC:g}"
                    )

    if "max_steer_angle_deg" in steering and steering["max_steer_angle_deg"] is not None:
        if not _is_finite_number(steering["max_steer_angle_deg"]):
            errors.append("steering.max_steer_angle_deg must be a finite number")
        else:
            max_steer = float(steering["max_steer_angle_deg"])
            if not (_MIN_MAX_STEER_DEG <= max_steer <= _MAX_MAX_STEER_DEG):
                errors.append(
                    "steering.max_steer_angle_deg must be "
                    f"{_MIN_MAX_STEER_DEG:g}-{_MAX_MAX_STEER_DEG:g}"
                )

    if "target_distance_cm" in steering and steering["target_distance_cm"] is not None:
        if not _is_finite_number(steering["target_distance_cm"]):
            errors.append("steering.target_distance_cm must be a finite number")
        else:
            target = float(steering["target_distance_cm"])
            if not (_MIN_TARGET_DISTANCE_CM <= target <= _MAX_TARGET_DISTANCE_CM):
                errors.append(
                    "steering.target_distance_cm must be "
                    f"{_MIN_TARGET_DISTANCE_CM:g}-{_MAX_TARGET_DISTANCE_CM:g}"
                )

    radii = steering.get("min_turn_radius_cm")
    if radii is not None:
        if isinstance(radii, (int, float)):
            if not _is_finite_number(radii):
                errors.append("steering.min_turn_radius_cm must be finite")
            else:
                value = float(radii)
                if not (_MIN_TURN_RADIUS_CM <= value <= _MAX_TURN_RADIUS_CM):
                    errors.append(
                        "steering.min_turn_radius_cm must be "
                        f"{_MIN_TURN_RADIUS_CM:g}-{_MAX_TURN_RADIUS_CM:g}"
                    )
        elif isinstance(radii, dict):
            for side in ("left", "right"):
                if side not in radii or radii[side] is None:
                    continue
                if not _is_finite_number(radii[side]):
                    errors.append(f"steering.min_turn_radius_cm.{side} must be finite")
                    continue
                value = float(radii[side])
                if not (_MIN_TURN_RADIUS_CM <= value <= _MAX_TURN_RADIUS_CM):
                    errors.append(
                        f"steering.min_turn_radius_cm.{side} must be "
                        f"{_MIN_TURN_RADIUS_CM:g}-{_MAX_TURN_RADIUS_CM:g}"
                    )
        else:
            errors.append("steering.min_turn_radius_cm must be a number or object")

    # Safety thresholds: validate only when present so partial updates that
    # omit them remain valid; resolve_safety_config enforces the pair later.
    for key in ("obstacle_too_close_cm", "obstacle_detected_cm"):
        if key not in steering or steering[key] is None:
            continue
        if not _is_finite_number(steering[key]):
            errors.append(f"steering.{key} must be a finite number")

    if not errors:
        try:
            from cat_follow.safety_config import resolve_safety_config

            class _Candidate:
                def get_all_calibration_data(self_inner):
                    return {"speed": speed, "steering": steering}

            resolve_safety_config(_Candidate())
        except ValueError as exc:
            errors.append(str(exc))
        except Exception as exc:  # noqa: BLE001
            errors.append(f"safety validation failed: {exc}")

    return errors


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
            self._speed = _load_json("speed_time_distance.json", self._dir)
            self._steering = _load_json("steering_limits.json", self._dir)

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
            value = self._steering.get("max_steer_angle_deg", 30.0)
            try:
                parsed = float(value)
            except (TypeError, ValueError):
                return 30.0
            if not math.isfinite(parsed) or parsed <= 0:
                return 30.0
            return parsed

    def get_min_turn_radii_cm(self) -> Tuple[float, float]:
        """Min turn radii in cm (left, right) for max curvature."""
        with self._lock:
            radii = self._steering.get("min_turn_radius_cm", {})
            if isinstance(radii, (int, float)):
                return (float(radii), float(radii))
            if not isinstance(radii, dict):
                return (40.0, 40.0)
            left = float(radii.get("left", 40.0))
            right = float(radii.get("right", 40.0))
            return (left, right)

    def get_target_distance_cm(self) -> float:
        """Closest approach distance (cm) for obstacle/approach logic. From steering_limits.json."""
        with self._lock:
            value = self._steering.get("target_distance_cm", 15.0)
            try:
                parsed = float(value)
            except (TypeError, ValueError):
                return 15.0
            if not math.isfinite(parsed):
                return 15.0
            return parsed

    def get_obstacle_too_close_cm(self) -> float | None:
        with self._lock:
            value = self._steering.get("obstacle_too_close_cm")
            if value is None:
                return None
            try:
                parsed = float(value)
            except (TypeError, ValueError):
                return None
            if not math.isfinite(parsed):
                return None
            return parsed

    def get_obstacle_detected_cm(self) -> float | None:
        with self._lock:
            value = self._steering.get("obstacle_detected_cm")
            if value is None:
                return None
            try:
                parsed = float(value)
            except (TypeError, ValueError):
                return None
            if not math.isfinite(parsed):
                return None
            return parsed

    # --- Setters for Web UI ---

    def get_all_calibration_data(self) -> dict:
        """Return all calibration data as a dictionary for the UI."""
        with self._lock:
            return {
                "speed": copy.deepcopy(self._speed),
                "steering": copy.deepcopy(self._steering),
            }

    def preview_merged_calibration(self, data: dict) -> dict:
        """Deep-merge ``data`` onto current calibration without mutating state."""

        if not isinstance(data, dict):
            raise TypeError("calibration payload must be an object")
        with self._lock:
            current = {
                "speed": copy.deepcopy(self._speed),
                "steering": copy.deepcopy(self._steering),
            }
        patch: dict = {}
        if "speed" in data:
            if data["speed"] is not None and not isinstance(data["speed"], dict):
                raise TypeError("speed must be an object")
            patch["speed"] = data.get("speed") or {}
        if "steering" in data:
            if data["steering"] is not None and not isinstance(data["steering"], dict):
                raise TypeError("steering must be an object")
            patch["steering"] = data.get("steering") or {}
        return _deep_merge_dict(current, patch)

    def apply_validated_calibration(self, data: dict) -> None:
        """Replace in-memory calibration with an already-validated document."""

        if not isinstance(data, dict):
            raise TypeError("calibration payload must be an object")
        speed = data.get("speed")
        steering = data.get("steering")
        if speed is not None and not isinstance(speed, dict):
            raise TypeError("speed must be an object")
        if steering is not None and not isinstance(steering, dict):
            raise TypeError("steering must be an object")
        with self._lock:
            if isinstance(speed, dict):
                self._speed = copy.deepcopy(speed)
            if isinstance(steering, dict):
                self._steering = copy.deepcopy(steering)

    def set_all_calibration_data(self, data: dict):
        """Update calibration from a dictionary (from UI) via deep merge.

        Prefer :meth:`preview_merged_calibration` + validation +
        :meth:`apply_validated_calibration` for request handlers so invalid
        payloads never mutate memory or disk.
        """
        candidate = self.preview_merged_calibration(data)
        errors = validate_calibration_candidate(candidate)
        if errors:
            raise ValueError("; ".join(errors))
        self.apply_validated_calibration(candidate)
