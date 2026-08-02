"""Safety threshold configuration (env defaults + optional calibration overrides).

Environment variables (startup defaults):
    CAT_FOLLOW_SAFETY_OBSTACLE_TOO_CLOSE_CM   — proximity stop distance (default 10)
    CAT_FOLLOW_SAFETY_OBSTACLE_DETECTED_CM    — veto ramp start (default 50)

``obstacle_too_close_cm`` acts as a floor under
``CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM``: DecisionEngine stops on whichever of
the two distances is more conservative.

Calibration JSON (``steering_limits.json``) may override at runtime when saved
from the Web UI:
    obstacle_too_close_cm
    obstacle_detected_cm
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import os

_PREFIX = "CAT_FOLLOW_SAFETY_"

# Contract spec default; kept as module constant for backward compatibility.
DEFAULT_OBSTACLE_TOO_CLOSE_CM = 10.0
DEFAULT_OBSTACLE_DETECTED_CM = 50.0

MIN_OBSTACLE_TOO_CLOSE_CM = 1.0
MAX_OBSTACLE_TOO_CLOSE_CM = 50.0
MIN_OBSTACLE_DETECTED_CM = 5.0
MAX_OBSTACLE_DETECTED_CM = 200.0


def _raw(name: str) -> str | None:
    value = os.getenv(f"{_PREFIX}{name}")
    if value is None:
        return None
    value = value.strip()
    return value or None


def _float(
    name: str,
    default: float,
    *,
    minimum: float,
    maximum: float,
) -> float:
    raw = _raw(name)
    if raw is None:
        return default
    value = float(raw)
    if not (minimum <= value <= maximum):
        raise ValueError(
            f"{_PREFIX}{name} must be between {minimum:g} and {maximum:g}"
        )
    return value


@dataclass(frozen=True)
class SafetyConfig:
    obstacle_too_close_cm: float = DEFAULT_OBSTACLE_TOO_CLOSE_CM
    obstacle_detected_cm: float = DEFAULT_OBSTACLE_DETECTED_CM

    def __post_init__(self) -> None:
        if not (MIN_OBSTACLE_TOO_CLOSE_CM <= self.obstacle_too_close_cm <= MAX_OBSTACLE_TOO_CLOSE_CM):
            raise ValueError("obstacle_too_close_cm out of allowed range")
        if not (MIN_OBSTACLE_DETECTED_CM <= self.obstacle_detected_cm <= MAX_OBSTACLE_DETECTED_CM):
            raise ValueError("obstacle_detected_cm out of allowed range")
        if self.obstacle_detected_cm <= self.obstacle_too_close_cm:
            raise ValueError("obstacle_detected_cm must be greater than obstacle_too_close_cm")


def load_safety_config_from_env() -> SafetyConfig:
    """Load safety thresholds from environment only."""

    return SafetyConfig(
        obstacle_too_close_cm=_float(
            "OBSTACLE_TOO_CLOSE_CM",
            DEFAULT_OBSTACLE_TOO_CLOSE_CM,
            minimum=MIN_OBSTACLE_TOO_CLOSE_CM,
            maximum=MAX_OBSTACLE_TOO_CLOSE_CM,
        ),
        obstacle_detected_cm=_float(
            "OBSTACLE_DETECTED_CM",
            DEFAULT_OBSTACLE_DETECTED_CM,
            minimum=MIN_OBSTACLE_DETECTED_CM,
            maximum=MAX_OBSTACLE_DETECTED_CM,
        ),
    )


def resolve_safety_config(calibration: Any | None = None) -> SafetyConfig:
    """Merge env defaults with optional calibration JSON overrides."""

    base = load_safety_config_from_env()
    if calibration is None:
        return base

    steering = {}
    try:
        data = calibration.get_all_calibration_data()
        steering = data.get("steering") or {}
    except Exception:  # noqa: BLE001
        steering = {}

    too_close = base.obstacle_too_close_cm
    detected = base.obstacle_detected_cm
    if isinstance(steering, dict):
        if steering.get("obstacle_too_close_cm") is not None:
            too_close = float(steering["obstacle_too_close_cm"])
        if steering.get("obstacle_detected_cm") is not None:
            detected = float(steering["obstacle_detected_cm"])

    return SafetyConfig(
        obstacle_too_close_cm=too_close,
        obstacle_detected_cm=detected,
    )


def safe_resolve_safety_config(
    calibration: Any | None = None,
) -> tuple[SafetyConfig | None, str | None]:
    """Resolve safety config without raising; report degradation instead.

    Returns ``(config, None)`` on success or ``(None, error_message)`` when
    env/calibration values are invalid (e.g. legacy corrupted JSON).
    """

    try:
        return resolve_safety_config(calibration), None
    except ValueError as exc:
        return None, str(exc)
    except Exception as exc:  # noqa: BLE001
        return None, f"safety config unavailable: {exc}"


def apply_safety_config_to_runtime(
    config: SafetyConfig,
    *,
    decision_engine: Any = None,
    range_adapter: Any = None,
    ros_bridge: Any = None,
) -> None:
    """Push resolved thresholds into live runtime components."""

    if decision_engine is not None and hasattr(decision_engine, "set_safety_thresholds"):
        decision_engine.set_safety_thresholds(config)
    if range_adapter is not None and hasattr(range_adapter, "set_safety_thresholds"):
        range_adapter.set_safety_thresholds(config)
    if ros_bridge is not None and hasattr(ros_bridge, "set_safety_thresholds"):
        ros_bridge.set_safety_thresholds(config)


__all__ = [
    "DEFAULT_OBSTACLE_DETECTED_CM",
    "DEFAULT_OBSTACLE_TOO_CLOSE_CM",
    "MAX_OBSTACLE_DETECTED_CM",
    "MAX_OBSTACLE_TOO_CLOSE_CM",
    "MIN_OBSTACLE_DETECTED_CM",
    "MIN_OBSTACLE_TOO_CLOSE_CM",
    "SafetyConfig",
    "apply_safety_config_to_runtime",
    "load_safety_config_from_env",
    "resolve_safety_config",
    "safe_resolve_safety_config",
]
