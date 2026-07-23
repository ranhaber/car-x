"""Validated action plans for the Movement tab sequence builder.

Only four predefined action kinds are supported.  Free-form scripting is
deliberately rejected so the web UI cannot execute arbitrary motor logic.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union


class ActionKind(str, Enum):
    DRIVE = "drive"
    STEER = "steer"
    WAIT = "wait"
    STOP = "stop"


# Safety caps for open-loop sequences (Movement tab defaults).
MAX_ACTIONS = 50
MAX_TOTAL_DURATION_S = 120.0
MIN_DURATION_S = 0.1
MAX_ACTION_DURATION_S = 15.0
MAX_SPEED_PCT = 50
MIN_SPEED_PCT = 1
# Default aligns with calibration/steering_limits.json; callers should pass
# the live calibration limit via validate_plan(max_steer_deg=...).
MAX_STEER_DEG = 30.0


@dataclass(frozen=True)
class DriveAction:
    kind: ActionKind = ActionKind.DRIVE
    direction: str = "forward"
    speed_pct: int = 30
    duration_s: float = 1.0

    def normalized_speed(self) -> float:
        magnitude = self.speed_pct / 100.0
        return magnitude if self.direction == "forward" else -magnitude


@dataclass(frozen=True)
class SteerAction:
    kind: ActionKind = ActionKind.STEER
    angle_deg: float = 0.0
    speed_pct: int = 30
    duration_s: float = 1.0
    max_steer_deg: float = MAX_STEER_DEG

    def normalized_speed(self) -> float:
        return self.speed_pct / 100.0

    def normalized_steering(self) -> float:
        limit = self.max_steer_deg
        if limit <= 0:
            return 0.0
        clamped = max(-limit, min(limit, self.angle_deg))
        return clamped / limit


@dataclass(frozen=True)
class WaitAction:
    kind: ActionKind = ActionKind.WAIT
    duration_s: float = 1.0


@dataclass(frozen=True)
class StopAction:
    kind: ActionKind = ActionKind.STOP
    duration_s: float = 0.5


ValidatedAction = Union[DriveAction, SteerAction, WaitAction, StopAction]


def _coerce_float(value: Any, field: str, errors: List[str]) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        errors.append(f"{field} must be a number")
        return None


def _coerce_int(value: Any, field: str, errors: List[str]) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError):
        errors.append(f"{field} must be an integer")
        return None


def _resolve_max_steer_deg(max_steer_deg: Optional[float]) -> float:
    if max_steer_deg is None:
        return MAX_STEER_DEG
    try:
        value = float(max_steer_deg)
    except (TypeError, ValueError) as exc:
        raise ValueError("max_steer_deg must be a positive number") from exc
    if not (value > 0):
        raise ValueError("max_steer_deg must be a positive number")
    return value


def validate_action(
    raw: Any,
    *,
    max_steer_deg: Optional[float] = None,
) -> Tuple[Optional[ValidatedAction], List[str]]:
    """Validate one action dict.  Returns (action, errors)."""

    errors: List[str] = []
    if not isinstance(raw, dict):
        return None, ["each action must be an object"]

    try:
        steer_limit = _resolve_max_steer_deg(max_steer_deg)
    except ValueError as exc:
        return None, [str(exc)]

    kind_raw = raw.get("type")
    if not isinstance(kind_raw, str):
        return None, ["action.type is required"]
    try:
        kind = ActionKind(kind_raw)
    except ValueError:
        return None, [f"unsupported action type {kind_raw!r}"]

    if kind == ActionKind.DRIVE:
        direction = raw.get("direction", "forward")
        if direction not in {"forward", "backward"}:
            errors.append("drive.direction must be 'forward' or 'backward'")
        speed_pct = _coerce_int(raw.get("speed_pct", 30), "drive.speed_pct", errors)
        duration_s = _coerce_float(raw.get("duration_s", 1.0), "drive.duration_s", errors)
        if speed_pct is not None and not (MIN_SPEED_PCT <= speed_pct <= MAX_SPEED_PCT):
            errors.append(f"drive.speed_pct must be {MIN_SPEED_PCT}-{MAX_SPEED_PCT}")
        if duration_s is not None and not (MIN_DURATION_S <= duration_s <= MAX_ACTION_DURATION_S):
            errors.append(
                f"drive.duration_s must be {MIN_DURATION_S}-{MAX_ACTION_DURATION_S}"
            )
        if errors:
            return None, errors
        return DriveAction(direction=direction, speed_pct=speed_pct, duration_s=duration_s), []

    if kind == ActionKind.STEER:
        angle_deg = _coerce_float(raw.get("angle_deg", 0.0), "steer.angle_deg", errors)
        speed_pct = _coerce_int(raw.get("speed_pct", 30), "steer.speed_pct", errors)
        duration_s = _coerce_float(raw.get("duration_s", 1.0), "steer.duration_s", errors)
        if angle_deg is not None and abs(angle_deg) > steer_limit:
            errors.append(f"steer.angle_deg must be within ±{steer_limit:g}")
        if speed_pct is not None and not (MIN_SPEED_PCT <= speed_pct <= MAX_SPEED_PCT):
            errors.append(f"steer.speed_pct must be {MIN_SPEED_PCT}-{MAX_SPEED_PCT}")
        if duration_s is not None and not (MIN_DURATION_S <= duration_s <= MAX_ACTION_DURATION_S):
            errors.append(
                f"steer.duration_s must be {MIN_DURATION_S}-{MAX_ACTION_DURATION_S}"
            )
        if errors:
            return None, errors
        return (
            SteerAction(
                angle_deg=angle_deg,
                speed_pct=speed_pct,
                duration_s=duration_s,
                max_steer_deg=steer_limit,
            ),
            [],
        )

    if kind == ActionKind.WAIT:
        duration_s = _coerce_float(raw.get("duration_s", 1.0), "wait.duration_s", errors)
        if duration_s is not None and not (MIN_DURATION_S <= duration_s <= MAX_ACTION_DURATION_S):
            errors.append(
                f"wait.duration_s must be {MIN_DURATION_S}-{MAX_ACTION_DURATION_S}"
            )
        if errors:
            return None, errors
        return WaitAction(duration_s=duration_s), []

    # STOP
    duration_s = _coerce_float(raw.get("duration_s", 0.5), "stop.duration_s", errors)
    if duration_s is not None and not (MIN_DURATION_S <= duration_s <= MAX_ACTION_DURATION_S):
        errors.append(
            f"stop.duration_s must be {MIN_DURATION_S}-{MAX_ACTION_DURATION_S}"
        )
    if errors:
        return None, errors
    return StopAction(duration_s=duration_s), []


def validate_plan(
    raw_actions: Any,
    *,
    max_steer_deg: Optional[float] = None,
) -> Tuple[List[ValidatedAction], List[str]]:
    """Validate a full action list for the Movement tab."""

    errors: List[str] = []
    if not isinstance(raw_actions, list):
        return [], ["actions must be a list"]
    if not raw_actions:
        return [], ["actions must not be empty"]
    if len(raw_actions) > MAX_ACTIONS:
        return [], [f"at most {MAX_ACTIONS} actions allowed"]

    try:
        steer_limit = _resolve_max_steer_deg(max_steer_deg)
    except ValueError as exc:
        return [], [str(exc)]

    validated: List[ValidatedAction] = []
    total_duration = 0.0
    for index, raw in enumerate(raw_actions):
        action, action_errors = validate_action(raw, max_steer_deg=steer_limit)
        if action_errors:
            errors.extend([f"action[{index}]: {msg}" for msg in action_errors])
            continue
        assert action is not None
        validated.append(action)
        total_duration += action.duration_s

    if total_duration > MAX_TOTAL_DURATION_S:
        errors.append(
            f"total planned duration must be <= {MAX_TOTAL_DURATION_S:g}s "
            f"(got {total_duration:.2f}s)"
        )
    if errors:
        return [], errors
    return validated, []


def plan_to_public_dict(actions: Sequence[ValidatedAction]) -> List[Dict[str, Any]]:
    """Serialize a validated plan for API responses."""

    out: List[Dict[str, Any]] = []
    for action in actions:
        if isinstance(action, DriveAction):
            out.append(
                {
                    "type": action.kind.value,
                    "direction": action.direction,
                    "speed_pct": action.speed_pct,
                    "duration_s": action.duration_s,
                }
            )
        elif isinstance(action, SteerAction):
            out.append(
                {
                    "type": action.kind.value,
                    "angle_deg": action.angle_deg,
                    "speed_pct": action.speed_pct,
                    "duration_s": action.duration_s,
                }
            )
        elif isinstance(action, WaitAction):
            out.append({"type": action.kind.value, "duration_s": action.duration_s})
        else:
            out.append({"type": action.kind.value, "duration_s": action.duration_s})
    return out
