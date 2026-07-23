"""Build validated action plans for calibration routines."""

from __future__ import annotations

from typing import List, Optional

from cat_follow.motion.action_plan import (
    MAX_SPEED_PCT,
    MAX_STEER_DEG,
    ValidatedAction,
    validate_plan,
)


def speed_test_plan(*, speed: int, duration_s: float) -> tuple[list[ValidatedAction], list[str]]:
    speed_pct = max(1, min(MAX_SPEED_PCT, int(speed)))
    return validate_plan(
        [
            {
                "type": "drive",
                "direction": "forward",
                "speed_pct": speed_pct,
                "duration_s": float(duration_s),
            },
            {"type": "stop", "duration_s": 0.5},
        ]
    )


def steer_test_plan(
    *,
    angle: float,
    speed: int,
    duration_s: float,
    max_steer_deg: Optional[float] = None,
) -> tuple[list[ValidatedAction], list[str]]:
    limit = float(MAX_STEER_DEG if max_steer_deg is None else max_steer_deg)
    angle_deg = max(-limit, min(limit, float(angle)))
    speed_pct = max(1, min(MAX_SPEED_PCT, int(speed)))
    return validate_plan(
        [
            {
                "type": "steer",
                "angle_deg": angle_deg,
                "speed_pct": speed_pct,
                "duration_s": float(duration_s),
            },
            {"type": "stop", "duration_s": 0.5},
        ],
        max_steer_deg=limit,
    )
