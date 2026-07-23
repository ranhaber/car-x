"""Tests for Movement tab action plan validation."""

import pytest

from cat_follow.motion.action_plan import (
    MAX_ACTIONS,
    MAX_SPEED_PCT,
    validate_action,
    validate_plan,
)


def test_validate_drive_action():
    action, errors = validate_action(
        {"type": "drive", "direction": "forward", "speed_pct": 20, "duration_s": 1.0}
    )
    assert not errors
    assert action.normalized_speed() == 0.2


def test_validate_drive_rejects_excessive_speed():
    _, errors = validate_action(
        {"type": "drive", "direction": "forward", "speed_pct": MAX_SPEED_PCT + 1, "duration_s": 1.0}
    )
    assert errors


def test_validate_plan_rejects_too_many_actions():
    raw = [{"type": "wait", "duration_s": 0.5} for _ in range(MAX_ACTIONS + 1)]
    _, errors = validate_plan(raw)
    assert any("at most" in err for err in errors)


def test_validate_plan_accepts_mixed_sequence():
    plan, errors = validate_plan(
        [
            {"type": "drive", "direction": "forward", "speed_pct": 10, "duration_s": 0.5},
            {"type": "wait", "duration_s": 0.5},
            {"type": "stop", "duration_s": 0.5},
        ]
    )
    assert not errors
    assert len(plan) == 3


def test_validate_steer_respects_custom_max():
    action, errors = validate_action(
        {"type": "steer", "angle_deg": 28, "speed_pct": 20, "duration_s": 1.0},
        max_steer_deg=30,
    )
    assert not errors
    assert action.normalized_steering() == pytest.approx(28 / 30)

    _, errors = validate_action(
        {"type": "steer", "angle_deg": 28, "speed_pct": 20, "duration_s": 1.0},
        max_steer_deg=25,
    )
    assert errors
