"""Tests for calibration motion plan builders."""

from cat_follow.motion.calibration_plans import speed_test_plan, steer_test_plan


def test_speed_test_plan_validates():
    plan, errors = speed_test_plan(speed=25, duration_s=1.0)
    assert not errors
    assert len(plan) == 2
    assert plan[0].speed_pct == 25


def test_steer_test_plan_clamps_angle():
    plan, errors = steer_test_plan(angle=40, speed=30, duration_s=2.0)
    assert not errors
    assert plan[0].angle_deg == 30.0


def test_steer_test_plan_respects_custom_max():
    plan, errors = steer_test_plan(
        angle=40, speed=30, duration_s=2.0, max_steer_deg=25.0
    )
    assert not errors
    assert plan[0].angle_deg == 25.0
