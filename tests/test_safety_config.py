"""Tests for safety threshold configuration."""

import os

from cat_follow.calibration import Calibration
from cat_follow.control.decision_engine import DecisionEngine
from cat_follow.control.fsm import FSM
from cat_follow.safety_config import (
    SafetyConfig,
    load_safety_config_from_env,
    resolve_safety_config,
)


def test_env_defaults(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_SAFETY_OBSTACLE_TOO_CLOSE_CM", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_SAFETY_OBSTACLE_DETECTED_CM", raising=False)
    cfg = load_safety_config_from_env()
    assert cfg.obstacle_too_close_cm == 10.0
    assert cfg.obstacle_detected_cm == 50.0


def test_env_override(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_SAFETY_OBSTACLE_TOO_CLOSE_CM", "20")
    monkeypatch.setenv("CAT_FOLLOW_SAFETY_OBSTACLE_DETECTED_CM", "60")
    cfg = load_safety_config_from_env()
    assert cfg.obstacle_too_close_cm == 20.0
    assert cfg.obstacle_detected_cm == 60.0


def test_calibration_override(monkeypatch, tmp_path):
    monkeypatch.delenv("CAT_FOLLOW_SAFETY_OBSTACLE_TOO_CLOSE_CM", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_SAFETY_OBSTACLE_DETECTED_CM", raising=False)
    calib_dir = tmp_path / "calib"
    calib_dir.mkdir()
    (calib_dir / "speed_time_distance.json").write_text("{}", encoding="utf-8")
    (calib_dir / "steering_limits.json").write_text(
        '{"obstacle_too_close_cm": 15, "obstacle_detected_cm": 55}',
        encoding="utf-8",
    )
    calib = Calibration(calib_dir=str(calib_dir))
    cfg = resolve_safety_config(calib)
    assert cfg.obstacle_too_close_cm == 15.0
    assert cfg.obstacle_detected_cm == 55.0


def test_decision_engine_runtime_update():
    engine = DecisionEngine(FSM(), obstacle_too_close_cm=10.0)
    engine.set_safety_thresholds(SafetyConfig(obstacle_too_close_cm=20.0, obstacle_detected_cm=50.0))
    assert engine.obstacle_too_close_cm == 20.0
