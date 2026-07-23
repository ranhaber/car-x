"""Tests for calibration validate-before-save and degraded safety endpoints."""

from __future__ import annotations

import json

from cat_follow.calibration import Calibration
from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState as PrototypeSharedState
from cat_follow.safety_config import safe_resolve_safety_config
from cat_follow.web_ui.app import create_app


def _write_valid_calib(tmp_path):
    calib_dir = tmp_path / "calib"
    calib_dir.mkdir()
    (calib_dir / "speed_time_distance.json").write_text(
        json.dumps({"speed_to_cm_per_sec": {"30": 12.0, "50": 22.0}}),
        encoding="utf-8",
    )
    (calib_dir / "steering_limits.json").write_text(
        json.dumps(
            {
                "max_steer_angle_deg": 30,
                "target_distance_cm": 15,
                "min_turn_radius_cm": {"left": 40.0, "right": 40.0},
                "obstacle_too_close_cm": 10,
                "obstacle_detected_cm": 50,
                "notes": "keep me",
            }
        ),
        encoding="utf-8",
    )
    return calib_dir


def test_invalid_calibration_post_leaves_disk_and_memory_unchanged(monkeypatch, tmp_path):
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", "1")
    calib_dir = _write_valid_calib(tmp_path)
    calib = Calibration(calib_dir=str(calib_dir))
    before = calib.get_all_calibration_data()
    disk_before = (calib_dir / "steering_limits.json").read_text(encoding="utf-8")

    app = create_app(
        shared=PrototypeSharedState(allocate_pool()),
        calibration=calib,
    )
    client = app.test_client()
    res = client.post(
        "/api/calibration",
        json={
            "steering": {
                "obstacle_too_close_cm": 80,
                "obstacle_detected_cm": 50,
            }
        },
    )
    assert res.status_code == 400
    after = calib.get_all_calibration_data()
    assert after == before
    assert (calib_dir / "steering_limits.json").read_text(encoding="utf-8") == disk_before


def test_partial_calibration_preserves_unrelated_keys(monkeypatch, tmp_path):
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", "1")
    calib_dir = _write_valid_calib(tmp_path)
    calib = Calibration(calib_dir=str(calib_dir))
    app = create_app(
        shared=PrototypeSharedState(allocate_pool()),
        calibration=calib,
    )
    client = app.test_client()
    res = client.post(
        "/api/calibration",
        json={"steering": {"max_steer_angle_deg": 28}},
    )
    assert res.status_code == 200
    data = calib.get_all_calibration_data()
    assert data["steering"]["max_steer_angle_deg"] == 28
    assert data["steering"]["target_distance_cm"] == 15
    assert data["steering"]["notes"] == "keep me"
    assert data["steering"]["obstacle_too_close_cm"] == 10


def test_legacy_invalid_calibration_get_is_degraded_not_500(monkeypatch, tmp_path):
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", "1")
    calib_dir = tmp_path / "bad"
    calib_dir.mkdir()
    (calib_dir / "speed_time_distance.json").write_text("{}", encoding="utf-8")
    (calib_dir / "steering_limits.json").write_text(
        json.dumps(
            {
                "obstacle_too_close_cm": 80,
                "obstacle_detected_cm": 50,
            }
        ),
        encoding="utf-8",
    )
    calib = Calibration(calib_dir=str(calib_dir))
    cfg, err = safe_resolve_safety_config(calib)
    assert cfg is None
    assert err

    app = create_app(
        shared=PrototypeSharedState(allocate_pool()),
        calibration=calib,
    )
    client = app.test_client()
    for path in (
        "/api/calibration",
        "/api/movement/safety",
        "/api/movement/sequence/status",
        "/api/status",
        "/api/config",
    ):
        res = client.get(path)
        assert res.status_code == 200, path
        body = res.get_json()
        if path == "/api/calibration":
            assert body["safety_effective"]["safety_degraded"] is True
        elif path == "/api/config":
            assert body["safety_effective"]["safety_degraded"] is True
        else:
            assert body.get("safety_degraded") is True


def test_validate_plan_uses_calibration_max_steer(monkeypatch, tmp_path):
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", "1")
    calib_dir = _write_valid_calib(tmp_path)
    # Raise max steer to 30 via file (already 30); accept 28 deg.
    calib = Calibration(calib_dir=str(calib_dir))
    app = create_app(
        shared=PrototypeSharedState(allocate_pool()),
        calibration=calib,
    )
    client = app.test_client()
    res = client.post(
        "/api/movement/sequence/validate",
        json={
            "actions": [
                {
                    "type": "steer",
                    "angle_deg": 28,
                    "speed_pct": 20,
                    "duration_s": 1.0,
                }
            ]
        },
    )
    assert res.status_code == 200
    assert res.get_json()["ok"] is True
