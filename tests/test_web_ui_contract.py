"""Tests for contract-aware web UI status / capabilities helpers."""

from __future__ import annotations

from cat_follow.control.types import (
    DecisionState,
    FSMSnapshot,
    FsmState,
    NavigationState,
    RangeBackend,
    RangeState,
    ReasonCode,
)
from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState as PrototypeSharedState
from cat_follow.perception.status import (
    get_perception_diagnostics,
    update_perception_diagnostics,
)
from cat_follow.runtime.shared_state import SharedState as RuntimeSharedState
from cat_follow.web_ui.app import create_app


def test_perception_diagnostics_roundtrip():
    update_perception_diagnostics(
        phase="TRACKING",
        backend="rknn",
        model_loaded=True,
        lores_active=True,
        motion=True,
        motion_gating=True,
    )
    d = get_perception_diagnostics()
    assert d.phase == "TRACKING"
    assert d.model_loaded is True
    assert d.lores_active is True


def test_motion_endpoint_open_when_no_token(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", "1")
    proto = PrototypeSharedState(allocate_pool())
    app = create_app(shared=proto)
    client = app.test_client()
    res = client.post("/api/target", json={"x": 1.0, "y": 2.0})
    assert res.status_code == 200


def test_motion_endpoint_requires_token_when_set(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", "s3cret")
    monkeypatch.setenv("CAT_FOLLOW_COMMS_TOKEN", "comms-s3cret")
    proto = PrototypeSharedState(allocate_pool())
    app = create_app(shared=proto)
    client = app.test_client()

    # Missing token -> 401
    res = client.post("/api/target", json={"x": 1.0, "y": 2.0})
    assert res.status_code == 401

    # Wrong token -> 401
    res = client.post(
        "/api/target", json={"x": 1.0, "y": 2.0}, headers={"X-Control-Token": "nope"}
    )
    assert res.status_code == 401

    # Correct token -> 200
    res = client.post(
        "/api/target", json={"x": 1.0, "y": 2.0}, headers={"X-Control-Token": "s3cret"}
    )
    assert res.status_code == 200


def test_stop_endpoint_never_requires_token(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", "s3cret")
    monkeypatch.setenv("CAT_FOLLOW_COMMS_TOKEN", "comms-s3cret")
    proto = PrototypeSharedState(allocate_pool())
    app = create_app(shared=proto)
    client = app.test_client()
    # Stopping the car must always be allowed, even without a token.
    assert client.post("/api/stop").status_code == 200
    assert client.post("/api/command/emergency_stop").status_code == 200


def test_status_prototype_mode(monkeypatch):
    monkeypatch.setattr(
        "cat_follow.web_ui.routes_status.range_sensor.get_last_distance_cm",
        lambda: 42.5,
    )
    proto = PrototypeSharedState(allocate_pool())
    proto.set_tracked_targets(
        {
            "PRIMARY_CAT": (1, 10.0, 20.0, 30.0, 40.0, 0.95, 0, 1.0),
            "SECONDARY_CAT": (2, 50.0, 60.0, 20.0, 25.0, 0.80, 1, 1.0),
        }
    )
    app = create_app(shared=proto)
    client = app.test_client()
    res = client.get("/api/status")
    assert res.status_code == 200
    data = res.get_json()
    assert data["mode"] == "prototype"
    assert data["ultrasonic_cm"] == 42.5
    assert "legacy" in data
    assert "stream_clients" in data
    assert "perception" in data
    assert data["tracked_targets"]["PRIMARY_CAT"]["track_id"] == 1
    assert data["tracked_targets"]["SECONDARY_CAT"]["confidence"] == 0.8
    assert data["cats"]["PRIMARY_CAT"]["track_id"] == 1
    assert data["cat_injection"]["enabled"] is False
    assert data["cat_injection"]["detection_fallback"] is False


def test_status_contract_mode(monkeypatch):
    monkeypatch.setattr(
        "cat_follow.web_ui.routes_status.range_sensor.get_last_distance_cm",
        lambda: None,
    )
    proto = PrototypeSharedState(allocate_pool())
    runtime = RuntimeSharedState()
    runtime.update_fsm(
        FSMSnapshot(state=FsmState.CHASE_A, last_transition_reason=ReasonCode.INIT)
    )
    runtime.update_decision(
        DecisionState(
            requested_state=FsmState.CHASE_A,
            speed=0.3,
            steering=0.1,
            brake=False,
            reason=ReasonCode.GLOBAL_CHASE,
            active_constraints=("lidar_veto", "navigation"),
        )
    )
    runtime.update_lidar_range(
        RangeState(
            fresh=True,
            backend=RangeBackend.LIDAR_C1,
            distance_cm=85.0,
            obstacle_detected=True,
            obstacle_critical=True,
        )
    )
    runtime.update_navigation(
        NavigationState(
            fresh=True,
            heading=0.5,
            heading_valid=True,
            path_correction=0.2,
            speed_limit=0.4,
        )
    )

    app = create_app(shared=proto, runtime_shared=runtime)
    client = app.test_client()
    res = client.get("/api/status")
    assert res.status_code == 200
    data = res.get_json()
    assert data["mode"] == "contract"
    assert data["fsm"]["state"] == "CHASE_A"
    assert data["state"] == "CHASE_A"
    assert "lidar_veto" in data["decision"]["active_constraints"]
    assert data["lidar"]["distance_cm"] == 85.0
    assert data["navigation"]["path_correction"] == 0.2


def test_stream_capabilities_endpoint():
    proto = PrototypeSharedState(allocate_pool())
    app = create_app(shared=proto)
    client = app.test_client()
    res = client.get("/api/stream/capabilities")
    assert res.status_code == 200
    data = res.get_json()
    assert data["mjpeg"] is True
    assert "h264" in data
    assert "resolutions" in data


def test_config_endpoint_readonly():
    proto = PrototypeSharedState(allocate_pool())
    app = create_app(shared=proto)
    client = app.test_client()
    res = client.get("/api/config")
    assert res.status_code == 200
    data = res.get_json()
    assert "camera" in data
    assert "perception" in data


def test_build_app_with_web_ui_flag():
    from cat_follow.runtime.app import build_app

    app = build_app(web_ui=True, web_ui_port=5099)
    assert app.web_ui_thread is not None
    assert app.web_ui_thread.daemon is True
