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
    res = client.post("/api/target", json={"x_cm": 100.0, "y_cm": 200.0})
    assert res.status_code == 200


def test_motion_endpoint_requires_token_when_set(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", "s3cret")
    monkeypatch.setenv("CAT_FOLLOW_COMMS_TOKEN", "comms-s3cret")
    proto = PrototypeSharedState(allocate_pool())
    app = create_app(shared=proto)
    client = app.test_client()

    # Missing token -> 401
    res = client.post("/api/target", json={"x_cm": 100.0, "y_cm": 200.0})
    assert res.status_code == 401

    # Wrong token -> 401
    res = client.post(
        "/api/target", json={"x_cm": 100.0, "y_cm": 200.0}, headers={"X-Control-Token": "nope"}
    )
    assert res.status_code == 401

    # Correct token -> 200
    res = client.post(
        "/api/target", json={"x_cm": 100.0, "y_cm": 200.0}, headers={"X-Control-Token": "s3cret"}
    )
    assert res.status_code == 200


def test_production_web_degrades_without_h264_and_refuses_mutation(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_REQUIRE_H264", "1")
    monkeypatch.setattr(
        "cat_follow.web_ui.routes_h264.init_h264_routes",
        lambda _ctx, _app: False,
    )

    proto = PrototypeSharedState(allocate_pool())
    app = create_app(shared=proto)
    client = app.test_client()

    assert client.get("/api/status").status_code == 200
    capabilities = client.get("/api/stream/capabilities").get_json()
    assert capabilities["h264"] is False
    response = client.post("/api/target", json={"x_cm": 100.0, "y_cm": 200.0})
    assert response.status_code == 503


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
    assert data["fsm"]["state"] == "GETTING_CLOSE"
    assert data["state"] == "GETTING_CLOSE"
    assert "lidar_veto" in data["decision"]["active_constraints"]
    assert data["lidar"]["distance_cm"] == 85.0
    assert data["navigation"]["path_correction"] == 0.2
    assert data["overhead"]["selected_target_id"] is None
    assert data["mission"]["active_target_id"] is None
    assert data["mission"]["handoff_deadline_ms"] is None
    assert data["mission"]["search_stage"] == 0
    assert data["mission"]["search_lock_observations"] == 0


def test_stream_capabilities_endpoint():
    proto = PrototypeSharedState(allocate_pool())
    app = create_app(shared=proto)
    client = app.test_client()
    res = client.get("/api/stream/capabilities")
    assert res.status_code == 200
    data = res.get_json()
    assert data["h264"] in (True, False)
    assert data["resolution"] == "640x480"


def test_start_chase_api_requires_target_id(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", "s3cret")
    monkeypatch.setenv("CAT_FOLLOW_COMMS_TOKEN", "comms-s3cret")

    from cat_follow.comms.comms_manager import CommsManager
    from cat_follow.control.decision_engine import DecisionEngine
    from cat_follow.control.fsm import FSM

    proto = PrototypeSharedState(allocate_pool())
    runtime = RuntimeSharedState()
    fsm = FSM()
    engine = DecisionEngine(fsm)
    comms = CommsManager(shared_state=runtime, ack_sink=lambda _ack: None)
    comms.bind_runtime(decision_engine=engine, fsm=fsm)

    app = create_app(shared=proto, runtime_shared=runtime, comms_manager=comms)
    client = app.test_client()

    missing = client.post(
        "/api/command/start_chase",
        headers={"X-Control-Token": "s3cret"},
    )
    assert missing.status_code == 400

    with_target = client.post(
        "/api/command/start_chase",
        json={"target_id": "cat-17"},
        headers={"X-Control-Token": "s3cret"},
    )
    assert with_target.status_code == 200
    body = with_target.get_json()
    assert body["ack"]["status"] == "rejected"


def test_target_api_carries_centimeters_to_the_nav2_goal(monkeypatch):
    """A UI target in cm must reach Nav2 as the same distance in meters."""

    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", "s3cret")
    monkeypatch.setenv("CAT_FOLLOW_COMMS_TOKEN", "comms-s3cret")

    from cat_follow.comms.comms_manager import CommsManager
    from cat_follow.control.decision_engine import DecisionEngine
    from cat_follow.control.fsm import FSM
    from cat_follow.control.types import CommandName, SharedSnapshot, SystemState
    from cat_follow.navigation.manager import NavigationManager
    from cat_follow.target_config import TargetRuntimeConfig

    class _Transport:
        def __init__(self):
            self.goals = []

        def submit_goal(self, intent):
            self.goals.append(intent)
            return f"goal-{len(self.goals)}"

        def cancel_goal(self, action_goal_id):
            pass

    from cat_follow.runtime.shared_state import now_monotonic_ms

    proto = PrototypeSharedState(allocate_pool())
    runtime = RuntimeSharedState()
    runtime.update_system(SystemState(startup_ready=True))
    now_ms = now_monotonic_ms()
    runtime.update_range(
        RangeState(received_ms=now_ms, fresh=True, distance_cm=100.0, confidence=1.0)
    )
    runtime.update_lidar_range(
        RangeState(
            received_ms=now_ms,
            fresh=True,
            backend=RangeBackend.LIDAR_C1,
            distance_cm=100.0,
            confidence=1.0,
        )
    )
    fsm = FSM()
    engine = DecisionEngine(fsm)
    comms = CommsManager(shared_state=runtime, ack_sink=lambda _ack: None)
    comms.bind_runtime(decision_engine=engine, fsm=fsm)

    app = create_app(shared=proto, runtime_shared=runtime, comms_manager=comms)
    client = app.test_client()
    res = client.post(
        "/api/target",
        json={"x_cm": 250.0, "y_cm": -80.0},
        headers={"X-Control-Token": "s3cret"},
    )
    assert res.status_code == 200
    body = res.get_json()
    assert body["x_cm"] == 250.0
    assert body["ack"]["status"] == "accepted"

    command = runtime.get_command()
    assert command.last_command == CommandName.GO_TO
    assert command.objective_x_cm == 250.0
    assert command.objective_y_cm == -80.0

    transport = _Transport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    manager.tick(SharedSnapshot(command=command), FsmState.GOTO, 1000)
    assert transport.goals, "GOTO should submit a Nav2 goal"
    intent = transport.goals[-1]
    assert intent.x_m == 2.5
    assert intent.y_m == -0.8


def test_target_api_rejects_non_finite_and_missing_centimeters(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", "1")
    proto = PrototypeSharedState(allocate_pool())
    app = create_app(shared=proto)
    client = app.test_client()

    assert client.post("/api/target", json={"x": 1.0, "y": 2.0}).status_code == 400
    assert (
        client.post("/api/target", json={"x_cm": "NaN", "y_cm": 0.0}).status_code
        == 400
    )


def test_config_endpoint_readonly():
    proto = PrototypeSharedState(allocate_pool())
    app = create_app(shared=proto)
    client = app.test_client()
    res = client.get("/api/config")
    assert res.status_code == 200
    data = res.get_json()
    assert "camera" in data
    assert "perception" in data
    assert data["active_runtime"]["applied_to_behavior"] is True
    assert data["active_runtime"]["values"]["brake_reverse_trigger_cm"] == 15.0
    assert data["active_runtime"]["values"]["sensor_recovery_sec"] == 2.0
    assert data["active_runtime"]["values"]["overhead_invalid_max_sec"] == 10.0
    assert data["target_runtime"]["applied_to_behavior"] is False
    assert (
        data["target_runtime"]["values"]["brake_reverse_trigger_cm"] == 15.0
    )


def test_status_exposes_effective_target_and_active_config(monkeypatch):
    monkeypatch.setattr(
        "cat_follow.web_ui.routes_status.range_sensor.get_last_distance_cm",
        lambda: None,
    )
    from cat_follow.target_config import load_target_runtime_config

    proto = PrototypeSharedState(allocate_pool())
    runtime = RuntimeSharedState()
    target_cfg = load_target_runtime_config()
    app = create_app(
        shared=proto,
        runtime_shared=runtime,
        target_runtime_config=target_cfg,
    )
    client = app.test_client()
    res = client.get("/api/status")
    assert res.status_code == 200
    data = res.get_json()
    assert data["effective_target_config"]["applied_to_behavior"] is False
    assert data["effective_active_config"]["applied_to_behavior"] is True
    assert data["effective_active_config"]["values"]["overhead_invalid_max_sec"] == 10.0
    assert (
        data["effective_active_config"]["values"]["brake_reverse_trigger_cm"]
        == 15.0
    )


def test_build_app_with_web_ui_flag():
    from cat_follow.runtime.app import build_app

    app = build_app(web_ui=True, web_ui_port=5099)
    assert app.web_ui_thread is not None
    assert app.web_ui_thread.daemon is True
