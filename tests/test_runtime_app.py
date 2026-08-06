"""Smoke test for the standalone runtime app."""

import json
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.comms.messages import (  # noqa: E402
    CommandMessage,
    TrackingCar,
    TrackingCat,
    TrackingMessage,
)
from cat_follow.control.types import (  # noqa: E402
    CommandName,
    FsmState,
    RangeBackend,
    RangeState,
)
from cat_follow.runtime.app import (  # noqa: E402
    _build_web_ui_thread,
    _make_default_backend,
    build_app,
)
from cat_follow.motion.motor_interface import NoOpMotorBackend  # noqa: E402


def _wait_until(predicate, timeout=2.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def _seed_healthy_sensors(app):
    from tests.test_comms_manager_helpers import durable_home

    now_ms = int(time.monotonic() * 1000)
    app.shared_state.update_range(
        RangeState(
            received_ms=now_ms,
            fresh=True,
            distance_cm=100.0,
            confidence=1.0,
        )
    )
    app.shared_state.update_lidar_range(
        RangeState(
            received_ms=now_ms,
            fresh=True,
            backend=RangeBackend.LIDAR_C1,
            distance_cm=100.0,
            confidence=1.0,
        )
    )
    app.shared_state.update_home(durable_home())


def test_build_app_wires_all_components(tmp_path):
    app = build_app(
        log_path=tmp_path / "telemetry.jsonl",
        stop_event=threading.Event(),
        target_rate_hz=200.0,  # fast for tests
    )
    assert app.shared_state is not None
    assert app.control_loop is not None
    assert app.comms_manager is not None
    assert app.motor_backend is not None
    assert app.recording_writer is not None
    assert app.perception_lifecycle_manager is not None
    assert app.target_runtime_config.brake_reverse_trigger_cm == 15.0
    # Slice 1 is observability-only: active control still uses the legacy
    # safety threshold until the BRAKE_REVERSE implementation slice.
    assert app.decision_engine.obstacle_too_close_cm == 10.0


def test_make_default_backend_passes_pan_forward_deg():
    """CLI injects a backend into build_app; forward must match MotorInterface."""
    backend = _make_default_backend(use_picarx=False, pan_forward_deg=8.5)
    assert isinstance(backend, NoOpMotorBackend)
    assert backend._pan_forward_deg == 8.5
    backend.emergency_stop()
    assert backend.look_applied[-1] == 8.5


def test_look_frame_half_width_default_matches_640_frame():
    from cat_follow.target_config import TargetRuntimeConfig

    cfg = TargetRuntimeConfig()
    assert cfg.look_frame_half_width_px == 320.0


def test_main_cli_wires_look_pan_forward_env_into_backend(monkeypatch):
    """argparse main path: LOOK_PAN_FORWARD_DEG → injected motor_backend."""
    import threading

    from cat_follow.runtime.app import main

    captured = {}

    class _FakeApp:
        def start(self) -> None:
            pass

        def stop(self, timeout: float = 2.0) -> None:
            pass

    def _fake_build_app(**kwargs):
        captured.update(kwargs)
        return _FakeApp()

    monkeypatch.setenv("CAT_FOLLOW_LOOK_PAN_FORWARD_DEG", "8.5")
    monkeypatch.setattr("cat_follow.runtime.app.build_app", _fake_build_app)
    monkeypatch.setattr(
        "cat_follow.runtime.app._install_signal_handlers", lambda _e: None
    )
    monkeypatch.setattr(threading.Event, "wait", lambda self, timeout=None: True)

    assert main([]) == 0
    backend = captured["motor_backend"]
    assert isinstance(backend, NoOpMotorBackend)
    assert backend._pan_forward_deg == 8.5


def test_build_app_applies_persisted_calibration_safety(tmp_path):
    from cat_follow.calibration import Calibration

    calib_dir = tmp_path / "calib"
    calib_dir.mkdir()
    (calib_dir / "speed_time_distance.json").write_text("{}", encoding="utf-8")
    (calib_dir / "steering_limits.json").write_text(
        '{"obstacle_too_close_cm": 22, "obstacle_detected_cm": 58}',
        encoding="utf-8",
    )
    calib = Calibration(calib_dir=str(calib_dir))

    app = build_app(
        log_path=tmp_path / "telemetry.jsonl",
        stop_event=threading.Event(),
        target_rate_hz=200.0,
        calibration=calib,
    )
    assert app.decision_engine.obstacle_too_close_cm == 22.0


def test_app_starts_runs_and_stops_cleanly(tmp_path):
    log_path = tmp_path / "telemetry.jsonl"
    app = build_app(
        log_path=log_path,
        stop_event=threading.Event(),
        target_rate_hz=200.0,
    )

    app.start()
    try:
        assert _wait_until(lambda: app.control_loop.tick_count >= 5, timeout=2.0)
    finally:
        app.stop()

    assert app.control_loop.tick_count >= 5
    assert log_path.exists()
    contents = log_path.read_text(encoding="utf-8")
    assert contents.strip(), "JSONL telemetry file should contain at least one event"
    events = [json.loads(line) for line in contents.splitlines() if line.strip()]
    config_events = [
        event for event in events if event["event_type"] == "configuration"
    ]
    assert config_events
    target = config_events[0]["data"]["target_runtime"]
    assert target["values"]["brake_reverse_trigger_cm"] == 15.0
    assert target["applied_to_behavior"] is False
    active = config_events[0]["data"]["active_runtime"]
    assert active["applied_to_behavior"] is True
    assert active["values"]["brake_reverse_trigger_cm"] == 15.0
    assert active["values"]["sensor_recovery_sec"] == 2.0
    assert active["values"]["overhead_invalid_max_sec"] == 10.0


def test_invalid_target_env_fails_build_app(monkeypatch, tmp_path):
    monkeypatch.setenv("CAT_FOLLOW_BRAKE_REVERSE_RESET_CM", "10")
    try:
        build_app(
            log_path=tmp_path / "telemetry.jsonl",
            stop_event=threading.Event(),
            target_rate_hz=200.0,
        )
        assert False, "expected ValueError from invalid target config"
    except ValueError as exc:
        assert "brake_reverse_reset_cm" in str(exc)


def test_target_brake_reverse_env_is_wired_into_decision_engine(monkeypatch, tmp_path):
    monkeypatch.setenv("CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM", "5")
    app = build_app(
        log_path=tmp_path / "telemetry.jsonl",
        stop_event=threading.Event(),
        target_rate_hz=200.0,
    )
    assert app.target_runtime_config.brake_reverse_trigger_cm == 5.0
    assert app.decision_engine.brake_reverse_trigger_cm == 5.0


def test_production_startup_degrades_when_web_and_tokens_unavailable(
    tmp_path, monkeypatch
):
    """Optional monitoring/auth failures must not suppress the core workers."""
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_REQUIRE_H264", "1")

    def fail_web_app(**_kwargs):
        raise RuntimeError("flask-sock unavailable")

    monkeypatch.setattr("cat_follow.web_ui.app.create_app", fail_web_app)

    proto_stop = threading.Event()
    started = set()

    def worker(name):
        started.add(name)
        proto_stop.wait(timeout=2.0)

    proto_threads = tuple(
        threading.Thread(
            target=worker,
            args=(name,),
            name=f"Production{name}",
            daemon=True,
        )
        for name in ("Camera", "Detector", "Tracker")
    )
    app = build_app(
        log_path=tmp_path / "telemetry.jsonl",
        target_rate_hz=200.0,
        web_ui=True,
        udp_listen_port=0,
        prototype_perception_threads=proto_threads,
        prototype_perception_stop_event=proto_stop,
    )

    assert app.web_ui_thread is None
    assert app.udp_receiver is None
    app.start()
    try:
        assert _wait_until(lambda: started == {"Camera", "Detector", "Tracker"})
        assert _wait_until(lambda: app.control_loop.tick_count >= 5)
    finally:
        app.stop()


def test_missing_tls_files_disable_only_web_server(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_SSL_CERTFILE", "/missing/web-ui.crt")
    monkeypatch.setenv("CAT_FOLLOW_WEB_SSL_KEYFILE", "/missing/web-ui.key")
    run_kwargs = {}
    ran = threading.Event()

    class FakeFlaskApp:
        def run(self, **kwargs):
            run_kwargs.update(kwargs)
            ran.set()

    monkeypatch.setattr(
        "cat_follow.web_ui.app.create_app", lambda **_kwargs: FakeFlaskApp()
    )
    thread = _build_web_ui_thread(
        runtime_shared=None,
        comms_manager=None,
        memory_shared=object(),
        picarx=None,
        port=5000,
        sequence_executor=object(),
    )
    assert thread is not None
    thread.start()
    thread.join(timeout=1.0)

    assert not ran.is_set()
    assert run_kwargs == {}


def test_app_processes_command_through_full_stack(tmp_path):
    app = build_app(
        log_path=tmp_path / "telemetry.jsonl",
        stop_event=threading.Event(),
        target_rate_hz=200.0,
    )
    _seed_healthy_sensors(app)
    app.start()

    try:
        # Provide target-scoped tracking before start_chase validation.
        app.comms_manager.submit_tracking(
            TrackingMessage(
                sequence=1,
                timestamp_ms=1,
                perimeter_id="yard-v3",
                selected_target_id="cat-17",
                car=TrackingCar(x=0.0, y=0.0, confidence=1.0),
                cat=TrackingCat(
                    x=10.0,
                    y=10.0,
                    confidence=1.0,
                    target_id="cat-17",
                ),
            )
        )
        ack = app.comms_manager.submit_command(
            CommandMessage(
                sequence=2001,
                timestamp_ms=2,
                command_id="cmd-app-start",
                command=CommandName.START_CHASE,
                params={"target_id": "cat-17"},
            )
        )
        assert ack.status.value == "accepted"
        assert ack.state == FsmState.GETTING_CLOSE
        assert ack.applied_control_sequence is not None
        assert _wait_until(
            lambda: app.fsm.state == FsmState.SEARCH, timeout=2.0
        )
    finally:
        app.stop()

    assert app.shared_state.get_decision().requested_state == FsmState.SEARCH


class _FakePrototypeVisionSS:
    """Minimal stub satisfying VisionAdapter's prototype-side contract."""

    def __init__(self, bbox=(0.0, 0.0, 0.0, 0.0, 0.0)):
        self._bbox = bbox

    def set_bbox(self, bbox):
        self._bbox = bbox

    def get_bbox_tracker(self):
        return self._bbox


def test_app_with_vision_adapter_publishes_vision_state(tmp_path):
    """build_app constructs a VisionAdapter from a prototype SS + image
    dimensions and the app populates ``SharedState.vision`` while running.
    """

    proto_ss = _FakePrototypeVisionSS(
        bbox=(300.0, 200.0, 40.0, 40.0, 1.0)  # centered, valid
    )

    app = build_app(
        log_path=tmp_path / "telemetry.jsonl",
        stop_event=threading.Event(),
        target_rate_hz=200.0,
        prototype_vision_shared_state=proto_ss,
        vision_image_width=640,
        vision_image_height=480,
    )
    assert app.vision_adapter is not None

    app.start()
    try:
        assert _wait_until(
            lambda: app.shared_state.get_vision().cat_visible, timeout=2.0
        )
        assert _wait_until(
            lambda: app.shared_state.get_vision().cat_visible_stable,
            timeout=2.0,
        )
    finally:
        app.stop()


def test_close_range_while_idle_remains_stationary(tmp_path):
    """Close range does not start reverse without an autonomous objective."""

    def read_close_obstacle():
        return 5.0  # below OBSTACLE_TOO_CLOSE_CM

    app = build_app(
        log_path=tmp_path / "telemetry.jsonl",
        stop_event=threading.Event(),
        target_rate_hz=200.0,
        range_read_distance=read_close_obstacle,
    )
    assert app.range_adapter is not None

    app.start()
    try:
        assert _wait_until(lambda: app.control_loop.tick_count >= 5)
        assert app.fsm.state == FsmState.IDLE
        decision = app.shared_state.get_decision()
        assert decision.speed == 0.0
    finally:
        app.stop()


def test_app_with_prototype_perception_threads_lifecycle(tmp_path):
    """Verify build_app accepts pre-built prototype perception threads and
    coordinates start/stop with the rest of the runtime."""

    proto_ss = _FakePrototypeVisionSS(
        bbox=(300.0, 200.0, 40.0, 40.0, 1.0)
    )

    proto_stop = threading.Event()
    started: list = []

    def fake_camera_loop(_ss, stop_event):
        started.append("camera")
        stop_event.wait(timeout=5.0)

    def fake_tracker_loop(_ss, stop_event):
        started.append("tracker")
        stop_event.wait(timeout=5.0)

    def fake_detector_loop(_ss, stop_event):
        started.append("detector")
        stop_event.wait(timeout=5.0)

    proto_threads = (
        threading.Thread(
            target=fake_camera_loop,
            args=(proto_ss, proto_stop),
            name="FakeCamera",
            daemon=True,
        ),
        threading.Thread(
            target=fake_tracker_loop,
            args=(proto_ss, proto_stop),
            name="FakeTracker",
            daemon=True,
        ),
        threading.Thread(
            target=fake_detector_loop,
            args=(proto_ss, proto_stop),
            name="FakeDetector",
            daemon=True,
        ),
    )

    distances = iter([42.0, 30.0, 20.0])

    def read_distance():
        try:
            return next(distances)
        except StopIteration:
            return 20.0

    app = build_app(
        log_path=tmp_path / "telemetry.jsonl",
        stop_event=threading.Event(),
        target_rate_hz=200.0,
        prototype_vision_shared_state=proto_ss,
        vision_image_width=640,
        vision_image_height=480,
        range_read_distance=read_distance,
        prototype_perception_threads=proto_threads,
        prototype_perception_stop_event=proto_stop,
    )
    assert app.vision_adapter is not None
    assert app.range_adapter is not None
    assert len(app.prototype_perception_threads) == 3

    app.start()
    try:
        # All three fake prototype threads should have started.
        assert _wait_until(lambda: len(started) == 3, timeout=2.0)
        # Vision should have populated the contract SS via the adapter.
        assert _wait_until(
            lambda: app.shared_state.get_vision().cat_visible, timeout=2.0
        )
    finally:
        app.stop()

    # After stop(), the prototype threads must have observed the stop event.
    for thread in app.prototype_perception_threads:
        assert not thread.is_alive(), thread.name


def test_app_with_udp_transport_round_trip(tmp_path):
    # An ACK server represents the overhead system listening for ACKs.
    ack_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ack_server.bind(("127.0.0.1", 0))
    ack_server.settimeout(2.0)
    ack_host, ack_port = ack_server.getsockname()

    app = build_app(
        log_path=tmp_path / "telemetry.jsonl",
        stop_event=threading.Event(),
        target_rate_hz=200.0,
        udp_listen_host="127.0.0.1",
        udp_listen_port=0,  # OS-assigned
        udp_target_host=ack_host,
        udp_target_port=ack_port,
    )
    _seed_healthy_sensors(app)
    app.start()

    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.bind(("127.0.0.1", 0))
    try:
        # Resolve the actual port the app bound to.
        receiver_address = app.udp_receiver.bound_address
        assert receiver_address is not None

        # Send tracking + start_chase via UDP.
        tracking = TrackingMessage(
            sequence=1,
            timestamp_ms=0,
            perimeter_id="yard-v3",
            selected_target_id="cat-17",
            car=TrackingCar(x=0.0, y=0.0, confidence=1.0),
            cat=TrackingCat(
                x=10.0,
                y=10.0,
                confidence=1.0,
                target_id="cat-17",
            ),
        )
        client.sendto(
            json.dumps(tracking.to_dict()).encode("utf-8"),
            receiver_address,
        )
        cmd = CommandMessage(
            sequence=2001,
            timestamp_ms=0,
            command_id="cmd-udp-start",
            command=CommandName.START_CHASE,
            params={"target_id": "cat-17"},
        )
        client.sendto(
            json.dumps(cmd.to_dict()).encode("utf-8"),
            receiver_address,
        )

        # Wait for the ACK to come back over UDP.
        data, _addr = ack_server.recvfrom(65535)
        payload = json.loads(data.decode("utf-8"))
        assert payload["status"] == "accepted"
        assert payload["command_id"] == "cmd-udp-start"

        # The close overhead target advances the FSM into SEARCH.
        assert _wait_until(
            lambda: app.fsm.state == FsmState.SEARCH, timeout=2.0
        )
    finally:
        client.close()
        ack_server.close()
        app.stop()
