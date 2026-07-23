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
from cat_follow.control.types import CommandName, FsmState  # noqa: E402
from cat_follow.runtime.app import build_app  # noqa: E402


def _wait_until(predicate, timeout=2.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


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


def test_app_processes_command_through_full_stack(tmp_path):
    app = build_app(
        log_path=tmp_path / "telemetry.jsonl",
        stop_event=threading.Event(),
        target_rate_hz=200.0,
    )

    # Provide tracking before start_chase so it passes validation.
    app.comms_manager.submit_tracking(
        TrackingMessage(
            sequence=1,
            timestamp_ms=1,
            car=TrackingCar(x=0.0, y=0.0, confidence=1.0),
            cat=TrackingCat(x=10.0, y=10.0, confidence=1.0),
        )
    )
    ack = app.comms_manager.submit_command(
        CommandMessage(
            sequence=2001,
            timestamp_ms=2,
            command_id="cmd-app-start",
            command=CommandName.START_CHASE,
        )
    )
    assert ack.status.value == "accepted"

    app.start()
    try:
        assert _wait_until(
            lambda: app.fsm.state == FsmState.CHASE_A, timeout=2.0
        )
    finally:
        app.stop()

    assert app.shared_state.get_decision().requested_state == FsmState.CHASE_A


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


def test_app_with_range_adapter_triggers_failsafe_on_obstacle(tmp_path):
    """A close obstacle reading should propagate through the range adapter
    and the control loop into a FAILSAFE transition with a brake command."""

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
        assert _wait_until(
            lambda: app.fsm.state == FsmState.FAILSAFE, timeout=2.0
        )
        decision = app.shared_state.get_decision()
        assert decision.brake is True
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
            car=TrackingCar(x=0.0, y=0.0, confidence=1.0),
            cat=TrackingCat(x=10.0, y=10.0, confidence=1.0),
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

        # The control loop should advance the FSM into CHASE_A.
        assert _wait_until(
            lambda: app.fsm.state == FsmState.CHASE_A, timeout=2.0
        )
    finally:
        client.close()
        ack_server.close()
        app.stop()
