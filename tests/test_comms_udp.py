"""Tests for the UDP receiver and sender.

All tests use loopback (``127.0.0.1``) and OS-assigned ports; no real
network is required.
"""

import json
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.comms.comms_manager import CommsManager  # noqa: E402
from cat_follow.comms.messages import (  # noqa: E402
    AckMessage,
    CommandMessage,
    TrackingCar,
    TrackingCat,
    TrackingMessage,
)
from cat_follow.comms.udp_receiver import UdpReceiver  # noqa: E402
from cat_follow.comms.udp_sender import UdpSender  # noqa: E402
from cat_follow.control.types import (  # noqa: E402
    AckStatus,
    AckType,
    CommandName,
    FsmState,
    ReasonCode,
    TelemetryEventType,
)
from cat_follow.runtime.shared_state import SharedState  # noqa: E402
from cat_follow.telemetry.async_logger import AsyncLogger, CallableSink  # noqa: E402


def _wait_until(predicate, timeout=2.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def _make_manager_and_acks():
    ss = SharedState()
    received_acks = []
    manager = CommsManager(shared_state=ss, ack_sink=received_acks.append)
    return manager, ss, received_acks


def _start_receiver(manager, *, logger=None):
    receiver = UdpReceiver(
        comms_manager=manager,
        bind_host="127.0.0.1",
        bind_port=0,
        logger=logger,
        recv_timeout_s=0.05,
    )
    receiver.start()
    address = receiver.bound_address
    assert address is not None, "receiver should be bound after start"
    return receiver, address


def _client_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))  # ephemeral source port
    return sock


# ── receiver: tracking ─────────────────────────────────────────────


def test_receiver_dispatches_tracking_message_to_comms_manager():
    manager, ss, _ = _make_manager_and_acks()
    receiver, address = _start_receiver(manager)
    client = _client_socket()
    try:
        msg = TrackingMessage(
            sequence=1,
            timestamp_ms=1000,
            car=TrackingCar(x=1.0, y=2.0, confidence=1.0),
            cat=TrackingCat(x=3.0, y=4.0, confidence=1.0),
        )
        client.sendto(json.dumps(msg.to_dict()).encode("utf-8"), address)
        assert _wait_until(lambda: ss.get_overhead().sequence == 1)
    finally:
        client.close()
        receiver.stop()

    overhead = ss.get_overhead()
    assert overhead.car.x == 1.0
    assert overhead.cat.y == 4.0


# ── receiver: command ─────────────────────────────────────────────


def test_receiver_dispatches_command_message_and_records_ack():
    manager, ss, acks = _make_manager_and_acks()
    receiver, address = _start_receiver(manager)
    client = _client_socket()
    try:
        cmd = CommandMessage(
            sequence=2001,
            timestamp_ms=10,
            command_id="cmd-set-home-1",
            command=CommandName.SET_HOME,
            params={"home": {"x": 5.0, "y": 6.0, "frame_id": "yard"}},
        )
        client.sendto(json.dumps(cmd.to_dict()).encode("utf-8"), address)
        assert _wait_until(lambda: ss.get_home().set is True)
    finally:
        client.close()
        receiver.stop()

    assert acks, "CommsManager should have produced an ACK"
    assert acks[-1].status == AckStatus.ACCEPTED
    assert ss.get_home().x == 5.0


# ── receiver: error handling ──────────────────────────────────────


def test_receiver_logs_invalid_json_and_keeps_running():
    captured = []
    logger = AsyncLogger(
        sink=CallableSink(captured.append),
        max_queue=64,
        flush_interval_s=0.05,
    )
    logger.start()
    manager, ss, _ = _make_manager_and_acks()
    receiver, address = _start_receiver(manager, logger=logger)
    client = _client_socket()
    try:
        client.sendto(b"not valid json", address)
        # Now send a valid tracking message; the receiver must still process it.
        msg = TrackingMessage(
            sequence=99,
            timestamp_ms=0,
            car=TrackingCar(x=0.0, y=0.0, confidence=1.0),
            cat=TrackingCat(x=0.0, y=0.0, confidence=1.0),
        )
        client.sendto(json.dumps(msg.to_dict()).encode("utf-8"), address)
        assert _wait_until(lambda: ss.get_overhead().sequence == 99)
        assert _wait_until(
            lambda: any(
                e["event_type"] == TelemetryEventType.THREAD_HEALTH.value
                and e["data"].get("event") == "udp_packet_dropped"
                for e in captured
            )
        )
    finally:
        client.close()
        receiver.stop()
        logger.stop()


def test_receiver_survives_commit_timeout_and_keeps_receiving():
    """A stalled control loop must not take the ingress thread down."""

    captured = []
    logger = AsyncLogger(
        sink=CallableSink(captured.append),
        max_queue=32,
        flush_interval_s=0.05,
    )
    logger.start()
    manager, ss, _ = _make_manager_and_acks()

    def _timeout(_msg):
        raise TimeoutError("timed out waiting for ACK on command_id='cmd-stalled'")

    manager.submit_command = _timeout
    receiver, address = _start_receiver(manager, logger=logger)
    client = _client_socket()
    try:
        cmd = CommandMessage(
            sequence=2001,
            timestamp_ms=10,
            command_id="cmd-stalled",
            command=CommandName.STOP_CHASE,
            params={},
        )
        client.sendto(json.dumps(cmd.to_dict()).encode("utf-8"), address)
        assert _wait_until(
            lambda: any(
                e["data"].get("cause") == "commit_timeout" for e in captured
            )
        )

        # Tracking ingress still works after the timed-out command.
        tracking = TrackingMessage(
            sequence=9,
            timestamp_ms=1000,
            car=TrackingCar(x=1.0, y=2.0, confidence=1.0),
            cat=TrackingCat(x=3.0, y=4.0, confidence=1.0),
        )
        client.sendto(json.dumps(tracking.to_dict()).encode("utf-8"), address)
        assert _wait_until(lambda: ss.get_overhead().sequence == 9)
    finally:
        client.close()
        receiver.stop()
        logger.stop()


def test_tracking_ingress_continues_while_a_command_is_committing():
    """A slow command commit must not stall overhead tracking ingress."""

    manager, ss, _ = _make_manager_and_acks()
    release = threading.Event()
    entered = threading.Event()

    def _blocking(_msg):
        entered.set()
        release.wait(timeout=5.0)

    manager.submit_command = _blocking
    receiver, address = _start_receiver(manager)
    client = _client_socket()
    try:
        cmd = CommandMessage(
            sequence=2003,
            timestamp_ms=10,
            command_id="cmd-slow",
            command=CommandName.STOP_CHASE,
            params={},
        )
        client.sendto(json.dumps(cmd.to_dict()).encode("utf-8"), address)
        assert entered.wait(timeout=2.0)

        tracking = TrackingMessage(
            sequence=13,
            timestamp_ms=1000,
            car=TrackingCar(x=1.0, y=2.0, confidence=1.0),
            cat=TrackingCat(x=3.0, y=4.0, confidence=1.0),
        )
        client.sendto(json.dumps(tracking.to_dict()).encode("utf-8"), address)
        assert _wait_until(lambda: ss.get_overhead().sequence == 13)
    finally:
        release.set()
        client.close()
        receiver.stop()


def test_receiver_survives_unexpected_handler_error():
    captured = []
    logger = AsyncLogger(
        sink=CallableSink(captured.append),
        max_queue=32,
        flush_interval_s=0.05,
    )
    logger.start()
    manager, ss, _ = _make_manager_and_acks()

    def _boom(_msg):
        raise RuntimeError("unexpected commit failure")

    manager.submit_command = _boom
    receiver, address = _start_receiver(manager, logger=logger)
    client = _client_socket()
    try:
        cmd = CommandMessage(
            sequence=2002,
            timestamp_ms=10,
            command_id="cmd-boom",
            command=CommandName.STOP_CHASE,
            params={},
        )
        client.sendto(json.dumps(cmd.to_dict()).encode("utf-8"), address)
        assert _wait_until(
            lambda: any(
                e["data"].get("cause") == "handler_error" for e in captured
            )
        )

        tracking = TrackingMessage(
            sequence=11,
            timestamp_ms=1000,
            car=TrackingCar(x=1.0, y=2.0, confidence=1.0),
            cat=TrackingCat(x=3.0, y=4.0, confidence=1.0),
        )
        client.sendto(json.dumps(tracking.to_dict()).encode("utf-8"), address)
        assert _wait_until(lambda: ss.get_overhead().sequence == 11)
    finally:
        client.close()
        receiver.stop()
        logger.stop()


def test_receiver_logs_unsupported_message_type():
    captured = []
    logger = AsyncLogger(
        sink=CallableSink(captured.append),
        max_queue=32,
        flush_interval_s=0.05,
    )
    logger.start()
    manager, _, _ = _make_manager_and_acks()
    receiver, address = _start_receiver(manager, logger=logger)
    client = _client_socket()
    try:
        client.sendto(b'{"type": "telemetry"}', address)
        assert _wait_until(
            lambda: any(
                e["event_type"] == TelemetryEventType.THREAD_HEALTH.value
                and e["data"].get("cause") == "unsupported_message_type"
                for e in captured
            )
        )
    finally:
        client.close()
        receiver.stop()
        logger.stop()


def test_receiver_logs_schema_version_error():
    captured = []
    logger = AsyncLogger(
        sink=CallableSink(captured.append),
        max_queue=32,
        flush_interval_s=0.05,
    )
    logger.start()
    manager, _, _ = _make_manager_and_acks()
    receiver, address = _start_receiver(manager, logger=logger)
    client = _client_socket()
    try:
        msg_dict = TrackingMessage(
            sequence=1,
            timestamp_ms=0,
            car=TrackingCar(x=0.0, y=0.0),
            cat=TrackingCat(x=0.0, y=0.0),
        ).to_dict()
        msg_dict["schema_version"] = 99
        client.sendto(json.dumps(msg_dict).encode("utf-8"), address)
        assert _wait_until(
            lambda: any(
                e["data"].get("cause") == "schema_version_error"
                for e in captured
                if e["event_type"] == TelemetryEventType.THREAD_HEALTH.value
            )
        )
    finally:
        client.close()
        receiver.stop()
        logger.stop()


# ── receiver: command auth ────────────────────────────────────────


def _start_receiver_with_token(manager, token, *, logger=None):
    receiver = UdpReceiver(
        comms_manager=manager,
        bind_host="127.0.0.1",
        bind_port=0,
        logger=logger,
        recv_timeout_s=0.05,
        command_token=token,
    )
    receiver.start()
    address = receiver.bound_address
    assert address is not None
    return receiver, address


def test_command_rejected_without_token_when_configured():
    captured = []
    logger = AsyncLogger(
        sink=CallableSink(captured.append),
        max_queue=32,
        flush_interval_s=0.05,
    )
    logger.start()
    manager, ss, acks = _make_manager_and_acks()
    receiver, address = _start_receiver_with_token(
        manager, "s3cret", logger=logger
    )
    client = _client_socket()
    try:
        cmd = CommandMessage(
            sequence=3001,
            timestamp_ms=0,
            command_id="cmd-unauth",
            command=CommandName.SET_HOME,
            params={"home": {"x": 1.0, "y": 1.0, "frame_id": "yard"}},
        )
        # No token attached -> must be dropped.
        client.sendto(json.dumps(cmd.to_dict()).encode("utf-8"), address)
        assert _wait_until(
            lambda: any(
                e["data"].get("cause") == "unauthorized_command"
                for e in captured
                if e["event_type"] == TelemetryEventType.THREAD_HEALTH.value
            )
        )
        # The command must not have been applied.
        assert ss.get_home().set is False
        assert not acks
    finally:
        client.close()
        receiver.stop()
        logger.stop()


def test_command_rejected_with_wrong_token():
    manager, ss, acks = _make_manager_and_acks()
    receiver, address = _start_receiver_with_token(manager, "s3cret")
    client = _client_socket()
    try:
        payload = CommandMessage(
            sequence=3002,
            timestamp_ms=0,
            command_id="cmd-wrong",
            command=CommandName.SET_HOME,
            params={"home": {"x": 1.0, "y": 1.0, "frame_id": "yard"}},
        ).to_dict()
        payload["token"] = "wrong"
        client.sendto(json.dumps(payload).encode("utf-8"), address)
        time.sleep(0.3)
        assert ss.get_home().set is False
        assert not acks
    finally:
        client.close()
        receiver.stop()


def test_command_accepted_with_correct_token():
    manager, ss, acks = _make_manager_and_acks()
    receiver, address = _start_receiver_with_token(manager, "s3cret")
    client = _client_socket()
    try:
        payload = CommandMessage(
            sequence=3003,
            timestamp_ms=0,
            command_id="cmd-auth",
            command=CommandName.SET_HOME,
            params={"home": {"x": 7.0, "y": 8.0, "frame_id": "yard"}},
        ).to_dict()
        payload["token"] = "s3cret"
        client.sendto(json.dumps(payload).encode("utf-8"), address)
        assert _wait_until(lambda: ss.get_home().set is True)
        assert acks[-1].status == AckStatus.ACCEPTED
        assert ss.get_home().x == 7.0
    finally:
        client.close()
        receiver.stop()


def test_tracking_not_gated_by_command_token():
    manager, ss, _ = _make_manager_and_acks()
    receiver, address = _start_receiver_with_token(manager, "s3cret")
    client = _client_socket()
    try:
        msg = TrackingMessage(
            sequence=7,
            timestamp_ms=0,
            car=TrackingCar(x=1.0, y=2.0, confidence=1.0),
            cat=TrackingCat(x=3.0, y=4.0, confidence=1.0),
        )
        client.sendto(json.dumps(msg.to_dict()).encode("utf-8"), address)
        assert _wait_until(lambda: ss.get_overhead().sequence == 7)
    finally:
        client.close()
        receiver.stop()


def test_command_auth_disabled_logs_warning():
    captured = []
    logger = AsyncLogger(
        sink=CallableSink(captured.append),
        max_queue=32,
        flush_interval_s=0.05,
    )
    logger.start()
    manager, _, _ = _make_manager_and_acks()
    # Explicit empty string -> auth disabled regardless of environment.
    receiver, _address = _start_receiver_with_token(manager, "", logger=logger)
    try:
        assert _wait_until(
            lambda: any(
                e["data"].get("event") == "udp_command_auth_disabled"
                for e in captured
                if e["event_type"] == TelemetryEventType.THREAD_HEALTH.value
            )
        )
    finally:
        receiver.stop()
        logger.stop()


# ── sender ────────────────────────────────────────────────────────


def test_sender_serializes_ack_to_json_over_udp():
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", 0))
    server.settimeout(2.0)
    host, port = server.getsockname()
    sender = UdpSender(target_host=host, target_port=port)
    try:
        ack = AckMessage(
            sequence=9001,
            timestamp_ms=5,
            ack_sequence=2002,
            ack_type=AckType.COMMAND,
            command_id="cmd-1",
            status=AckStatus.ACCEPTED,
            state=FsmState.IDLE,
            reason=ReasonCode.STOP_CHASE_ACCEPTED,
            cause=None,
        )
        sender(ack)  # use the callable form
        data, _addr = server.recvfrom(65535)
    finally:
        sender.close()
        server.close()

    payload = json.loads(data.decode("utf-8"))
    assert payload["type"] == "ack"
    assert payload["status"] == "accepted"
    assert payload["command_id"] == "cmd-1"
    assert payload["cause"] is None


def test_sender_socket_error_is_logged_and_swallowed():
    captured = []
    logger = AsyncLogger(
        sink=CallableSink(captured.append),
        max_queue=16,
        flush_interval_s=0.05,
    )
    logger.start()
    sender = UdpSender(target_host="127.0.0.1", target_port=1, logger=logger)
    try:
        # Close the socket so any subsequent send raises OSError, and verify
        # the sender logs rather than crashes.
        sender.close()
        ack = AckMessage(
            sequence=1,
            timestamp_ms=0,
            ack_sequence=1,
            ack_type=AckType.COMMAND,
            command_id="cmd-bad",
            status=AckStatus.ACCEPTED,
            state=FsmState.IDLE,
            reason=ReasonCode.STOP_CHASE_ACCEPTED,
            cause=None,
        )
        sender.send(ack)  # must not raise
        assert _wait_until(
            lambda: any(
                e["data"].get("event") == "udp_send_failed"
                for e in captured
                if e["event_type"] == TelemetryEventType.THREAD_HEALTH.value
            )
        )
    finally:
        logger.stop()


# ── round trip: receiver -> CommsManager -> sender ─────────────────


def test_full_round_trip_command_to_ack_via_udp():
    # Bind a server socket to receive the outgoing ACK.
    ack_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ack_server.bind(("127.0.0.1", 0))
    ack_server.settimeout(2.0)
    ack_host, ack_port = ack_server.getsockname()
    sender = UdpSender(target_host=ack_host, target_port=ack_port)

    ss = SharedState()
    manager = CommsManager(shared_state=ss, ack_sink=sender)
    receiver, recv_address = _start_receiver(manager)
    client = _client_socket()
    try:
        cmd = CommandMessage(
            sequence=2002,
            timestamp_ms=0,
            command_id="cmd-roundtrip",
            command=CommandName.STOP_CHASE,
        )
        client.sendto(json.dumps(cmd.to_dict()).encode("utf-8"), recv_address)
        data, _addr = ack_server.recvfrom(65535)
    finally:
        client.close()
        receiver.stop()
        sender.close()
        ack_server.close()

    payload = json.loads(data.decode("utf-8"))
    assert payload["type"] == "ack"
    assert payload["command_id"] == "cmd-roundtrip"
    assert payload["status"] == "accepted"
    assert payload["ack_sequence"] == cmd.sequence
