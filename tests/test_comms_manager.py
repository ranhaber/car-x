"""Tests for the in-process CommsManager."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.comms.comms_manager import CommsManager  # noqa: E402
from cat_follow.comms.messages import (  # noqa: E402
    CommandMessage,
    TrackingCar,
    TrackingCat,
    TrackingMessage,
)
from cat_follow.control.types import (  # noqa: E402
    AckStatus,
    CommandName,
    FsmState,
    ReasonCode,
    RejectionCause,
)
from cat_follow.runtime.shared_state import SharedState  # noqa: E402


def _make_manager():
    ss = SharedState()
    received = []
    manager = CommsManager(shared_state=ss, ack_sink=received.append)
    return manager, ss, received


def _tracking(sequence=1, *, car_conf=1.0, cat_conf=1.0):
    return TrackingMessage(
        sequence=sequence,
        timestamp_ms=sequence * 100,
        car=TrackingCar(x=10.0, y=20.0, heading=0.0, heading_valid=False, confidence=car_conf),
        cat=TrackingCat(x=30.0, y=40.0, confidence=cat_conf),
    )


def _command(name, *, sequence=2001, command_id=None, params=None):
    return CommandMessage(
        sequence=sequence,
        timestamp_ms=sequence * 10,
        command_id=command_id or f"cmd-{sequence}",
        command=name,
        params=params or {},
    )


# ── tracking ────────────────────────────────────────────────────────


def test_tracking_updates_overhead_state():
    manager, ss, _ = _make_manager()

    accepted = manager.submit_tracking(_tracking(sequence=10))
    assert accepted is True

    overhead = ss.get_overhead()
    assert overhead.sequence == 10
    assert overhead.car.x == 10.0
    assert overhead.cat.y == 40.0
    assert overhead.car.confidence == 1.0
    assert overhead.received_ms > 0
    assert overhead.fresh is True


def test_tracking_drops_duplicates_and_out_of_order():
    manager, ss, _ = _make_manager()
    manager.submit_tracking(_tracking(sequence=5))
    assert manager.submit_tracking(_tracking(sequence=5)) is False
    assert manager.submit_tracking(_tracking(sequence=4)) is False
    overhead = ss.get_overhead()
    assert overhead.sequence == 5  # still the latest accepted


# ── set_home ────────────────────────────────────────────────────────


def test_set_home_accepted_updates_home_state():
    manager, ss, acks = _make_manager()
    cmd = _command(
        CommandName.SET_HOME,
        params={"home": {"x": 100.0, "y": 200.0, "frame_id": "yard"}},
    )
    ack = manager.submit_command(cmd)

    assert ack.status == AckStatus.ACCEPTED
    assert ack.cause is None
    assert ack.command_id == cmd.command_id
    home = ss.get_home()
    assert home.set is True
    assert home.x == 100.0
    assert home.y == 200.0
    assert home.source_command_id == cmd.command_id
    assert acks == [ack]


def test_set_home_rejects_missing_payload():
    manager, ss, _ = _make_manager()
    ack = manager.submit_command(_command(CommandName.SET_HOME, params={}))

    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.HOME_MISSING
    assert ss.get_home().set is False


def test_set_home_rejects_invalid_frame():
    manager, ss, _ = _make_manager()
    ack = manager.submit_command(
        _command(
            CommandName.SET_HOME,
            params={"home": {"x": 0.0, "y": 0.0, "frame_id": "wrong"}},
        )
    )
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.HOME_INVALID
    assert ss.get_home().set is False


# ── start_chase ─────────────────────────────────────────────────────


def test_start_chase_accepted_when_tracking_valid():
    manager, _, _ = _make_manager()
    manager.submit_tracking(_tracking(sequence=1))

    ack = manager.submit_command(_command(CommandName.START_CHASE))
    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.START_CHASE_ACCEPTED


def test_start_chase_rejected_when_cat_position_invalid():
    manager, _, _ = _make_manager()
    manager.submit_tracking(_tracking(sequence=1, cat_conf=0.0))

    ack = manager.submit_command(_command(CommandName.START_CHASE))
    assert ack.status == AckStatus.REJECTED
    assert ack.reason == ReasonCode.START_CHASE_REJECTED
    assert ack.cause == RejectionCause.CAT_POSITION_INVALID


def test_start_chase_rejected_when_no_tracking_yet():
    manager, _, _ = _make_manager()
    ack = manager.submit_command(_command(CommandName.START_CHASE))
    assert ack.status == AckStatus.REJECTED
    # With default zero-confidence overhead, the car-position check fires first.
    assert ack.cause == RejectionCause.CAR_POSITION_INVALID


# ── stop_chase / return_home ───────────────────────────────────────


def test_stop_chase_accepted_in_normal_state():
    manager, _, _ = _make_manager()
    ack = manager.submit_command(_command(CommandName.STOP_CHASE))
    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.STOP_CHASE_ACCEPTED


def test_return_home_accepted_with_home():
    manager, ss, _ = _make_manager()
    ack = manager.submit_command(
        _command(
            CommandName.RETURN_HOME,
            params={"home": {"x": 5.0, "y": 6.0, "frame_id": "yard"}},
        )
    )
    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.RETURN_HOME_ACCEPTED
    home = ss.get_home()
    assert home.x == 5.0
    assert home.set is True


def test_return_home_rejected_without_home_payload():
    manager, _, _ = _make_manager()
    ack = manager.submit_command(_command(CommandName.RETURN_HOME, params={}))
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.HOME_MISSING


# ── go_to ──────────────────────────────────────────────────────────


def test_go_to_accepted_with_target():
    manager, _, _ = _make_manager()
    ack = manager.submit_command(
        _command(
            CommandName.GO_TO,
            params={"target": {"x": 50.0, "y": 60.0, "frame_id": "yard"}},
        )
    )
    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.GO_TO_ACCEPTED


def test_go_to_rejected_without_target():
    manager, _, _ = _make_manager()
    ack = manager.submit_command(_command(CommandName.GO_TO, params={}))
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.TARGET_INVALID


# ── clear_failsafe ─────────────────────────────────────────────────


def test_clear_failsafe_rejected_without_confirmation():
    manager, _, _ = _make_manager()
    ack = manager.submit_command(
        _command(CommandName.CLEAR_FAILSAFE, params={"operator_confirmed": False})
    )
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.OPERATOR_CONFIRMATION_REQUIRED


def test_clear_failsafe_accepted_with_confirmation():
    manager, _, _ = _make_manager()
    ack = manager.submit_command(
        _command(CommandName.CLEAR_FAILSAFE, params={"operator_confirmed": True})
    )
    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.CLEAR_FAILSAFE_ACCEPTED


# ── shared state command group ─────────────────────────────────────


def test_command_state_is_published_on_accept():
    manager, ss, _ = _make_manager()
    cmd = _command(
        CommandName.SET_HOME,
        params={"home": {"x": 1.0, "y": 2.0, "frame_id": "yard"}},
    )
    manager.submit_command(cmd)

    cs = ss.get_command()
    assert cs.last_command_id == cmd.command_id
    assert cs.last_command == CommandName.SET_HOME
    assert cs.last_status == AckStatus.ACCEPTED
    assert cs.last_cause is None


def test_command_state_is_published_on_reject():
    manager, ss, _ = _make_manager()
    cmd = _command(CommandName.SET_HOME, params={})
    manager.submit_command(cmd)

    cs = ss.get_command()
    assert cs.last_command_id == cmd.command_id
    assert cs.last_status == AckStatus.REJECTED
    assert cs.last_cause == RejectionCause.HOME_MISSING


# ── idempotency ────────────────────────────────────────────────────


def test_duplicate_command_returns_cached_result_with_new_ack_sequence():
    manager, ss, acks = _make_manager()
    cmd = _command(
        CommandName.SET_HOME,
        params={"home": {"x": 1.0, "y": 2.0, "frame_id": "yard"}},
    )
    first_ack = manager.submit_command(cmd)

    # Mutate home behind the manager's back to verify retry does NOT touch it.
    home_before_retry = ss.get_home()

    retry_msg = CommandMessage(
        sequence=cmd.sequence + 1,
        timestamp_ms=cmd.timestamp_ms + 1,
        command_id=cmd.command_id,
        command=cmd.command,
        params=cmd.params,
    )
    second_ack = manager.submit_command(retry_msg)

    # Same logical result, new ack envelope.
    assert second_ack.status == first_ack.status
    assert second_ack.command_id == first_ack.command_id
    assert second_ack.ack_sequence == retry_msg.sequence
    assert second_ack.sequence != first_ack.sequence
    assert acks == [first_ack, second_ack]
    # Home was NOT re-applied (no source_command_id refresh, same instance).
    assert ss.get_home() is home_before_retry


def test_command_id_cache_size_is_bounded():
    ss = SharedState()
    manager = CommsManager(
        shared_state=ss, ack_sink=lambda ack: None, command_id_cache_size=3
    )

    for i in range(5):
        manager.submit_command(
            _command(CommandName.STOP_CHASE, sequence=2000 + i, command_id=f"cmd-{i}")
        )

    assert manager.cached_command_ids() == 3
