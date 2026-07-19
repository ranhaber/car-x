"""Tests for the comms wire-form dataclasses."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.comms.messages import (  # noqa: E402
    AckMessage,
    CommandMessage,
    SchemaVersionError,
    TrackingCar,
    TrackingCat,
    TrackingMessage,
)
from cat_follow.control.types import (  # noqa: E402
    AckStatus,
    AckType,
    CommandName,
    FsmState,
    ReasonCode,
    RejectionCause,
)


# ── tracking ────────────────────────────────────────────────────────


def test_tracking_message_round_trip():
    msg = TrackingMessage(
        sequence=42,
        timestamp_ms=12345,
        car=TrackingCar(x=1.0, y=2.0, heading=0.5, heading_valid=True, confidence=1.0),
        cat=TrackingCat(x=10.0, y=20.0, confidence=1.0),
    )
    payload = msg.to_dict()
    assert payload["type"] == "tracking"
    assert payload["frame_id"] == "yard"
    restored = TrackingMessage.from_dict(payload)
    assert restored == msg


def test_tracking_from_dict_rejects_wrong_type():
    payload = TrackingMessage(
        sequence=1,
        timestamp_ms=0,
        car=TrackingCar(x=0.0, y=0.0),
        cat=TrackingCat(x=0.0, y=0.0),
    ).to_dict()
    payload["type"] = "command"
    with pytest.raises(ValueError):
        TrackingMessage.from_dict(payload)


def test_unsupported_schema_version_raises():
    payload = TrackingMessage(
        sequence=1,
        timestamp_ms=0,
        car=TrackingCar(x=0.0, y=0.0),
        cat=TrackingCat(x=0.0, y=0.0),
    ).to_dict()
    payload["schema_version"] = 99
    with pytest.raises(SchemaVersionError):
        TrackingMessage.from_dict(payload)


# ── command ────────────────────────────────────────────────────────


def test_command_message_round_trip():
    msg = CommandMessage(
        sequence=2002,
        timestamp_ms=10,
        command_id="cmd-0001",
        command=CommandName.START_CHASE,
        params={},
    )
    payload = msg.to_dict()
    assert payload["type"] == "command"
    assert payload["command"] == "start_chase"
    restored = CommandMessage.from_dict(payload)
    assert restored == msg


def test_command_message_with_params_round_trip():
    msg = CommandMessage(
        sequence=2003,
        timestamp_ms=20,
        command_id="cmd-set-home-1",
        command=CommandName.SET_HOME,
        params={"home": {"x": 100.0, "y": 200.0, "frame_id": "yard"}},
    )
    restored = CommandMessage.from_dict(msg.to_dict())
    assert restored == msg


# ── ack ────────────────────────────────────────────────────────────


def test_ack_message_round_trip_accepted():
    ack = AckMessage(
        sequence=9001,
        timestamp_ms=11,
        ack_sequence=2002,
        ack_type=AckType.COMMAND,
        command_id="cmd-0001",
        status=AckStatus.ACCEPTED,
        state=FsmState.CHASE_A,
        reason=ReasonCode.START_CHASE_ACCEPTED,
        cause=None,
    )
    payload = ack.to_dict()
    assert payload["status"] == "accepted"
    assert payload["cause"] is None
    restored = AckMessage.from_dict(payload)
    assert restored == ack


def test_ack_message_round_trip_rejected():
    ack = AckMessage(
        sequence=9002,
        timestamp_ms=12,
        ack_sequence=2003,
        ack_type=AckType.COMMAND,
        command_id="cmd-0002",
        status=AckStatus.REJECTED,
        state=FsmState.IDLE,
        reason=ReasonCode.START_CHASE_REJECTED,
        cause=RejectionCause.CAT_POSITION_INVALID,
    )
    payload = ack.to_dict()
    assert payload["status"] == "rejected"
    assert payload["cause"] == "cat_position_invalid"
    restored = AckMessage.from_dict(payload)
    assert restored == ack
