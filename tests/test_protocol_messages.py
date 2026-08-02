"""Tests for Protocol V1 wire schemas."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.comms.messages import (  # noqa: E402
    AckMessage,
    CommandMessage,
    MissionEventMessage,
    TrackingCar,
    TrackingCat,
    TrackingMessage,
)
from cat_follow.control.types import (  # noqa: E402
    AckStatus,
    AckType,
    CommandName,
    FsmState,
    MissionEventName,
    ReasonCode,
)


def test_overhead_observation_round_trip():
    msg = TrackingMessage(
        sequence=42,
        timestamp_ms=12345,
        perimeter_id="yard-v3",
        calibration_version=7,
        selected_target_id="cat-17",
        car=TrackingCar(x=1.0, y=2.0, heading=0.5, confidence=0.96),
        cat=TrackingCat(
            x=10.0,
            y=20.0,
            confidence=0.93,
            target_id="cat-17",
            inside_perimeter=True,
        ),
    )
    payload = msg.to_dict()
    payload["type"] = "overhead_observation"
    payload["observation_seq"] = payload.pop("sequence")
    payload["observed_at_ms"] = payload.pop("timestamp_ms")

    restored = TrackingMessage.from_dict(payload)

    assert restored.selected_target_id == "cat-17"
    assert restored.cat.target_id == "cat-17"
    assert restored.perimeter_id == "yard-v3"


def test_command_start_chase_carries_target_id():
    msg = CommandMessage(
        sequence=1,
        timestamp_ms=10,
        command_id="cmd-1",
        command=CommandName.START_CHASE,
        params={"target_id": "cat-17"},
    )
    payload = msg.to_dict()

    assert payload["args"]["target_id"] == "cat-17"
    assert payload["name"] == "START_CHASE"
    restored = CommandMessage.from_dict(payload)
    assert restored.target_id == "cat-17"


def test_mission_event_round_trip():
    msg = MissionEventMessage(
        event_id="evt-31bd",
        mission_id="mission-204",
        timestamp_ms=1000,
        name=MissionEventName.PRIMARY_CAT_LEFT_PERIMETER,
        target_id="cat-17",
        perimeter_id="yard-v3",
        observation_sequence=1901,
    )
    payload = msg.to_dict()
    restored = MissionEventMessage.from_dict(payload)
    assert restored == msg


def test_ack_v1_fields_round_trip():
    ack = AckMessage(
        sequence=1,
        timestamp_ms=2,
        ack_sequence=3,
        ack_type=AckType.MISSION_EVENT,
        command_id="evt-1",
        status=AckStatus.ACCEPTED,
        state=FsmState.IDLE,
        reason=ReasonCode.PRIMARY_TARGET_EXIT_HANDOFF,
        applied_control_sequence=42,
    )
    payload = ack.to_dict()
    assert payload["applied"] is True
    assert payload["resulting_state"] == "IDLE"
    assert payload["applied_control_seq"] == 42
    restored = AckMessage.from_dict(payload)
    assert restored.applied_control_sequence == 42
