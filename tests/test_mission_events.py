"""Tests for mission-event transactional handling."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.control.types import (  # noqa: E402
    AckStatus,
    FsmState,
    ReasonCode,
    RejectionCause,
)
from tests.test_comms_manager_helpers import (  # noqa: E402
    DEFAULT_TARGET_ID,
    command_message,
    drive_into_brake_reverse,
    make_manager,
    mission_event_message,
    start_chase_command,
    tracking_message,
)
from cat_follow.control.types import CommandName  # noqa: E402


def test_matching_primary_left_event_enters_idle():
    manager, ss, acks, fsm, _ = make_manager()
    manager.submit_tracking(tracking_message(sequence=10))
    manager.submit_command(start_chase_command(command_id="cmd-start"))
    assert fsm.state == FsmState.GETTING_CLOSE

    ack = manager.submit_mission_event(
        mission_event_message(
            event_id="evt-1",
            observation_sequence=10,
            target_id=DEFAULT_TARGET_ID,
        )
    )

    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.PRIMARY_TARGET_EXIT_HANDOFF
    assert ack.state == FsmState.IDLE
    assert fsm.state == FsmState.IDLE
    mission = ss.get_mission()
    assert mission.active_target_id is None
    assert mission.blocked_target_id == DEFAULT_TARGET_ID
    assert mission.blocked_through_observation_seq == 10
    assert mission.handoff_deadline_ms is not None


def test_handoff_rejects_restart_from_stale_exit_observation():
    manager, _, _, _, _ = make_manager()
    manager.submit_tracking(tracking_message(sequence=10))
    manager.submit_command(start_chase_command(command_id="cmd-start"))
    manager.submit_mission_event(
        mission_event_message(event_id="evt-exit", observation_sequence=10)
    )

    ack = manager.submit_command(
        start_chase_command(command_id="cmd-stale-restart")
    )

    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.STALE_OBSERVATION


def test_wrong_target_event_rejected():
    manager, _, _, fsm, _ = make_manager()
    manager.submit_tracking(tracking_message(sequence=10))
    manager.submit_command(start_chase_command(command_id="cmd-start"))

    ack = manager.submit_mission_event(
        mission_event_message(
            event_id="evt-wrong",
            target_id="cat-other",
            observation_sequence=10,
        )
    )

    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.WRONG_TARGET
    assert fsm.state == FsmState.GETTING_CLOSE


def test_duplicate_event_id_returns_cached_result():
    manager, _, acks, _, _ = make_manager()
    manager.submit_tracking(tracking_message(sequence=10))
    manager.submit_command(start_chase_command(command_id="cmd-start"))

    first = manager.submit_mission_event(
        mission_event_message(event_id="evt-dup", observation_sequence=10)
    )
    second = manager.submit_mission_event(
        mission_event_message(event_id="evt-dup", observation_sequence=10)
    )

    assert second.applied_control_sequence == first.applied_control_sequence
    assert second.state == first.state
    assert len(acks) == 3


def test_primary_left_from_brake_reverse_accepted_when_saved_objective_is_chase():
    manager, _, _, fsm, engine = make_manager()
    manager.submit_tracking(tracking_message(sequence=10))
    manager.submit_command(start_chase_command(command_id="cmd-start"))
    drive_into_brake_reverse(engine)
    assert fsm.state == FsmState.BRAKE_REVERSE
    assert engine.brake_saved_state == FsmState.GETTING_CLOSE

    ack = manager.submit_mission_event(
        mission_event_message(event_id="evt-exit", observation_sequence=10)
    )

    assert ack.status == AckStatus.ACCEPTED
    assert fsm.state == FsmState.IDLE
    # The interrupted chase no longer exists, so nothing may resume it.
    assert engine.brake_reverse_phase is None
    assert engine.brake_saved_state is None


def test_primary_left_from_brake_reverse_rejected_when_saved_objective_is_not_chase():
    manager, _, _, fsm, engine = make_manager()
    manager.submit_tracking(tracking_message(sequence=10))
    manager.submit_command(start_chase_command(command_id="cmd-start"))
    manager.submit_command(
        command_message(CommandName.RETURN_HOME, sequence=2100, params={})
    )
    assert fsm.state == FsmState.RETURN_HOME
    drive_into_brake_reverse(engine)
    assert fsm.state == FsmState.BRAKE_REVERSE
    assert engine.brake_saved_state == FsmState.RETURN_HOME

    ack = manager.submit_mission_event(
        mission_event_message(event_id="evt-exit", observation_sequence=10)
    )

    # Transition matrix 9.2: the exit only applies to a BRAKE_REVERSE that
    # interrupted a chase.
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.INVALID_STATE
    assert fsm.state == FsmState.BRAKE_REVERSE
    assert engine.brake_saved_state == FsmState.RETURN_HOME
