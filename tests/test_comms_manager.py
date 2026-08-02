"""Tests for the in-process CommsManager."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.comms.comms_manager import CommsManager  # noqa: E402
from cat_follow.comms.messages import CommandMessage  # noqa: E402
from cat_follow.control.types import (  # noqa: E402
    AckStatus,
    AckType,
    CommandName,
    FsmState,
    ReasonCode,
    RejectionCause,
)
from tests.test_comms_manager_helpers import (  # noqa: E402
    DEFAULT_TARGET_ID,
    command_message as _command,
    drive_into_brake_reverse,
    make_manager as _make_manager,
    mission_event_message,
    start_chase_command,
    tracking_message as _tracking,
)


# ── tracking ────────────────────────────────────────────────────────


def test_tracking_updates_overhead_state():
    manager, ss, _, _, _ = _make_manager()

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
    manager, ss, _, _, _ = _make_manager()
    manager.submit_tracking(_tracking(sequence=5))
    assert manager.submit_tracking(_tracking(sequence=5)) is False
    assert manager.submit_tracking(_tracking(sequence=4)) is False
    overhead = ss.get_overhead()
    assert overhead.sequence == 5  # still the latest accepted


# ── set_home ────────────────────────────────────────────────────────


def test_set_home_accepted_updates_home_state():
    manager, ss, acks, _, _ = _make_manager()
    cmd = _command(
        CommandName.SET_HOME,
        params={"home": {"x_cm": 100.0, "y_cm": 200.0, "frame_id": "yard"}},
    )
    ack = manager.submit_command(cmd)

    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.SET_HOME_ACCEPTED
    assert ack.cause is None
    assert ack.command_id == cmd.command_id
    home = ss.get_home()
    assert home.set is True
    assert home.valid is True
    assert home.x == 100.0
    assert home.y == 200.0
    assert home.home_version == 1
    assert home.source_command_id == cmd.command_id
    assert acks == [ack]


def test_set_home_rejects_missing_payload():
    manager, ss, _, _, _ = _make_manager(home=None)
    ack = manager.submit_command(_command(CommandName.SET_HOME, params={}))

    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.HOME_MISSING
    assert ss.get_home().set is False


def test_set_home_rejects_invalid_frame():
    manager, ss, _, _, _ = _make_manager(home=None)
    ack = manager.submit_command(
        _command(
            CommandName.SET_HOME,
            params={"home": {"x_cm": 0.0, "y_cm": 0.0, "frame_id": "wrong"}},
        )
    )
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.HOME_INVALID
    assert ss.get_home().set is False


# ── start_chase ─────────────────────────────────────────────────────


def test_start_chase_accepted_when_tracking_valid():
    manager, _, _, fsm, _ = _make_manager()
    manager.submit_tracking(_tracking(sequence=1))

    ack = manager.submit_command(start_chase_command())
    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.START_CHASE_ACCEPTED
    assert ack.state == FsmState.GETTING_CLOSE
    assert ack.applied_control_sequence == 1
    assert fsm.state == FsmState.GETTING_CLOSE


def test_start_chase_rejected_without_durable_home():
    manager, _, _, _, _ = _make_manager(home=None)
    manager.submit_tracking(_tracking(sequence=1))

    ack = manager.submit_command(start_chase_command())
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.HOME_MISSING


def test_start_chase_rejected_when_required_sensor_unhealthy():
    manager, _, _, _, _ = _make_manager(sensors_healthy=False)
    manager.submit_tracking(_tracking(sequence=1))

    ack = manager.submit_command(start_chase_command())

    assert ack.status == AckStatus.REJECTED
    assert ack.reason == ReasonCode.START_CHASE_REJECTED
    assert ack.cause == RejectionCause.MOTION_UNSAFE


def test_start_chase_rejected_when_cat_position_invalid():
    manager, _, _, _, _ = _make_manager()
    manager.submit_tracking(_tracking(sequence=1, cat_conf=0.0))

    ack = manager.submit_command(start_chase_command())
    assert ack.status == AckStatus.REJECTED
    assert ack.reason == ReasonCode.START_CHASE_REJECTED
    assert ack.cause == RejectionCause.CAT_POSITION_INVALID


def test_start_chase_rejected_when_no_tracking_yet():
    manager, _, _, _, _ = _make_manager()
    ack = manager.submit_command(start_chase_command())
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.CAR_POSITION_INVALID


def test_start_chase_rejects_missing_target_id():
    manager, _, _, _, _ = _make_manager()
    manager.submit_tracking(_tracking(sequence=1))

    ack = manager.submit_command(_command(CommandName.START_CHASE))

    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.TARGET_INVALID


def test_stop_chase_wrong_target_rejected():
    manager, _, _, fsm, _ = _make_manager()
    manager.submit_tracking(_tracking(sequence=1))
    manager.submit_command(start_chase_command(command_id="cmd-start"))

    ack = manager.submit_command(
        _command(
            CommandName.STOP_CHASE,
            command_id="cmd-stop",
            params={"target_id": "cat-other"},
        )
    )

    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.WRONG_TARGET
    assert fsm.state == FsmState.GETTING_CLOSE


# ── stop_chase / return_home ───────────────────────────────────────


def test_stop_chase_accepted_in_normal_state():
    manager, _, _, _, _ = _make_manager()
    ack = manager.submit_command(_command(CommandName.STOP_CHASE))
    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.STOP_CHASE_ACCEPTED


def test_return_home_accepted_with_durable_home():
    from tests.test_comms_manager_helpers import durable_home

    manager, ss, _, fsm, _ = _make_manager(home=durable_home(x=5.0, y=6.0))
    ack = manager.submit_command(_command(CommandName.RETURN_HOME, params={}))
    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.RETURN_HOME_ACCEPTED
    assert fsm.state == FsmState.RETURN_HOME
    home = ss.get_home()
    assert home.x == 5.0
    assert home.set is True


def test_return_home_without_home_enters_failsafe():
    manager, _, _, fsm, _ = _make_manager(home=None)
    ack = manager.submit_command(_command(CommandName.RETURN_HOME, params={}))
    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.FAILSAFE_TRIGGERED
    assert fsm.state == FsmState.FAILSAFE


# ── brake reverse context teardown ─────────────────────────────────


def _brake_reverse_manager():
    """Manager whose engine sits mid-reverse with one attempt already spent."""

    manager, ss, acks, fsm, engine = _make_manager()
    manager.submit_tracking(_tracking(sequence=1))
    manager.submit_command(start_chase_command(command_id="cmd-start"))
    for now_ms in (1000, 1020, 1120):
        drive_into_brake_reverse(engine, now_ms=now_ms)
    assert fsm.state == FsmState.BRAKE_REVERSE
    assert engine.brake_reverse_phase is not None
    assert engine.brake_reverse_attempts == 1
    return manager, ss, acks, fsm, engine


@pytest.mark.parametrize(
    "command,params,expected_state",
    [
        (CommandName.STOP_CHASE, {}, FsmState.IDLE),
        (CommandName.RETURN_HOME, {}, FsmState.RETURN_HOME),
        (CommandName.EMERGENCY_STOP, {}, FsmState.FAILSAFE),
    ],
)
def test_command_leaving_brake_reverse_clears_engine_brake_context(
    command, params, expected_state
):
    manager, _, _, fsm, engine = _brake_reverse_manager()

    ack = manager.submit_command(
        _command(command, sequence=2100, params=params)
    )

    assert ack.status == AckStatus.ACCEPTED
    assert fsm.state == expected_state
    # The phase is only meaningful inside BRAKE_REVERSE; leaving it stale would
    # let the next control tick reverse or resume a cancelled objective.
    assert engine.brake_reverse_phase is None
    assert engine.brake_saved_state is None
    # Ending the objective also returns a full attempt budget to the next one.
    assert engine.brake_reverse_attempts == 0


# ── go_to ──────────────────────────────────────────────────────────


def test_go_to_accepted_with_target():
    manager, _, _, _, _ = _make_manager()
    ack = manager.submit_command(
        _command(
            CommandName.GO_TO,
            params={"target": {"x_cm": 50.0, "y_cm": 60.0, "frame_id": "yard"}},
        )
    )
    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.GO_TO_ACCEPTED


def test_go_to_rejected_without_target():
    manager, _, _, _, _ = _make_manager()
    ack = manager.submit_command(_command(CommandName.GO_TO, params={}))
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.TARGET_INVALID


def test_go_to_publishes_objective_in_centimeters():
    manager, ss, _, _, _ = _make_manager()
    manager.submit_command(
        _command(
            CommandName.GO_TO,
            params={"target": {"x_cm": 250.0, "y_cm": -80.0, "frame_id": "yard"}},
        )
    )
    cs = ss.get_command()
    assert cs.objective_x_cm == 250.0
    assert cs.objective_y_cm == -80.0
    assert cs.objective_frame_id == "yard"


def test_go_to_rejected_with_non_finite_target():
    manager, _, _, _, _ = _make_manager()
    ack = manager.submit_command(
        _command(
            CommandName.GO_TO,
            params={
                "target": {"x_cm": float("nan"), "y_cm": 0.0, "frame_id": "yard"}
            },
        )
    )
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.TARGET_INVALID


def test_go_to_rejected_outside_home_or_idle():
    manager, _, _, fsm, _ = _make_manager()
    manager.submit_tracking(_tracking(sequence=1))
    manager.submit_command(start_chase_command())
    assert fsm.state == FsmState.GETTING_CLOSE

    ack = manager.submit_command(
        _command(
            CommandName.GO_TO,
            sequence=2050,
            params={"target": {"x_cm": 50.0, "y_cm": 60.0, "frame_id": "yard"}},
        )
    )
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.INVALID_STATE
    assert fsm.state == FsmState.GETTING_CLOSE


# ── clear_failsafe ─────────────────────────────────────────────────


def test_clear_failsafe_rejected_without_confirmation():
    manager, _, _, _, _ = _make_manager()
    ack = manager.submit_command(
        _command(CommandName.CLEAR_FAILSAFE, params={"operator_confirmed": False})
    )
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.OPERATOR_CONFIRMATION_REQUIRED


def test_clear_failsafe_accepted_with_confirmation():
    manager, _, _, fsm, _ = _make_manager()
    manager.submit_command(_command(CommandName.EMERGENCY_STOP, sequence=2100))
    assert fsm.state == FsmState.FAILSAFE

    ack = manager.submit_command(
        _command(
            CommandName.CLEAR_FAILSAFE,
            sequence=2101,
            params={"operator_confirmed": True},
        )
    )
    assert ack.status == AckStatus.ACCEPTED
    assert ack.reason == ReasonCode.CLEAR_FAILSAFE_ACCEPTED
    assert fsm.state == FsmState.IDLE


def test_clear_failsafe_rejected_when_not_in_failsafe():
    manager, _, _, fsm, _ = _make_manager()
    ack = manager.submit_command(
        _command(CommandName.CLEAR_FAILSAFE, params={"operator_confirmed": True})
    )
    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.INVALID_STATE
    assert fsm.state == FsmState.IDLE


# ── shared state command group ─────────────────────────────────────


def test_command_state_is_published_on_accept():
    manager, ss, _, _, _ = _make_manager()
    cmd = _command(
        CommandName.SET_HOME,
        params={"home": {"x_cm": 1.0, "y_cm": 2.0, "frame_id": "yard"}},
    )
    manager.submit_command(cmd)

    cs = ss.get_command()
    assert cs.last_command_id == cmd.command_id
    assert cs.last_command == CommandName.SET_HOME
    assert cs.last_status == AckStatus.ACCEPTED
    assert cs.last_cause is None


def test_command_state_is_published_on_reject():
    manager, ss, _, _, _ = _make_manager()
    cmd = _command(CommandName.SET_HOME, params={})
    manager.submit_command(cmd)

    cs = ss.get_command()
    assert cs.last_command_id == cmd.command_id
    assert cs.last_status == AckStatus.REJECTED
    assert cs.last_cause == RejectionCause.HOME_MISSING


# ── idempotency ────────────────────────────────────────────────────


def test_duplicate_command_returns_cached_result_with_new_ack_sequence():
    manager, ss, acks, _, _ = _make_manager()
    cmd = _command(
        CommandName.SET_HOME,
        params={"home": {"x_cm": 1.0, "y_cm": 2.0, "frame_id": "yard"}},
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
    assert second_ack.applied_control_sequence == first_ack.applied_control_sequence
    assert second_ack.state == first_ack.state
    assert second_ack.ack_sequence == retry_msg.sequence
    assert second_ack.sequence != first_ack.sequence
    assert acks == [first_ack, second_ack]
    # Home was NOT re-applied (no source_command_id refresh, same instance).
    assert ss.get_home() is home_before_retry


def test_command_id_cache_size_is_bounded():
    manager, ss, _, _, _ = _make_manager(command_id_cache_size=3)

    for i in range(5):
        manager.submit_command(
            _command(CommandName.STOP_CHASE, sequence=2000 + i, command_id=f"cmd-{i}")
        )

    assert manager.cached_command_ids() == 3


def test_emergency_stop_cache_is_bounded():
    manager, _, _, _, _ = _make_manager(command_id_cache_size=3)

    for i in range(5):
        manager.submit_command(
            _command(
                CommandName.EMERGENCY_STOP,
                sequence=2100 + i,
                command_id=f"estop-{i}",
            )
        )

    assert manager.cached_command_ids() == 3


# ── units ──────────────────────────────────────────────────────────


def test_yard_payload_accepts_legacy_unsuffixed_centimeters():
    manager, ss, _, _, _ = _make_manager()
    manager.submit_command(
        _command(
            CommandName.SET_HOME,
            params={"home": {"x": 100.0, "y": 200.0, "frame_id": "yard"}},
        )
    )
    home = ss.get_home()
    assert (home.x, home.y) == (100.0, 200.0)
    assert (home.x_m, home.y_m) == (1.0, 2.0)


# ── emergency stop ─────────────────────────────────────────────────


def test_emergency_stop_ack_reports_state_after_hook_runs():
    manager, ss, _, fsm, _ = _make_manager()
    observed = []

    def _hook():
        observed.append(fsm.state)

    manager._on_emergency_stop = _hook
    ack = manager.submit_command(_command(CommandName.EMERGENCY_STOP))

    assert observed == [FsmState.FAILSAFE]
    assert ack.state == FsmState.FAILSAFE
    assert ss.get_fsm().state == FsmState.FAILSAFE


# ── mission events ─────────────────────────────────────────────────


def test_mission_event_rejection_keeps_mission_event_ack_type():
    manager, _, _, _, _ = _make_manager()
    ack = manager.submit_mission_event(
        mission_event_message(target_id="not-the-active-target")
    )

    assert ack.status == AckStatus.REJECTED
    assert ack.cause == RejectionCause.WRONG_TARGET
    assert ack.ack_type == AckType.MISSION_EVENT


def test_mission_event_invalid_state_keeps_mission_event_ack_type():
    manager, _, _, _, _ = _make_manager()
    manager.submit_tracking(_tracking(sequence=1))
    manager.submit_command(start_chase_command())
    manager.submit_command(_command(CommandName.STOP_CHASE, sequence=2050))

    ack = manager.submit_mission_event(mission_event_message())

    assert ack.status == AckStatus.REJECTED
    assert ack.ack_type == AckType.MISSION_EVENT


# ── commit timeout ─────────────────────────────────────────────────


def test_commit_timeout_leaves_no_waiter_or_stranded_ack():
    manager, _, _, fsm, engine = _make_manager()
    # A bound control loop that never ticks: the submitter must give up and
    # leave nothing behind for the next command to trip over.
    manager.bind_runtime(control_loop=object(), decision_engine=engine, fsm=fsm)
    manager._commit_timeout_s = 0.05

    with pytest.raises(TimeoutError):
        manager.submit_command(
            _command(CommandName.STOP_CHASE, command_id="cmd-stalled")
        )

    assert manager.pending_ack_waiters() == 0
    assert manager.uncollected_acks() == 0

    # The late commit must not strand an ACK nobody will ever collect.
    manager.apply_pending_transactions(
        applied_control_seq=7, decision_engine=engine, fsm=fsm
    )
    assert manager.uncollected_acks() == 0
