"""Tests for the contract-driven FSM validator."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.control.fsm import FSM, is_transition_allowed  # noqa: E402
from cat_follow.control.types import (  # noqa: E402
    FsmEvent,
    FsmState,
    ReasonCode,
)


def _apply(fsm, event, reason=ReasonCode.INIT):
    return fsm.apply(event, reason=reason, now_ms=1)


# ── default state ────────────────────────────────────────────────────


def test_initial_state_is_idle():
    fsm = FSM()
    assert fsm.state == FsmState.IDLE
    snapshot = fsm.snapshot()
    assert snapshot.state == FsmState.IDLE
    assert snapshot.previous_state is None
    assert snapshot.last_rejected_transition is None
    assert snapshot.authority == "FSM"


# ── direct transitions ──────────────────────────────────────────────


def test_idle_to_chase_a_on_start_chase_accepted():
    fsm = FSM()
    result = _apply(fsm, FsmEvent.START_CHASE_ACCEPTED, ReasonCode.START_CHASE_ACCEPTED)
    assert result.accepted is True
    assert result.from_state == FsmState.IDLE
    assert result.to_state == FsmState.CHASE_A
    assert fsm.state == FsmState.CHASE_A


def test_chase_a_to_track_b_on_cat_visible_stable():
    fsm = FSM(initial_state=FsmState.CHASE_A)
    result = _apply(fsm, FsmEvent.CAT_VISIBLE_STABLE, ReasonCode.LOCAL_TRACK)
    assert result.accepted is True
    assert fsm.state == FsmState.TRACK_B


def test_track_b_to_brake_on_final_approach_ready():
    fsm = FSM(initial_state=FsmState.TRACK_B)
    result = _apply(fsm, FsmEvent.FINAL_APPROACH_READY, ReasonCode.FINAL_APPROACH)
    assert result.accepted is True
    assert fsm.state == FsmState.BRAKE


def test_brake_to_track_b_on_brake_aborted_cat_moved():
    fsm = FSM(initial_state=FsmState.BRAKE)
    result = _apply(fsm, FsmEvent.BRAKE_ABORTED_CAT_MOVED, ReasonCode.BRAKE_ABORTED_CAT_MOVED)
    assert result.accepted is True
    assert fsm.state == FsmState.TRACK_B


def test_track_b_to_chase_a_on_cat_lost():
    fsm = FSM(initial_state=FsmState.TRACK_B)
    result = _apply(fsm, FsmEvent.CAT_LOST, ReasonCode.CAT_LOST_FALLBACK)
    assert result.accepted is True
    assert fsm.state == FsmState.CHASE_A


def test_idle_to_goto_on_go_to_accepted():
    fsm = FSM()
    result = _apply(fsm, FsmEvent.GO_TO_ACCEPTED, ReasonCode.GO_TO_ACCEPTED)
    assert result.accepted is True
    assert fsm.state == FsmState.GOTO


def test_goto_to_idle_on_go_to_complete():
    fsm = FSM(initial_state=FsmState.GOTO)
    result = _apply(fsm, FsmEvent.GO_TO_COMPLETE, ReasonCode.GO_TO_COMPLETE)
    assert result.accepted is True
    assert fsm.state == FsmState.IDLE


def test_return_home_to_home_on_complete():
    fsm = FSM(initial_state=FsmState.RETURN_HOME)
    result = _apply(fsm, FsmEvent.RETURN_HOME_COMPLETE, ReasonCode.RETURN_HOME_COMPLETE)
    assert result.accepted is True
    assert fsm.state == FsmState.HOME


def test_failsafe_to_idle_on_clear_failsafe_accepted():
    fsm = FSM(initial_state=FsmState.FAILSAFE)
    result = _apply(fsm, FsmEvent.CLEAR_FAILSAFE_ACCEPTED, ReasonCode.CLEAR_FAILSAFE_ACCEPTED)
    assert result.accepted is True
    assert fsm.state == FsmState.IDLE


# ── pattern rules ──────────────────────────────────────────────────


def test_stop_chase_accepted_from_any_chase_state_goes_to_idle():
    for chase_state in (FsmState.CHASE_A, FsmState.TRACK_B, FsmState.BRAKE):
        fsm = FSM(initial_state=chase_state)
        result = _apply(fsm, FsmEvent.STOP_CHASE_ACCEPTED, ReasonCode.STOP_CHASE_ACCEPTED)
        assert result.accepted is True, chase_state
        assert fsm.state == FsmState.IDLE


def test_stop_chase_accepted_from_idle_is_rejected():
    fsm = FSM()
    result = _apply(fsm, FsmEvent.STOP_CHASE_ACCEPTED, ReasonCode.STOP_CHASE_ACCEPTED)
    assert result.accepted is False
    assert fsm.state == FsmState.IDLE


def test_return_home_accepted_from_non_failsafe_states():
    for state in (
        FsmState.HOME,
        FsmState.IDLE,
        FsmState.CHASE_A,
        FsmState.TRACK_B,
        FsmState.BRAKE,
        FsmState.GOTO,
        FsmState.RETURN_HOME,
    ):
        fsm = FSM(initial_state=state)
        result = _apply(fsm, FsmEvent.RETURN_HOME_ACCEPTED, ReasonCode.RETURN_HOME_ACCEPTED)
        assert result.accepted is True, state
        assert fsm.state == FsmState.RETURN_HOME


def test_return_home_accepted_from_failsafe_is_rejected():
    fsm = FSM(initial_state=FsmState.FAILSAFE)
    result = _apply(fsm, FsmEvent.RETURN_HOME_ACCEPTED, ReasonCode.RETURN_HOME_ACCEPTED)
    assert result.accepted is False
    assert fsm.state == FsmState.FAILSAFE


def test_obstacle_too_close_from_any_state_goes_to_failsafe():
    for state in FsmState:
        fsm = FSM(initial_state=state)
        result = _apply(fsm, FsmEvent.OBSTACLE_TOO_CLOSE, ReasonCode.OBSTACLE_TOO_CLOSE)
        assert result.accepted is True, state
        assert fsm.state == FsmState.FAILSAFE


def test_failsafe_triggered_and_emergency_stop_go_to_failsafe():
    for event in (FsmEvent.FAILSAFE_TRIGGERED, FsmEvent.EMERGENCY_STOP_ACCEPTED):
        fsm = FSM(initial_state=FsmState.CHASE_A)
        result = _apply(fsm, event, ReasonCode.FAILSAFE_TRIGGERED)
        assert result.accepted is True, event
        assert fsm.state == FsmState.FAILSAFE


# ── rejection behavior ─────────────────────────────────────────────


def test_invalid_transition_is_rejected_and_recorded():
    fsm = FSM()
    result = _apply(fsm, FsmEvent.FINAL_APPROACH_READY, ReasonCode.FINAL_APPROACH)
    assert result.accepted is False
    assert fsm.state == FsmState.IDLE  # unchanged

    snapshot = fsm.snapshot()
    assert snapshot.state == FsmState.IDLE
    assert snapshot.last_rejected_transition is not None
    assert "IDLE" in snapshot.last_rejected_transition
    assert "final_approach_ready" in snapshot.last_rejected_transition


def test_successful_transition_clears_rejected_descriptor():
    fsm = FSM()
    _apply(fsm, FsmEvent.FINAL_APPROACH_READY, ReasonCode.FINAL_APPROACH)
    assert fsm.snapshot().last_rejected_transition is not None

    _apply(fsm, FsmEvent.START_CHASE_ACCEPTED, ReasonCode.START_CHASE_ACCEPTED)
    snapshot = fsm.snapshot()
    assert snapshot.last_rejected_transition is None
    assert snapshot.previous_state == FsmState.IDLE
    assert snapshot.state == FsmState.CHASE_A


def test_is_transition_allowed_helper_matches_apply():
    fsm = FSM()
    assert is_transition_allowed(FsmState.IDLE, FsmEvent.START_CHASE_ACCEPTED) is True
    assert is_transition_allowed(FsmState.IDLE, FsmEvent.FINAL_APPROACH_READY) is False
    assert is_transition_allowed(FsmState.FAILSAFE, FsmEvent.RETURN_HOME_ACCEPTED) is False
    assert is_transition_allowed(FsmState.FAILSAFE, FsmEvent.CLEAR_FAILSAFE_ACCEPTED) is True


# ── snapshot ───────────────────────────────────────────────────────


def test_snapshot_records_transition_metadata():
    fsm = FSM()
    fsm.apply(
        FsmEvent.START_CHASE_ACCEPTED,
        reason=ReasonCode.START_CHASE_ACCEPTED,
        now_ms=12345,
    )
    snapshot = fsm.snapshot()
    assert snapshot.state == FsmState.CHASE_A
    assert snapshot.previous_state == FsmState.IDLE
    assert snapshot.last_transition_ms == 12345
    assert snapshot.last_transition_reason == ReasonCode.START_CHASE_ACCEPTED
    assert snapshot.fresh is True
    assert snapshot.authority == "FSM"
