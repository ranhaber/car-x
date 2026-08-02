"""Tests for the contract-driven FSM validator."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.control.fsm import FSM, is_transition_allowed, _resolve_transition
from cat_follow.control.types import (
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


def test_idle_to_getting_close_on_start_chase_accepted():
    fsm = FSM()
    result = _apply(fsm, FsmEvent.START_CHASE_ACCEPTED, ReasonCode.START_CHASE_ACCEPTED)
    assert result.accepted is True
    assert result.from_state == FsmState.IDLE
    assert result.to_state == FsmState.GETTING_CLOSE
    assert fsm.state == FsmState.GETTING_CLOSE


def test_getting_close_to_search_on_distance_ready():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    result = _apply(
        fsm, FsmEvent.SEARCH_ENTRY_READY, ReasonCode.SEARCH_ENTRY
    )
    assert result.accepted is True
    assert fsm.state == FsmState.SEARCH


def test_search_to_chase_on_associated_local_lock():
    fsm = FSM(initial_state=FsmState.SEARCH)
    result = _apply(
        fsm, FsmEvent.LOCAL_TRACK_ACQUIRED, ReasonCode.LOCAL_TRACK
    )
    assert result.accepted is True
    assert fsm.state == FsmState.CHASE


def test_legacy_final_approach_event_does_not_enter_brake_reverse():
    fsm = FSM(initial_state=FsmState.CHASE)
    result = _apply(fsm, FsmEvent.FINAL_APPROACH_READY, ReasonCode.FINAL_APPROACH)
    assert result.accepted is False
    assert fsm.state == FsmState.CHASE


def test_brake_reverse_has_no_behavior_before_safety_slice():
    fsm = FSM(initial_state=FsmState.BRAKE_REVERSE)
    result = _apply(fsm, FsmEvent.BRAKE_ABORTED_CAT_MOVED, ReasonCode.BRAKE_ABORTED_CAT_MOVED)
    assert result.accepted is False
    assert fsm.state == FsmState.BRAKE_REVERSE


def test_chase_to_getting_close_on_far_cat_lost():
    fsm = FSM(initial_state=FsmState.CHASE)
    result = _apply(
        fsm, FsmEvent.CAT_LOST_FAR, ReasonCode.CAT_LOST_FAR
    )
    assert result.accepted is True
    assert fsm.state == FsmState.GETTING_CLOSE


def test_chase_to_search_on_near_cat_lost():
    fsm = FSM(initial_state=FsmState.CHASE)
    result = _apply(
        fsm, FsmEvent.CAT_LOST_NEAR, ReasonCode.CAT_LOST_NEAR
    )
    assert result.accepted is True
    assert fsm.state == FsmState.SEARCH


def test_brake_reverse_triggered_from_normal_driving_state():
    assert (
        is_transition_allowed(
            FsmState.GETTING_CLOSE, FsmEvent.BRAKE_REVERSE_TRIGGERED
        )
        is True
    )
    fsm = FSM(initial_state=FsmState.GOTO)
    result = _apply(
        fsm,
        FsmEvent.BRAKE_REVERSE_TRIGGERED,
        ReasonCode.BRAKE_REVERSE_TRIGGERED,
    )
    assert result.accepted is True
    assert fsm.state == FsmState.BRAKE_REVERSE


def test_brake_reverse_cleared_restores_saved_state():
    fsm = FSM(initial_state=FsmState.BRAKE_REVERSE)
    result = fsm.apply(
        FsmEvent.BRAKE_REVERSE_CLEARED,
        reason=ReasonCode.BRAKE_REVERSE_CLEAR,
        resume_state=FsmState.CHASE,
    )
    assert result.accepted is True
    assert fsm.state == FsmState.CHASE


def test_is_transition_allowed_honors_resume_state():
    assert (
        is_transition_allowed(
            FsmState.BRAKE_REVERSE,
            FsmEvent.BRAKE_REVERSE_CLEARED,
            resume_state=FsmState.CHASE,
        )
        is True
    )
    assert (
        is_transition_allowed(
            FsmState.BRAKE_REVERSE,
            FsmEvent.BRAKE_REVERSE_CLEARED,
            resume_state=FsmState.FAILSAFE,
        )
        is False
    )
    assert (
        is_transition_allowed(
            FsmState.BRAKE_REVERSE, FsmEvent.BRAKE_REVERSE_CLEARED
        )
        is False
    )


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


def test_search_reachable_only_from_chase_matrix_events():
    assert (
        _resolve_transition(
            FsmState.GETTING_CLOSE, FsmEvent.SEARCH_ENTRY_READY
        )
        == FsmState.SEARCH
    )
    assert (
        _resolve_transition(FsmState.CHASE, FsmEvent.CAT_LOST_NEAR)
        == FsmState.SEARCH
    )


def test_stop_chase_accepted_from_any_chase_state_goes_to_idle():
    for chase_state in (
        FsmState.GETTING_CLOSE,
        FsmState.SEARCH,
        FsmState.CHASE,
        FsmState.BRAKE_REVERSE,
    ):
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
        FsmState.GETTING_CLOSE,
        FsmState.SEARCH,
        FsmState.CHASE,
        FsmState.BRAKE_REVERSE,
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
        fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
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
    assert snapshot.state == FsmState.GETTING_CLOSE


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
    assert snapshot.state == FsmState.GETTING_CLOSE
    assert snapshot.previous_state == FsmState.IDLE
    assert snapshot.last_transition_ms == 12345
    assert snapshot.last_transition_reason == ReasonCode.START_CHASE_ACCEPTED
    assert snapshot.fresh is True
    assert snapshot.authority == "FSM"
