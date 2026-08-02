"""Tests for target control contract types."""

import os
import sys
from dataclasses import FrozenInstanceError

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.control.types import (  # noqa: E402
    AckStatus,
    CommandName,
    DecisionInput,
    DecisionOutput,
    FsmState,
    HomeState,
    OverheadState,
    ReasonCode,
    SharedSnapshot,
    TargetSource,
)


def test_fsm_states_use_canonical_redesign_names():
    assert {state.value for state in FsmState} == {
        "HOME",
        "IDLE",
        "GETTING_CLOSE",
        "SEARCH",
        "CHASE",
        "BRAKE_REVERSE",
        "GOTO",
        "RETURN_HOME",
        "FAILSAFE",
    }


def test_v1_fsm_names_adapt_to_canonical_names():
    assert FsmState.CHASE_A is FsmState.GETTING_CLOSE
    assert FsmState.TRACK_B is FsmState.CHASE
    assert FsmState("CHASE_A") is FsmState.GETTING_CLOSE
    assert FsmState("TRACK_B") is FsmState.CHASE
    assert FsmState("BRAKE") is FsmState.BRAKE_REVERSE


def test_command_and_ack_enums_match_contract():
    assert CommandName.START_CHASE.value == "start_chase"
    assert CommandName.GO_TO.value == "go_to"
    assert AckStatus.ACCEPTED.value == "accepted"
    assert AckStatus.REJECTED.value == "rejected"
    assert {status.value for status in AckStatus} == {"accepted", "rejected"}


def test_shared_snapshot_defaults_are_safe_and_immutable():
    snapshot = SharedSnapshot()
    assert snapshot.fsm.state == FsmState.IDLE
    assert snapshot.home.set is False
    assert snapshot.overhead.car.confidence == 0.0
    assert snapshot.decision.speed == 0.0
    assert snapshot.decision.steering == 0.0

    with pytest.raises(FrozenInstanceError):
        snapshot.home = HomeState(set=True)


def test_decision_input_excludes_previous_decision():
    snapshot = SharedSnapshot()
    decision_input = DecisionInput(
        now_ms=123,
        overhead=snapshot.overhead,
        home=snapshot.home,
        vision=snapshot.vision,
        range=snapshot.range,
        navigation=snapshot.navigation,
        system=snapshot.system,
        fsm=snapshot.fsm,
        command=snapshot.command,
    )

    assert not hasattr(decision_input, "decision")
    assert decision_input.now_ms == 123


def test_decision_output_uses_normalized_motion_and_debug_target():
    output = DecisionOutput(
        timestamp_ms=10,
        requested_state=FsmState.CHASE_A,
        speed=0.5,
        steering=-0.25,
        brake=False,
        reason=ReasonCode.GLOBAL_CHASE,
        active_constraints=("navigation_constraint",),
        target_x=230.0,
        target_y=410.0,
        target_source=TargetSource.CAT_GLOBAL,
    )

    assert output.requested_state == FsmState.CHASE_A
    assert -1.0 <= output.speed <= 1.0
    assert -1.0 <= output.steering <= 1.0
    assert output.target_source == TargetSource.CAT_GLOBAL


def test_overhead_default_tracking_confidence_is_unusable():
    overhead = OverheadState()
    assert overhead.frame_id == "yard"
    assert overhead.car.confidence == 0.0
    assert overhead.car.heading_valid is False
    assert overhead.cat.confidence == 0.0
