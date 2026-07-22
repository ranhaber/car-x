"""Tests for the V1 DecisionEngine shell."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.control.decision_engine import (  # noqa: E402
    OBSTACLE_TOO_CLOSE_CM,
    OVERHEAD_STALE_FAILSAFE_MS,
    OVERHEAD_STALE_WARNING_MS,
    DecisionEngine,
)
from cat_follow.control.fsm import FSM  # noqa: E402
from cat_follow.control.types import (  # noqa: E402
    AckStatus,
    CommandName,
    CommandState,
    DecisionInput,
    FSMSnapshot,
    FsmState,
    HomeState,
    NavigationState,
    OverheadState,
    RangeState,
    ReasonCode,
    SystemState,
    TargetSource,
    VisionState,
)


# ── helpers ─────────────────────────────────────────────────────────


def _make_input(
    *,
    fsm_state: FsmState = FsmState.IDLE,
    now_ms: int = 1000,
    overhead_received_ms: int = 1000,
    overhead_fresh: bool = True,
    range_distance_cm=None,
    range_fresh: bool = False,
    range_critical: bool = False,
    range_received_ms: int = None,
    command: CommandState = None,
    vision: VisionState = None,
) -> DecisionInput:
    return DecisionInput(
        now_ms=now_ms,
        overhead=OverheadState(
            received_ms=overhead_received_ms,
            fresh=overhead_fresh,
            sequence=1,
        ),
        home=HomeState(),
        vision=vision if vision is not None else VisionState(),
        range=RangeState(
            received_ms=now_ms if range_received_ms is None else range_received_ms,
            fresh=range_fresh,
            distance_cm=range_distance_cm,
            obstacle_critical=range_critical,
        ),
        navigation=NavigationState(),
        system=SystemState(),
        fsm=FSMSnapshot(state=fsm_state),
        command=command or CommandState(),
    )


def _accepted_command(command_id, command):
    return CommandState(
        last_command_id=command_id,
        last_command=command,
        last_status=AckStatus.ACCEPTED,
    )


def _rejected_command(command_id, command):
    return CommandState(
        last_command_id=command_id,
        last_command=command,
        last_status=AckStatus.REJECTED,
    )


def _make_engine(initial_state: FsmState = FsmState.IDLE):
    fsm = FSM(initial_state=initial_state)
    return DecisionEngine(fsm), fsm


# ── default behavior ───────────────────────────────────────────────


def test_idle_default_tick_is_safe_stop():
    engine, fsm = _make_engine(FsmState.IDLE)
    decision = engine.tick(_make_input(fsm_state=FsmState.IDLE))

    assert decision.requested_state == FsmState.IDLE
    assert decision.speed == 0.0
    assert decision.steering == 0.0
    assert decision.brake is False
    assert decision.reason == ReasonCode.INIT
    assert decision.active_constraints == ()
    assert decision.target_source == TargetSource.NONE
    assert fsm.state == FsmState.IDLE


def test_failsafe_state_emits_brake_command():
    engine, fsm = _make_engine(FsmState.FAILSAFE)
    decision = engine.tick(_make_input(fsm_state=FsmState.FAILSAFE))

    assert decision.requested_state == FsmState.FAILSAFE
    assert decision.brake is True
    assert decision.reason == ReasonCode.FAILSAFE_TRIGGERED
    assert fsm.state == FsmState.FAILSAFE


def test_state_default_reasons_match_state():
    cases = [
        (FsmState.HOME, ReasonCode.INIT),
        (FsmState.CHASE_A, ReasonCode.GLOBAL_CHASE),
        (FsmState.TRACK_B, ReasonCode.LOCAL_TRACK),
        (FsmState.BRAKE, ReasonCode.FINAL_APPROACH),
        (FsmState.GOTO, ReasonCode.GO_TO_ACCEPTED),
        (FsmState.RETURN_HOME, ReasonCode.RETURN_HOME_ACCEPTED),
    ]
    for state, expected_reason in cases:
        engine, fsm = _make_engine(state)
        # TRACK_B is only stable while a fresh, visible cat is locked; otherwise
        # the wired CAT_LOST transition falls back to CHASE_A.  Provide one so
        # this state's default reason (LOCAL_TRACK) is observable.
        vision = None
        if state == FsmState.TRACK_B:
            vision = VisionState(
                received_ms=1000, fresh=True, cat_visible=True, cat_visible_stable=True
            )
        decision = engine.tick(_make_input(fsm_state=state, vision=vision))
        # Chase states with no fresh overhead packet are still expected to
        # emit the state default reason in the V1 shell because the freshness
        # rule only fires when overhead has actually been received and is
        # too old.  See test_overhead_expired_in_chase_triggers_failsafe.
        assert decision.reason == expected_reason, state
        assert decision.speed == 0.0
        assert decision.steering == 0.0
        assert decision.brake is False


# ── obstacle veto ──────────────────────────────────────────────────


def test_range_below_threshold_triggers_failsafe():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            range_distance_cm=OBSTACLE_TOO_CLOSE_CM - 1.0,
            range_fresh=True,
        )
    )

    assert fsm.state == FsmState.FAILSAFE
    assert decision.requested_state == FsmState.FAILSAFE
    assert decision.brake is True
    assert decision.reason == ReasonCode.OBSTACLE_TOO_CLOSE
    assert "obstacle_too_close" in decision.active_constraints


def test_range_at_threshold_does_not_trigger_failsafe():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            range_distance_cm=OBSTACLE_TOO_CLOSE_CM,
            range_fresh=True,
        )
    )

    assert fsm.state == FsmState.CHASE_A
    assert decision.requested_state == FsmState.CHASE_A
    assert "obstacle_too_close" not in decision.active_constraints


def test_stale_range_does_not_trigger_obstacle_veto():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    # Very close but the sample is old (aged out); freshness is now computed
    # from received_ms, not the sticky fresh flag.
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            now_ms=100_000,
            overhead_received_ms=100_000,
            range_distance_cm=1.0,
            range_received_ms=1,
            range_fresh=True,
        )
    )

    assert fsm.state == FsmState.CHASE_A
    assert "obstacle_too_close" not in decision.active_constraints


def test_critical_obstacle_severity_triggers_failsafe():
    engine, fsm = _make_engine(FsmState.TRACK_B)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.TRACK_B,
            range_fresh=True,
            range_distance_cm=50.0,
            range_critical=True,
        )
    )

    assert fsm.state == FsmState.FAILSAFE
    assert decision.brake is True
    assert decision.reason == ReasonCode.OBSTACLE_VETO
    assert "obstacle_veto" in decision.active_constraints


# ── vision-driven chase handoff ────────────────────────────────────


def test_chase_a_to_track_b_on_stable_vision():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    vision = VisionState(
        received_ms=1000, fresh=True, cat_visible=True, cat_visible_stable=True
    )
    engine.tick(_make_input(fsm_state=FsmState.CHASE_A, vision=vision))
    assert fsm.state == FsmState.TRACK_B


def test_track_b_to_chase_a_on_vision_aged_out():
    engine, fsm = _make_engine(FsmState.TRACK_B)
    # Vision was received long ago -> aged past VISION_STALE_MS -> cat lost.
    vision = VisionState(
        received_ms=1, fresh=True, cat_visible=True, cat_visible_stable=True
    )
    engine.tick(
        _make_input(
            fsm_state=FsmState.TRACK_B,
            now_ms=100_000,
            overhead_received_ms=100_000,
            vision=vision,
        )
    )
    assert fsm.state == FsmState.CHASE_A


def test_track_b_stays_with_fresh_visible_cat():
    engine, fsm = _make_engine(FsmState.TRACK_B)
    vision = VisionState(
        received_ms=1000, fresh=True, cat_visible=True, cat_visible_stable=True
    )
    engine.tick(_make_input(fsm_state=FsmState.TRACK_B, vision=vision))
    assert fsm.state == FsmState.TRACK_B


# ── overhead freshness ─────────────────────────────────────────────


def test_overhead_stale_warning_in_chase_state_adds_constraint():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    now_ms = 10_000
    received_ms = now_ms - (OVERHEAD_STALE_WARNING_MS + 50)  # >300ms, <700ms
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            now_ms=now_ms,
            overhead_received_ms=received_ms,
            overhead_fresh=False,
        )
    )

    assert fsm.state == FsmState.CHASE_A
    assert "overhead_stale" in decision.active_constraints
    # No failsafe yet, but reduced behavior is the caller's job for V1.


def test_overhead_expired_in_chase_state_triggers_failsafe():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    now_ms = 10_000
    received_ms = now_ms - (OVERHEAD_STALE_FAILSAFE_MS + 50)  # >700ms
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            now_ms=now_ms,
            overhead_received_ms=received_ms,
            overhead_fresh=False,
        )
    )

    assert fsm.state == FsmState.FAILSAFE
    assert decision.requested_state == FsmState.FAILSAFE
    assert decision.brake is True
    assert decision.reason == ReasonCode.OVERHEAD_EXPIRED
    assert "overhead_expired" in decision.active_constraints


def test_overhead_freshness_ignored_in_idle_state():
    engine, fsm = _make_engine(FsmState.IDLE)
    now_ms = 10_000
    received_ms = now_ms - 9_000  # very stale
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            now_ms=now_ms,
            overhead_received_ms=received_ms,
            overhead_fresh=False,
        )
    )

    assert fsm.state == FsmState.IDLE
    assert "overhead_stale" not in decision.active_constraints
    assert "overhead_expired" not in decision.active_constraints


def test_overhead_never_received_does_not_trigger_failsafe():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            now_ms=10_000,
            overhead_received_ms=0,  # never received
            overhead_fresh=False,
        )
    )

    assert fsm.state == FsmState.CHASE_A
    assert "overhead_expired" not in decision.active_constraints


# ── safety precedence ──────────────────────────────────────────────


def test_obstacle_too_close_takes_precedence_over_overhead_stale():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    now_ms = 10_000
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            now_ms=now_ms,
            overhead_received_ms=now_ms - 5000,  # very expired
            range_distance_cm=5.0,  # too close
            range_fresh=True,
        )
    )

    assert fsm.state == FsmState.FAILSAFE
    assert decision.reason == ReasonCode.OBSTACLE_TOO_CLOSE
    # The overhead-expired constraint should not be present because we
    # short-circuited at the obstacle veto.
    assert "obstacle_too_close" in decision.active_constraints
    assert "overhead_expired" not in decision.active_constraints


# ── output shape ───────────────────────────────────────────────────


def test_output_uses_normalized_motion_and_zero_default():
    engine, _ = _make_engine(FsmState.CHASE_A)
    decision = engine.tick(_make_input(fsm_state=FsmState.CHASE_A))
    assert -1.0 <= decision.speed <= 1.0
    assert -1.0 <= decision.steering <= 1.0
    assert decision.target_x is None
    assert decision.target_y is None
    assert decision.target_source == TargetSource.NONE
    assert decision.rejected_transition is False


# ── command consumption (Milestone 2 extension) ────────────────────


def test_accepted_start_chase_command_fires_fsm_event():
    engine, fsm = _make_engine(FsmState.IDLE)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            command=_accepted_command("cmd-1", CommandName.START_CHASE),
        )
    )
    assert fsm.state == FsmState.CHASE_A
    assert decision.requested_state == FsmState.CHASE_A


def test_accepted_stop_chase_transitions_to_idle_and_skips_overhead_stale():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    now_ms = 10_000
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            now_ms=now_ms,
            overhead_received_ms=now_ms - 5_000,  # very stale
            overhead_fresh=False,
            command=_accepted_command("cmd-stop", CommandName.STOP_CHASE),
        )
    )
    assert fsm.state == FsmState.IDLE
    assert decision.requested_state == FsmState.IDLE
    # Because we transitioned out of chase before the freshness check,
    # the overhead-stale constraint must NOT be added.
    assert "overhead_stale" not in decision.active_constraints
    assert "overhead_expired" not in decision.active_constraints


def test_command_id_is_consumed_only_once():
    engine, fsm = _make_engine(FsmState.IDLE)
    cmd_state = _accepted_command("cmd-go", CommandName.GO_TO)

    engine.tick(_make_input(fsm_state=FsmState.IDLE, command=cmd_state))
    assert fsm.state == FsmState.GOTO

    # Re-tick with the same command_id present in shared state.  The engine
    # must not re-fire the FSM event.
    fsm.apply_force = None  # sanity: not present
    engine.tick(_make_input(fsm_state=FsmState.GOTO, command=cmd_state))
    assert fsm.state == FsmState.GOTO


def test_rejected_command_is_consumed_without_firing_event():
    engine, fsm = _make_engine(FsmState.IDLE)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            command=_rejected_command("cmd-bad-start", CommandName.START_CHASE),
        )
    )
    # No transition because the command was rejected upstream.
    assert fsm.state == FsmState.IDLE
    assert decision.requested_state == FsmState.IDLE


def test_set_home_command_does_not_fire_fsm_event():
    engine, fsm = _make_engine(FsmState.IDLE)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            command=_accepted_command("cmd-home", CommandName.SET_HOME),
        )
    )
    assert fsm.state == FsmState.IDLE
    assert decision.requested_state == FsmState.IDLE


def test_emergency_stop_command_transitions_to_failsafe():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            command=_accepted_command("cmd-estop", CommandName.EMERGENCY_STOP),
        )
    )
    assert fsm.state == FsmState.FAILSAFE
    assert decision.brake is True
    assert decision.requested_state == FsmState.FAILSAFE
    assert decision.reason == ReasonCode.FAILSAFE_TRIGGERED


def test_clear_failsafe_command_returns_to_idle():
    engine, fsm = _make_engine(FsmState.FAILSAFE)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.FAILSAFE,
            command=_accepted_command("cmd-clear", CommandName.CLEAR_FAILSAFE),
        )
    )
    assert fsm.state == FsmState.IDLE
    assert decision.requested_state == FsmState.IDLE
