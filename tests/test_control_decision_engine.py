"""Tests for the V1 DecisionEngine shell."""

import os
import sys

import pytest

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
    CarTrackingState,
    TrackingObjectState,
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
    range_distance_cm=100.0,
    range_fresh: bool = True,
    range_critical: bool = False,
    range_received_ms: int = None,
    range_confidence: float | None = None,
    lidar_distance_cm: float | None = 100.0,
    lidar_fresh: bool = True,
    lidar_received_ms: int | None = None,
    lidar_confidence: float = 1.0,
    command: CommandState = None,
    vision: VisionState = None,
    target_id: str | None = "cat-17",
    car_x: float = 0.0,
    car_y: float = 0.0,
    cat_x: float = 300.0,
    cat_y: float = 0.0,
    overhead_confidence: float = 1.0,
    home: HomeState = None,
    navigation: NavigationState = None,
) -> DecisionInput:
    if range_confidence is None:
        range_confidence = (
            1.0 if range_fresh and range_distance_cm is not None else 0.0
        )
    return DecisionInput(
        now_ms=now_ms,
        overhead=OverheadState(
            received_ms=overhead_received_ms,
            fresh=overhead_fresh,
            sequence=1,
            selected_target_id=target_id,
            car=CarTrackingState(
                x=car_x, y=car_y, confidence=overhead_confidence
            ),
            cat=TrackingObjectState(
                x=cat_x,
                y=cat_y,
                confidence=overhead_confidence,
                target_id=target_id,
            ),
        ),
        home=home or HomeState(),
        vision=vision if vision is not None else VisionState(),
        range=RangeState(
            received_ms=now_ms if range_received_ms is None else range_received_ms,
            fresh=range_fresh,
            distance_cm=range_distance_cm,
            confidence=range_confidence,
            obstacle_critical=range_critical,
        ),
        navigation=navigation or NavigationState(),
        system=SystemState(),
        fsm=FSMSnapshot(state=fsm_state),
        command=command or CommandState(),
        lidar=RangeState(
            received_ms=(
                now_ms if lidar_received_ms is None else lidar_received_ms
            ),
            fresh=lidar_fresh,
            distance_cm=lidar_distance_cm,
            confidence=lidar_confidence,
        ),
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
    engine = DecisionEngine(fsm)
    if initial_state in {
        FsmState.GETTING_CLOSE,
        FsmState.SEARCH,
        FsmState.CHASE,
    }:
        engine.set_active_target_id("cat-17")
    return engine, fsm


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
        (FsmState.GETTING_CLOSE, ReasonCode.GLOBAL_CHASE),
        (FsmState.SEARCH, ReasonCode.GLOBAL_CHASE),
        (FsmState.CHASE, ReasonCode.LOCAL_TRACK),
        (FsmState.GOTO, ReasonCode.GO_TO_ACCEPTED),
        (FsmState.RETURN_HOME, ReasonCode.RETURN_HOME_ACCEPTED),
    ]
    for state, expected_reason in cases:
        engine, fsm = _make_engine(state)
        # CHASE is only stable while a fresh, visible cat is locked; otherwise
        # the wired CAT_LOST transition falls back to GETTING_CLOSE. Provide one so
        # this state's default reason (LOCAL_TRACK) is observable.
        vision = None
        if state == FsmState.CHASE:
            vision = VisionState(
                received_ms=1000,
                fresh=True,
                cat_visible=True,
                cat_visible_stable=True,
                associated_target_id="cat-17",
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


def test_range_below_reverse_threshold_enters_brake_reverse():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            range_distance_cm=OBSTACLE_TOO_CLOSE_CM - 1.0,
            range_fresh=True,
        )
    )

    assert fsm.state == FsmState.BRAKE_REVERSE
    assert decision.requested_state == FsmState.BRAKE_REVERSE
    assert decision.brake is True
    assert decision.reason == ReasonCode.BRAKE_REVERSE_ACTIVE
    assert "brake_reverse" in decision.active_constraints


def test_range_at_reverse_threshold_does_not_trigger():
    engine, fsm = _make_engine(FsmState.CHASE_A)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            range_distance_cm=15.0,
            range_fresh=True,
        )
    )

    assert fsm.state == FsmState.CHASE_A
    assert decision.requested_state == FsmState.CHASE_A
    assert "brake_reverse" not in decision.active_constraints


def test_stale_range_starts_sensor_health_hold():
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
    assert decision.reason == ReasonCode.SENSOR_HEALTH_HOLD
    assert "ultrasonic_unhealthy" in decision.active_constraints


def test_legacy_critical_flag_does_not_bypass_distance_policy():
    engine, fsm = _make_engine(FsmState.GETTING_CLOSE)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.GETTING_CLOSE,
            range_fresh=True,
            range_distance_cm=50.0,
            range_critical=True,
        )
    )

    assert fsm.state == FsmState.GETTING_CLOSE
    assert decision.reason == ReasonCode.GLOBAL_CHASE
    assert "obstacle_veto" not in decision.active_constraints


# ── chase transition matrix ────────────────────────────────────────


def test_getting_close_enters_search_at_distance_threshold():
    engine, fsm = _make_engine(FsmState.GETTING_CLOSE)
    engine.tick(
        _make_input(
            fsm_state=FsmState.GETTING_CLOSE,
            cat_x=200.0,
        )
    )
    assert fsm.state == FsmState.SEARCH


def test_search_requires_three_new_associated_observations():
    engine, fsm = _make_engine(FsmState.SEARCH)
    for sequence in (1, 2):
        engine.tick(
            _make_input(
                fsm_state=FsmState.SEARCH,
                vision=VisionState(
                    received_ms=1000,
                    fresh=True,
                    cat_visible=True,
                    observation_sequence=sequence,
                    associated_target_id="cat-17",
                ),
            )
        )
        assert fsm.state == FsmState.SEARCH
    engine.tick(
        _make_input(
            fsm_state=FsmState.SEARCH,
            vision=VisionState(
                received_ms=1000,
                fresh=True,
                cat_visible=True,
                observation_sequence=3,
                associated_target_id="cat-17",
            ),
        )
    )
    assert fsm.state == FsmState.CHASE


def test_chase_local_loss_near_transitions_directly_to_search():
    engine, fsm = _make_engine(FsmState.CHASE)
    engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE,
            cat_x=150.0,
            vision=VisionState(received_ms=1),
        )
    )
    assert fsm.state == FsmState.SEARCH


def test_chase_local_loss_far_transitions_directly_to_getting_close():
    engine, fsm = _make_engine(FsmState.CHASE)
    engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE,
            cat_x=250.0,
            vision=VisionState(received_ms=1),
        )
    )
    assert fsm.state == FsmState.GETTING_CLOSE


def test_overhead_invalid_is_retained_in_getting_close():
    engine, fsm = _make_engine(FsmState.GETTING_CLOSE)
    now_ms = 10_000
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.GETTING_CLOSE,
            now_ms=now_ms,
            overhead_received_ms=now_ms - OVERHEAD_STALE_WARNING_MS - 1,
        )
    )
    assert fsm.state == FsmState.GETTING_CLOSE
    assert "overhead_invalid_retention" in decision.active_constraints


def test_overhead_retention_timeout_returns_home_when_safe():
    engine, fsm = _make_engine(FsmState.GETTING_CLOSE)
    stale_received_ms = 1
    engine.tick(
        _make_input(
            fsm_state=FsmState.GETTING_CLOSE,
            now_ms=1000,
            overhead_received_ms=stale_received_ms,
        )
    )
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.GETTING_CLOSE,
            now_ms=11_000,
            overhead_received_ms=stale_received_ms,
            home=HomeState(set=True),
        )
    )
    assert fsm.state == FsmState.RETURN_HOME
    assert decision.reason == ReasonCode.OVERHEAD_EXPIRED


def test_overhead_target_change_stops_in_idle():
    engine, fsm = _make_engine(FsmState.GETTING_CLOSE)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.GETTING_CLOSE,
            target_id="cat-other",
        )
    )
    assert fsm.state == FsmState.IDLE
    assert decision.reason == ReasonCode.TARGET_ID_CHANGED


def test_search_second_interval_timeout_returns_home():
    engine, fsm = _make_engine(FsmState.SEARCH)
    for now_ms in (1000, 11_000, 21_000):
        decision = engine.tick(
            _make_input(
                fsm_state=FsmState.SEARCH,
                now_ms=now_ms,
                overhead_received_ms=now_ms,
                home=HomeState(set=True),
                vision=VisionState(
                    received_ms=now_ms,
                    observation_sequence=now_ms,
                ),
            )
        )
    assert fsm.state == FsmState.RETURN_HOME
    assert decision.reason == ReasonCode.SEARCH_EXHAUSTED


def test_handoff_timeout_returns_home_when_safe():
    engine, fsm = _make_engine(FsmState.IDLE)
    engine.start_handoff(
        1000, exited_target_id="cat-17", observation_sequence=5
    )
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            now_ms=11_000,
            home=HomeState(set=True),
        )
    )
    assert fsm.state == FsmState.RETURN_HOME
    assert decision.reason == ReasonCode.HANDOFF_TIMEOUT


def test_search_navigation_is_capped_at_search_speed():
    engine, fsm = _make_engine(FsmState.SEARCH)
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.SEARCH,
            navigation=NavigationState(
                received_ms=1000,
                fresh=True,
                speed_limit=1.0,
                path_correction=0.2,
            ),
        )
    )
    assert fsm.state == FsmState.SEARCH
    assert decision.speed == pytest.approx(1.0 / 3.0)
    assert decision.steering == 0.2


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


def test_brake_reverse_takes_precedence_over_overhead_stale():
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

    assert fsm.state == FsmState.BRAKE_REVERSE
    assert decision.reason == ReasonCode.BRAKE_REVERSE_ACTIVE
    # The overhead-expired constraint should not be present because we
    # short-circuited at the obstacle veto.
    assert "brake_reverse" in decision.active_constraints
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


def test_rejected_start_chase_transition_leaves_no_active_target():
    engine, fsm = _make_engine(FsmState.GOTO)
    assert engine.active_target_id is None

    engine.tick(
        _make_input(
            fsm_state=FsmState.GOTO,
            command=CommandState(
                last_command_id="cmd-start",
                last_command=CommandName.START_CHASE,
                last_status=AckStatus.ACCEPTED,
                target_id="cat-17",
            ),
        )
    )

    # START_CHASE is not a legal transition out of GOTO, so the objective must
    # not be bound to the requested target.
    assert fsm.state == FsmState.GOTO
    assert engine.active_target_id is None


def test_stop_chase_releases_target_even_without_fsm_transition():
    engine, fsm = _make_engine(FsmState.IDLE)
    engine.set_active_target_id("cat-17")

    engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            command=_accepted_command("cmd-stop", CommandName.STOP_CHASE),
        )
    )

    assert fsm.state == FsmState.IDLE
    assert engine.active_target_id is None


def test_mission_event_publication_does_not_swallow_its_command():
    engine, fsm = _make_engine(FsmState.IDLE)

    engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            command=CommandState(
                last_command_id="cmd-go",
                last_command=CommandName.GO_TO,
                last_status=AckStatus.ACCEPTED,
                mission_event_id="evt-1",
            ),
        )
    )

    assert engine.last_consumed_mission_event_id == "evt-1"
    assert fsm.state == FsmState.GOTO


def test_manual_sequence_drives_when_sensors_usable():
    from cat_follow.motion.action_plan import DriveAction
    from cat_follow.motion.sequence_executor import MotionSequenceExecutor

    executor = MotionSequenceExecutor(heartbeat_timeout_ms=1000)
    executor.start([DriveAction(speed_pct=30, duration_s=1.0)], now_ms=1000)
    fsm = FSM(initial_state=FsmState.IDLE)
    engine = DecisionEngine(fsm, sequence_executor=executor)

    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            now_ms=1100,
            range_distance_cm=OBSTACLE_TOO_CLOSE_CM + 5.0,
            range_fresh=True,
            range_received_ms=1100,
        )
    )
    assert decision.reason == ReasonCode.MANUAL_SEQUENCE
    assert decision.speed == 0.3
    assert "manual_sequence" in decision.active_constraints


def test_manual_sequence_aborts_when_fsm_leaves_idle():
    from cat_follow.motion.action_plan import DriveAction
    from cat_follow.motion.sequence_executor import MotionSequenceExecutor

    executor = MotionSequenceExecutor(heartbeat_timeout_ms=1000)
    executor.start([DriveAction(speed_pct=30, duration_s=5.0)], now_ms=1000)
    fsm = FSM(initial_state=FsmState.CHASE_A)
    engine = DecisionEngine(fsm, sequence_executor=executor)

    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.CHASE_A,
            now_ms=1100,
            range_distance_cm=OBSTACLE_TOO_CLOSE_CM + 5.0,
            range_fresh=True,
            range_received_ms=1100,
        )
    )

    assert not executor.is_running
    assert decision.brake is True
    assert "sequence_blocked_fsm" in decision.active_constraints


def test_manual_sequence_aborts_on_obstacle_without_resuming():
    from cat_follow.motion.action_plan import DriveAction
    from cat_follow.motion.sequence_executor import MotionSequenceExecutor

    executor = MotionSequenceExecutor(heartbeat_timeout_ms=1000)
    executor.start([DriveAction(speed_pct=30, duration_s=5.0)], now_ms=1000)
    fsm = FSM(initial_state=FsmState.IDLE)
    engine = DecisionEngine(fsm, sequence_executor=executor)

    blocked = engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            now_ms=1100,
            range_distance_cm=OBSTACLE_TOO_CLOSE_CM - 1.0,
            range_fresh=True,
            range_received_ms=1100,
        )
    )
    assert blocked.reason == ReasonCode.OBSTACLE_TOO_CLOSE
    assert not executor.is_running

    cleared = engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            now_ms=1200,
            range_distance_cm=OBSTACLE_TOO_CLOSE_CM + 5.0,
            range_fresh=True,
            range_received_ms=1200,
        )
    )
    assert cleared.reason != ReasonCode.MANUAL_SEQUENCE
    assert cleared.speed == 0.0


def test_manual_sequence_obstacle_abort_does_not_latch_failsafe():
    from cat_follow.motion.action_plan import DriveAction
    from cat_follow.motion.sequence_executor import MotionSequenceExecutor

    executor = MotionSequenceExecutor(heartbeat_timeout_ms=1000)
    executor.start([DriveAction(speed_pct=30, duration_s=5.0)], now_ms=1000)
    fsm = FSM(initial_state=FsmState.IDLE)
    engine = DecisionEngine(fsm, sequence_executor=executor)

    blocked = engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            now_ms=1100,
            range_distance_cm=engine.close_obstacle_trigger_cm - 1.0,
            range_fresh=True,
            range_received_ms=1100,
        )
    )

    assert blocked.speed == 0.0
    assert blocked.brake is True
    assert "obstacle_too_close" in blocked.active_constraints
    assert not executor.is_running
    # A routine Movement-tab obstacle must not require CLEAR_FAILSAFE, matching
    # the recoverable BRAKE_REVERSE behavior of the autonomous path.
    assert fsm.state == FsmState.IDLE

    executor.start([DriveAction(speed_pct=30, duration_s=1.0)], now_ms=1200)
    resumed = engine.tick(
        _make_input(
            fsm_state=FsmState.IDLE,
            now_ms=1300,
            range_distance_cm=100.0,
            range_fresh=True,
            range_received_ms=1300,
        )
    )
    assert resumed.reason == ReasonCode.MANUAL_SEQUENCE


def test_close_obstacle_does_not_move_stationary_idle():
    engine, fsm = _make_engine(FsmState.IDLE)
    at_15cm = engine.tick(
        _make_input(
            now_ms=1000,
            range_distance_cm=15.0,
            range_fresh=True,
            range_received_ms=1000,
        )
    )
    assert fsm.state == FsmState.IDLE
    assert at_15cm.reason != ReasonCode.OBSTACLE_TOO_CLOSE

    engine2, fsm2 = _make_engine(FsmState.IDLE)
    at_9cm = engine2.tick(
        _make_input(
            now_ms=1000,
            range_distance_cm=9.0,
            range_fresh=True,
            range_received_ms=1000,
        )
    )
    assert fsm2.state == FsmState.IDLE
    assert at_9cm.speed == 0.0


def test_configurable_brake_reverse_threshold():
    from cat_follow.target_config import TargetRuntimeConfig

    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(
        fsm,
        target_runtime_config=TargetRuntimeConfig(
            brake_reverse_trigger_cm=20.0,
            brake_reverse_reset_cm=25.0,
        ),
    )
    decision = engine.tick(
        _make_input(
            fsm_state=FsmState.GETTING_CLOSE,
            now_ms=1000,
            range_distance_cm=15.0,
            range_fresh=True,
            range_received_ms=1000,
        )
    )
    assert fsm.state == FsmState.BRAKE_REVERSE
    assert decision.reason == ReasonCode.BRAKE_REVERSE_ACTIVE

