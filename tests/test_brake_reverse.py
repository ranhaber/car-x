"""Safety tests for dual-sensor hold and bounded BRAKE_REVERSE."""

from dataclasses import replace

from cat_follow.control.decision_engine import DecisionEngine
from cat_follow.control.fsm import FSM, TransitionResult
from cat_follow.control.types import (
    AckStatus,
    BrakeReversePhase,
    CommandName,
    CommandState,
    DecisionInput,
    FSMSnapshot,
    FsmEvent,
    FsmState,
    HomeState,
    NavigationState,
    OverheadState,
    RangeBackend,
    RangeState,
    ReasonCode,
    SystemState,
    VisionState,
)
from cat_follow.safety_config import SafetyConfig
from cat_follow.target_config import TargetRuntimeConfig


def _sensor(now_ms, distance_cm, backend):
    return RangeState(
        received_ms=now_ms,
        fresh=True,
        backend=backend,
        distance_cm=distance_cm,
        confidence=1.0,
    )


def _input(
    now_ms,
    *,
    state=FsmState.GETTING_CLOSE,
    ultrasonic_cm=100.0,
    lidar_cm=100.0,
    ultrasonic_healthy=True,
    lidar_healthy=True,
    command=None,
):
    return DecisionInput(
        now_ms=now_ms,
        overhead=OverheadState(received_ms=now_ms, fresh=True, sequence=1),
        home=HomeState(),
        vision=VisionState(),
        range=(
            _sensor(now_ms, ultrasonic_cm, RangeBackend.ULTRASONIC)
            if ultrasonic_healthy
            else RangeState()
        ),
        lidar=(
            _sensor(now_ms, lidar_cm, RangeBackend.LIDAR_C1)
            if lidar_healthy
            else RangeState(backend=RangeBackend.LIDAR_C1)
        ),
        navigation=NavigationState(),
        system=SystemState(),
        fsm=FSMSnapshot(state=state),
        command=command or CommandState(),
    )


def test_dual_sensor_loss_holds_then_failsafe():
    fsm = FSM(initial_state=FsmState.GOTO)
    engine = DecisionEngine(fsm)

    first = engine.tick(_input(1000, state=FsmState.GOTO, lidar_healthy=False))
    assert fsm.state == FsmState.GOTO
    assert first.reason == ReasonCode.SENSOR_HEALTH_HOLD
    assert first.speed == 0.0

    recovered_too_late = engine.tick(
        _input(3000, state=FsmState.GOTO, lidar_healthy=False)
    )
    assert fsm.state == FsmState.FAILSAFE
    assert recovered_too_late.reason == ReasonCode.SENSOR_HEALTH_TIMEOUT


def test_dual_sensor_recovery_before_deadline_resumes_objective():
    fsm = FSM(initial_state=FsmState.GOTO)
    engine = DecisionEngine(fsm)

    engine.tick(_input(1000, state=FsmState.GOTO, ultrasonic_healthy=False))
    recovered = engine.tick(_input(2999, state=FsmState.GOTO))

    assert fsm.state == FsmState.GOTO
    assert recovered.reason != ReasonCode.SENSOR_HEALTH_HOLD
    assert "sensor_health_hold" not in recovered.active_constraints


def test_brake_reverse_phases_are_centered_and_time_bounded():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)

    entry = engine.tick(_input(1000, ultrasonic_cm=14.0))
    assert fsm.state == FsmState.BRAKE_REVERSE
    assert entry.speed == 0.0
    assert entry.steering == 0.0
    assert engine.brake_reverse_phase == BrakeReversePhase.CENTER

    centered = engine.tick(_input(1020, state=FsmState.BRAKE_REVERSE))
    assert centered.speed == 0.0
    assert centered.steering == 0.0
    assert engine.brake_reverse_phase == BrakeReversePhase.SETTLE

    settling = engine.tick(_input(1119, state=FsmState.BRAKE_REVERSE))
    assert settling.speed == 0.0

    reversing = engine.tick(_input(1120, state=FsmState.BRAKE_REVERSE))
    assert reversing.speed == -0.30
    assert reversing.steering == 0.0
    assert engine.brake_reverse_attempts == 1

    still_reversing = engine.tick(_input(1619, state=FsmState.BRAKE_REVERSE))
    assert still_reversing.speed == -0.30

    stopped = engine.tick(_input(1620, state=FsmState.BRAKE_REVERSE))
    assert stopped.speed == 0.0
    assert stopped.brake is True
    assert engine.brake_reverse_phase == BrakeReversePhase.STOP_EXIT

    engine.tick(_input(1640, state=FsmState.BRAKE_REVERSE))
    restored = engine.tick(_input(1660, state=FsmState.BRAKE_REVERSE))
    assert restored.speed == 0.0
    assert fsm.state == FsmState.GETTING_CLOSE
    assert restored.reason == ReasonCode.BRAKE_REVERSE_CLEAR


def test_sensor_loss_during_brake_reverse_fails_immediately():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)
    engine.tick(_input(1000, ultrasonic_cm=14.0))

    decision = engine.tick(
        _input(1020, state=FsmState.BRAKE_REVERSE, lidar_healthy=False)
    )

    assert fsm.state == FsmState.FAILSAFE
    assert decision.reason == ReasonCode.SENSOR_HEALTH_TIMEOUT
    assert "brake_reverse_sensor_loss" in decision.active_constraints


def test_blocked_after_max_attempts_enters_failsafe():
    cfg = TargetRuntimeConfig(
        brake_reverse_settle_ms=0,
        brake_reverse_duration_sec=0.0,
        brake_reverse_max_attempts=2,
    )
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm, target_runtime_config=cfg)

    times = iter(range(1000, 1020))
    decision = engine.tick(_input(next(times), ultrasonic_cm=10.0))
    while fsm.state != FsmState.FAILSAFE:
        decision = engine.tick(
            _input(
                next(times),
                state=FsmState.BRAKE_REVERSE,
                ultrasonic_cm=10.0,
            )
        )

    assert engine.brake_reverse_attempts == 2
    assert decision.reason == ReasonCode.BRAKE_REVERSE_EXHAUSTED


def test_home_degraded_without_sensor_failsafe():
    fsm = FSM(initial_state=FsmState.HOME)
    engine = DecisionEngine(fsm)

    decision = engine.tick(_input(1000, state=FsmState.HOME, lidar_healthy=False))

    assert fsm.state == FsmState.HOME
    assert decision.reason == ReasonCode.INIT
    assert "sensor_health_degraded" in decision.active_constraints


def test_sensor_hold_boundary_1999ms_recovers_2000ms_failsafe():
    fsm = FSM(initial_state=FsmState.GOTO)
    engine = DecisionEngine(fsm)

    engine.tick(_input(1000, state=FsmState.GOTO, lidar_healthy=False))
    recovered = engine.tick(_input(2999, state=FsmState.GOTO, lidar_healthy=False))
    assert fsm.state == FsmState.GOTO
    assert recovered.reason == ReasonCode.SENSOR_HEALTH_HOLD

    failed = engine.tick(_input(3000, state=FsmState.GOTO, lidar_healthy=False))
    assert fsm.state == FsmState.FAILSAFE
    assert failed.reason == ReasonCode.SENSOR_HEALTH_TIMEOUT


def test_stop_chase_ignored_when_saved_objective_is_not_chase():
    fsm = FSM(initial_state=FsmState.GOTO)
    engine = DecisionEngine(fsm)
    engine.tick(_input(1000, state=FsmState.GOTO, ultrasonic_cm=10.0))
    assert fsm.state == FsmState.BRAKE_REVERSE

    stop = CommandState(
        last_command_id="cmd-stop",
        last_command=CommandName.STOP_CHASE,
        last_status=AckStatus.ACCEPTED,
    )
    engine.tick(
        _input(
            1100,
            state=FsmState.BRAKE_REVERSE,
            ultrasonic_cm=10.0,
            command=stop,
        )
    )

    assert fsm.state == FsmState.BRAKE_REVERSE


def test_proximity_helpers_ignore_unusable_sensor_readings():
    engine = DecisionEngine(FSM(initial_state=FsmState.GETTING_CLOSE))
    faulted = _input(1000, ultrasonic_healthy=False, lidar_healthy=False)

    assert engine._any_sensor_below(faulted, 15.0) is False
    assert engine._both_sensors_above(faulted, 20.0) is False

    stale = replace(
        _input(100000),
        range=_sensor(1, 1.0, RangeBackend.ULTRASONIC),
        lidar=_sensor(1, 1.0, RangeBackend.LIDAR_C1),
    )
    assert engine._any_sensor_below(stale, 15.0) is False


def test_clearance_reset_after_both_sensors_above_20cm_for_2s():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)

    engine.tick(_input(1000, ultrasonic_cm=14.0))
    engine.tick(_input(1020, state=FsmState.BRAKE_REVERSE))
    engine.tick(_input(1120, state=FsmState.BRAKE_REVERSE))
    assert engine.brake_reverse_attempts == 1

    fsm.force_state(FsmState.IDLE, reason=ReasonCode.INIT, now_ms=2000)
    engine.tick(_input(2000, state=FsmState.IDLE, ultrasonic_cm=25.0, lidar_cm=25.0))
    engine.tick(_input(4000, state=FsmState.IDLE, ultrasonic_cm=25.0, lidar_cm=25.0))

    assert engine.brake_reverse_attempts == 0


def test_operator_failsafe_distance_widens_the_close_trigger():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)

    # 20 cm clears the 15 cm target-architecture trigger.
    engine.tick(_input(1000, ultrasonic_cm=20.0))
    assert fsm.state == FsmState.GETTING_CLOSE

    engine.set_safety_thresholds(
        SafetyConfig(obstacle_too_close_cm=25.0, obstacle_detected_cm=50.0)
    )
    assert engine.close_obstacle_trigger_cm == 25.0

    engine.tick(_input(1100, ultrasonic_cm=20.0))
    assert fsm.state == FsmState.BRAKE_REVERSE


def test_attempt_reset_clearance_keeps_margin_above_effective_trigger():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)
    engine.set_safety_thresholds(
        SafetyConfig(obstacle_too_close_cm=25.0, obstacle_detected_cm=50.0)
    )
    # Default config keeps a 5 cm margin between trigger and reset distance.
    assert engine.close_obstacle_clear_cm == 30.0

    engine.tick(_input(1000, ultrasonic_cm=14.0))
    engine.tick(_input(1020, state=FsmState.BRAKE_REVERSE))
    engine.tick(_input(1120, state=FsmState.BRAKE_REVERSE))
    assert engine.brake_reverse_attempts == 1

    fsm.force_state(FsmState.IDLE, reason=ReasonCode.INIT, now_ms=2000)
    engine.tick(_input(2000, state=FsmState.IDLE, ultrasonic_cm=28.0, lidar_cm=28.0))
    engine.tick(_input(4000, state=FsmState.IDLE, ultrasonic_cm=28.0, lidar_cm=28.0))
    assert engine.brake_reverse_attempts == 1

    engine.tick(_input(6000, state=FsmState.IDLE, ultrasonic_cm=32.0, lidar_cm=32.0))
    engine.tick(_input(8000, state=FsmState.IDLE, ultrasonic_cm=32.0, lidar_cm=32.0))
    assert engine.brake_reverse_attempts == 0


# ── phase context ownership ────────────────────────────────────────


class _RejectBrakeTriggerFSM(FSM):
    """FSM that refuses ``BRAKE_REVERSE_TRIGGERED``.

    Stands in for the race the real transition table cannot be forced into: the
    comms thread leaving the driving state after the control tick sampled it.
    """

    def apply(self, event, *, reason, now_ms=None, resume_state=None):
        if event == FsmEvent.BRAKE_REVERSE_TRIGGERED:
            return TransitionResult(
                accepted=False,
                from_state=self.state,
                to_state=self.state,
                rejected_descriptor="test-rejected",
            )
        return super().apply(
            event, reason=reason, now_ms=now_ms, resume_state=resume_state
        )


def test_rejected_brake_reverse_trigger_holds_zero_motion():
    fsm = _RejectBrakeTriggerFSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)

    decision = engine.tick(_input(1000, ultrasonic_cm=10.0))

    assert decision.speed == 0.0
    assert decision.brake is True
    assert "brake_reverse_transition_rejected" in decision.active_constraints
    # A phase armed against a refused transition would command reverse on the
    # next tick while the FSM says the car is doing something else.
    assert engine.brake_reverse_phase is None
    assert engine.brake_saved_state is None
    assert fsm.state == FsmState.GETTING_CLOSE


def test_phase_cleared_when_fsm_left_brake_reverse_between_ticks():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)
    engine.tick(_input(1000, ultrasonic_cm=10.0))
    assert fsm.state == FsmState.BRAKE_REVERSE
    assert engine.brake_reverse_phase is not None

    # The comms transaction path applies an operator stop between ticks.
    fsm.apply(
        FsmEvent.STOP_CHASE_ACCEPTED,
        reason=ReasonCode.STOP_CHASE_ACCEPTED,
        now_ms=1100,
    )
    assert fsm.state == FsmState.IDLE

    decision = engine.tick(
        _input(1200, state=FsmState.IDLE, ultrasonic_cm=10.0)
    )

    assert engine.brake_reverse_phase is None
    assert engine.brake_saved_state is None
    assert decision.speed == 0.0
    assert fsm.state == FsmState.IDLE


def test_brake_reverse_output_refuses_to_drive_a_stale_phase():
    fsm = FSM(initial_state=FsmState.CHASE)
    engine = DecisionEngine(fsm)
    engine.tick(_input(1000, state=FsmState.CHASE, ultrasonic_cm=10.0))
    assert fsm.state == FsmState.BRAKE_REVERSE

    # Simulates the state changing after tick() routed into the brake path.
    fsm.force_state(FsmState.IDLE, reason=ReasonCode.STOP_CHASE_ACCEPTED)
    decision = engine._brake_reverse_output(
        _input(1100, state=FsmState.IDLE, ultrasonic_cm=10.0)
    )

    assert decision.speed == 0.0
    assert decision.brake is True
    assert "brake_reverse_context_stale" in decision.active_constraints
    assert engine.brake_reverse_phase is None
    assert engine.brake_saved_state is None
    # Abandoning a cancelled objective is not a fault, so no failsafe latch.
    assert fsm.state == FsmState.IDLE


def test_start_handoff_clears_brake_reverse_context():
    fsm = FSM(initial_state=FsmState.CHASE)
    engine = DecisionEngine(fsm)
    engine.tick(_input(1000, state=FsmState.CHASE, ultrasonic_cm=10.0))
    assert engine.brake_saved_state == FsmState.CHASE

    engine.start_handoff(
        1100, exited_target_id="cat-17", observation_sequence=11
    )

    assert engine.brake_reverse_phase is None
    assert engine.brake_saved_state is None


def test_operator_stop_resets_the_attempt_budget():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)
    engine.tick(_input(1000, ultrasonic_cm=10.0))
    engine.tick(_input(1020, state=FsmState.BRAKE_REVERSE, ultrasonic_cm=10.0))
    engine.tick(_input(1120, state=FsmState.BRAKE_REVERSE, ultrasonic_cm=10.0))
    assert engine.brake_reverse_attempts == 1

    stop = CommandState(
        last_command_id="cmd-stop",
        last_command=CommandName.STOP_CHASE,
        last_status=AckStatus.ACCEPTED,
    )
    engine.tick(
        _input(
            1200,
            state=FsmState.BRAKE_REVERSE,
            ultrasonic_cm=10.0,
            command=stop,
        )
    )

    assert fsm.state == FsmState.IDLE
    # The budget is scoped to the objective the operator just cancelled.
    assert engine.brake_reverse_attempts == 0
    assert engine.brake_reverse_phase is None


def test_clearing_brake_reverse_releases_the_inherited_perception_policy():
    fsm = FSM(initial_state=FsmState.CHASE)
    engine = DecisionEngine(fsm)
    engine.tick(_input(1000, state=FsmState.CHASE, ultrasonic_cm=14.0))
    assert engine.lifecycle_context().brake_saved_detector is True

    engine.tick(_input(1020, state=FsmState.BRAKE_REVERSE))
    engine.tick(_input(1120, state=FsmState.BRAKE_REVERSE))
    engine.tick(_input(1620, state=FsmState.BRAKE_REVERSE))
    engine.tick(_input(1640, state=FsmState.BRAKE_REVERSE))
    restored = engine.tick(_input(1660, state=FsmState.BRAKE_REVERSE))

    assert fsm.state == FsmState.CHASE
    assert restored.reason == ReasonCode.BRAKE_REVERSE_CLEAR
    # The lifecycle manager only consults these while the FSM is in
    # BRAKE_REVERSE, so leaving them set would misreport a later entry.
    context = engine.lifecycle_context()
    assert context.brake_saved_detector is False
    assert context.brake_saved_recording is False
    assert context.brake_saved_state is None
