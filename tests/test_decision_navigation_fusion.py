"""Tests for lidar veto + NavigationState fusion in the DecisionEngine."""

from cat_follow.control.decision_engine import (
    MAX_SPEED,
    OBSTACLE_TOO_CLOSE_CM,
    DecisionEngine,
)
from cat_follow.control.fsm import FSM
from cat_follow.control.types import (
    CommandState,
    DecisionInput,
    FSMSnapshot,
    FsmState,
    HomeState,
    NavigationState,
    OverheadState,
    RangeBackend,
    RangeState,
    SystemState,
    VisionState,
)


def _fresh_range(now_ms=1000):
    """A fresh, valid, far ultrasonic reading (usable obstacle sensor)."""
    return RangeState(
        received_ms=now_ms,
        fresh=True,
        distance_cm=100.0,
        confidence=1.0,
    )


def _input(
    *,
    fsm_state=FsmState.CHASE_A,
    now_ms=1000,
    navigation=None,
    lidar=None,
    range=None,
) -> DecisionInput:
    return DecisionInput(
        now_ms=now_ms,
        overhead=OverheadState(received_ms=now_ms, fresh=True, sequence=1),
        home=HomeState(),
        vision=VisionState(),
        range=range if range is not None else _fresh_range(now_ms),
        navigation=navigation or NavigationState(),
        system=SystemState(),
        fsm=FSMSnapshot(state=fsm_state),
        command=CommandState(),
        lidar=lidar or RangeState(),
    )


def _engine(state=FsmState.CHASE_A):
    fsm = FSM(initial_state=state)
    return DecisionEngine(fsm), fsm


def test_lidar_close_triggers_failsafe():
    engine, fsm = _engine(FsmState.CHASE_A)
    lidar = RangeState(
        received_ms=1000,
        fresh=True,
        backend=RangeBackend.LIDAR_C1,
        distance_cm=OBSTACLE_TOO_CLOSE_CM - 2.0,
    )
    decision = engine.tick(_input(lidar=lidar))
    assert fsm.state == FsmState.FAILSAFE
    assert decision.brake is True
    assert "obstacle_too_close" in decision.active_constraints
    assert "lidar_obstacle" in decision.active_constraints


def test_lidar_critical_triggers_veto():
    engine, fsm = _engine(FsmState.CHASE_A)
    lidar = RangeState(
        received_ms=1000,
        fresh=True,
        backend=RangeBackend.LIDAR_C1,
        distance_cm=40.0,
        obstacle_critical=True,
    )
    decision = engine.tick(_input(lidar=lidar))
    assert fsm.state == FsmState.FAILSAFE
    assert "obstacle_veto" in decision.active_constraints
    assert "lidar_veto" in decision.active_constraints


def test_stale_lidar_does_not_veto():
    engine, fsm = _engine(FsmState.CHASE_A)
    # Very close but the sample is old (received long ago) -> aged out.
    lidar = RangeState(received_ms=1, fresh=True, distance_cm=1.0)
    decision = engine.tick(_input(now_ms=100000, lidar=lidar))
    assert fsm.state == FsmState.CHASE_A
    assert "obstacle_too_close" not in decision.active_constraints


def test_navigation_drives_goto_speed_and_steer():
    engine, _ = _engine(FsmState.GOTO)
    nav = NavigationState(received_ms=1000, fresh=True, path_correction=0.5, speed_limit=0.6)
    decision = engine.tick(_input(fsm_state=FsmState.GOTO, navigation=nav))
    assert decision.steering == 0.5
    assert abs(decision.speed - 0.6 * MAX_SPEED) < 1e-9
    assert "navigation" in decision.active_constraints


def test_navigation_path_correction_is_clamped():
    engine, _ = _engine(FsmState.CHASE_A)
    nav = NavigationState(received_ms=1000, fresh=True, path_correction=5.0, speed_limit=2.0)
    decision = engine.tick(_input(fsm_state=FsmState.CHASE_A, navigation=nav))
    # Nav2 path_correction still biases steering (clamped)...
    assert decision.steering == 1.0
    # ...but in CHASE_A Nav2 is advisory only: local pursuit owns speed, which is
    # 0 in the V1 shell, so the car holds rather than being driven by Nav2.
    assert decision.speed == 0.0
    assert "nav_advisory" in decision.active_constraints


def test_navigation_chase_a_holds_speed_but_biases_steer():
    engine, _ = _engine(FsmState.CHASE_A)
    nav = NavigationState(received_ms=1000, fresh=True, path_correction=0.3, speed_limit=0.9)
    decision = engine.tick(_input(fsm_state=FsmState.CHASE_A, navigation=nav))
    assert abs(decision.steering - 0.3) < 1e-9
    assert decision.speed == 0.0
    assert "navigation" in decision.active_constraints
    assert "nav_advisory" in decision.active_constraints


def test_stale_navigation_keeps_zero_motion():
    engine, _ = _engine(FsmState.GOTO)
    # Old receipt time -> navigation ages out -> zero motion.
    nav = NavigationState(received_ms=1, fresh=True, path_correction=0.5, speed_limit=0.6)
    decision = engine.tick(_input(now_ms=100000, fsm_state=FsmState.GOTO, navigation=nav))
    assert decision.speed == 0.0
    assert decision.steering == 0.0


def test_navigation_drive_fails_closed_without_usable_obstacle_sensor():
    """CHASE_A/GOTO must not drive on nav constraints when no obstacle sensor
    is fresh + valid (dead/faulted range and lidar)."""
    engine, _ = _engine(FsmState.GOTO)
    nav = NavigationState(received_ms=1000, fresh=True, path_correction=0.5, speed_limit=0.6)
    faulted_range = RangeState(received_ms=1000, fresh=True, distance_cm=None, confidence=0.0)
    decision = engine.tick(
        _input(fsm_state=FsmState.GOTO, navigation=nav, range=faulted_range)
    )
    assert decision.speed == 0.0
    assert decision.steering == 0.0
    assert "obstacle_sensor_unavailable" in decision.active_constraints
