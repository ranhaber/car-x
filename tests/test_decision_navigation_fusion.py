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


def _input(
    *,
    fsm_state=FsmState.CHASE_A,
    now_ms=1000,
    navigation=None,
    lidar=None,
) -> DecisionInput:
    return DecisionInput(
        now_ms=now_ms,
        overhead=OverheadState(received_ms=now_ms, fresh=True, sequence=1),
        home=HomeState(),
        vision=VisionState(),
        range=RangeState(),
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
    lidar = RangeState(fresh=False, distance_cm=1.0)  # very close but stale
    decision = engine.tick(_input(lidar=lidar))
    assert fsm.state == FsmState.CHASE_A
    assert "obstacle_too_close" not in decision.active_constraints


def test_navigation_drives_goto_speed_and_steer():
    engine, _ = _engine(FsmState.GOTO)
    nav = NavigationState(fresh=True, path_correction=0.5, speed_limit=0.6)
    decision = engine.tick(_input(fsm_state=FsmState.GOTO, navigation=nav))
    assert decision.steering == 0.5
    assert abs(decision.speed - 0.6 * MAX_SPEED) < 1e-9
    assert "navigation" in decision.active_constraints


def test_navigation_path_correction_is_clamped():
    engine, _ = _engine(FsmState.CHASE_A)
    nav = NavigationState(fresh=True, path_correction=5.0, speed_limit=2.0)
    decision = engine.tick(_input(fsm_state=FsmState.CHASE_A, navigation=nav))
    assert decision.steering == 1.0
    assert decision.speed == MAX_SPEED


def test_stale_navigation_keeps_zero_motion():
    engine, _ = _engine(FsmState.GOTO)
    nav = NavigationState(fresh=False, path_correction=0.5, speed_limit=0.6)
    decision = engine.tick(_input(fsm_state=FsmState.GOTO, navigation=nav))
    assert decision.speed == 0.0
    assert decision.steering == 0.0
