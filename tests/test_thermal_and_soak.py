"""Critical thermal return policy (SAFE-15..18) and host soak harness."""

from cat_follow.control.decision_engine import DecisionEngine
from cat_follow.control.fsm import FSM
from cat_follow.control.types import (
    DecisionInput,
    FSMSnapshot,
    FsmState,
    HomeState,
    NavigationState,
    OverheadState,
    RangeBackend,
    RangeState,
    ReasonCode,
    SystemState,
    ThermalState,
    VisionState,
    CommandState,
)
from cat_follow.target_config import TargetRuntimeConfig
from tests.test_comms_manager_helpers import durable_home


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
    state=FsmState.GOTO,
    ultrasonic_cm=100.0,
    lidar_cm=100.0,
    ultrasonic_healthy=True,
    lidar_healthy=True,
    thermal=ThermalState.NORMAL,
    home=None,
    navigation=None,
):
    return DecisionInput(
        now_ms=now_ms,
        overhead=OverheadState(received_ms=now_ms, fresh=True, sequence=1),
        home=home or HomeState(),
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
        navigation=navigation or NavigationState(received_ms=now_ms, fresh=True),
        system=SystemState(thermal_state=thermal),
        fsm=FSMSnapshot(state=state),
        command=CommandState(),
    )


def test_safe_15_critical_thermal_returns_home_when_safe():
    fsm = FSM(initial_state=FsmState.GOTO)
    engine = DecisionEngine(fsm)
    home = durable_home()
    engine.freeze_mission_home(home)

    out = engine.tick(
        _input(
            1000,
            state=FsmState.GOTO,
            thermal=ThermalState.CRITICAL,
            home=home,
            navigation=NavigationState(
                received_ms=1000,
                fresh=True,
                healthy=True,
                path_viable=True,
            ),
        )
    )
    assert fsm.state == FsmState.RETURN_HOME
    assert out.reason == ReasonCode.THERMAL_CRITICAL
    assert "thermal_critical" in out.active_constraints


def test_safe_15_critical_thermal_failsafe_when_unsafe_flag():
    fsm = FSM(initial_state=FsmState.CHASE)
    engine = DecisionEngine(
        fsm,
        target_runtime_config=TargetRuntimeConfig(thermal_critical_unsafe=True),
    )
    out = engine.tick(
        _input(1000, state=FsmState.CHASE, thermal=ThermalState.CRITICAL)
    )
    assert fsm.state == FsmState.FAILSAFE
    assert out.reason == ReasonCode.THERMAL_CRITICAL


def test_safe_16_return_home_continues_under_critical_cap():
    fsm = FSM(initial_state=FsmState.RETURN_HOME)
    engine = DecisionEngine(fsm)
    home = durable_home()
    engine.freeze_mission_home(home)
    out = engine.tick(
        _input(
            1000,
            state=FsmState.RETURN_HOME,
            thermal=ThermalState.CRITICAL,
            home=home,
            navigation=NavigationState(
                received_ms=1000,
                fresh=True,
                healthy=True,
                path_viable=True,
                speed_limit=1.0,
                speed_cap_mps=0.30,
                heading_valid=True,
            ),
        )
    )
    assert fsm.state == FsmState.RETURN_HOME
    assert "thermal_critical_return_cap" in out.active_constraints
    assert out.speed <= (0.08 / 0.30) + 1e-9


def test_safe_17_home_idle_remain_stopped_on_critical_thermal():
    for state in (FsmState.HOME, FsmState.IDLE):
        fsm = FSM(initial_state=state)
        engine = DecisionEngine(fsm)
        out = engine.tick(
            _input(1000, state=state, thermal=ThermalState.CRITICAL)
        )
        assert out.speed == 0.0
        assert out.brake is True
        assert fsm.state == state


def test_safe_18_brake_reverse_thermal_replaces_with_return_or_fail():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)
    home = durable_home()
    engine.freeze_mission_home(home)
    engine.tick(_input(1000, ultrasonic_cm=14.0, home=home))
    assert fsm.state == FsmState.BRAKE_REVERSE
    out = engine.tick(
        _input(
            1100,
            state=FsmState.BRAKE_REVERSE,
            thermal=ThermalState.CRITICAL,
            home=home,
            navigation=NavigationState(
                received_ms=1100, fresh=True, healthy=True, path_viable=True
            ),
        )
    )
    assert fsm.state == FsmState.RETURN_HOME
    assert out.reason == ReasonCode.THERMAL_CRITICAL


def test_host_soak_sensor_drop_and_thermal_never_drives_unhealthy():
    """Long host soak: intermittent dual-sensor loss + thermal spikes."""

    fsm = FSM(initial_state=FsmState.GOTO)
    engine = DecisionEngine(fsm)
    home = durable_home()
    engine.freeze_mission_home(home)
    now = 1000
    drove_while_unhealthy = False
    for i in range(250):
        now += 20
        ultra_ok = (i % 17) != 0
        lidar_ok = (i % 23) != 0
        thermal = (
            ThermalState.CRITICAL if i in {80, 81, 82} else ThermalState.NORMAL
        )
        out = engine.tick(
            _input(
                now,
                state=fsm.state,
                ultrasonic_healthy=ultra_ok,
                lidar_healthy=lidar_ok,
                thermal=thermal,
                home=home,
                navigation=NavigationState(
                    received_ms=now,
                    fresh=True,
                    healthy=True,
                    path_viable=True,
                    speed_limit=0.5,
                ),
            )
        )
        health = engine.dual_sensor_health
        assert health is not None
        if not (ultra_ok and lidar_ok) and fsm.state not in {
            FsmState.HOME,
            FsmState.IDLE,
            FsmState.FAILSAFE,
            FsmState.RETURN_HOME,
        }:
            if out.speed != 0.0:
                drove_while_unhealthy = True
        if health.hold_active:
            assert out.speed == 0.0
    assert drove_while_unhealthy is False
    assert fsm.state in {
        FsmState.GOTO,
        FsmState.RETURN_HOME,
        FsmState.FAILSAFE,
        FsmState.IDLE,
        FsmState.HOME,
    }
