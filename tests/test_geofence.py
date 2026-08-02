"""Car geofence containment and DecisionEngine breach failsafe tests."""

from cat_follow.control.decision_engine import DecisionEngine
from cat_follow.control.fsm import FSM
from cat_follow.control.types import (
    CommandState,
    DecisionInput,
    FsmState,
    GeofenceState,
    HomeState,
    NavigationState,
    OverheadState,
    RangeBackend,
    RangeState,
    ReasonCode,
    SystemState,
    VisionState,
    FSMSnapshot,
)
from cat_follow.navigation.geofence import (
    GeofencePolygon,
    evaluate_geofence,
    point_in_polygon,
)


SQUARE = GeofencePolygon(
    geofence_id="yard-inner-v1",
    vertices_m=((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),
)


def test_point_in_polygon_inside_and_outside():
    assert point_in_polygon(5.0, 5.0, SQUARE.vertices_m)
    assert not point_in_polygon(11.0, 5.0, SQUARE.vertices_m)


def test_evaluate_geofence_confirms_breach():
    state = evaluate_geofence(
        SQUARE,
        pose_x_m=11.0,
        pose_y_m=5.0,
        pose_received_ms=1000,
        now_ms=1000,
    )
    assert state.configured
    assert state.breach_confirmed
    assert state.car_inside is False
    assert state.localization_valid_for_containment


def test_geofence_breach_latches_failsafe():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)
    engine.set_active_target_id("cat-17")
    decision = engine.tick(
        DecisionInput(
            now_ms=1000,
            overhead=OverheadState(),
            home=HomeState(set=True, valid=True),
            vision=VisionState(),
            range=RangeState(
                received_ms=1000, fresh=True, distance_cm=100.0, confidence=1.0
            ),
            lidar=RangeState(
                received_ms=1000,
                fresh=True,
                backend=RangeBackend.LIDAR_C1,
                distance_cm=100.0,
                confidence=1.0,
            ),
            navigation=NavigationState(),
            system=SystemState(),
            fsm=FSMSnapshot(state=FsmState.GETTING_CLOSE),
            command=CommandState(),
            geofence=GeofenceState(
                configured=True,
                car_geofence_id="yard-inner-v1",
                car_inside=False,
                localization_valid_for_containment=True,
                breach_confirmed=True,
                breach_at_ms=1000,
            ),
        )
    )
    assert fsm.state == FsmState.FAILSAFE
    assert decision.brake
    assert "geofence_breach" in decision.active_constraints


def _driving_input(now_ms, geofence, *, home=None):
    return DecisionInput(
        now_ms=now_ms,
        overhead=OverheadState(),
        home=home or HomeState(set=True, valid=True),
        vision=VisionState(),
        range=RangeState(
            received_ms=now_ms, fresh=True, distance_cm=100.0, confidence=1.0
        ),
        lidar=RangeState(
            received_ms=now_ms,
            fresh=True,
            backend=RangeBackend.LIDAR_C1,
            distance_cm=100.0,
            confidence=1.0,
        ),
        navigation=NavigationState(),
        system=SystemState(),
        fsm=FSMSnapshot(state=FsmState.GETTING_CLOSE),
        command=CommandState(),
        geofence=geofence,
    )


def test_unobservable_containment_reports_geofence_reason_not_sensor_hold():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)
    engine.set_active_target_id("cat-17")

    decision = engine.tick(
        _driving_input(
            1000,
            GeofenceState(
                configured=True,
                car_geofence_id="yard-inner-v1",
                localization_valid_for_containment=False,
            ),
        )
    )

    assert fsm.state == FsmState.GETTING_CLOSE
    assert decision.brake
    assert decision.reason == ReasonCode.GEOFENCE_UNOBSERVABLE
    assert "geofence_unobservable" in decision.active_constraints


def test_unobservable_containment_without_safe_return_latches_failsafe():
    fsm = FSM(initial_state=FsmState.GETTING_CLOSE)
    engine = DecisionEngine(fsm)
    engine.set_active_target_id("cat-17")

    decision = engine.tick(
        _driving_input(
            1000,
            GeofenceState(
                configured=True,
                car_geofence_id="yard-inner-v1",
                localization_valid_for_containment=False,
            ),
            home=HomeState(set=False, valid=False),
        )
    )

    assert fsm.state == FsmState.FAILSAFE
    assert decision.reason == ReasonCode.GEOFENCE_UNOBSERVABLE
    assert "geofence_unobservable" in decision.active_constraints
