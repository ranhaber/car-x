"""Tests for lidar veto + NavigationState fusion in the DecisionEngine."""

from cat_follow.control.decision_engine import (
    MAX_SPEED,
    OBSTACLE_TOO_CLOSE_CM,
    DecisionEngine,
)
from cat_follow.control.fsm import FSM
from cat_follow.control.types import (
    CommandState,
    CarTrackingState,
    DecisionInput,
    FSMSnapshot,
    FsmState,
    HomeState,
    LookDriveMode,
    NavigationState,
    OverheadState,
    RangeBackend,
    RangeState,
    ReasonCode,
    SystemState,
    TrackingObjectState,
    VisionState,
)
from cat_follow.target_config import TargetRuntimeConfig


def _fresh_range(now_ms=1000):
    """A fresh, valid, far ultrasonic reading (usable obstacle sensor)."""
    return RangeState(
        received_ms=now_ms,
        fresh=True,
        distance_cm=100.0,
        confidence=1.0,
    )


def _fresh_lidar(now_ms=1000):
    return RangeState(
        received_ms=now_ms,
        fresh=True,
        backend=RangeBackend.LIDAR_C1,
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
    vision=None,
    home=None,
) -> DecisionInput:
    return DecisionInput(
        now_ms=now_ms,
        overhead=OverheadState(
            received_ms=now_ms,
            fresh=True,
            sequence=1,
            selected_target_id="cat-17",
            car=CarTrackingState(confidence=1.0),
            cat=TrackingObjectState(
                x=300.0,
                confidence=1.0,
                target_id="cat-17",
            ),
        ),
        home=home or HomeState(),
        vision=vision or VisionState(),
        range=range if range is not None else _fresh_range(now_ms),
        navigation=navigation or NavigationState(),
        system=SystemState(),
        fsm=FSMSnapshot(state=fsm_state),
        command=CommandState(),
        lidar=lidar if lidar is not None else _fresh_lidar(now_ms),
    )


def _engine(state=FsmState.CHASE_A, *, config=None):
    fsm = FSM(initial_state=state)
    engine = DecisionEngine(fsm, target_runtime_config=config)
    if state in {FsmState.GETTING_CLOSE, FsmState.SEARCH, FsmState.CHASE}:
        engine.set_active_target_id("cat-17")
    return engine, fsm


def test_lidar_close_triggers_brake_reverse():
    engine, fsm = _engine(FsmState.CHASE_A)
    lidar = RangeState(
        received_ms=1000,
        fresh=True,
        backend=RangeBackend.LIDAR_C1,
        distance_cm=OBSTACLE_TOO_CLOSE_CM - 2.0,
        confidence=1.0,
    )
    decision = engine.tick(_input(lidar=lidar))
    assert fsm.state == FsmState.BRAKE_REVERSE
    assert decision.brake is True
    assert "brake_reverse" in decision.active_constraints


def test_lidar_critical_flag_does_not_override_distance_policy():
    engine, fsm = _engine(FsmState.CHASE_A)
    lidar = RangeState(
        received_ms=1000,
        fresh=True,
        backend=RangeBackend.LIDAR_C1,
        distance_cm=40.0,
        confidence=1.0,
        obstacle_critical=True,
    )
    decision = engine.tick(_input(lidar=lidar))
    assert fsm.state == FsmState.CHASE_A
    assert "obstacle_veto" not in decision.active_constraints


def test_stale_lidar_starts_health_hold():
    engine, fsm = _engine(FsmState.CHASE_A)
    # Very close but the sample is old (received long ago) -> aged out.
    lidar = RangeState(received_ms=1, fresh=True, distance_cm=1.0)
    decision = engine.tick(_input(now_ms=100000, lidar=lidar))
    assert fsm.state == FsmState.CHASE_A
    assert "sensor_health_hold" in decision.active_constraints


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
    assert decision.steering == 1.0
    assert decision.speed == 2.0
    assert "navigation" in decision.active_constraints


def test_navigation_getting_close_drives_speed_and_steer():
    engine, _ = _engine(FsmState.CHASE_A)
    nav = NavigationState(received_ms=1000, fresh=True, path_correction=0.3, speed_limit=0.9)
    decision = engine.tick(_input(fsm_state=FsmState.CHASE_A, navigation=nav))
    assert abs(decision.steering - 0.3) < 1e-9
    assert decision.speed == 0.9
    assert "navigation" in decision.active_constraints


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
    assert "sensor_health_hold" in decision.active_constraints


def test_chase_clamps_camera_request_without_adding_nav_correction():
    # Large vision offset with small path_correction → BODY_STEER (not path turn).
    engine, fsm = _engine(FsmState.CHASE)
    nav = NavigationState(
        received_ms=1000,
        fresh=True,
        authority="NavigationManager",
        healthy=True,
        path_viable=True,
        safe_steering_min=-0.2,
        safe_steering_max=0.3,
        path_correction=0.1,
        speed_limit=0.5,
        envelope_source="costmap_sweep",
    )
    vision = VisionState(
        received_ms=1000,
        fresh=True,
        cat_visible=True,
        associated_target_id="cat-17",
        x_offset_norm=0.8,
        x_offset_px=200.0,
    )
    decision = engine.tick(
        _input(
            fsm_state=FsmState.CHASE,
            navigation=nav,
            vision=vision,
        )
    )
    assert fsm.state == FsmState.CHASE
    assert decision.steering == 0.3
    assert decision.steering != 0.9  # camera + path_correction is forbidden
    assert decision.target_source.value == "cat_local"
    assert decision.look.mode == LookDriveMode.BODY_STEER
    assert "camera_steering_clamped" in decision.active_constraints


def test_chase_look_at_follows_path_not_camera():
    cfg = TargetRuntimeConfig(
        look_mode_dwell_ms=0,
        look_pan_slew_deg_s=1000.0,
        look_n_enter_px=40.0,
        look_n_exit_px=80.0,
        look_path_turn_threshold=0.35,
        look_px_per_deg=10.0,
    )
    engine, fsm = _engine(FsmState.CHASE, config=cfg)
    nav = NavigationState(
        received_ms=1000,
        fresh=True,
        authority="NavigationManager",
        healthy=True,
        path_viable=True,
        safe_steering_min=-0.5,
        safe_steering_max=0.5,
        path_correction=0.12,
        speed_limit=0.5,
        envelope_source="costmap_sweep",
    )
    vision = VisionState(
        received_ms=1000,
        fresh=True,
        cat_visible=True,
        associated_target_id="cat-17",
        x_offset_norm=0.05,
        x_offset_px=16.0,
    )
    decision = engine.tick(
        _input(
            fsm_state=FsmState.CHASE,
            navigation=nav,
            vision=vision,
        )
    )
    assert fsm.state == FsmState.CHASE
    assert decision.look.mode == LookDriveMode.LOOK_AT
    assert abs(decision.steering - 0.12) < 1e-6
    assert "look_at_path_follow" in decision.active_constraints
    assert "camera_steering_clamped" not in decision.active_constraints


def test_chase_hold_from_look_drive_zeros_speed():
    """HOLD from look/drive (unusable envelope) must not keep chase speed."""
    engine, fsm = _engine(FsmState.CHASE)
    nav = NavigationState(
        received_ms=1000,
        fresh=True,
        authority="NavigationManager",
        healthy=True,
        path_viable=True,
        safe_steering_min=0.4,
        safe_steering_max=-0.4,  # inverted → look_drive HOLD
        path_correction=0.2,
        speed_limit=0.5,
        envelope_source="costmap_sweep",
    )
    vision = VisionState(
        received_ms=1000,
        fresh=True,
        cat_visible=True,
        associated_target_id="cat-17",
        x_offset_norm=0.4,
        x_offset_px=80.0,
    )
    decision = engine.tick(
        _input(
            fsm_state=FsmState.CHASE,
            navigation=nav,
            vision=vision,
        )
    )
    assert fsm.state == FsmState.CHASE
    assert decision.speed == 0.0
    assert decision.brake is True
    assert decision.look.mode == LookDriveMode.HOLD
    assert "look_drive_hold" in decision.active_constraints
    assert decision.reason == ReasonCode.NAVIGATION_PATH_BLOCKED
    assert decision.look.reason == "envelope_unusable"


def test_chase_pan_reset_timeout_uses_look_drive_hold_reason():
    """Pan-reset timeout HOLD is LOOK_DRIVE_HOLD, not path-blocked."""
    cfg = TargetRuntimeConfig(
        look_pan_reset_timeout_ms=50,
        look_pan_slew_deg_s=1.0,
        look_control_period_ms=20,
        look_mode_dwell_ms=0,
        look_n_enter_px=5.0,
        look_n_exit_px=10.0,
        look_px_per_deg=1.0,
    )
    engine, fsm = _engine(FsmState.CHASE, config=cfg)
    nav = NavigationState(
        received_ms=1000,
        fresh=True,
        authority="NavigationManager",
        healthy=True,
        path_viable=True,
        safe_steering_min=-0.2,
        safe_steering_max=0.5,
        path_correction=0.0,
        speed_limit=0.5,
        envelope_source="costmap_sweep",
    )
    vision = VisionState(
        received_ms=1000,
        fresh=True,
        cat_visible=True,
        associated_target_id="cat-17",
        x_offset_norm=0.8,
        x_offset_px=200.0,
    )
    engine._look_drive._pan_deg = 40.0
    first = engine.tick(
        _input(
            fsm_state=FsmState.CHASE,
            navigation=nav,
            vision=vision,
            now_ms=1000,
        )
    )
    assert first.look.mode == LookDriveMode.PAN_RESET
    timed_out = engine.tick(
        _input(
            fsm_state=FsmState.CHASE,
            navigation=nav,
            vision=vision,
            now_ms=1100,
        )
    )
    assert fsm.state == FsmState.CHASE
    assert timed_out.look.mode == LookDriveMode.HOLD
    assert timed_out.speed == 0.0
    assert timed_out.brake is True
    assert timed_out.reason == ReasonCode.LOOK_DRIVE_HOLD
    assert "look_pan_reset_timeout" in timed_out.active_constraints


def test_brake_reverse_emits_calibrated_forward_look():
    cfg = TargetRuntimeConfig(
        look_pan_forward_deg=8.0,
        look_pan_slew_deg_s=1000.0,
        brake_reverse_settle_ms=50,
        brake_reverse_duration_sec=1.0,
    )
    engine, fsm = _engine(FsmState.CHASE, config=cfg)
    lidar = RangeState(
        received_ms=1000,
        fresh=True,
        backend=RangeBackend.LIDAR_C1,
        distance_cm=OBSTACLE_TOO_CLOSE_CM - 2.0,
        confidence=1.0,
    )
    d1 = engine.tick(_input(fsm_state=FsmState.CHASE, lidar=lidar, now_ms=1000))
    assert fsm.state == FsmState.BRAKE_REVERSE
    assert abs(d1.look.pan_deg - 8.0) < 1e-6
    # STOP_ENTRY -> CENTER
    engine.tick(_input(fsm_state=FsmState.BRAKE_REVERSE, lidar=lidar, now_ms=1100))
    # CENTER -> SETTLE, then settle elapsed -> REVERSE motion
    d_rev = engine.tick(
        _input(fsm_state=FsmState.BRAKE_REVERSE, lidar=lidar, now_ms=1200)
    )
    assert d_rev.brake is False
    assert d_rev.speed < 0.0
    assert abs(d_rev.look.pan_deg - 8.0) < 1e-6
    assert d_rev.look.pan_forward_deg == 8.0


def test_chase_stops_when_navigation_path_is_not_viable():
    engine, _ = _engine(FsmState.CHASE)
    nav = NavigationState(
        received_ms=1000,
        fresh=True,
        authority="NavigationManager",
        healthy=True,
        path_viable=False,
        speed_limit=0.5,
    )
    vision = VisionState(
        received_ms=1000,
        fresh=True,
        cat_visible=True,
        associated_target_id="cat-17",
    )
    decision = engine.tick(
        _input(
            fsm_state=FsmState.CHASE,
            navigation=nav,
            vision=vision,
        )
    )
    assert decision.speed == 0.0
    assert decision.brake
    assert decision.reason.value == "navigation_path_blocked"


def test_correlated_goto_completion_enters_idle():
    engine, fsm = _engine(FsmState.GOTO)
    nav = NavigationState(completion_qualified=True)
    decision = engine.tick(
        _input(fsm_state=FsmState.GOTO, navigation=nav)
    )
    assert fsm.state == FsmState.IDLE
    assert decision.speed == 0.0


def test_exhausted_chase_navigation_returns_home_when_safe():
    engine, fsm = _engine(FsmState.GETTING_CLOSE)
    nav = NavigationState(failures_exhausted=True)
    engine.tick(
        _input(
            fsm_state=FsmState.GETTING_CLOSE,
            navigation=nav,
            home=HomeState(set=True),
        )
    )
    assert fsm.state == FsmState.RETURN_HOME


def test_exhausted_return_home_navigation_enters_failsafe():
    engine, fsm = _engine(FsmState.RETURN_HOME)
    nav = NavigationState(failures_exhausted=True)
    decision = engine.tick(
        _input(fsm_state=FsmState.RETURN_HOME, navigation=nav)
    )
    assert fsm.state == FsmState.FAILSAFE
    assert decision.brake


def test_exhausted_getting_close_without_home_enters_failsafe():
    engine, fsm = _engine(FsmState.GETTING_CLOSE)
    nav = NavigationState(failures_exhausted=True)
    decision = engine.tick(
        _input(
            fsm_state=FsmState.GETTING_CLOSE,
            navigation=nav,
            home=HomeState(set=False),
        )
    )
    assert fsm.state == FsmState.FAILSAFE
    assert decision.brake
    assert "safe_return_unavailable" in decision.active_constraints


def test_exhausted_goto_enters_idle():
    engine, fsm = _engine(FsmState.GOTO)
    nav = NavigationState(failures_exhausted=True)
    decision = engine.tick(
        _input(fsm_state=FsmState.GOTO, navigation=nav)
    )
    assert fsm.state == FsmState.IDLE
    assert decision.speed == 0.0
    assert decision.brake


def test_qualified_return_home_completion_enters_home():
    engine, fsm = _engine(FsmState.RETURN_HOME)
    nav = NavigationState(completion_qualified=True)
    decision = engine.tick(
        _input(fsm_state=FsmState.RETURN_HOME, navigation=nav)
    )
    assert fsm.state == FsmState.HOME
    assert decision.speed == 0.0


def test_exhausted_chase_return_home_clears_chase_target_publication():
    engine, fsm = _engine(FsmState.GETTING_CLOSE)
    engine.tick(_input(fsm_state=FsmState.GETTING_CLOSE, home=HomeState(set=True)))

    engine.tick(
        _input(
            fsm_state=FsmState.GETTING_CLOSE,
            navigation=NavigationState(failures_exhausted=True),
            home=HomeState(set=True),
        )
    )
    assert fsm.state == FsmState.RETURN_HOME

    driving = engine.tick(
        _input(
            fsm_state=FsmState.RETURN_HOME,
            navigation=NavigationState(
                received_ms=1000, fresh=True, path_correction=0.2, speed_limit=0.4
            ),
            home=HomeState(set=True),
        )
    )
    assert driving.target_source.value == "home"
    assert driving.target_x is None
    assert driving.target_y is None


def test_exhausted_chase_failsafes_when_return_transition_is_rejected(monkeypatch):
    engine, fsm = _engine(FsmState.GETTING_CLOSE)

    def _reject(event, **kwargs):
        from cat_follow.control.fsm import TransitionResult

        return TransitionResult(
            accepted=False,
            from_state=fsm.state,
            to_state=fsm.state,
            rejected_descriptor="forced_rejection",
        )

    monkeypatch.setattr(fsm, "apply", _reject)
    decision = engine.tick(
        _input(
            fsm_state=FsmState.GETTING_CLOSE,
            navigation=NavigationState(failures_exhausted=True),
            home=HomeState(set=True),
        )
    )
    assert decision.brake
    assert "return_home_transition_rejected" in decision.active_constraints
    assert "safe_return_unavailable" not in decision.active_constraints


def test_nav_success_without_completion_qualification_stays_in_goto():
    engine, fsm = _engine(FsmState.GOTO)
    nav = NavigationState(
        received_ms=1000,
        fresh=True,
        path_correction=0.4,
        speed_limit=0.5,
        completion_qualified=False,
    )
    decision = engine.tick(_input(fsm_state=FsmState.GOTO, navigation=nav))
    assert fsm.state == FsmState.GOTO
    assert decision.steering == 0.4
