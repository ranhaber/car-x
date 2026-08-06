"""Unit tests for look/drive mode selection."""

from cat_follow.control.look_drive import LookDriveController
from cat_follow.control.types import (
    FsmState,
    LookDriveMode,
    NavigationState,
    VisionState,
)
from cat_follow.target_config import TargetRuntimeConfig


def _nav(**kwargs):
    base = dict(
        path_viable=True,
        safe_steering_min=-0.5,
        safe_steering_max=0.5,
        path_correction=0.0,
        envelope_source="costmap_sweep",
    )
    base.update(kwargs)
    return NavigationState(**base)


def _vision(**kwargs):
    base = dict(
        fresh=True,
        cat_visible=True,
        x_offset_norm=0.1,
        x_offset_px=16.0,
        associated_target_id="cat-17",
        association_ambiguous=False,
    )
    base.update(kwargs)
    return VisionState(**base)


def test_look_at_uses_path_not_camera_steer():
    cfg = TargetRuntimeConfig(
        look_mode_dwell_ms=0,
        look_pan_slew_deg_s=1000.0,
        look_n_enter_px=40.0,
        look_n_exit_px=80.0,
    )
    ctl = LookDriveController(cfg, pan_forward_deg=0.0)
    d = ctl.tick(
        now_ms=1000,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=20.0, x_offset_norm=0.2),
        navigation=_nav(path_correction=0.15),
        hold_motion=False,
        path_correction=0.15,
        camera_request=0.9,
    )
    assert d.mode == LookDriveMode.LOOK_AT
    assert abs(d.steering - 0.15) < 1e-6
    assert d.steering != 0.9


def test_look_at_hysteresis_keeps_until_n_exit():
    cfg = TargetRuntimeConfig(
        look_mode_dwell_ms=0,
        look_pan_slew_deg_s=1000.0,
        look_n_enter_px=20.0,
        look_n_exit_px=60.0,
        look_px_per_deg=10.0,
    )
    ctl = LookDriveController(cfg, pan_forward_deg=0.0)
    d1 = ctl.tick(
        now_ms=1000,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=10.0),
        navigation=_nav(),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.0,
    )
    assert d1.mode == LookDriveMode.LOOK_AT
    # Between N_enter and N_exit: stay in LOOK_AT.
    d2 = ctl.tick(
        now_ms=1500,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=40.0),
        navigation=_nav(),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.0,
    )
    assert d2.mode == LookDriveMode.LOOK_AT
    # At/above N_exit: leave LOOK_AT.
    d3 = ctl.tick(
        now_ms=2000,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=60.0),
        navigation=_nav(),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.0,
    )
    assert d3.mode in {LookDriveMode.PAN_RESET, LookDriveMode.BODY_STEER}


def test_path_needs_turn_prefers_path_follow_after_look_at():
    cfg = TargetRuntimeConfig(
        look_mode_dwell_ms=0,
        look_pan_slew_deg_s=1000.0,
        look_n_enter_px=40.0,
        look_n_exit_px=80.0,
        look_path_turn_threshold=0.2,
        look_px_per_deg=10.0,
    )
    ctl = LookDriveController(cfg, pan_forward_deg=0.0)
    d1 = ctl.tick(
        now_ms=1000,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=10.0),
        navigation=_nav(path_correction=0.0),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.1,
    )
    assert d1.mode == LookDriveMode.LOOK_AT
    d2 = ctl.tick(
        now_ms=1500,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=10.0),
        navigation=_nav(path_correction=0.5),
        hold_motion=False,
        path_correction=0.5,
        camera_request=0.1,
    )
    assert d2.mode == LookDriveMode.PAN_RESET
    # Finish reset immediately (pan already near forward).
    ctl._pan_deg = 0.0
    d3 = ctl.tick(
        now_ms=1600,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=10.0),
        navigation=_nav(path_correction=0.5),
        hold_motion=False,
        path_correction=0.5,
        camera_request=0.1,
    )
    assert d3.mode == LookDriveMode.PATH_FOLLOW


def test_body_steer_requires_pan_forward_and_clamps():
    cfg = TargetRuntimeConfig(
        look_mode_dwell_ms=0,
        look_pan_slew_deg_s=1000.0,
        look_path_turn_threshold=0.05,
        look_px_per_deg=1.0,
        look_n_enter_px=5.0,
        look_n_exit_px=10.0,
    )
    ctl = LookDriveController(cfg, pan_forward_deg=0.0)
    d = ctl.tick(
        now_ms=1000,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=200.0, x_offset_norm=0.8),
        navigation=_nav(
            path_correction=0.0,
            safe_steering_min=-0.2,
            safe_steering_max=0.3,
        ),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.8,
    )
    assert d.mode == LookDriveMode.BODY_STEER
    assert abs(d.steering - 0.3) < 1e-6
    assert d.look.pan_deg == 0.0


def test_pan_reset_blocks_vision_steer():
    cfg = TargetRuntimeConfig(
        look_mode_dwell_ms=0,
        look_pan_slew_deg_s=30.0,
        look_control_period_ms=20,
        look_pan_reset_timeout_ms=5000,
        look_n_enter_px=5.0,
        look_n_exit_px=10.0,
        look_px_per_deg=1.0,
    )
    ctl = LookDriveController(cfg, pan_forward_deg=0.0)
    ctl._pan_deg = 40.0
    d = ctl.tick(
        now_ms=1000,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=200.0, x_offset_norm=0.8),
        navigation=_nav(
            path_correction=0.0,
            safe_steering_min=-0.2,
            safe_steering_max=0.3,
        ),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.8,
    )
    assert d.mode == LookDriveMode.PAN_RESET
    assert d.freeze_chassis_steer is True
    assert d.look.pan_deg < 40.0


def test_pan_reset_timeout_zeros_held_steer():
    cfg = TargetRuntimeConfig(
        look_mode_dwell_ms=0,
        look_pan_slew_deg_s=1.0,
        look_control_period_ms=20,
        look_pan_reset_timeout_ms=100,
        look_n_enter_px=5.0,
        look_n_exit_px=10.0,
        look_px_per_deg=1.0,
    )
    ctl = LookDriveController(cfg, pan_forward_deg=0.0)
    ctl._pan_deg = 40.0
    d1 = ctl.tick(
        now_ms=1000,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=200.0, x_offset_norm=0.8),
        navigation=_nav(
            path_correction=0.0,
            safe_steering_min=-0.2,
            safe_steering_max=0.3,
        ),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.8,
    )
    assert d1.mode == LookDriveMode.PAN_RESET
    # Expire PAN_RESET while pan still off-forward.
    d2 = ctl.tick(
        now_ms=1200,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=200.0, x_offset_norm=0.8),
        navigation=_nav(
            path_correction=0.0,
            safe_steering_min=-0.2,
            safe_steering_max=0.3,
        ),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.8,
    )
    assert d2.mode == LookDriveMode.HOLD
    assert d2.steering == 0.0
    assert d2.constraint == "look_pan_reset_timeout"


def test_pan_toward_error_increases_pan_for_cat_on_right():
    cfg = TargetRuntimeConfig(
        look_mode_dwell_ms=0,
        look_pan_slew_deg_s=1000.0,
        look_n_enter_px=80.0,
        look_n_exit_px=120.0,
        look_px_per_deg=10.0,
        look_center_deadband_px=0.0,
    )
    ctl = LookDriveController(cfg, pan_forward_deg=0.0)
    d = ctl.tick(
        now_ms=1000,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=20.0, x_offset_norm=0.1),
        navigation=_nav(path_correction=0.0),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.0,
    )
    assert d.mode == LookDriveMode.LOOK_AT
    # Cat right → pan_deg increases (Picarx pan-right convention).
    assert d.look.pan_deg > 0.0


def test_hold_forces_zero_and_forward_pan():
    cfg = TargetRuntimeConfig(look_pan_slew_deg_s=1000.0)
    ctl = LookDriveController(cfg, pan_forward_deg=8.0)
    ctl._pan_deg = 25.0
    d = ctl.tick(
        now_ms=1000,
        fsm_state=FsmState.CHASE,
        vision=_vision(),
        navigation=_nav(path_viable=False),
        hold_motion=False,
        path_correction=0.2,
        camera_request=0.5,
    )
    assert d.mode == LookDriveMode.HOLD
    assert d.steering == 0.0
    assert abs(d.look.pan_deg - 8.0) < 1e-6


def test_non_chase_forces_path_follow_forward_pan():
    cfg = TargetRuntimeConfig(look_pan_slew_deg_s=1000.0)
    ctl = LookDriveController(cfg, pan_forward_deg=8.0)
    d = ctl.tick(
        now_ms=1000,
        fsm_state=FsmState.GETTING_CLOSE,
        vision=_vision(),
        navigation=_nav(path_correction=0.25),
        hold_motion=False,
        path_correction=0.25,
        camera_request=0.9,
    )
    assert d.mode == LookDriveMode.PATH_FOLLOW
    assert abs(d.steering - 0.25) < 1e-6
    assert abs(d.look.pan_deg - 8.0) < 1e-6


def test_force_forward_snaps_without_slew():
    cfg = TargetRuntimeConfig(look_pan_slew_deg_s=1.0)
    ctl = LookDriveController(cfg, pan_forward_deg=8.0)
    ctl._pan_deg = 40.0
    look = ctl.force_forward(
        reason="envelope_invalid", pixel_error_px=1.0, now_ms=5000
    )
    assert look.pan_deg == 8.0
    assert ctl.pan_deg == 8.0
    assert look.mode == LookDriveMode.HOLD
    assert ctl._mode_entered_ms == 5000


def test_hold_motion_zeros_stale_held_steer():
    """hold_motion HOLD must clear LookDriveDecision.steering after body steer."""
    cfg = TargetRuntimeConfig(
        look_mode_dwell_ms=0,
        look_n_enter_px=5.0,
        look_n_exit_px=10.0,
        look_px_per_deg=1.0,
    )
    ctl = LookDriveController(cfg, pan_forward_deg=0.0)
    d_body = ctl.tick(
        now_ms=1000,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=200.0, x_offset_norm=0.8),
        navigation=_nav(
            path_correction=0.0,
            safe_steering_min=-0.2,
            safe_steering_max=0.55,
        ),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.55,
    )
    assert d_body.mode == LookDriveMode.BODY_STEER
    assert abs(d_body.steering - 0.55) < 1e-6
    d_hold = ctl.tick(
        now_ms=1020,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=200.0, x_offset_norm=0.8),
        navigation=_nav(
            path_correction=0.0,
            safe_steering_min=-0.2,
            safe_steering_max=0.55,
        ),
        hold_motion=True,
        path_correction=0.0,
        camera_request=0.55,
    )
    assert d_hold.mode == LookDriveMode.HOLD
    assert d_hold.steering == 0.0
    assert d_hold.look.reason == "hold_motion"


def test_envelope_unusable_zeros_stale_held_steer():
    """envelope_unusable HOLD must not return pre-HOLD camera steer."""
    cfg = TargetRuntimeConfig(
        look_mode_dwell_ms=0,
        look_n_enter_px=5.0,
        look_n_exit_px=10.0,
        look_px_per_deg=1.0,
    )
    ctl = LookDriveController(cfg, pan_forward_deg=0.0)
    d_body = ctl.tick(
        now_ms=1000,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=200.0, x_offset_norm=0.8),
        navigation=_nav(
            path_correction=0.0,
            safe_steering_min=-0.2,
            safe_steering_max=0.55,
        ),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.55,
    )
    assert d_body.mode == LookDriveMode.BODY_STEER
    assert abs(d_body.steering - 0.55) < 1e-6
    d_hold = ctl.tick(
        now_ms=1020,
        fsm_state=FsmState.CHASE,
        vision=_vision(x_offset_px=200.0, x_offset_norm=0.8),
        navigation=_nav(
            path_correction=0.0,
            path_viable=False,
            safe_steering_min=-0.2,
            safe_steering_max=0.55,
        ),
        hold_motion=False,
        path_correction=0.0,
        camera_request=0.55,
    )
    assert d_hold.mode == LookDriveMode.HOLD
    assert d_hold.steering == 0.0
    assert d_hold.look.reason == "envelope_unusable"
