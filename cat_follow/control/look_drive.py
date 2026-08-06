"""Look/drive mode selection for CHASE fusion.

Pure policy module: no ROS or hardware.  DecisionEngine owns the instance and
applies the resulting chassis steer + LookCommand atomically each tick.

See ``cat_follow/docs/Look_Drive_Path_Design.md``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from cat_follow.control.types import (
    FsmState,
    LookCommand,
    LookDriveMode,
    NavigationState,
    VisionState,
)
from cat_follow.target_config import TargetRuntimeConfig


# States that force pan to calibrated forward and never use vision chassis.
_FORCE_FORWARD_STATES = frozenset(
    {
        FsmState.HOME,
        FsmState.IDLE,
        FsmState.GETTING_CLOSE,
        FsmState.SEARCH,
        FsmState.GOTO,
        FsmState.RETURN_HOME,
        FsmState.BRAKE_REVERSE,
        FsmState.FAILSAFE,
    }
)


@dataclass(frozen=True)
class LookDriveDecision:
    """Chassis steer policy plus pan command for one tick."""

    mode: LookDriveMode
    steering: float
    freeze_chassis_steer: bool
    look: LookCommand
    constraint: Optional[str] = None


def pixel_error_px(
    vision: VisionState, *, half_frame_width_px: float
) -> float:
    """Horizontal pixel error; positive means cat is right of center."""

    if vision.x_offset_px is not None:
        return float(vision.x_offset_px)
    return float(vision.x_offset_norm) * float(half_frame_width_px)


def pan_can_center(
    error_px: float,
    *,
    pan_deg: float,
    pan_min_deg: float,
    pan_max_deg: float,
    px_per_deg: float,
) -> bool:
    """True when remaining pan travel can null the pixel error."""

    if px_per_deg <= 0.0:
        return False
    # Positive error (cat right) → need higher pan_deg (Picarx pan-right).
    needed = pan_deg + (error_px / px_per_deg)
    return pan_min_deg <= needed <= pan_max_deg


class LookDriveController:
    """Stateful look/drive selector with hysteresis and PAN_RESET gating."""

    def __init__(
        self,
        config: Optional[TargetRuntimeConfig] = None,
        *,
        pan_forward_deg: float = 0.0,
        pan_min_deg: float = -90.0,
        pan_max_deg: float = 90.0,
    ) -> None:
        self._config = config or TargetRuntimeConfig()
        self._pan_forward_deg = float(pan_forward_deg)
        self._pan_min_deg = float(pan_min_deg)
        self._pan_max_deg = float(pan_max_deg)
        self._mode = LookDriveMode.PATH_FOLLOW
        self._mode_entered_ms = 0
        self._pan_deg = float(pan_forward_deg)
        self._held_steer = 0.0
        self._pan_reset_started_ms: Optional[int] = None
        self._pending_after_reset = LookDriveMode.BODY_STEER

    @property
    def mode(self) -> LookDriveMode:
        return self._mode

    @property
    def pan_deg(self) -> float:
        return self._pan_deg

    def tick(
        self,
        *,
        now_ms: int,
        fsm_state: FsmState,
        vision: VisionState,
        navigation: NavigationState,
        hold_motion: bool,
        path_correction: float,
        camera_request: float,
    ) -> LookDriveDecision:
        cfg = self._config
        error_px = pixel_error_px(
            vision, half_frame_width_px=cfg.look_frame_half_width_px
        )
        abs_err = abs(error_px)
        pan_at_forward = (
            abs(self._pan_deg - self._pan_forward_deg)
            <= cfg.look_pan_forward_deadband_deg
        )
        envelope_ok = (
            navigation.path_viable
            and navigation.safe_steering_min
            <= navigation.safe_steering_max
        )
        track_ok = (
            vision.fresh
            and vision.cat_visible
            and not vision.association_ambiguous
            and vision.associated_target_id is not None
        )
        can_center = pan_can_center(
            error_px,
            pan_deg=self._pan_deg,
            pan_min_deg=self._pan_min_deg,
            pan_max_deg=self._pan_max_deg,
            px_per_deg=cfg.look_px_per_deg,
        )
        path_needs_turn = abs(path_correction) > cfg.look_path_turn_threshold

        if hold_motion:
            return self._fail_closed_hold(
                now_ms=now_ms,
                error_px=error_px,
                camera_request=camera_request,
                reason="hold_motion",
                constraint="look_drive_hold",
            )

        if fsm_state in _FORCE_FORWARD_STATES:
            return self._enter(
                LookDriveMode.PATH_FOLLOW,
                now_ms=now_ms,
                steering=path_correction,
                freeze=False,
                pan_target=self._pan_forward_deg,
                error_px=error_px,
                camera_request=camera_request,
                reason=f"fsm_{fsm_state.value}",
            )

        if fsm_state != FsmState.CHASE:
            return self._enter(
                LookDriveMode.PATH_FOLLOW,
                now_ms=now_ms,
                steering=path_correction,
                freeze=False,
                pan_target=self._pan_forward_deg,
                error_px=error_px,
                camera_request=camera_request,
                reason="non_chase",
            )

        # CHASE requires a usable envelope.
        if not envelope_ok:
            return self._fail_closed_hold(
                now_ms=now_ms,
                error_px=error_px,
                camera_request=camera_request,
                reason="envelope_unusable",
                constraint="look_drive_hold",
            )

        # CHASE
        if self._mode == LookDriveMode.PAN_RESET:
            return self._tick_pan_reset(
                now_ms=now_ms,
                error_px=error_px,
                camera_request=camera_request,
                path_correction=path_correction,
                pan_at_forward=pan_at_forward,
                track_ok=track_ok,
                navigation=navigation,
            )

        look_eligible = track_ok and can_center and not path_needs_turn
        # Hysteresis: enter only inside N_enter; keep until N_exit.
        look_at_enter = look_eligible and abs_err <= cfg.look_n_enter_px
        look_at_keep = look_eligible and abs_err < cfg.look_n_exit_px
        look_at_exit = not look_at_keep

        if self._mode == LookDriveMode.LOOK_AT:
            if look_at_exit and self._dwell_elapsed(now_ms):
                # Path-required turns prefer PATH_FOLLOW after reset; body
                # vision steer only when the track is still usable for chase.
                if path_needs_turn or not track_ok:
                    self._pending_after_reset = LookDriveMode.PATH_FOLLOW
                else:
                    self._pending_after_reset = LookDriveMode.BODY_STEER
                return self._start_pan_reset(
                    now_ms=now_ms,
                    error_px=error_px,
                    camera_request=camera_request,
                    held_steer=path_correction,
                    reason="leave_look_at",
                )
            pan_target = self._pan_toward_error(error_px, now_ms)
            return self._enter(
                LookDriveMode.LOOK_AT,
                now_ms=now_ms,
                steering=clamp_path_steer(path_correction, navigation),
                freeze=False,
                pan_target=pan_target,
                error_px=error_px,
                camera_request=camera_request,
                reason="look_at",
                constraint="look_at_path_follow",
            )

        if self._mode == LookDriveMode.BODY_STEER:
            if look_at_enter and self._dwell_elapsed(now_ms) and pan_at_forward:
                pan_target = self._pan_toward_error(error_px, now_ms)
                return self._enter(
                    LookDriveMode.LOOK_AT,
                    now_ms=now_ms,
                    steering=clamp_path_steer(path_correction, navigation),
                    freeze=False,
                    pan_target=pan_target,
                    error_px=error_px,
                    camera_request=camera_request,
                    reason="enter_look_at",
                    constraint="look_at_path_follow",
                )
            if not pan_at_forward:
                self._pending_after_reset = LookDriveMode.BODY_STEER
                return self._start_pan_reset(
                    now_ms=now_ms,
                    error_px=error_px,
                    camera_request=camera_request,
                    held_steer=self._held_steer,
                    reason="body_steer_pan_gate",
                )
            safe_min = navigation.safe_steering_min
            safe_max = navigation.safe_steering_max
            steer = max(safe_min, min(safe_max, camera_request))
            self._held_steer = steer
            return self._enter(
                LookDriveMode.BODY_STEER,
                now_ms=now_ms,
                steering=steer,
                freeze=False,
                pan_target=self._pan_forward_deg,
                error_px=error_px,
                camera_request=camera_request,
                reason="body_steer",
                constraint="camera_steering_clamped",
            )

        # PATH_FOLLOW / HOLD recovery into CHASE
        if look_at_enter and pan_at_forward:
            pan_target = self._pan_toward_error(error_px, now_ms)
            return self._enter(
                LookDriveMode.LOOK_AT,
                now_ms=now_ms,
                steering=clamp_path_steer(path_correction, navigation),
                freeze=False,
                pan_target=pan_target,
                error_px=error_px,
                camera_request=camera_request,
                reason="enter_look_at",
                constraint="look_at_path_follow",
            )

        if track_ok and (
            not can_center
            or abs_err > cfg.look_n_enter_px
            or path_needs_turn
        ):
            if path_needs_turn and can_center and abs_err <= cfg.look_n_enter_px:
                # Path turn with already-centered track: follow path, no body
                # vision steer.
                return self._enter(
                    LookDriveMode.PATH_FOLLOW,
                    now_ms=now_ms,
                    steering=clamp_path_steer(path_correction, navigation),
                    freeze=False,
                    pan_target=self._pan_forward_deg,
                    error_px=error_px,
                    camera_request=camera_request,
                    reason="path_needs_turn",
                )
            if not pan_at_forward:
                self._pending_after_reset = (
                    LookDriveMode.PATH_FOLLOW
                    if path_needs_turn
                    else LookDriveMode.BODY_STEER
                )
                return self._start_pan_reset(
                    now_ms=now_ms,
                    error_px=error_px,
                    camera_request=camera_request,
                    held_steer=path_correction,
                    reason="need_body_steer"
                    if not path_needs_turn
                    else "path_needs_turn_reset",
                )
            if path_needs_turn:
                return self._enter(
                    LookDriveMode.PATH_FOLLOW,
                    now_ms=now_ms,
                    steering=clamp_path_steer(path_correction, navigation),
                    freeze=False,
                    pan_target=self._pan_forward_deg,
                    error_px=error_px,
                    camera_request=camera_request,
                    reason="path_needs_turn",
                )
            safe_min = navigation.safe_steering_min
            safe_max = navigation.safe_steering_max
            steer = max(safe_min, min(safe_max, camera_request))
            return self._enter(
                LookDriveMode.BODY_STEER,
                now_ms=now_ms,
                steering=steer,
                freeze=False,
                pan_target=self._pan_forward_deg,
                error_px=error_px,
                camera_request=camera_request,
                reason="body_steer",
                constraint="camera_steering_clamped",
            )

        return self._enter(
            LookDriveMode.PATH_FOLLOW,
            now_ms=now_ms,
            steering=clamp_path_steer(path_correction, navigation),
            freeze=False,
            pan_target=self._pan_forward_deg,
            error_px=error_px,
            camera_request=camera_request,
            reason="path_follow",
        )

    def force_forward(
        self,
        *,
        reason: str,
        now_ms: int,
        pixel_error_px: float = 0.0,
        camera_request: float = 0.0,
        mode: LookDriveMode = LookDriveMode.HOLD,
    ) -> LookCommand:
        """Snap controller pan to calibrated forward without a second slew tick.

        Used when DecisionEngine already advanced look_drive once this cycle
        (e.g. inverted envelope after BODY_STEER) and must fail closed without
        double-ticking.  ``now_ms`` is required so dwell state stays coherent.
        """

        self._mode = mode
        self._mode_entered_ms = int(now_ms)
        self._pan_deg = self._pan_forward_deg
        self._pan_reset_started_ms = None
        self._held_steer = 0.0
        return LookCommand(
            mode=mode,
            pan_deg=self._pan_forward_deg,
            pan_forward_deg=self._pan_forward_deg,
            reason=reason,
            pixel_error_px=float(pixel_error_px),
            camera_request=float(camera_request),
        )

    def _tick_pan_reset(
        self,
        *,
        now_ms: int,
        error_px: float,
        camera_request: float,
        path_correction: float,
        pan_at_forward: bool,
        track_ok: bool,
        navigation: NavigationState,
    ) -> LookDriveDecision:
        cfg = self._config
        started = self._pan_reset_started_ms or now_ms
        if pan_at_forward:
            nxt = self._pending_after_reset
            if nxt == LookDriveMode.BODY_STEER and track_ok:
                self._mode = LookDriveMode.BODY_STEER
                self._mode_entered_ms = now_ms
                self._pan_reset_started_ms = None
                self._pan_deg = self._pan_forward_deg
                steer = max(
                    navigation.safe_steering_min,
                    min(navigation.safe_steering_max, camera_request),
                )
                self._held_steer = steer
                return LookDriveDecision(
                    mode=LookDriveMode.BODY_STEER,
                    steering=steer,
                    freeze_chassis_steer=False,
                    look=LookCommand(
                        mode=LookDriveMode.BODY_STEER,
                        pan_deg=self._pan_forward_deg,
                        pan_forward_deg=self._pan_forward_deg,
                        reason="pan_reset_done_body_steer",
                        pixel_error_px=error_px,
                        camera_request=camera_request,
                    ),
                    constraint="camera_steering_clamped",
                )
            return self._enter(
                LookDriveMode.PATH_FOLLOW,
                now_ms=now_ms,
                steering=clamp_path_steer(path_correction, navigation),
                freeze=False,
                pan_target=self._pan_forward_deg,
                error_px=error_px,
                camera_request=camera_request,
                reason="pan_reset_done_path",
            )
        if now_ms - started >= cfg.look_pan_reset_timeout_ms:
            # Fail closed: clear held vision steer before freeze HOLD.
            return self._fail_closed_hold(
                now_ms=now_ms,
                error_px=error_px,
                camera_request=camera_request,
                reason="pan_reset_timeout",
                constraint="look_pan_reset_timeout",
            )
        pan_target = self._slew_toward(
            self._pan_forward_deg, now_ms, dt_hint_ms=cfg.look_control_period_ms
        )
        return LookDriveDecision(
            mode=LookDriveMode.PAN_RESET,
            steering=self._held_steer,
            freeze_chassis_steer=True,
            look=LookCommand(
                mode=LookDriveMode.PAN_RESET,
                pan_deg=pan_target,
                pan_forward_deg=self._pan_forward_deg,
                reason="pan_reset",
                pixel_error_px=error_px,
                camera_request=camera_request,
            ),
            constraint="pan_reset_freeze_steer",
        )

    def _start_pan_reset(
        self,
        *,
        now_ms: int,
        error_px: float,
        camera_request: float,
        held_steer: float,
        reason: str,
    ) -> LookDriveDecision:
        self._held_steer = float(held_steer)
        self._pan_reset_started_ms = now_ms
        self._mode = LookDriveMode.PAN_RESET
        self._mode_entered_ms = now_ms
        pan_target = self._slew_toward(
            self._pan_forward_deg,
            now_ms,
            dt_hint_ms=self._config.look_control_period_ms,
        )
        return LookDriveDecision(
            mode=LookDriveMode.PAN_RESET,
            steering=self._held_steer,
            freeze_chassis_steer=True,
            look=LookCommand(
                mode=LookDriveMode.PAN_RESET,
                pan_deg=pan_target,
                pan_forward_deg=self._pan_forward_deg,
                reason=reason,
                pixel_error_px=error_px,
                camera_request=camera_request,
            ),
            constraint="pan_reset_freeze_steer",
        )

    def _fail_closed_hold(
        self,
        *,
        now_ms: int,
        error_px: float,
        camera_request: float,
        reason: str,
        constraint: str = "look_drive_hold",
    ) -> LookDriveDecision:
        """Enter HOLD with steering cleared at the LookDriveDecision boundary.

        ``_enter(..., freeze=True)`` reuses ``_held_steer`` for output.  Any
        fail-closed HOLD must zero that cache first so consumers never see a
        stale camera/path request (even when DecisionEngine also stops).
        """

        self._held_steer = 0.0
        return self._enter(
            LookDriveMode.HOLD,
            now_ms=now_ms,
            steering=0.0,
            freeze=True,
            pan_target=self._pan_forward_deg,
            error_px=error_px,
            camera_request=camera_request,
            reason=reason,
            constraint=constraint,
        )

    def _enter(
        self,
        mode: LookDriveMode,
        *,
        now_ms: int,
        steering: float,
        freeze: bool,
        pan_target: float,
        error_px: float,
        camera_request: float,
        reason: str,
        constraint: Optional[str] = None,
    ) -> LookDriveDecision:
        if mode != self._mode:
            self._mode = mode
            self._mode_entered_ms = now_ms
            if mode != LookDriveMode.PAN_RESET:
                self._pan_reset_started_ms = None
        if mode == LookDriveMode.LOOK_AT:
            self._pan_deg = float(pan_target)
        else:
            self._pan_deg = self._slew_toward(
                pan_target,
                now_ms,
                dt_hint_ms=self._config.look_control_period_ms,
            )
        steer = max(-1.0, min(1.0, float(steering)))
        if not freeze:
            self._held_steer = steer
        return LookDriveDecision(
            mode=mode,
            steering=self._held_steer if freeze else steer,
            freeze_chassis_steer=freeze,
            look=LookCommand(
                mode=mode,
                pan_deg=self._pan_deg,
                pan_forward_deg=self._pan_forward_deg,
                reason=reason,
                pixel_error_px=error_px,
                camera_request=camera_request,
            ),
            constraint=constraint,
        )

    def _dwell_elapsed(self, now_ms: int) -> bool:
        return (now_ms - self._mode_entered_ms) >= self._config.look_mode_dwell_ms

    def _pan_toward_error(self, error_px: float, now_ms: int) -> float:
        cfg = self._config
        if cfg.look_px_per_deg <= 0.0:
            return self._pan_deg
        # Positive error (cat right) → pan right (increase pan_deg; Picarx
        # set_cam_pan_angle convention matches stare_at_you / bull_fight).
        desired = self._pan_deg + (error_px / cfg.look_px_per_deg)
        desired = max(self._pan_min_deg, min(self._pan_max_deg, desired))
        # Small deadband in pixels: hold pan.
        if abs(error_px) <= cfg.look_center_deadband_px:
            desired = self._pan_deg
        return self._slew_toward(
            desired, now_ms, dt_hint_ms=cfg.look_control_period_ms
        )

    def _slew_toward(
        self, target: float, now_ms: int, *, dt_hint_ms: int
    ) -> float:
        cfg = self._config
        dt_s = max(0.001, float(dt_hint_ms) / 1000.0)
        max_step = cfg.look_pan_slew_deg_s * dt_s
        delta = float(target) - self._pan_deg
        if abs(delta) <= max_step:
            self._pan_deg = float(target)
        else:
            self._pan_deg += max_step if delta > 0 else -max_step
        self._pan_deg = max(self._pan_min_deg, min(self._pan_max_deg, self._pan_deg))
        return self._pan_deg


def clamp_path_steer(
    path_correction: float, navigation: NavigationState
) -> float:
    """Clamp path_correction into a published envelope when one is present."""

    steer = max(-1.0, min(1.0, path_correction))
    source = (navigation.envelope_source or "none").lower()
    if source in {"point", "costmap_sweep"}:
        if navigation.safe_steering_min <= navigation.safe_steering_max:
            steer = max(
                navigation.safe_steering_min,
                min(navigation.safe_steering_max, steer),
            )
    elif navigation.safe_steering_min < navigation.safe_steering_max:
        # Explicit non-degenerate band without provenance still clamps.
        steer = max(
            navigation.safe_steering_min,
            min(navigation.safe_steering_max, steer),
        )
    return steer


__all__ = [
    "LookDriveController",
    "LookDriveDecision",
    "clamp_path_steer",
    "pan_can_center",
    "pixel_error_px",
]
