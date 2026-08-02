"""DecisionEngine shell for the contract-driven runtime.

This is the V1 Milestone 1 shell.  It honors the safety precedence and
freshness rules from the Interface and Data Contract Specification but does
not yet implement pursuit math (steering toward target, speed control,
handover blending, etc.).  Those land in later milestones once perception
and motion modules are wired in.

The shell is enough to:

- enforce ``failsafe > obstacle veto > pursuit logic`` precedence;
- request FSM transitions on critical safety events
  (``obstacle_too_close``, ``failsafe_triggered``);
- detect overhead staleness/expiry and downgrade chase behavior;
- emit a :class:`DecisionOutput` with active constraints and a reason code;
- keep the car at zero speed/steering until later milestones add real
  pursuit control.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, List, Optional

from cat_follow.control.fsm import FSM, NORMAL_DRIVING_STATES

if TYPE_CHECKING:
    from cat_follow.motion.sequence_executor import MotionSequenceExecutor
from cat_follow.control.types import (
    AckStatus,
    BrakeReversePhase,
    CommandName,
    DecisionInput,
    DecisionOutput,
    FsmEvent,
    FsmState,
    MissionState,
    RangeState,
    ReasonCode,
    TargetSource,
    ThermalState,
)
from cat_follow.navigation.dual_sensor_health import (
    DualSensorHealthState,
    build_dual_sensor_health,
)
from cat_follow.navigation.safe_return import (
    home_map_pose_m,
    safe_return_possible,
)
from cat_follow.target_config import TargetRuntimeConfig


# ── Tunable constants (mirror Interface spec section 13.14) ─────────


# Distance below which any obstacle is treated as a critical safety event.
# Instance thresholds on :class:`DecisionEngine` may override this at runtime.
OBSTACLE_TOO_CLOSE_CM = 10.0

# Overhead packet ages that trigger downgraded behavior or failsafe.
OVERHEAD_STALE_WARNING_MS = 300
OVERHEAD_STALE_FAILSAFE_MS = 700

# Age (ms) beyond which a producer's last sample is no longer trusted for
# safety/control decisions.  Per Interface spec 12.6 the stored ``fresh`` flag
# is advisory only: freshness MUST be recomputed at decision time from
# ``received_ms`` against local monotonic time.  A dead adapter/bridge thread
# stops advancing ``received_ms`` and therefore ages out (fail-closed) instead
# of leaving its last sample authoritative forever.
RANGE_STALE_MS = 500
LIDAR_STALE_MS = 500
NAVIGATION_STALE_MS = 500
# Vision loss window preserves the current CHASE -> GETTING_CLOSE fallback.
VISION_STALE_MS = 350

# Normalized full-scale speed used when navigation drives the chassis. Speeds
# and steering are normalized to [0, 1] and [-1, 1] respectively.
MAX_SPEED = 1.0
PLANNER_FULL_SCALE_MPS = 0.30

# States where a fresh NavigationState (from Nav2 via the ros_bridge) is
# allowed to drive path_correction / speed_limit into the motion command.
_NAV_DRIVE_STATES = frozenset(
    {
        FsmState.GETTING_CLOSE,
        FsmState.SEARCH,
        FsmState.CHASE,
        FsmState.GOTO,
        FsmState.RETURN_HOME,
    }
)

# FSM states that allow the Movement-tab open-loop sequence executor.
_SEQUENCE_DRIVE_STATES = frozenset({FsmState.IDLE, FsmState.HOME})


# Mapping of accepted command -> (FSM event, reason code) used to translate
# commands published into ``SharedState.command`` by ``CommsManager`` into
# FSM transitions.  ``set_home`` is intentionally omitted because it is a
# state-data side effect, not an FSM event.
_COMMAND_TO_EVENT = {
    CommandName.START_CHASE: (FsmEvent.START_CHASE_ACCEPTED, ReasonCode.START_CHASE_ACCEPTED),
    CommandName.STOP_CHASE: (FsmEvent.STOP_CHASE_ACCEPTED, ReasonCode.STOP_CHASE_ACCEPTED),
    CommandName.RETURN_HOME: (FsmEvent.RETURN_HOME_ACCEPTED, ReasonCode.RETURN_HOME_ACCEPTED),
    CommandName.GO_TO: (FsmEvent.GO_TO_ACCEPTED, ReasonCode.GO_TO_ACCEPTED),
    CommandName.EMERGENCY_STOP: (FsmEvent.EMERGENCY_STOP_ACCEPTED, ReasonCode.FAILSAFE_TRIGGERED),
    CommandName.CLEAR_FAILSAFE: (FsmEvent.CLEAR_FAILSAFE_ACCEPTED, ReasonCode.CLEAR_FAILSAFE_ACCEPTED),
}


# Operator commands that end the current objective outright.  The brake-reverse
# attempt budget is scoped to an objective, so these start the next one fresh
# instead of inheriting a partially spent budget.
_OBJECTIVE_ENDING_COMMANDS = frozenset(
    {
        CommandName.STOP_CHASE,
        CommandName.RETURN_HOME,
        CommandName.EMERGENCY_STOP,
        CommandName.CLEAR_FAILSAFE,
    }
)


# Mapping of FSM state to the V1 default reason code emitted by the shell
# when no other higher-priority condition is active.
_STATE_DEFAULT_REASON = {
    FsmState.HOME: ReasonCode.INIT,
    FsmState.IDLE: ReasonCode.INIT,
    FsmState.GETTING_CLOSE: ReasonCode.GLOBAL_CHASE,
    FsmState.SEARCH: ReasonCode.GLOBAL_CHASE,
    FsmState.CHASE: ReasonCode.LOCAL_TRACK,
    FsmState.GOTO: ReasonCode.GO_TO_ACCEPTED,
    FsmState.RETURN_HOME: ReasonCode.RETURN_HOME_ACCEPTED,
    FsmState.FAILSAFE: ReasonCode.FAILSAFE_TRIGGERED,
}


class DecisionEngine:
    """V1 shell that produces a safe :class:`DecisionOutput` per tick.

    The engine owns its FSM reference so it can request transitions when
    safety conditions fire.  Snapshot publication and motor execution remain
    the caller's responsibility.
    """

    def __init__(
        self,
        fsm: FSM,
        sequence_executor: Optional["MotionSequenceExecutor"] = None,
        *,
        obstacle_too_close_cm: float = OBSTACLE_TOO_CLOSE_CM,
        target_runtime_config: Optional[TargetRuntimeConfig] = None,
    ) -> None:
        self._fsm = fsm
        self._sequence = sequence_executor
        self._last_consumed_command_id: Optional[str] = None
        self._last_consumed_mission_event_id: Optional[str] = None
        self._active_target_id: Optional[str] = None
        self._obstacle_too_close_cm = float(obstacle_too_close_cm)
        self._target_config = target_runtime_config or TargetRuntimeConfig()
        self._sensor_hold_started_ms: Optional[int] = None
        self._brake_saved_state: Optional[FsmState] = None
        self._brake_phase: Optional[BrakeReversePhase] = None
        self._brake_phase_started_ms = 0
        self._brake_attempts = 0
        self._clearance_started_ms: Optional[int] = None
        self._overhead_invalid_started_ms: Optional[int] = None
        self._last_valid_target_x: Optional[float] = None
        self._last_valid_target_y: Optional[float] = None
        self._search_started_ms: Optional[int] = None
        self._search_stage = 0
        self._search_lock_count = 0
        self._last_vision_observation_sequence: Optional[int] = None
        self._handoff_deadline_ms: Optional[int] = None
        self._last_event_observation_seq = -1
        self._blocked_target_id: Optional[str] = None
        self._blocked_through_observation_seq = -1
        self._home_version_frozen: Optional[int] = None
        self._frozen_home_x: Optional[float] = None
        self._frozen_home_y: Optional[float] = None
        self._frozen_home_x_m: Optional[float] = None
        self._frozen_home_y_m: Optional[float] = None
        self._frozen_home_yaw_rad: float = 0.0
        self._frozen_home_frame_id: str = "yard"
        self._chase_recording_requested = False
        self._recording_postroll_deadline_ms: Optional[int] = None
        self._goto_request_yolo = False
        self._goto_request_recording = False
        self._brake_saved_detector = False
        self._brake_saved_recording = False
        self._thermal_critical_return_active = False
        self._last_dual_sensor_health: Optional[DualSensorHealthState] = None
        self._last_hold_reason: Optional[str] = None

    def set_safety_thresholds(self, config) -> None:
        """Apply runtime safety threshold updates from :mod:`safety_config`."""

        self._obstacle_too_close_cm = float(config.obstacle_too_close_cm)

    @property
    def obstacle_too_close_cm(self) -> float:
        return self._obstacle_too_close_cm

    @property
    def brake_reverse_trigger_cm(self) -> float:
        return self._target_config.brake_reverse_trigger_cm

    @property
    def close_obstacle_trigger_cm(self) -> float:
        """Proximity distance that stops motion, honoring both knobs.

        ``brake_reverse_trigger_cm`` is the target-architecture trigger and
        ``obstacle_too_close_cm`` is the operator-facing failsafe distance from
        SafetyConfig / the calibration UI.  Both are live-tunable, so the gate
        uses whichever is more conservative instead of silently ignoring one.
        """

        return max(
            self._target_config.brake_reverse_trigger_cm,
            self._obstacle_too_close_cm,
        )

    @property
    def close_obstacle_clear_cm(self) -> float:
        """Clearance distance that resets the brake-reverse attempt counter.

        Keeps the configured ``reset_cm - trigger_cm`` margin above whatever
        trigger distance is effective, so the validated "reset exceeds trigger"
        invariant still holds when ``obstacle_too_close_cm`` dominates.
        """

        margin_cm = (
            self._target_config.brake_reverse_reset_cm
            - self._target_config.brake_reverse_trigger_cm
        )
        return self.close_obstacle_trigger_cm + margin_cm

    def tick(self, decision_input: DecisionInput) -> DecisionOutput:
        constraints: List[str] = []

        # Consume accepted command preemption first.  The FSM rejects ordinary
        # commands from FAILSAFE, while an accepted CLEAR_FAILSAFE may return
        # to stationary IDLE; sensor gates still prevent later unsafe motion.
        self._consume_accepted_command(decision_input)
        if (
            self._brake_phase is not None
            and self._fsm.state != FsmState.BRAKE_REVERSE
        ):
            # Covers this tick's own command as well as any transition the comms
            # transaction path applied between ticks: the phase context must
            # never outlive the state that owns it.
            self.clear_brake_reverse_context()

        # FAILSAFE is latched unless the explicit clearance command above was
        # accepted by the command validator.
        if self._fsm.state == FsmState.FAILSAFE:
            self._abort_sequence("failsafe")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.FAILSAFE_TRIGGERED,
                constraints=constraints,
                brake=True,
            )

        # Car geofence breach is precedence #2: latch FAILSAFE immediately.
        geofence = decision_input.geofence
        if geofence.configured and geofence.breach_confirmed:
            return self._enter_failsafe(
                decision_input,
                reason=ReasonCode.GEOFENCE_BREACH,
                constraint="geofence_breach",
            )

        current_state = self._fsm.state
        if current_state == FsmState.FAILSAFE:
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.FAILSAFE_TRIGGERED,
                constraints=constraints,
                brake=True,
            )

        range_healthy, lidar_healthy = self._required_sensor_health(decision_input)
        both_healthy = range_healthy and lidar_healthy
        self._publish_dual_sensor_health(
            decision_input,
            range_healthy=range_healthy,
            lidar_healthy=lidar_healthy,
        )

        # Critical thermal policy (precedence after geofence / failsafe latch).
        thermal_output = self._apply_thermal_policy(
            decision_input,
            constraints,
            range_healthy=range_healthy,
            lidar_healthy=lidar_healthy,
        )
        if thermal_output is not None:
            return thermal_output

        # Loss of containment observability while driving: stop and escalate
        # if safe return is impossible.
        if (
            current_state in NORMAL_DRIVING_STATES
            and geofence.configured
            and not geofence.localization_valid_for_containment
        ):
            constraints.append("geofence_unobservable")
            # NAV-13: stop first, then fail only when safe return cannot be
            # established.  Geofence is excluded from the predicate because a
            # confirmed breach already latched FAILSAFE above and the
            # observability loss is the trigger condition itself.
            ok, _ = safe_return_possible(
                home=decision_input.home,
                mission=decision_input.mission,
                range_healthy=range_healthy,
                lidar_healthy=lidar_healthy,
                geofence=None,
                navigation=decision_input.navigation,
            )
            if not ok:
                return self._enter_failsafe(
                    decision_input,
                    reason=ReasonCode.GEOFENCE_UNOBSERVABLE,
                    constraint="geofence_unobservable",
                )
            # NAV-13 asks for a stop, not a state change: the objective is held
            # at zero motion so it resumes when localization recovers.  Do not
            # convert this into RETURN_HOME without changing NAV-13 first.
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.GEOFENCE_UNOBSERVABLE,
                constraints=constraints,
                brake=True,
            )

        # BRAKE_REVERSE has no sensor recovery grace period.
        if current_state == FsmState.BRAKE_REVERSE:
            if not both_healthy:
                return self._enter_failsafe(
                    decision_input,
                    reason=ReasonCode.SENSOR_HEALTH_TIMEOUT,
                    constraint="brake_reverse_sensor_loss",
                )
            return self._brake_reverse_output(decision_input)

        # Both proximity sources are required for every autonomous objective.
        # Stationary HOME/IDLE merely report degraded status and stay stopped.
        if current_state in NORMAL_DRIVING_STATES:
            if not both_healthy:
                if self._sensor_hold_started_ms is None:
                    self._sensor_hold_started_ms = decision_input.now_ms
                hold_age_ms = (
                    decision_input.now_ms - self._sensor_hold_started_ms
                )
                if hold_age_ms >= int(
                    self._target_config.sensor_recovery_sec * 1000
                ):
                    return self._enter_failsafe(
                        decision_input,
                        reason=ReasonCode.SENSOR_HEALTH_TIMEOUT,
                        constraint="sensor_health_timeout",
                    )
                constraints.append("sensor_health_hold")
                self._last_hold_reason = "sensor_health_hold"
                if not range_healthy:
                    constraints.append("ultrasonic_unhealthy")
                if not lidar_healthy:
                    constraints.append("lidar_unhealthy")
                self._publish_dual_sensor_health(
                    decision_input,
                    range_healthy=range_healthy,
                    lidar_healthy=lidar_healthy,
                    hold_active=True,
                    hold_reason="sensor_health_hold",
                )
                return self._safe_stop_output(
                    decision_input,
                    reason=ReasonCode.SENSOR_HEALTH_HOLD,
                    constraints=constraints,
                    brake=True,
                )
            self._sensor_hold_started_ms = None
            self._last_hold_reason = None
        else:
            self._sensor_hold_started_ms = None
            self._last_hold_reason = None
            if (
                current_state in {FsmState.HOME, FsmState.IDLE}
                and not both_healthy
            ):
                constraints.append("sensor_health_degraded")
                if not range_healthy:
                    constraints.append("ultrasonic_unhealthy")
                if not lidar_healthy:
                    constraints.append("lidar_unhealthy")

        self._publish_dual_sensor_health(
            decision_input,
            range_healthy=range_healthy,
            lidar_healthy=lidar_healthy,
        )

        self._update_brake_attempt_reset(decision_input, both_healthy)

        # A close reading from either healthy required sensor starts the bounded
        # reverse sequence.  The old 10 cm direct FAILSAFE path is superseded.
        if (
            current_state in NORMAL_DRIVING_STATES
            and self._any_sensor_below(
                decision_input, self.close_obstacle_trigger_cm
            )
        ):
            return self._start_brake_reverse(decision_input, current_state)

        chase_transition_output = self._apply_chase_matrix(
            decision_input,
            constraints,
            range_healthy=range_healthy,
            lidar_healthy=lidar_healthy,
        )
        if chase_transition_output is not None:
            return chase_transition_output

        # State-specific shell behavior.
        current_state = self._fsm.state
        navigation_output = self._apply_navigation_events(
            decision_input,
            constraints,
            range_healthy=range_healthy,
            lidar_healthy=lidar_healthy,
        )
        if navigation_output is not None:
            return navigation_output
        current_state = self._fsm.state

        # Movement-tab open-loop sequence (contract runtime only).
        sequence_output = self._sequence_drive_output(decision_input, constraints)
        if sequence_output is not None:
            return sequence_output

        # Navigation-assisted drive (GETTING_CLOSE / GOTO) when Nav2 publishes
        #    fresh constraints via the ros_bridge.  Safety precedence above is
        #    already enforced (failsafe > obstacle veto > this).  Navigation
        #    freshness is recomputed from received_ms so a dead ros_bridge (or a
        #    silent planner, via the bridge's cmd_vel aging) fails closed here.
        nav_fresh = self._age_fresh(
            decision_input.navigation.received_ms,
            NAVIGATION_STALE_MS,
            decision_input.now_ms,
        )
        if current_state in _NAV_DRIVE_STATES and nav_fresh:
            return self._navigation_drive_output(
                decision_input, current_state, constraints
            )

        # All other states (or no fresh navigation): emit zero-motion shell
        # output with a state-appropriate reason.  Real pursuit math lands in
        # later milestones once VisionTracker pursuit control is wired in.
        reason = _STATE_DEFAULT_REASON.get(current_state, ReasonCode.INIT)
        return self._safe_stop_output(
            decision_input,
            reason=reason,
            constraints=constraints,
            brake=False,
        )

    # ── helpers ─────────────────────────────────────────────────────

    @property
    def dual_sensor_health(self) -> Optional[DualSensorHealthState]:
        return self._last_dual_sensor_health

    def _publish_dual_sensor_health(
        self,
        decision_input: DecisionInput,
        *,
        range_healthy: bool,
        lidar_healthy: bool,
        hold_active: bool = False,
        hold_reason: Optional[str] = None,
    ) -> None:
        hold_started = self._sensor_hold_started_ms
        recovery_deadline = None
        if hold_started is not None:
            recovery_deadline = hold_started + int(
                self._target_config.sensor_recovery_sec * 1000
            )
        self._last_dual_sensor_health = build_dual_sensor_health(
            ultrasonic=decision_input.range,
            lidar=decision_input.lidar,
            now_ms=decision_input.now_ms,
            ultrasonic_stale_ms=RANGE_STALE_MS,
            lidar_stale_ms=LIDAR_STALE_MS,
            hold_active=hold_active,
            hold_started_ms=hold_started if hold_active else None,
            hold_reason=hold_reason if hold_active else None,
            recovery_deadline_ms=recovery_deadline if hold_active else None,
            costmap_layer_enabled=bool(
                self._target_config.nav_ultrasonic_costmap
            ),
        )
        # Unused health booleans retained for call-site clarity / future telemetry.
        _ = (range_healthy, lidar_healthy)

    def _apply_thermal_policy(
        self,
        decision_input: DecisionInput,
        constraints: List[str],
        *,
        range_healthy: bool,
        lidar_healthy: bool,
    ) -> Optional[DecisionOutput]:
        """Apply critical thermal return / failsafe policy (SAFE-15..18)."""

        thermal = decision_input.system.thermal_state
        if thermal != ThermalState.CRITICAL:
            if self._fsm.state != FsmState.RETURN_HOME:
                self._thermal_critical_return_active = False
            return None

        constraints.append("thermal_critical")
        current_state = self._fsm.state
        unsafe = bool(self._target_config.thermal_critical_unsafe)

        if current_state in {FsmState.HOME, FsmState.IDLE, FsmState.FAILSAFE}:
            self._thermal_critical_return_active = False
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.THERMAL_CRITICAL,
                constraints=constraints,
                brake=True,
            )

        if current_state == FsmState.BRAKE_REVERSE:
            self.clear_brake_reverse_context()
            if unsafe:
                return self._enter_failsafe(
                    decision_input,
                    reason=ReasonCode.THERMAL_CRITICAL,
                    constraint="thermal_critical_unsafe",
                )
            return self._return_home_or_failsafe(
                decision_input,
                constraints,
                event=FsmEvent.RETURN_HOME_ACCEPTED,
                reason=ReasonCode.THERMAL_CRITICAL,
                range_healthy=range_healthy,
                lidar_healthy=lidar_healthy,
            )

        if current_state == FsmState.RETURN_HOME:
            if unsafe:
                return self._enter_failsafe(
                    decision_input,
                    reason=ReasonCode.THERMAL_CRITICAL,
                    constraint="thermal_critical_unsafe",
                )
            self._thermal_critical_return_active = True
            constraints.append("thermal_critical_return")
            return None

        if unsafe:
            return self._enter_failsafe(
                decision_input,
                reason=ReasonCode.THERMAL_CRITICAL,
                constraint="thermal_critical_unsafe",
            )
        self._thermal_critical_return_active = True
        return self._return_home_or_failsafe(
            decision_input,
            constraints,
            event=FsmEvent.RETURN_HOME_ACCEPTED,
            reason=ReasonCode.THERMAL_CRITICAL,
            range_healthy=range_healthy,
            lidar_healthy=lidar_healthy,
        )

    @property
    def brake_reverse_phase(self) -> Optional[BrakeReversePhase]:
        return self._brake_phase

    @property
    def brake_reverse_attempts(self) -> int:
        return self._brake_attempts

    @property
    def brake_saved_state(self) -> Optional[FsmState]:
        return self._brake_saved_state

    @property
    def active_target_id(self) -> Optional[str]:
        return self._active_target_id

    @property
    def last_consumed_mission_event_id(self) -> Optional[str]:
        """Most recent mission event already applied by the comms path."""

        return self._last_consumed_mission_event_id

    def set_active_target_id(self, target_id: Optional[str]) -> None:
        self._active_target_id = target_id
        if target_id is not None:
            self._handoff_deadline_ms = None
            self._blocked_target_id = None
            self._blocked_through_observation_seq = -1

    def start_handoff(
        self,
        now_ms: int,
        *,
        exited_target_id: str,
        observation_sequence: int,
    ) -> None:
        self._active_target_id = None
        self._blocked_target_id = exited_target_id
        self._blocked_through_observation_seq = observation_sequence
        self._handoff_deadline_ms = now_ms + int(
            self._target_config.handoff_wait_sec * 1000
        )
        self._reset_chase_context()
        # The exit may be applied from BRAKE_REVERSE, so the interrupted
        # objective the phase would restore no longer exists.
        self.clear_brake_reverse_context()

    def freeze_mission_home(self, home) -> None:
        self._home_version_frozen = int(home.home_version)
        self._frozen_home_x = float(home.x)
        self._frozen_home_y = float(home.y)
        frozen_x_m, frozen_y_m = home_map_pose_m(home)
        self._frozen_home_x_m = frozen_x_m
        self._frozen_home_y_m = frozen_y_m
        self._frozen_home_yaw_rad = float(home.yaw_rad)
        self._frozen_home_frame_id = str(home.frame_id or "yard")

    def clear_mission_home_freeze(self) -> None:
        self._home_version_frozen = None
        self._frozen_home_x = None
        self._frozen_home_y = None
        self._frozen_home_x_m = None
        self._frozen_home_y_m = None
        self._frozen_home_yaw_rad = 0.0
        self._frozen_home_frame_id = "yard"

    def request_chase_recording(self) -> None:
        self._chase_recording_requested = True

    def clear_chase_recording(self) -> None:
        self._chase_recording_requested = False

    def start_recording_postroll(self, now_ms: int) -> None:
        self._recording_postroll_deadline_ms = now_ms + int(
            self._target_config.recording_postroll_sec * 1000
        )
        self._chase_recording_requested = False

    def clear_expired_recording_postroll(self, now_ms: int) -> None:
        deadline = self._recording_postroll_deadline_ms
        if deadline is not None and now_ms >= int(deadline):
            self._recording_postroll_deadline_ms = None

    def set_goto_perception_flags(
        self, *, request_yolo: bool, request_recording: bool
    ) -> None:
        self._goto_request_yolo = bool(request_yolo)
        self._goto_request_recording = bool(request_recording)

    def freeze_brake_perception_policy(
        self, *, detector_requested: bool, recording_requested: bool
    ) -> None:
        self._brake_saved_detector = bool(detector_requested)
        self._brake_saved_recording = bool(recording_requested)

    def clear_brake_reverse_context(self, *, reset_attempts: bool = False) -> None:
        """Drop the BRAKE_REVERSE phase, saved objective, and perception freeze.

        The FSM owns the mode; this context is only meaningful while the FSM
        actually sits in ``BRAKE_REVERSE``.  Every exit path -- the engine's own
        clearance, failsafe, thermal abort, and the comms transaction path that
        moves the FSM between control ticks -- routes through here so a stale
        phase can never drive the motors or resume a dead objective.

        The attempt budget belongs to the objective rather than to one phase, so
        it survives by default and is cleared only by the measured dual-sensor
        clearance timer in :py:meth:`_update_brake_attempt_reset`.  Callers that
        end the objective outright on operator command pass ``reset_attempts``
        so the next objective starts with a full budget.
        """

        self._brake_phase = None
        self._brake_phase_started_ms = 0
        self._brake_saved_state = None
        self._brake_saved_detector = False
        self._brake_saved_recording = False
        if reset_attempts:
            self._brake_attempts = 0
            self._clearance_started_ms = None

    def cancel_handoff(self) -> None:
        self._handoff_deadline_ms = None

    @property
    def mission_state(self) -> MissionState:
        return MissionState(
            active_target_id=self._active_target_id,
            last_event_observation_seq=self._last_event_observation_seq,
            blocked_target_id=self._blocked_target_id,
            blocked_through_observation_seq=(
                self._blocked_through_observation_seq
            ),
            handoff_deadline_ms=self._handoff_deadline_ms,
            overhead_invalid_started_ms=self._overhead_invalid_started_ms,
            search_stage=self._search_stage,
            search_lock_observations=self._search_lock_count,
            home_version_frozen=self._home_version_frozen,
            frozen_home_x=self._frozen_home_x,
            frozen_home_y=self._frozen_home_y,
            frozen_home_x_m=self._frozen_home_x_m,
            frozen_home_y_m=self._frozen_home_y_m,
            frozen_home_yaw_rad=self._frozen_home_yaw_rad,
            frozen_home_frame_id=self._frozen_home_frame_id,
            chase_recording_requested=self._chase_recording_requested,
            recording_postroll_deadline_ms=self._recording_postroll_deadline_ms,
            goto_request_yolo=self._goto_request_yolo,
            goto_request_recording=self._goto_request_recording,
        )

    def lifecycle_context(self):
        from cat_follow.perception.perception_lifecycle_manager import (
            LifecycleMissionContext,
        )

        return LifecycleMissionContext(
            chase_recording_requested=self._chase_recording_requested,
            recording_postroll_deadline_ms=self._recording_postroll_deadline_ms,
            goto_request_yolo=self._goto_request_yolo,
            goto_request_recording=self._goto_request_recording,
            brake_saved_detector=self._brake_saved_detector,
            brake_saved_recording=self._brake_saved_recording,
            brake_saved_state=self._brake_saved_state,
        )

    def note_command_applied(self, command_id: str) -> None:
        self._last_consumed_command_id = command_id

    def note_mission_event_applied(
        self, event_id: str, observation_sequence: Optional[int] = None
    ) -> None:
        self._last_consumed_mission_event_id = event_id
        if observation_sequence is not None:
            self._last_event_observation_seq = observation_sequence

    def _apply_navigation_events(
        self,
        decision_input: DecisionInput,
        constraints: List[str],
        *,
        range_healthy: bool,
        lidar_healthy: bool,
    ) -> Optional[DecisionOutput]:
        nav = decision_input.navigation
        state = self._fsm.state
        now_ms = decision_input.now_ms

        if nav.completion_qualified:
            if state == FsmState.GOTO:
                self._fsm.apply(
                    FsmEvent.GO_TO_COMPLETE,
                    reason=ReasonCode.NAVIGATION_COMPLETE,
                    now_ms=now_ms,
                )
            elif state == FsmState.RETURN_HOME:
                self._fsm.apply(
                    FsmEvent.RETURN_HOME_COMPLETE,
                    reason=ReasonCode.NAVIGATION_COMPLETE,
                    now_ms=now_ms,
                )
                self.clear_chase_recording()
                self._recording_postroll_deadline_ms = None
            else:
                return None
            constraints.append("navigation_completion_qualified")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.NAVIGATION_COMPLETE,
                constraints=constraints,
                brake=True,
            )

        if not nav.failures_exhausted:
            return None

        if state == FsmState.RETURN_HOME:
            return self._enter_failsafe(
                decision_input,
                reason=ReasonCode.NAVIGATION_FAILURES_EXHAUSTED,
                constraint="navigation_failures_exhausted",
            )
        if state == FsmState.GOTO:
            result = self._fsm.apply(
                FsmEvent.NAVIGATION_FAILURES_EXHAUSTED,
                reason=ReasonCode.NAVIGATION_FAILURES_EXHAUSTED,
                now_ms=now_ms,
            )
            if not result.accepted:
                return self._enter_failsafe(
                    decision_input,
                    reason=ReasonCode.NAVIGATION_FAILURES_EXHAUSTED,
                    constraint="navigation_failures_exhausted",
                )
            constraints.append("navigation_failures_exhausted")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.NAVIGATION_FAILURES_EXHAUSTED,
                constraints=constraints,
                brake=True,
            )
        if state in {FsmState.GETTING_CLOSE, FsmState.SEARCH, FsmState.CHASE}:
            constraints.append("navigation_failures_exhausted")
            return self._return_home_or_failsafe(
                decision_input,
                constraints,
                event=FsmEvent.NAVIGATION_FAILURES_EXHAUSTED,
                reason=ReasonCode.NAVIGATION_FAILURES_EXHAUSTED,
                range_healthy=range_healthy,
                lidar_healthy=lidar_healthy,
            )
        return None

    def _required_sensor_health(
        self, decision_input: DecisionInput
    ) -> tuple[bool, bool]:
        return (
            self._obstacle_sensor_usable(
                decision_input.range, RANGE_STALE_MS, decision_input.now_ms
            ),
            self._obstacle_sensor_usable(
                decision_input.lidar, LIDAR_STALE_MS, decision_input.now_ms
            ),
        )

    def _required_sensor_distances_cm(
        self, decision_input: DecisionInput
    ) -> tuple[Optional[float], Optional[float]]:
        """Return (ultrasonic, lidar) readings, or None when not usable.

        A stale or faulted sensor (``distance_cm is None`` / zero confidence)
        yields ``None`` so proximity math never compares against a missing
        reading; escalation for missing sensors belongs to the dual-sensor
        health gate, not to these predicates.
        """

        return (
            (
                decision_input.range.distance_cm
                if self._obstacle_sensor_usable(
                    decision_input.range, RANGE_STALE_MS, decision_input.now_ms
                )
                else None
            ),
            (
                decision_input.lidar.distance_cm
                if self._obstacle_sensor_usable(
                    decision_input.lidar, LIDAR_STALE_MS, decision_input.now_ms
                )
                else None
            ),
        )

    def _any_sensor_below(
        self, decision_input: DecisionInput, threshold_cm: float
    ) -> bool:
        """True when a usable required sensor reads closer than the threshold."""

        return any(
            distance_cm is not None and distance_cm < threshold_cm
            for distance_cm in self._required_sensor_distances_cm(decision_input)
        )

    def _both_sensors_above(
        self, decision_input: DecisionInput, threshold_cm: float
    ) -> bool:
        """True only when both required sensors are usable and read clear."""

        return all(
            distance_cm is not None and distance_cm > threshold_cm
            for distance_cm in self._required_sensor_distances_cm(decision_input)
        )

    def _enter_failsafe(
        self,
        decision_input: DecisionInput,
        *,
        reason: ReasonCode,
        constraint: str,
    ) -> DecisionOutput:
        self._abort_sequence(constraint)
        self._fsm.apply(
            FsmEvent.FAILSAFE_TRIGGERED,
            reason=reason,
            now_ms=decision_input.now_ms,
        )
        self.clear_brake_reverse_context()
        return self._safe_stop_output(
            decision_input,
            reason=reason,
            constraints=[constraint],
            brake=True,
        )

    def _start_brake_reverse(
        self, decision_input: DecisionInput, interrupted_state: FsmState
    ) -> DecisionOutput:
        self._abort_sequence("brake_reverse")
        result = self._fsm.apply(
            FsmEvent.BRAKE_REVERSE_TRIGGERED,
            reason=ReasonCode.BRAKE_REVERSE_TRIGGERED,
            now_ms=decision_input.now_ms,
        )
        if not result.accepted:
            # The FSM accepts this event from every normal driving state, so a
            # rejection means another authority (comms command path) already
            # left that state after this tick read its snapshot.  Arming the
            # phase anyway would command reverse motion while the FSM says
            # otherwise, so hold zero motion and let the next tick re-evaluate
            # the obstacle against the state that actually applies.
            self.clear_brake_reverse_context()
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.OBSTACLE_TOO_CLOSE,
                constraints=[
                    "obstacle_too_close",
                    "brake_reverse_transition_rejected",
                ],
                brake=True,
            )

        self._brake_saved_state = interrupted_state
        # Inherit perception demand from the interrupted objective.
        detector = interrupted_state in {
            FsmState.SEARCH,
            FsmState.CHASE,
        } or (
            interrupted_state == FsmState.GOTO and self._goto_request_yolo
        )
        recording = self._chase_recording_requested or (
            interrupted_state == FsmState.GOTO and self._goto_request_recording
        )
        self.freeze_brake_perception_policy(
            detector_requested=detector,
            recording_requested=recording,
        )
        self._brake_phase = BrakeReversePhase.STOP_ENTRY
        self._brake_phase_started_ms = decision_input.now_ms
        return self._brake_reverse_output(decision_input)

    def _brake_reverse_output(
        self, decision_input: DecisionInput
    ) -> DecisionOutput:
        if self._fsm.state != FsmState.BRAKE_REVERSE:
            # A command applied on the comms thread left BRAKE_REVERSE after
            # this tick sampled the state.  Drop the orphaned phase instead of
            # reversing or restoring the objective the operator just cancelled.
            self.clear_brake_reverse_context()
            return self._stale_brake_context_output(decision_input)

        phase = self._brake_phase
        if phase is None:
            return self._enter_failsafe(
                decision_input,
                reason=ReasonCode.FAILSAFE_TRIGGERED,
                constraint="brake_reverse_context_missing",
            )

        constraints = ["brake_reverse", f"brake_reverse_{phase.value}"]
        now_ms = decision_input.now_ms

        if phase == BrakeReversePhase.STOP_ENTRY:
            self._brake_phase = BrakeReversePhase.CENTER
            self._brake_phase_started_ms = now_ms
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.BRAKE_REVERSE_ACTIVE,
                constraints=constraints,
                brake=True,
            )

        if phase == BrakeReversePhase.CENTER:
            self._brake_phase = BrakeReversePhase.SETTLE
            self._brake_phase_started_ms = now_ms
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.BRAKE_REVERSE_ACTIVE,
                constraints=constraints,
                brake=True,
            )

        if phase == BrakeReversePhase.SETTLE:
            if (
                now_ms - self._brake_phase_started_ms
                >= self._target_config.brake_reverse_settle_ms
            ):
                self._brake_phase = BrakeReversePhase.REVERSE
                self._brake_phase_started_ms = now_ms
                self._brake_attempts += 1
                phase = BrakeReversePhase.REVERSE
                constraints = [
                    "brake_reverse",
                    f"brake_reverse_{phase.value}",
                ]
            else:
                return self._safe_stop_output(
                    decision_input,
                    reason=ReasonCode.BRAKE_REVERSE_ACTIVE,
                    constraints=constraints,
                    brake=True,
                )

        if phase == BrakeReversePhase.REVERSE:
            duration_ms = int(
                self._target_config.brake_reverse_duration_sec * 1000
            )
            if now_ms - self._brake_phase_started_ms < duration_ms:
                # Re-read immediately before the only motion-producing branch
                # in this method: a command applied between the guard above and
                # here must win over an in-flight reverse.
                if self._fsm.state != FsmState.BRAKE_REVERSE:
                    self.clear_brake_reverse_context()
                    return self._stale_brake_context_output(decision_input)
                return DecisionOutput(
                    timestamp_ms=now_ms,
                    requested_state=FsmState.BRAKE_REVERSE,
                    speed=self._target_config.brake_reverse_normalized,
                    steering=0.0,
                    brake=False,
                    reason=ReasonCode.BRAKE_REVERSE_ACTIVE,
                    active_constraints=tuple(constraints),
                )
            self._brake_phase = BrakeReversePhase.STOP_EXIT
            self._brake_phase_started_ms = now_ms
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.BRAKE_REVERSE_ACTIVE,
                constraints=constraints,
                brake=True,
            )

        if phase == BrakeReversePhase.STOP_EXIT:
            self._brake_phase = BrakeReversePhase.RECHECK
            self._brake_phase_started_ms = now_ms
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.BRAKE_REVERSE_ACTIVE,
                constraints=constraints,
                brake=True,
            )

        if self._any_sensor_below(
            decision_input, self.close_obstacle_trigger_cm
        ):
            if self._brake_attempts >= self._target_config.brake_reverse_max_attempts:
                return self._enter_failsafe(
                    decision_input,
                    reason=ReasonCode.BRAKE_REVERSE_EXHAUSTED,
                    constraint="brake_reverse_exhausted",
                )
            self._brake_phase = BrakeReversePhase.STOP_ENTRY
            self._brake_phase_started_ms = now_ms
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.BRAKE_REVERSE_ACTIVE,
                constraints=["brake_reverse", "brake_reverse_retry"],
                brake=True,
            )

        saved_state = self._brake_saved_state
        if self._fsm.state != FsmState.BRAKE_REVERSE:
            # Losing the race here is an operator command, not a fault, so the
            # restore must be abandoned rather than escalated to FAILSAFE.
            self.clear_brake_reverse_context()
            return self._stale_brake_context_output(decision_input)
        result = self._fsm.apply(
            FsmEvent.BRAKE_REVERSE_CLEARED,
            reason=ReasonCode.BRAKE_REVERSE_CLEAR,
            now_ms=now_ms,
            resume_state=saved_state,
        )
        if not result.accepted:
            return self._enter_failsafe(
                decision_input,
                reason=ReasonCode.FAILSAFE_TRIGGERED,
                constraint="brake_reverse_restore_failed",
            )
        self.clear_brake_reverse_context()
        return self._safe_stop_output(
            decision_input,
            reason=ReasonCode.BRAKE_REVERSE_CLEAR,
            constraints=["brake_reverse_clear"],
            brake=True,
        )

    def _stale_brake_context_output(
        self, decision_input: DecisionInput
    ) -> DecisionOutput:
        """Zero-motion output for a brake phase the FSM has already left."""

        return self._safe_stop_output(
            decision_input,
            reason=_STATE_DEFAULT_REASON.get(
                self._fsm.state, ReasonCode.INIT
            ),
            constraints=["brake_reverse_context_stale"],
            brake=True,
        )

    def _update_brake_attempt_reset(
        self, decision_input: DecisionInput, both_healthy: bool
    ) -> None:
        if (
            both_healthy
            and self._both_sensors_above(
                decision_input, self.close_obstacle_clear_cm
            )
        ):
            if self._clearance_started_ms is None:
                self._clearance_started_ms = decision_input.now_ms
            elif (
                decision_input.now_ms - self._clearance_started_ms
                >= int(self._target_config.brake_reverse_reset_sec * 1000)
            ):
                self._brake_attempts = 0
        else:
            self._clearance_started_ms = None

    def _consume_accepted_command(self, decision_input: DecisionInput) -> None:
        """Apply any newly-accepted command from ``SharedState.command``.

        Each command_id is consumed at most once; duplicate retries that
        CommsManager already deduplicates by ``command_id`` will not
        re-fire FSM events here either.  Rejected commands are still marked
        as consumed so the engine doesn't keep retrying them.
        """

        command = decision_input.command
        if command.mission_event_id is not None:
            # Mission events are validated and applied by the comms transaction
            # path (target match, perimeter, observation sequence), which calls
            # note_mission_event_applied before publishing.  Record the id for
            # observability, then keep going: returning here would swallow any
            # command carried by the same publication.
            self._last_consumed_mission_event_id = command.mission_event_id

        command_id = command.last_command_id
        if command_id is None or command_id == self._last_consumed_command_id:
            return

        # Mark as consumed before attempting the transition so a rejected
        # command doesn't replay forever.
        self._last_consumed_command_id = command_id

        if command.last_status != AckStatus.ACCEPTED:
            return
        if command.last_command is None:
            return

        if (
            command.last_command == CommandName.STOP_CHASE
            and self._fsm.state == FsmState.BRAKE_REVERSE
            and self._brake_saved_state
            not in {
                FsmState.GETTING_CLOSE,
                FsmState.SEARCH,
                FsmState.CHASE,
            }
        ):
            # Rejected per transition matrix 9.2: no state or objective mutation.
            return

        mapping = _COMMAND_TO_EVENT.get(command.last_command)
        if mapping is None:
            return  # set_home and unknown commands are not FSM events

        event, reason = mapping
        result = self._fsm.apply(event, reason=reason, now_ms=decision_input.now_ms)

        if command.last_command in _OBJECTIVE_ENDING_COMMANDS:
            self.clear_brake_reverse_context(reset_attempts=True)

        if command.last_command == CommandName.START_CHASE:
            # Only bind the objective when the FSM actually entered the chase;
            # a rejected start must not leave an active target behind.
            if result.accepted:
                self._active_target_id = command.target_id
        elif command.last_command == CommandName.STOP_CHASE:
            # Stopping always releases the objective, including the idempotent
            # HOME/IDLE case where the FSM has no transition to make.
            self._active_target_id = None

    def _apply_chase_matrix(
        self,
        decision_input: DecisionInput,
        constraints: List[str],
        *,
        range_healthy: bool,
        lidar_healthy: bool,
    ) -> Optional[DecisionOutput]:
        """Apply Slice 5 chase transitions and bounded retention timers."""

        state = self._fsm.state
        now_ms = decision_input.now_ms

        if state == FsmState.IDLE and self._handoff_deadline_ms is not None:
            if now_ms >= self._handoff_deadline_ms:
                self._handoff_deadline_ms = None
                return self._return_home_or_failsafe(
                    decision_input,
                    constraints,
                    event=FsmEvent.HANDOFF_TIMEOUT,
                    reason=ReasonCode.HANDOFF_TIMEOUT,
                    range_healthy=range_healthy,
                    lidar_healthy=lidar_healthy,
                )
            constraints.append("handoff_wait")
            return None

        if state not in {
            FsmState.GETTING_CLOSE,
            FsmState.SEARCH,
            FsmState.CHASE,
        }:
            return None

        overhead_valid, target_distance_cm = self._overhead_target_status(
            decision_input
        )
        selected_target = decision_input.overhead.selected_target_id
        if (
            state in {FsmState.GETTING_CLOSE, FsmState.SEARCH}
            and selected_target is not None
            and self._active_target_id is not None
            and selected_target != self._active_target_id
        ):
            self._fsm.apply(
                FsmEvent.TARGET_ID_CHANGED,
                reason=ReasonCode.TARGET_ID_CHANGED,
                now_ms=now_ms,
            )
            self._active_target_id = None
            self._reset_chase_context()
            constraints.append("target_id_changed")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.TARGET_ID_CHANGED,
                constraints=constraints,
                brake=True,
            )

        if state in {FsmState.GETTING_CLOSE, FsmState.SEARCH}:
            if overhead_valid:
                self._overhead_invalid_started_ms = None
                self._last_valid_target_x = decision_input.overhead.cat.x
                self._last_valid_target_y = decision_input.overhead.cat.y
            else:
                if self._overhead_invalid_started_ms is None:
                    self._overhead_invalid_started_ms = now_ms
                invalid_age = now_ms - self._overhead_invalid_started_ms
                constraints.extend(
                    ["overhead_invalid_retention", "search_speed_cap"]
                )
                if invalid_age >= int(
                    self._target_config.overhead_invalid_max_sec * 1000
                ):
                    return self._return_home_or_failsafe(
                        decision_input,
                        constraints,
                        event=FsmEvent.OVERHEAD_RETENTION_EXPIRED,
                        reason=ReasonCode.OVERHEAD_EXPIRED,
                        range_healthy=range_healthy,
                        lidar_healthy=lidar_healthy,
                    )

        if (
            state == FsmState.GETTING_CLOSE
            and overhead_valid
            and target_distance_cm is not None
            and target_distance_cm
            <= self._target_config.search_entry_distance_cm
        ):
            self._fsm.apply(
                FsmEvent.SEARCH_ENTRY_READY,
                reason=ReasonCode.SEARCH_ENTRY,
                now_ms=now_ms,
            )
            self._begin_search(now_ms)
            state = self._fsm.state

        if state == FsmState.SEARCH:
            search_output = self._apply_search_acquisition(
                decision_input,
                constraints,
                range_healthy=range_healthy,
                lidar_healthy=lidar_healthy,
            )
            if search_output is not None:
                return search_output

        if state == FsmState.CHASE:
            vision = decision_input.vision
            vision_fresh = self._age_fresh(
                vision.received_ms,
                self._target_config.local_track_stale_ms,
                now_ms,
            )
            associated = (
                vision.associated_target_id == self._active_target_id
                and not vision.association_ambiguous
            )
            if not vision_fresh or not vision.cat_visible or not associated:
                if overhead_valid and target_distance_cm is not None:
                    near = (
                        target_distance_cm
                        <= self._target_config.search_entry_distance_cm
                    )
                    self._fsm.apply(
                        (
                            FsmEvent.CAT_LOST_NEAR
                            if near
                            else FsmEvent.CAT_LOST_FAR
                        ),
                        reason=(
                            ReasonCode.CAT_LOST_NEAR
                            if near
                            else ReasonCode.CAT_LOST_FAR
                        ),
                        now_ms=now_ms,
                    )
                    if near:
                        self._begin_search(now_ms)
                else:
                    return self._return_home_or_failsafe(
                        decision_input,
                        constraints,
                        event=FsmEvent.OVERHEAD_RETENTION_EXPIRED,
                        reason=ReasonCode.OVERHEAD_EXPIRED,
                        range_healthy=range_healthy,
                        lidar_healthy=lidar_healthy,
                    )

        return None

    def _apply_search_acquisition(
        self,
        decision_input: DecisionInput,
        constraints: List[str],
        *,
        range_healthy: bool,
        lidar_healthy: bool,
    ) -> Optional[DecisionOutput]:
        vision = decision_input.vision
        now_ms = decision_input.now_ms
        vision_fresh = self._age_fresh(
            vision.received_ms,
            self._target_config.local_track_stale_ms,
            now_ms,
        )
        new_observation = (
            vision.observation_sequence
            != self._last_vision_observation_sequence
        )
        if new_observation:
            self._last_vision_observation_sequence = (
                vision.observation_sequence
            )
            associated = (
                vision_fresh
                and vision.cat_visible
                and not vision.association_ambiguous
                and vision.associated_target_id == self._active_target_id
            )
            self._search_lock_count = (
                self._search_lock_count + 1 if associated else 0
            )

        if (
            self._search_lock_count
            >= self._target_config.search_lock_observations
        ):
            self._fsm.apply(
                FsmEvent.LOCAL_TRACK_ACQUIRED,
                reason=ReasonCode.LOCAL_TRACK,
                now_ms=now_ms,
            )
            self._search_started_ms = None
            self._search_stage = 0
            return None

        constraints.append("search_acquiring")
        if self._search_started_ms is None:
            self._search_started_ms = now_ms
        interval_ms = int(self._target_config.search_interval_sec * 1000)
        if now_ms - self._search_started_ms >= interval_ms:
            if self._search_stage == 0:
                # Stage 1 asks NavigationManager for one observation waypoint;
                # whether a collision-free waypoint exists is decided there, so
                # this constraint only reports that the stage is active.
                self._search_stage = 1
                self._search_started_ms = now_ms
                self._search_lock_count = 0
                constraints.append("search_observation_stage")
            else:
                return self._return_home_or_failsafe(
                    decision_input,
                    constraints,
                    event=FsmEvent.SEARCH_EXHAUSTED,
                    reason=ReasonCode.SEARCH_EXHAUSTED,
                    range_healthy=range_healthy,
                    lidar_healthy=lidar_healthy,
                )
        return None

    def _overhead_target_status(
        self, decision_input: DecisionInput
    ) -> tuple[bool, Optional[float]]:
        overhead = decision_input.overhead
        age_fresh = self._age_fresh(
            overhead.received_ms,
            OVERHEAD_STALE_WARNING_MS,
            decision_input.now_ms,
        )
        threshold = self._target_config.overhead_min_confidence
        min_confidence = 0.0 if threshold is None else threshold
        values = (
            overhead.car.x,
            overhead.car.y,
            overhead.cat.x,
            overhead.cat.y,
        )
        valid = (
            age_fresh
            and overhead.selected_target_id == self._active_target_id
            and overhead.cat.target_id == self._active_target_id
            and overhead.cat.inside_perimeter
            and overhead.car.confidence > min_confidence
            and overhead.cat.confidence > min_confidence
            and all(math.isfinite(value) for value in values)
        )
        if not valid:
            return False, None
        return True, math.hypot(
            overhead.cat.x - overhead.car.x,
            overhead.cat.y - overhead.car.y,
        )

    def _return_home_or_failsafe(
        self,
        decision_input: DecisionInput,
        constraints: List[str],
        *,
        event: FsmEvent,
        reason: ReasonCode,
        range_healthy: bool,
        lidar_healthy: bool,
    ) -> DecisionOutput:
        self._reset_chase_context()
        ok, _ = safe_return_possible(
            home=decision_input.home,
            mission=decision_input.mission,
            range_healthy=range_healthy,
            lidar_healthy=lidar_healthy,
            geofence=decision_input.geofence,
            navigation=decision_input.navigation,
        )
        if ok:
            result = self._fsm.apply(
                event,
                reason=reason,
                now_ms=decision_input.now_ms,
            )
            if result.accepted:
                constraints.append("return_home_fallback")
                return self._safe_stop_output(
                    decision_input,
                    reason=reason,
                    constraints=constraints,
                    brake=True,
                )
            # Safe return was possible but the FSM refused the transition, which
            # is a different fault than an unsafe return.
            return self._enter_failsafe(
                decision_input,
                reason=reason,
                constraint="return_home_transition_rejected",
            )
        return self._enter_failsafe(
            decision_input,
            reason=reason,
            constraint="safe_return_unavailable",
        )

    def _begin_search(self, now_ms: int) -> None:
        self._search_started_ms = now_ms
        self._search_stage = 0
        self._search_lock_count = 0
        self._last_vision_observation_sequence = None

    def _reset_chase_context(self) -> None:
        self._overhead_invalid_started_ms = None
        self._last_valid_target_x = None
        self._last_valid_target_y = None
        self._search_started_ms = None
        self._search_stage = 0
        self._search_lock_count = 0
        self._last_vision_observation_sequence = None

    @staticmethod
    def _overhead_age_ms(decision_input: DecisionInput) -> Optional[int]:
        """Return monotonic age of latest overhead packet, or None if never received."""

        if decision_input.overhead.received_ms <= 0:
            return None
        return decision_input.now_ms - decision_input.overhead.received_ms

    @staticmethod
    def _age_fresh(received_ms: int, ttl_ms: int, now_ms: int) -> bool:
        """Age-based freshness computed at decision time (Interface spec 12.6).

        Ignores the sticky published ``fresh`` flag: a group is only trusted
        when it was actually received (``received_ms > 0``) and its age is
        within ``ttl_ms``.  A dead producer thread ages out and fails closed.
        """
        if received_ms <= 0:
            return False
        return (now_ms - received_ms) <= ttl_ms

    @staticmethod
    def _obstacle_sensor_usable(rs: RangeState, ttl_ms: int, now_ms: int) -> bool:
        """True when an obstacle sensor is fresh AND returning a valid reading.

        Fail-closed helper for the drive-permission gate: a fresh-but-faulted
        sensor (``distance_cm is None`` / ``confidence == 0``) does NOT count as
        "clear" -- otherwise a stuck sensor that keeps publishing ``None`` would
        silently disable obstacle protection (see review finding #11).
        """
        if rs.received_ms <= 0:
            return False
        if (now_ms - rs.received_ms) > ttl_ms:
            return False
        if rs.distance_cm is None or rs.confidence <= 0.0:
            return False
        return True

    def _sequence_drive_output(
        self,
        decision_input: DecisionInput,
        constraints: List[str],
    ) -> Optional[DecisionOutput]:
        """Apply Movement-tab sequence motion when the executor is active."""

        if self._sequence is None or not self._sequence.is_running:
            return None

        current_state = self._fsm.state
        if current_state not in _SEQUENCE_DRIVE_STATES:
            self._abort_sequence(f"fsm_blocked:{current_state.value}")
            constraints.append("sequence_blocked_fsm")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.MANUAL_SEQUENCE,
                constraints=constraints,
                brake=True,
            )

        range_usable = self._obstacle_sensor_usable(
            decision_input.range, RANGE_STALE_MS, decision_input.now_ms
        )
        lidar_usable = self._obstacle_sensor_usable(
            decision_input.lidar, LIDAR_STALE_MS, decision_input.now_ms
        )
        if not (range_usable and lidar_usable):
            self._abort_sequence("obstacle_sensor_unavailable")
            constraints.append("obstacle_sensor_unavailable")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.MANUAL_SEQUENCE,
                constraints=constraints,
                brake=True,
            )

        if self._any_sensor_below(
            decision_input, self.close_obstacle_trigger_cm
        ):
            # Same trigger distance as the autonomous path, which enters the
            # recoverable BRAKE_REVERSE sequence.  BRAKE_REVERSE is unreachable
            # from IDLE/HOME, so the manual equivalent is to abort the plan and
            # hold zero motion: the operator must submit a new sequence, but a
            # routine obstacle no longer latches FAILSAFE.
            self._abort_sequence("obstacle_too_close")
            constraints.append("obstacle_too_close")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.OBSTACLE_TOO_CLOSE,
                constraints=constraints,
                brake=True,
            )

        # Advance only after all state and safety gates pass. A blocked sequence
        # is aborted above, so elapsed wall time can never fast-forward an
        # open-loop plan and later resume it without a new operator request.
        self._sequence.advance(decision_input.now_ms)
        cmd = self._sequence.motion_command(decision_input.now_ms)
        if cmd is None:
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.MANUAL_SEQUENCE,
                constraints=constraints,
                brake=True,
            )

        constraints.append("manual_sequence")
        return DecisionOutput(
            timestamp_ms=decision_input.now_ms,
            requested_state=current_state,
            speed=cmd.speed,
            steering=cmd.steering,
            brake=cmd.brake,
            reason=ReasonCode.MANUAL_SEQUENCE,
            active_constraints=tuple(constraints),
        )

    def _abort_sequence(self, reason: str) -> None:
        """Abort an active Movement sequence without allowing auto-resume."""

        if self._sequence is not None and self._sequence.is_running:
            self._sequence.stop(reason)

    def _navigation_drive_output(
        self,
        decision_input: DecisionInput,
        current_state: FsmState,
        constraints: List[str],
    ) -> DecisionOutput:
        """Apply current Nav2 output for GOTO and overhead chase movement."""
        nav = decision_input.navigation
        speed_cap = max(0.0, nav.speed_limit) * MAX_SPEED
        if nav.speed_cap_mps > 0.0:
            speed_cap = min(
                speed_cap,
                nav.speed_cap_mps / PLANNER_FULL_SCALE_MPS,
            )
        if (
            self._thermal_critical_return_active
            and current_state == FsmState.RETURN_HOME
        ):
            thermal_cap = (
                self._target_config.thermal_critical_return_speed_cap_mps
                / PLANNER_FULL_SCALE_MPS
            )
            speed_cap = min(speed_cap, thermal_cap)
            constraints.append("thermal_critical_return_cap")

        if nav.authority == "NavigationManager" and not nav.path_viable:
            constraints.append("navigation_path_blocked")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.NAVIGATION_PATH_BLOCKED,
                constraints=constraints,
                brake=True,
            )

        if current_state == FsmState.CHASE:
            camera_request = max(
                -1.0, min(1.0, decision_input.vision.x_offset_norm)
            )
            safe_min = max(-1.0, min(1.0, nav.safe_steering_min))
            safe_max = max(-1.0, min(1.0, nav.safe_steering_max))
            if safe_min > safe_max:
                constraints.append("navigation_envelope_invalid")
                return self._safe_stop_output(
                    decision_input,
                    reason=ReasonCode.NAVIGATION_PATH_BLOCKED,
                    constraints=constraints,
                    brake=True,
                )
            # Canonical non-additive fusion: camera requests steering and Nav2
            # supplies the permitted envelope.  path_correction is not summed.
            final_steer = max(safe_min, min(safe_max, camera_request))
            constraints.append("camera_steering_clamped")
        else:
            final_steer = max(-1.0, min(1.0, nav.path_correction))

        if current_state == FsmState.GOTO:
            final_speed = speed_cap
            target_source = TargetSource.GO_TO
        elif current_state == FsmState.RETURN_HOME:
            final_speed = speed_cap
            target_source = TargetSource.HOME
        elif current_state == FsmState.CHASE:
            final_speed = speed_cap
            target_source = TargetSource.CAT_LOCAL
        else:
            final_speed = speed_cap
            if (
                current_state == FsmState.SEARCH
                or self._overhead_invalid_started_ms is not None
            ):
                search_cap = min(
                    1.0,
                    self._target_config.search_speed_cap_mps
                    / PLANNER_FULL_SCALE_MPS,
                )
                final_speed = min(final_speed, search_cap)
                constraints.append("search_speed_cap")
            target_source = TargetSource.CAT_GLOBAL

        constraints.append("navigation")
        if nav.no_progress:
            constraints.append("no_progress")
        if nav.dead_end:
            constraints.append("dead_end")

        # Only chase objectives may publish the retained overhead cat position;
        # GOTO/RETURN_HOME goals are owned by NavigationManager.
        chase_target = target_source in {
            TargetSource.CAT_GLOBAL,
            TargetSource.CAT_LOCAL,
        }
        return DecisionOutput(
            timestamp_ms=decision_input.now_ms,
            requested_state=self._fsm.state,
            speed=final_speed,
            steering=final_steer,
            brake=False,
            reason=_STATE_DEFAULT_REASON.get(current_state, ReasonCode.INIT),
            active_constraints=tuple(constraints),
            target_x=self._last_valid_target_x if chase_target else None,
            target_y=self._last_valid_target_y if chase_target else None,
            target_source=target_source,
            rejected_transition=False,
        )

    def _safe_stop_output(
        self,
        decision_input: DecisionInput,
        *,
        reason: ReasonCode,
        constraints: List[str],
        brake: bool,
    ) -> DecisionOutput:
        return DecisionOutput(
            timestamp_ms=decision_input.now_ms,
            requested_state=self._fsm.state,
            speed=0.0,
            steering=0.0,
            brake=brake,
            reason=reason,
            active_constraints=tuple(constraints),
            target_x=None,
            target_y=None,
            target_source=TargetSource.NONE,
            rejected_transition=False,
        )


__all__ = [
    "DecisionEngine",
    "MAX_SPEED",
    "OBSTACLE_TOO_CLOSE_CM",
    "OVERHEAD_STALE_FAILSAFE_MS",
    "OVERHEAD_STALE_WARNING_MS",
]
