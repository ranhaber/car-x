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

from typing import TYPE_CHECKING, List, Optional

from cat_follow.control.fsm import FSM

if TYPE_CHECKING:
    from cat_follow.motion.sequence_executor import MotionSequenceExecutor
from cat_follow.control.types import (
    AckStatus,
    CommandName,
    DecisionInput,
    DecisionOutput,
    FsmEvent,
    FsmState,
    RangeState,
    ReasonCode,
    TargetSource,
)


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
# Vision loss window mirrors CAMERA_LOSS_FALLBACK_MS (TRACK_B -> CHASE_A).
VISION_STALE_MS = 350

# Normalized full-scale speed used when navigation drives the chassis. Speeds
# and steering are normalized to [0, 1] and [-1, 1] respectively.
MAX_SPEED = 1.0

# Chase-state set used by stage logic and overhead-freshness rules.
_CHASE_STATES = frozenset(
    {FsmState.CHASE_A, FsmState.TRACK_B, FsmState.BRAKE}
)

# States where a fresh NavigationState (from Nav2 via the ros_bridge) is
# allowed to drive path_correction / speed_limit into the motion command.
_NAV_DRIVE_STATES = frozenset({FsmState.CHASE_A, FsmState.GOTO})

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


# Mapping of FSM state to the V1 default reason code emitted by the shell
# when no other higher-priority condition is active.
_STATE_DEFAULT_REASON = {
    FsmState.HOME: ReasonCode.INIT,
    FsmState.IDLE: ReasonCode.INIT,
    FsmState.CHASE_A: ReasonCode.GLOBAL_CHASE,
    FsmState.TRACK_B: ReasonCode.LOCAL_TRACK,
    FsmState.BRAKE: ReasonCode.FINAL_APPROACH,
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
    ) -> None:
        self._fsm = fsm
        self._sequence = sequence_executor
        self._last_consumed_command_id: Optional[str] = None
        self._obstacle_too_close_cm = float(obstacle_too_close_cm)

    def set_safety_thresholds(self, config) -> None:
        """Apply runtime safety threshold updates from :mod:`safety_config`."""

        self._obstacle_too_close_cm = float(config.obstacle_too_close_cm)

    @property
    def obstacle_too_close_cm(self) -> float:
        return self._obstacle_too_close_cm

    def tick(self, decision_input: DecisionInput) -> DecisionOutput:
        constraints: List[str] = []

        # 1. Hard failsafe: range/lidar obstacle within ``OBSTACLE_TOO_CLOSE_CM``.
        range_close = self._range_obstacle_too_close(decision_input)
        lidar_close = self._lidar_obstacle_too_close(decision_input)
        if range_close or lidar_close:
            self._abort_sequence("obstacle_too_close")
            self._fsm.apply(
                FsmEvent.OBSTACLE_TOO_CLOSE,
                reason=ReasonCode.OBSTACLE_TOO_CLOSE,
                now_ms=decision_input.now_ms,
            )
            constraints.append("obstacle_too_close")
            if lidar_close:
                constraints.append("lidar_obstacle")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.OBSTACLE_TOO_CLOSE,
                constraints=constraints,
                brake=True,
            )

        # 2. Critical obstacle veto (severity-based), from ultrasonic or lidar.
        #    Freshness is recomputed from received_ms (not the sticky flag).
        range_critical = (
            self._age_fresh(
                decision_input.range.received_ms, RANGE_STALE_MS, decision_input.now_ms
            )
            and decision_input.range.obstacle_critical
        )
        lidar_critical = (
            self._age_fresh(
                decision_input.lidar.received_ms, LIDAR_STALE_MS, decision_input.now_ms
            )
            and decision_input.lidar.obstacle_critical
        )
        if range_critical or lidar_critical:
            self._abort_sequence("obstacle_veto")
            self._fsm.apply(
                FsmEvent.FAILSAFE_TRIGGERED,
                reason=ReasonCode.OBSTACLE_VETO,
                now_ms=decision_input.now_ms,
            )
            constraints.append("obstacle_veto")
            if lidar_critical:
                constraints.append("lidar_veto")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.OBSTACLE_VETO,
                constraints=constraints,
                brake=True,
            )

        # 3. Consume any newly-accepted command and translate it into an FSM
        #    event.  Commands are validated by ``CommsManager``; here we only
        #    fire the FSM transition when status is ``accepted``.
        self._consume_accepted_command(decision_input)

        # 4. Overhead freshness checks while in a chase state.  Use the
        #    *current* FSM state (post command consumption) rather than the
        #    snapshot's state so a freshly-applied stop_chase doesn't get
        #    mis-classified as chase here.
        current_state = self._fsm.state
        is_chase = current_state in _CHASE_STATES
        overhead_age_ms = self._overhead_age_ms(decision_input)

        if is_chase and overhead_age_ms is not None:
            if overhead_age_ms > OVERHEAD_STALE_FAILSAFE_MS:
                self._abort_sequence("overhead_expired")
                self._fsm.apply(
                    FsmEvent.FAILSAFE_TRIGGERED,
                    reason=ReasonCode.OVERHEAD_EXPIRED,
                    now_ms=decision_input.now_ms,
                )
                constraints.append("overhead_expired")
                return self._safe_stop_output(
                    decision_input,
                    reason=ReasonCode.OVERHEAD_EXPIRED,
                    constraints=constraints,
                    brake=True,
                )
            if overhead_age_ms > OVERHEAD_STALE_WARNING_MS:
                constraints.append("overhead_stale")

        # 4b. Vision-driven chase handoff.  Without this the CAT_VISIBLE_STABLE
        #     / CAT_LOST transitions are unreachable and TRACK_B is dead.
        self._apply_vision_events(decision_input)

        # 5. State-specific shell behavior.
        current_state = self._fsm.state

        if current_state == FsmState.FAILSAFE:
            self._abort_sequence("failsafe")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.FAILSAFE_TRIGGERED,
                constraints=constraints,
                brake=True,
            )

        # 5b. Movement-tab open-loop sequence (contract runtime only).
        sequence_output = self._sequence_drive_output(decision_input, constraints)
        if sequence_output is not None:
            return sequence_output

        # 6. Navigation-assisted drive (CHASE_A / GOTO) when Nav2 is publishing
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
            # Fail-closed: only drive when at least one obstacle sensor is fresh
            # AND returning a valid reading.  With no usable proximity sensing we
            # refuse to drive blind (review findings #1, #11).
            range_usable = self._obstacle_sensor_usable(
                decision_input.range, RANGE_STALE_MS, decision_input.now_ms
            )
            lidar_usable = self._obstacle_sensor_usable(
                decision_input.lidar, LIDAR_STALE_MS, decision_input.now_ms
            )
            if not (range_usable or lidar_usable):
                constraints.append("obstacle_sensor_unavailable")
                return self._safe_stop_output(
                    decision_input,
                    reason=ReasonCode.OBSTACLE_VETO,
                    constraints=constraints,
                    brake=False,
                )
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

    def _consume_accepted_command(self, decision_input: DecisionInput) -> None:
        """Apply any newly-accepted command from ``SharedState.command``.

        Each command_id is consumed at most once; duplicate retries that
        CommsManager already deduplicates by ``command_id`` will not
        re-fire FSM events here either.  Rejected commands are still marked
        as consumed so the engine doesn't keep retrying them.
        """

        command = decision_input.command
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

        mapping = _COMMAND_TO_EVENT.get(command.last_command)
        if mapping is None:
            return  # set_home and unknown commands are not FSM events

        event, reason = mapping
        self._fsm.apply(event, reason=reason, now_ms=decision_input.now_ms)

    def _apply_vision_events(self, decision_input: DecisionInput) -> None:
        """Emit vision-driven FSM events for the chase handoff.

        - ``CHASE_A`` + fresh, stable cat  -> ``CAT_VISIBLE_STABLE`` (-> TRACK_B)
        - ``TRACK_B`` + cat lost/aged out   -> ``CAT_LOST``          (-> CHASE_A)

        Vision freshness is recomputed from ``received_ms`` (the adapter only
        advances it on a genuinely new tracker observation), so a frozen tracker
        ages out and reads as "cat lost" rather than a sticky visible lock.
        """
        vision = decision_input.vision
        state = self._fsm.state
        vision_fresh = self._age_fresh(
            vision.received_ms, VISION_STALE_MS, decision_input.now_ms
        )

        if state == FsmState.CHASE_A:
            if vision_fresh and vision.cat_visible_stable:
                self._fsm.apply(
                    FsmEvent.CAT_VISIBLE_STABLE,
                    reason=ReasonCode.LOCAL_TRACK,
                    now_ms=decision_input.now_ms,
                )
        elif state == FsmState.TRACK_B:
            if not vision_fresh or not vision.cat_visible:
                self._fsm.apply(
                    FsmEvent.CAT_LOST,
                    reason=ReasonCode.CAT_LOST_FALLBACK,
                    now_ms=decision_input.now_ms,
                )

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

    def _range_obstacle_too_close(self, decision_input: DecisionInput) -> bool:
        rs = decision_input.range
        if not self._age_fresh(rs.received_ms, RANGE_STALE_MS, decision_input.now_ms):
            return False
        if rs.distance_cm is None:
            return False
        return rs.distance_cm < self._obstacle_too_close_cm

    def _lidar_obstacle_too_close(self, decision_input: DecisionInput) -> bool:
        ls = decision_input.lidar
        if not self._age_fresh(ls.received_ms, LIDAR_STALE_MS, decision_input.now_ms):
            return False
        if ls.distance_cm is None:
            return False
        return ls.distance_cm < self._obstacle_too_close_cm

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
        if not (range_usable or lidar_usable):
            self._abort_sequence("obstacle_sensor_unavailable")
            constraints.append("obstacle_sensor_unavailable")
            return self._safe_stop_output(
                decision_input,
                reason=ReasonCode.MANUAL_SEQUENCE,
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
        """Blend Nav2 constraints into the motion command.

        Authority model (``final_steer = clamp(pursuit_steer + path_correction)``,
        ``final_speed = min(pursuit_speed, speed_limit * MAX_SPEED)``):

        - ``GOTO`` is a legitimate autonomous navigation goal, so Nav2 drives to
          the target with ``speed_limit`` acting as the throttle.
        - ``CHASE_A`` pursuit authority is *local* (owned by vision pursuit, not
          the ROS bridge).  Nav2 is advisory only here: ``speed_limit`` is a cap
          and ``path_correction`` a steering bias on top of the local pursuit
          term.  The V1 shell has no pursuit term yet, so ``pursuit_speed = 0``
          and the car holds in CHASE_A until vision pursuit is wired -- Nav2 is
          no longer the sole motor driver.
        """
        nav = decision_input.navigation
        pursuit_steer = 0.0
        speed_cap = max(0.0, nav.speed_limit) * MAX_SPEED

        final_steer = max(-1.0, min(1.0, pursuit_steer + nav.path_correction))

        if current_state == FsmState.GOTO:
            final_speed = speed_cap
            target_source = TargetSource.GO_TO
        else:
            # CHASE_A: local pursuit owns speed; Nav2 only caps it (advisory).
            pursuit_speed = 0.0
            final_speed = max(0.0, min(pursuit_speed, speed_cap))
            constraints.append("nav_advisory")
            target_source = TargetSource.CAT_GLOBAL

        constraints.append("navigation")
        if nav.no_progress:
            constraints.append("no_progress")
        if nav.dead_end:
            constraints.append("dead_end")

        return DecisionOutput(
            timestamp_ms=decision_input.now_ms,
            requested_state=self._fsm.state,
            speed=final_speed,
            steering=final_steer,
            brake=False,
            reason=_STATE_DEFAULT_REASON.get(current_state, ReasonCode.INIT),
            active_constraints=tuple(constraints),
            target_x=None,
            target_y=None,
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
