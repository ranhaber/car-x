"""State-derived Nav2 goal lifecycle and navigation qualification.

The manager is deliberately transport-agnostic.  Production wires a ROS
``NavigateToPose`` transport; tests use a synchronous fake.  DecisionEngine
remains the sole drivetrain authority and consumes only the enriched
``NavigationState`` returned here.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import replace
import math
import threading
from typing import Callable, List, Optional, Protocol

from cat_follow.control.types import (
    CommandName,
    FsmState,
    NavigationFailureClass,
    NavigationGoalIntent,
    NavigationObjectiveType,
    NavigationResult,
    NavigationResultStatus,
    NavigationState,
    SharedSnapshot,
    TelemetryEventType,
    TelemetrySeverity,
)
from cat_follow.navigation.safe_return import (
    frozen_home_pose,
    home_has_map_pose,
    home_is_valid,
)
from cat_follow.target_config import TargetRuntimeConfig


NAV_POSE_STALE_MS = 500
DEFAULT_MAX_FAILURES = 2

# Superseded action IDs are forgotten once their terminal result arrives.  A
# transport that never reports one (detached node, lost server) must not grow
# the set without bound over a long mission.
MAX_EXPECTED_REPLACEMENTS = 32


class NavigationTransport(Protocol):
    """Minimal action-client boundary owned by :class:`NavigationManager`.

    Both methods are called *without* the manager lock held and must return
    promptly.  A transport must not report a terminal result synchronously
    from inside :meth:`submit_goal`; the manager records the correlation ID
    only after the call returns.
    """

    def submit_goal(self, intent: NavigationGoalIntent) -> str:
        """Submit ``intent`` and return the Nav2 action goal correlation ID."""

    def cancel_goal(self, action_goal_id: str) -> None:
        """Cancel a submitted action goal immediately."""


YardTransform = Callable[
    [float, float, float, str], tuple[float, float, float, str]
]
ObservationWaypointProvider = Callable[
    [NavigationGoalIntent], Optional[tuple[float, float, float, str]]
]


def default_yard_to_map(
    x_cm: float, y_cm: float, yaw_rad: float, frame_id: str
) -> tuple[float, float, float, str]:
    """Development transform for aligned yard/map frames.

    Production calibration may inject a different transform.  The default only
    performs the contract's centimeters-to-meters unit conversion.
    """

    if frame_id not in {"yard", "map"}:
        raise ValueError(f"unsupported navigation source frame: {frame_id!r}")
    return x_cm / 100.0, y_cm / 100.0, yaw_rad, "map"


class NavigationManager:
    """Own Nav2 objective submission, correlation, retries, and completion."""

    def __init__(
        self,
        config: TargetRuntimeConfig,
        *,
        transport: Optional[NavigationTransport] = None,
        yard_to_map: YardTransform = default_yard_to_map,
        observation_waypoint_provider: Optional[
            ObservationWaypointProvider
        ] = None,
        max_failures: int = DEFAULT_MAX_FAILURES,
        logger=None,
    ) -> None:
        if max_failures < 1:
            raise ValueError("max_failures must be >= 1")
        self._config = config
        self._transport = transport
        self._yard_to_map = yard_to_map
        self._observation_waypoint_provider = observation_waypoint_provider
        self._max_failures = max_failures
        self._logger = logger
        self._lock = threading.RLock()
        self._intent_counter = 0
        self._active: Optional[NavigationGoalIntent] = None
        self._last_result: Optional[NavigationResult] = None
        self._expected_replacements: "OrderedDict[str, None]" = OrderedDict()
        # Bumped whenever the active goal changes so a submit that is still in
        # flight cannot adopt its action ID after being superseded.
        self._submit_token = 0
        self._failure_count = 0
        self._failures_exhausted = False
        self._succeeded_correlation: Optional[tuple[str, str]] = None
        self._dwell_started_ms: Optional[int] = None
        self._ignored_late_results = 0
        self._observation_stage_handled = False

    @property
    def active_intent(self) -> Optional[NavigationGoalIntent]:
        with self._lock:
            return self._active

    @property
    def ignored_late_results(self) -> int:
        with self._lock:
            return self._ignored_late_results

    @property
    def healthy(self) -> bool:
        with self._lock:
            return self._transport is not None

    def set_transport(self, transport: Optional[NavigationTransport]) -> None:
        with self._lock:
            self._transport = transport
            self._submit_token += 1

    def cancel(self, *, expected_replacement: bool = False) -> None:
        """Cancel the active action immediately, bypassing refresh filters."""

        with self._lock:
            active = self._active
            transport = self._transport
            if active is None:
                return
            action_id = active.action_goal_id
            self._submit_token += 1
            if expected_replacement and action_id:
                self._remember_replacement(action_id)
                self._active = replace(active, expected_replacement=True)
            else:
                self._active = None
                self._reset_completion()
        if action_id and transport is not None:
            transport.cancel_goal(action_id)

    def handle_result(
        self,
        *,
        goal_intent_id: str,
        action_goal_id: str,
        status: NavigationResultStatus,
        completed_at_ms: int,
        result_code: Optional[int] = None,
        failure_class: Optional[NavigationFailureClass] = None,
    ) -> bool:
        """Accept a correlated terminal result; ignore neutral/late results."""

        with self._lock:
            if action_goal_id in self._expected_replacements:
                self._expected_replacements.pop(action_goal_id, None)
                self._log_result(
                    goal_intent_id,
                    action_goal_id,
                    status,
                    "expected_replacement",
                )
                return False
            active = self._active
            if (
                active is None
                or active.goal_intent_id != goal_intent_id
                or active.action_goal_id != action_goal_id
            ):
                self._ignored_late_results += 1
                self._log_result(
                    goal_intent_id,
                    action_goal_id,
                    status,
                    "correlation_mismatch",
                )
                return False

            result = NavigationResult(
                goal_intent_id=goal_intent_id,
                action_goal_id=action_goal_id,
                status=status,
                result_code=result_code,
                failure_class=failure_class,
                completed_at_ms=completed_at_ms,
            )
            self._last_result = result
            self._log_result(
                goal_intent_id, action_goal_id, status, "accepted"
            )
            if status == NavigationResultStatus.SUCCEEDED:
                self._succeeded_correlation = (
                    goal_intent_id,
                    action_goal_id,
                )
                self._dwell_started_ms = None
                return True

            if status == NavigationResultStatus.CANCELED:
                failure_class = (
                    failure_class or NavigationFailureClass.PREEMPTED
                )
            self._last_result = replace(result, failure_class=failure_class)
            self._failure_count += 1
            self._failures_exhausted = (
                self._failure_count >= self._max_failures
            )
            if not self._failures_exhausted:
                # Clear correlation so the same objective is re-submitted on
                # the next control tick with a new action goal ID.
                self._active = replace(active, action_goal_id=None)
            return True

    def tick(
        self,
        snapshot: SharedSnapshot,
        state: FsmState,
        now_ms: int,
    ) -> NavigationState:
        """Synchronize the state-derived objective and enrich nav policy."""

        raw = snapshot.navigation
        desired = self._desired_goal(snapshot, state, now_ms)
        if desired is None:
            if state in {
                FsmState.HOME,
                FsmState.IDLE,
                FsmState.BRAKE_REVERSE,
                FsmState.FAILSAFE,
            }:
                self.cancel()
        else:
            self._sync_goal(desired, now_ms)

        with self._lock:
            active = self._active
            completion = self._qualify_completion(raw, active, state, now_ms)
            stationary_search = bool(
                state == FsmState.SEARCH
                and snapshot.mission.search_stage > 0
                and active is None
            )
            path_viable = bool(
                not stationary_search
                and (
                    raw.path_viable
                    or (
                        raw.received_ms > 0
                        and raw.fresh
                        and not raw.dead_end
                        and not raw.no_progress
                    )
                )
            )
            safe_min = raw.safe_steering_min
            safe_max = raw.safe_steering_max
            if path_viable and safe_min == safe_max == 0.0:
                safe_min, safe_max = -1.0, 1.0
            speed_cap = (
                raw.speed_cap_mps
                if raw.speed_cap_mps > 0.0
                else max(0.0, raw.speed_limit) * 0.30
            )
            return replace(
                raw,
                authority="NavigationManager",
                healthy=self._transport is not None,
                path_viable=path_viable,
                safe_steering_min=safe_min,
                safe_steering_max=safe_max,
                speed_cap_mps=speed_cap,
                goal_intent=active,
                last_result=self._last_result,
                completion_qualified=completion,
                failures_exhausted=self._failures_exhausted,
            )

    def _desired_goal(
        self, snapshot: SharedSnapshot, state: FsmState, now_ms: int
    ) -> Optional[NavigationGoalIntent]:
        if state in {FsmState.GETTING_CLOSE, FsmState.SEARCH}:
            overhead = snapshot.overhead
            target_id = snapshot.overhead.selected_target_id
            if (
                target_id is None
                or overhead.cat.target_id != target_id
                or not overhead.cat.inside_perimeter
            ):
                return None
            x_m, y_m, yaw, frame = self._yard_to_map(
                overhead.cat.x,
                overhead.cat.y,
                0.0,
                overhead.frame_id,
            )
            objective = (
                NavigationObjectiveType.GETTING_CLOSE
                if state == FsmState.GETTING_CLOSE
                else NavigationObjectiveType.SEARCH
            )
            target_goal = self._new_desired(
                objective,
                x_m,
                y_m,
                yaw,
                frame,
                now_ms,
                target_id=target_id,
                moving=True,
            )
            if state == FsmState.SEARCH and snapshot.mission.search_stage > 0:
                if not self._observation_stage_handled:
                    self._observation_stage_handled = True
                    provider = self._observation_waypoint_provider
                    waypoint = provider(target_goal) if provider else None
                    if waypoint is None:
                        self.cancel()
                        return None
                    x_m, y_m, yaw, frame = waypoint
                    return self._new_desired(
                        NavigationObjectiveType.SEARCH_OBSERVATION,
                        x_m,
                        y_m,
                        yaw,
                        frame,
                        now_ms,
                        target_id=target_id,
                    )
                active = self._active
                if (
                    active is not None
                    and active.objective_type
                    == NavigationObjectiveType.SEARCH_OBSERVATION
                ):
                    return replace(active, action_goal_id=None)
                return None
            self._observation_stage_handled = False
            return target_goal

        if state == FsmState.GOTO:
            self._observation_stage_handled = False
            command = snapshot.command
            if (
                command.last_command != CommandName.GO_TO
                or command.objective_x_cm is None
                or command.objective_y_cm is None
            ):
                return None
            x_m, y_m, yaw, frame = self._yard_to_map(
                command.objective_x_cm,
                command.objective_y_cm,
                command.objective_yaw_rad,
                command.objective_frame_id or "yard",
            )
            return self._new_desired(
                NavigationObjectiveType.GOTO,
                x_m,
                y_m,
                yaw,
                frame,
                now_ms,
            )

        if state == FsmState.RETURN_HOME:
            self._observation_stage_handled = False
            home = snapshot.home
            if snapshot.mission.home_version_frozen is not None:
                # A frozen mission home must resolve through the same predicate
                # safe_return_possible() uses, or RETURN_HOME gets no goal.
                pose = frozen_home_pose(snapshot.mission, home)
                if pose is None:
                    return None
                _x_cm, _y_cm, yaw, _frame, x_m, y_m = pose
                return self._new_desired(
                    NavigationObjectiveType.RETURN_HOME,
                    x_m,
                    y_m,
                    yaw,
                    "map",
                    now_ms,
                )
            if not home_is_valid(home):
                return None
            if home_has_map_pose(home):
                x_m, y_m, yaw, frame = (
                    float(home.x_m),
                    float(home.y_m),
                    float(home.yaw_rad),
                    "map",
                )
            else:
                x_m, y_m, yaw, frame = self._yard_to_map(
                    home.x,
                    home.y,
                    home.yaw_rad,
                    home.frame_id,
                )
            return self._new_desired(
                NavigationObjectiveType.RETURN_HOME,
                x_m,
                y_m,
                yaw,
                frame,
                now_ms,
            )
        return None

    def _new_desired(
        self,
        objective: NavigationObjectiveType,
        x_m: float,
        y_m: float,
        yaw: float,
        frame: str,
        now_ms: int,
        *,
        target_id: Optional[str] = None,
        moving: bool = False,
    ) -> NavigationGoalIntent:
        return NavigationGoalIntent(
            goal_intent_id="",
            objective_type=objective,
            target_id=target_id,
            frame_id=frame,
            x_m=x_m,
            y_m=y_m,
            yaw_rad=yaw,
            moving_goal=moving,
            requested_at_ms=now_ms,
            last_refresh_ms=now_ms,
        )

    def _sync_goal(
        self, desired: NavigationGoalIntent, now_ms: int
    ) -> None:
        cancel_ids: List[str] = []
        with self._lock:
            transport = self._transport
            active = self._active
            same_objective = bool(
                active
                and active.objective_type == desired.objective_type
                and active.target_id == desired.target_id
            )
            if same_objective and active is not None:
                if self._failures_exhausted:
                    # The retry budget for this objective is spent.  Hold the
                    # last correlation so DecisionEngine can route on
                    # failures_exhausted instead of submitting more goals.
                    return
                if active.action_goal_id is None:
                    pending = replace(
                        desired, goal_intent_id=active.goal_intent_id
                    )
                else:
                    if not desired.moving_goal:
                        return
                    min_interval_ms = int(
                        1000.0 / self._config.nav_moving_goal_max_hz
                    )
                    displacement_cm = (
                        math.hypot(
                            desired.x_m - active.x_m,
                            desired.y_m - active.y_m,
                        )
                        * 100.0
                    )
                    if (
                        now_ms - active.last_refresh_ms < min_interval_ms
                        or displacement_cm
                        < self._config.nav_moving_goal_min_displacement_cm
                    ):
                        return
                    old_action_id = active.action_goal_id
                    self._remember_replacement(old_action_id)
                    cancel_ids.append(old_action_id)
                    pending = replace(
                        desired,
                        goal_intent_id=active.goal_intent_id,
                        requested_at_ms=active.requested_at_ms,
                        refresh_count=active.refresh_count + 1,
                        last_refresh_ms=now_ms,
                    )
            else:
                if active is not None and active.action_goal_id:
                    self._remember_replacement(active.action_goal_id)
                    cancel_ids.append(active.action_goal_id)
                self._intent_counter += 1
                self._failure_count = 0
                self._failures_exhausted = False
                self._reset_completion()
                pending = replace(
                    desired,
                    goal_intent_id=f"gi-{self._intent_counter:06d}",
                )
            token = self._begin_submit(pending)
        self._execute_goal_io(transport, pending, token, cancel_ids)

    def _begin_submit(self, intent: NavigationGoalIntent) -> int:
        """Publish ``intent`` as active before any transport I/O runs."""

        self._active = replace(
            intent, action_goal_id=None, expected_replacement=False
        )
        self._submit_token += 1
        return self._submit_token

    def _execute_goal_io(
        self,
        transport: Optional[NavigationTransport],
        intent: NavigationGoalIntent,
        token: int,
        cancel_ids: List[str],
    ) -> None:
        """Run cancel/submit without the manager lock held.

        Holding the lock across the action client would let a slow transport
        block result callbacks and DecisionEngine reads of the same state.
        """

        if transport is None:
            return
        for action_id in cancel_ids:
            transport.cancel_goal(action_id)
        action_goal_id = str(transport.submit_goal(intent))
        superseded = False
        with self._lock:
            active = self._active
            if (
                self._submit_token == token
                and active is not None
                and active.goal_intent_id == intent.goal_intent_id
                and active.action_goal_id is None
            ):
                self._active = replace(active, action_goal_id=action_goal_id)
            else:
                # Cancelled or replaced while the submit was in flight; the
                # action must not outlive the intent that requested it.
                self._remember_replacement(action_goal_id)
                superseded = True
        if superseded:
            transport.cancel_goal(action_goal_id)
            return
        self._log_goal(intent, action_goal_id)

    def _remember_replacement(self, action_goal_id: str) -> None:
        """Mark a superseded action ID so its terminal result stays neutral."""

        self._expected_replacements.pop(action_goal_id, None)
        self._expected_replacements[action_goal_id] = None
        while len(self._expected_replacements) > MAX_EXPECTED_REPLACEMENTS:
            self._expected_replacements.popitem(last=False)

    def _log_goal(
        self, intent: NavigationGoalIntent, action_goal_id: str
    ) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.NAVIGATION_GOAL,
            severity=TelemetrySeverity.INFO,
            source="NavigationManager",
            state=None,
            data={
                "goal_intent_id": intent.goal_intent_id,
                "objective_type": intent.objective_type.value,
                "target_id": intent.target_id,
                "moving_goal": intent.moving_goal,
                "action_goal_id": action_goal_id,
                "refresh_count": intent.refresh_count,
            },
        )

    def _qualify_completion(
        self,
        raw: NavigationState,
        active: Optional[NavigationGoalIntent],
        state: FsmState,
        now_ms: int,
    ) -> bool:
        if (
            active is None
            or state not in {FsmState.GOTO, FsmState.RETURN_HOME}
            or self._succeeded_correlation
            != (active.goal_intent_id, active.action_goal_id)
        ):
            self._dwell_started_ms = None
            return False
        pose_fresh = (
            raw.pose_received_ms > 0
            and now_ms - raw.pose_received_ms <= NAV_POSE_STALE_MS
        )
        xy_error_cm = (
            math.hypot(
                raw.pose_x_m - active.x_m,
                raw.pose_y_m - active.y_m,
            )
            * 100.0
        )
        yaw_error = abs(
            math.atan2(
                math.sin(raw.pose_yaw_rad - active.yaw_rad),
                math.cos(raw.pose_yaw_rad - active.yaw_rad),
            )
        )
        in_tolerance = (
            pose_fresh
            and xy_error_cm <= self._config.nav_completion_xy_cm
            and yaw_error <= self._config.nav_completion_yaw_rad
        )
        if not in_tolerance:
            self._dwell_started_ms = None
            return False
        if self._dwell_started_ms is None:
            self._dwell_started_ms = now_ms
        dwell_ms = int(self._config.nav_completion_dwell_sec * 1000)
        qualified = now_ms - self._dwell_started_ms >= dwell_ms
        if self._last_result is not None:
            self._last_result = replace(
                self._last_result,
                pose_qualified=True,
                dwell_qualified=qualified,
            )
        return qualified

    def _reset_completion(self) -> None:
        self._succeeded_correlation = None
        self._dwell_started_ms = None

    def _log_result(
        self,
        goal_intent_id: str,
        action_goal_id: str,
        status: NavigationResultStatus,
        disposition: str,
    ) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.NAVIGATION_RESULT,
            severity=(
                TelemetrySeverity.WARNING
                if disposition == "correlation_mismatch"
                else TelemetrySeverity.INFO
            ),
            source="NavigationManager",
            state=None,
            data={
                "goal_intent_id": goal_intent_id,
                "action_goal_id": action_goal_id,
                "status": status.value,
                "disposition": disposition,
            },
        )


__all__ = [
    "DEFAULT_MAX_FAILURES",
    "NAV_POSE_STALE_MS",
    "NavigationManager",
    "NavigationTransport",
    "ObservationWaypointProvider",
    "default_yard_to_map",
]
