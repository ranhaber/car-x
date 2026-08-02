"""Apply queued commands and mission events at the control-loop boundary."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import isfinite
from typing import TYPE_CHECKING, Any, Dict, Optional

from cat_follow.comms.messages import (
    CommandMessage,
    MissionEventMessage,
    PendingTransaction,
)
from cat_follow.control.decision_engine import LIDAR_STALE_MS, RANGE_STALE_MS
from cat_follow.control.fsm import CHASE_STATES
from cat_follow.control.types import (
    AckStatus,
    AckType,
    CommandName,
    CommandState,
    FsmEvent,
    FsmState,
    HomeState,
    MissionEventName,
    MissionState,
    ReasonCode,
    RejectionCause,
)
from cat_follow.home.store import HomePersistError, HomeStore
from cat_follow.navigation.geofence import point_in_polygon
from cat_follow.navigation.safe_return import (
    frozen_home_pose,
    home_is_valid,
    safe_return_possible,
)
from cat_follow.runtime.shared_state import SharedState, now_monotonic_ms

if TYPE_CHECKING:
    from cat_follow.control.decision_engine import DecisionEngine
    from cat_follow.control.fsm import FSM
    from cat_follow.navigation.geofence import GeofencePolygon


@dataclass(frozen=True)
class TransactionResult:
    """Outcome of a command or mission event applied at a control boundary."""

    ack_type: AckType
    status: AckStatus
    state: FsmState
    reason: ReasonCode
    cause: Optional[RejectionCause]
    applied_control_seq: int


def apply_pending_transaction(
    txn: PendingTransaction,
    *,
    shared_state: SharedState,
    fsm: "FSM",
    engine: "DecisionEngine",
    applied_control_seq: int,
    authority: str = "CommsManager",
    on_start_chase=None,
    home_store: Optional[HomeStore] = None,
    geofence_polygon: Optional["GeofencePolygon"] = None,
) -> TransactionResult:
    if txn.command is not None:
        result = _apply_command(
            txn.command,
            shared_state=shared_state,
            fsm=fsm,
            engine=engine,
            authority=authority,
            on_start_chase=on_start_chase,
            home_store=home_store,
            geofence_polygon=geofence_polygon,
        )
    elif txn.mission_event is not None:
        result = _apply_mission_event(
            txn.mission_event,
            shared_state=shared_state,
            fsm=fsm,
            engine=engine,
            authority=authority,
        )
    else:
        raise ValueError("pending transaction must carry a command or mission event")

    shared_state.update_fsm(fsm.snapshot(received_ms=now_monotonic_ms()))
    return TransactionResult(
        ack_type=result.ack_type,
        status=result.status,
        state=result.state,
        reason=result.reason,
        cause=result.cause,
        applied_control_seq=applied_control_seq,
    )


def _apply_command(
    msg: CommandMessage,
    *,
    shared_state: SharedState,
    fsm: "FSM",
    engine: "DecisionEngine",
    authority: str,
    on_start_chase,
    home_store: Optional[HomeStore],
    geofence_polygon: Optional["GeofencePolygon"],
) -> "_ApplyResult":
    state = fsm.state
    handlers = {
        CommandName.SET_HOME: _apply_set_home,
        CommandName.START_CHASE: _apply_start_chase,
        CommandName.STOP_CHASE: _apply_stop_chase,
        CommandName.RETURN_HOME: _apply_return_home,
        CommandName.GO_TO: _apply_go_to,
        CommandName.EMERGENCY_STOP: _apply_emergency_stop,
        CommandName.CLEAR_FAILSAFE: _apply_clear_failsafe,
    }
    handler = handlers.get(msg.command)
    if handler is None:
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.INVALID_COMMAND,
        )

    result = handler(
        msg,
        state=state,
        shared_state=shared_state,
        fsm=fsm,
        engine=engine,
        home_store=home_store,
        geofence_polygon=geofence_polygon,
    )
    if result.status == AckStatus.ACCEPTED and on_start_chase is not None:
        if msg.command == CommandName.START_CHASE:
            try:
                on_start_chase()
            except Exception:
                pass

    _publish_command_state(msg, result, shared_state, authority)
    engine.note_command_applied(msg.command_id)
    return result.with_state(fsm.state)


def _apply_mission_event(
    msg: MissionEventMessage,
    *,
    shared_state: SharedState,
    fsm: "FSM",
    engine: "DecisionEngine",
    authority: str,
) -> "_ApplyResult":
    state = fsm.state
    mission = shared_state.get_mission()
    overhead = shared_state.get_overhead()

    def rejected(cause: RejectionCause) -> "_ApplyResult":
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            cause,
            ack_type=AckType.MISSION_EVENT,
        )

    if msg.target_id != mission.active_target_id:
        return rejected(RejectionCause.WRONG_TARGET)
    if msg.perimeter_id and overhead.perimeter_id and (
        msg.perimeter_id != overhead.perimeter_id
    ):
        return rejected(RejectionCause.INVALID_PARAMS)
    if msg.observation_sequence < overhead.sequence:
        return rejected(RejectionCause.STALE_OBSERVATION)
    if msg.observation_sequence <= mission.last_event_observation_seq:
        return rejected(RejectionCause.DUPLICATE_SUPERSEDED)
    if msg.name != MissionEventName.PRIMARY_CAT_LEFT_PERIMETER:
        return rejected(RejectionCause.INVALID_PARAMS)
    if state not in {
        FsmState.GETTING_CLOSE,
        FsmState.SEARCH,
        FsmState.CHASE,
        FsmState.BRAKE_REVERSE,
    }:
        return rejected(RejectionCause.INVALID_STATE)
    if (
        state == FsmState.BRAKE_REVERSE
        and engine.brake_saved_state
        not in {
            FsmState.GETTING_CLOSE,
            FsmState.SEARCH,
            FsmState.CHASE,
        }
    ):
        # Transition matrix 9.2: the exit only applies to a BRAKE_REVERSE that
        # interrupted a chase.  The FSM table cannot see the saved objective, so
        # the predicate lives here alongside the same rule for STOP_CHASE.
        return rejected(RejectionCause.INVALID_STATE)

    applied_ms = now_monotonic_ms()
    transition = fsm.apply(
        FsmEvent.PRIMARY_CAT_LEFT_PERIMETER,
        reason=ReasonCode.PRIMARY_TARGET_EXIT_HANDOFF,
        now_ms=applied_ms,
    )
    if not transition.accepted:
        return rejected(RejectionCause.INVALID_STATE)
    engine.start_handoff(
        applied_ms,
        exited_target_id=msg.target_id,
        observation_sequence=msg.observation_sequence,
    )
    engine.start_recording_postroll(applied_ms)
    engine.note_mission_event_applied(
        msg.event_id, msg.observation_sequence
    )
    shared_state.update_mission(
        engine.mission_state
    )
    shared_state.update_command(
        CommandState(
            timestamp_ms=msg.timestamp_ms,
            received_ms=now_monotonic_ms(),
            fresh=True,
            authority=authority,
            target_id=msg.target_id,
            mission_event_id=msg.event_id,
            mission_event_name=msg.name,
            mission_event_observation_sequence=msg.observation_sequence,
        )
    )
    return _ApplyResult(
        ack_type=AckType.MISSION_EVENT,
        status=AckStatus.ACCEPTED,
        state=fsm.state,
        reason=ReasonCode.PRIMARY_TARGET_EXIT_HANDOFF,
        cause=None,
    )


@dataclass(frozen=True)
class _ApplyResult:
    ack_type: AckType
    status: AckStatus
    state: FsmState
    reason: ReasonCode
    cause: Optional[RejectionCause]

    @classmethod
    def rejected(
        cls,
        state: FsmState,
        reason: ReasonCode,
        cause: RejectionCause,
        *,
        ack_type: AckType = AckType.COMMAND,
    ) -> "_ApplyResult":
        return cls(
            ack_type=ack_type,
            status=AckStatus.REJECTED,
            state=state,
            reason=reason,
            cause=cause,
        )

    def with_state(self, state: FsmState) -> "_ApplyResult":
        return _ApplyResult(
            ack_type=self.ack_type,
            status=self.status,
            state=state,
            reason=self.reason,
            cause=self.cause,
        )


def _publish_command_state(
    msg: CommandMessage,
    result: _ApplyResult,
    shared_state: SharedState,
    authority: str,
) -> None:
    objective = None
    if msg.command == CommandName.GO_TO:
        objective = msg.params.get("target")
    elif msg.command == CommandName.RETURN_HOME:
        # RETURN_HOME no longer carries geometry; publish frozen/durable home.
        pose = frozen_home_pose(
            shared_state.get_mission(), shared_state.get_home()
        )
        if pose is not None:
            objective = {
                "x_cm": pose[0],
                "y_cm": pose[1],
                "yaw_rad": pose[2],
                "frame_id": pose[3],
            }
    if not isinstance(objective, dict):
        objective = {}
    objective_x_cm = _coordinate_cm(objective, "x")
    objective_y_cm = _coordinate_cm(objective, "y")
    shared_state.update_command(
        CommandState(
            timestamp_ms=msg.timestamp_ms,
            received_ms=now_monotonic_ms(),
            fresh=True,
            authority=authority,
            last_command_id=msg.command_id,
            last_command=msg.command,
            last_status=result.status,
            last_reason=result.reason,
            last_cause=result.cause,
            target_id=msg.target_id,
            objective_x_cm=(
                float(objective_x_cm) if objective_x_cm is not None else None
            ),
            objective_y_cm=(
                float(objective_y_cm) if objective_y_cm is not None else None
            ),
            objective_yaw_rad=float(objective.get("yaw_rad", 0.0)),
            objective_frame_id=(
                str(objective["frame_id"])
                if objective.get("frame_id") is not None
                else None
            ),
            request_yolo=bool(msg.params.get("request_yolo", False)),
            request_recording=bool(msg.params.get("request_recording", False)),
        )
    )


def _motion_sensors_healthy(shared_state: SharedState) -> bool:
    now_ms = now_monotonic_ms()
    range_state = shared_state.get_range()
    lidar_state = shared_state.get_lidar_range()

    def usable(state, ttl_ms):
        return (
            state.received_ms > 0
            and now_ms - state.received_ms <= ttl_ms
            and state.distance_cm is not None
            and state.confidence > 0.0
        )

    return usable(range_state, RANGE_STALE_MS) and usable(
        lidar_state, LIDAR_STALE_MS
    )


def _mission_home_frozen(mission: MissionState) -> bool:
    return (
        mission.home_version_frozen is not None
        or mission.active_target_id is not None
    )


def _resolve_set_home_pose(
    msg: CommandMessage, shared_state: SharedState
) -> tuple[Optional[tuple[float, float, str, float, float, float]], Optional[RejectionCause]]:
    """Return ((x_cm, y_cm, frame, yaw_rad, x_m, y_m), None) or (None, cause).

    The operator-facing home payload is centimeters in the yard frame; the
    meter pair exists only because Nav2 poses are metric.
    """

    home_payload = msg.params.get("home")
    if isinstance(home_payload, dict):
        validation = _validate_yard_point_cm(home_payload)
        if validation is not None:
            return None, validation
        x_cm, y_cm, frame_id = _yard_point_cm(home_payload)
        yaw_rad = float(home_payload.get("yaw_rad", 0.0))
        return (
            x_cm,
            y_cm,
            frame_id,
            yaw_rad,
            x_cm / 100.0,
            y_cm / 100.0,
        ), None

    nav = shared_state.get_navigation()
    now_ms = now_monotonic_ms()
    if nav.pose_received_ms <= 0 or now_ms - nav.pose_received_ms > 500:
        return None, RejectionCause.HOME_MISSING
    return (
        nav.pose_x_m * 100.0,
        nav.pose_y_m * 100.0,
        "yard",
        float(nav.pose_yaw_rad),
        float(nav.pose_x_m),
        float(nav.pose_y_m),
    ), None


def _apply_set_home(
    msg: CommandMessage,
    *,
    state: FsmState,
    shared_state: SharedState,
    fsm: "FSM",
    engine: "DecisionEngine",
    home_store: Optional[HomeStore] = None,
    geofence_polygon: Optional["GeofencePolygon"] = None,
) -> _ApplyResult:
    if state not in {FsmState.HOME, FsmState.IDLE}:
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.INVALID_STATE,
        )
    if _mission_home_frozen(shared_state.get_mission()):
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.INVALID_STATE,
        )
    if not shared_state.get_system().startup_ready:
        return _ApplyResult.rejected(
            state,
            ReasonCode.STARTUP_NOT_READY,
            RejectionCause.STARTUP_NOT_READY,
        )

    pose, cause = _resolve_set_home_pose(msg, shared_state)
    if pose is None:
        return _ApplyResult.rejected(
            state, ReasonCode.TRANSITION_REJECTED, cause or RejectionCause.HOME_MISSING
        )
    x_cm, y_cm, frame_id, yaw_rad, x_m, y_m = pose

    if geofence_polygon is not None:
        if not point_in_polygon(x_m, y_m, geofence_polygon.vertices_m):
            return _ApplyResult.rejected(
                state,
                ReasonCode.TRANSITION_REJECTED,
                RejectionCause.GEOFENCE_INVALID,
            )

    try:
        if home_store is not None:
            home = home_store.commit(
                x=x_cm,
                y=y_cm,
                frame_id=frame_id,
                yaw_rad=yaw_rad,
                x_m=x_m,
                y_m=y_m,
                source_command_id=msg.command_id,
                timestamp_ms=msg.timestamp_ms,
            )
        else:
            previous = shared_state.get_home()
            home = HomeState(
                timestamp_ms=msg.timestamp_ms,
                received_ms=now_monotonic_ms(),
                fresh=True,
                authority="HomeStore",
                set=True,
                valid=True,
                x=x_cm,
                y=y_cm,
                frame_id=frame_id,
                x_m=x_m,
                y_m=y_m,
                yaw_rad=yaw_rad,
                home_version=int(previous.home_version) + 1,
                source_command_id=msg.command_id,
                persisted_at_ms=msg.timestamp_ms,
            )
    except HomePersistError:
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.HOME_PERSIST_FAILED,
        )

    shared_state.update_home(home)
    return _ApplyResult(
        ack_type=AckType.COMMAND,
        status=AckStatus.ACCEPTED,
        state=state,
        reason=ReasonCode.SET_HOME_ACCEPTED,
        cause=None,
    )


def _apply_start_chase(
    msg: CommandMessage,
    *,
    state: FsmState,
    shared_state: SharedState,
    fsm: "FSM",
    engine: "DecisionEngine",
    home_store: Optional[HomeStore] = None,
    geofence_polygon: Optional["GeofencePolygon"] = None,
) -> _ApplyResult:
    if not msg.target_id:
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.TARGET_INVALID,
        )
    if state not in {FsmState.HOME, FsmState.IDLE}:
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.INVALID_STATE,
        )
    if not shared_state.get_system().startup_ready:
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.STARTUP_NOT_READY,
        )
    home = shared_state.get_home()
    if not home_is_valid(home):
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.HOME_MISSING,
        )
    geofence = shared_state.get_geofence()
    if geofence.configured and (
        geofence.breach_confirmed or not geofence.car_inside
    ):
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.GEOFENCE_INVALID,
        )
    overhead = shared_state.get_overhead()
    mission = shared_state.get_mission()
    if (
        mission.blocked_target_id == msg.target_id
        and overhead.sequence <= mission.blocked_through_observation_seq
    ):
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.STALE_OBSERVATION,
        )
    if overhead.car.confidence < 1.0:
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.CAR_POSITION_INVALID,
        )
    if overhead.cat.confidence < 1.0:
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.CAT_POSITION_INVALID,
        )
    if (
        overhead.selected_target_id != msg.target_id
        or overhead.cat.target_id != msg.target_id
        or not overhead.cat.inside_perimeter
    ):
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.TARGET_INVALID,
        )
    if overhead.received_ms <= 0:
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.TRACKING_STALE,
        )
    if not _motion_sensors_healthy(shared_state):
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.MOTION_UNSAFE,
        )

    transition = fsm.apply(
        FsmEvent.START_CHASE_ACCEPTED,
        reason=ReasonCode.START_CHASE_ACCEPTED,
        now_ms=now_monotonic_ms(),
    )
    if not transition.accepted:
        return _ApplyResult.rejected(
            state,
            ReasonCode.START_CHASE_REJECTED,
            RejectionCause.INVALID_STATE,
        )
    engine.set_active_target_id(msg.target_id)
    engine.freeze_mission_home(home)
    engine.request_chase_recording()
    if home_store is not None:
        home_store.mark_frozen(True)
        shared_state.update_home(replace(home, frozen_for_mission=True))
    else:
        shared_state.update_home(replace(home, frozen_for_mission=True))
    shared_state.update_mission(engine.mission_state)
    return _ApplyResult(
        ack_type=AckType.COMMAND,
        status=AckStatus.ACCEPTED,
        state=fsm.state,
        reason=ReasonCode.START_CHASE_ACCEPTED,
        cause=None,
    )


def _apply_stop_chase(
    msg: CommandMessage,
    *,
    state: FsmState,
    shared_state: SharedState,
    fsm: "FSM",
    engine: "DecisionEngine",
    home_store: Optional[HomeStore] = None,
    geofence_polygon: Optional["GeofencePolygon"] = None,
) -> _ApplyResult:
    mission = shared_state.get_mission()
    if state == FsmState.FAILSAFE:
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.FAILSAFE_ACTIVE,
        )
    if (
        msg.target_id is not None
        and msg.target_id != mission.active_target_id
    ):
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.WRONG_TARGET,
        )
    if (
        state == FsmState.BRAKE_REVERSE
        and engine.brake_saved_state
        not in {
            FsmState.GETTING_CLOSE,
            FsmState.SEARCH,
            FsmState.CHASE,
        }
    ):
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.INVALID_STATE,
        )

    # Outside a chase state the transition is a no-op and the command stays
    # accepted: stopping something already stopped is idempotent success, and
    # the mission/home-freeze teardown below still has to run. Skipping the
    # FSM call in that case (rather than calling it and ignoring the result)
    # avoids logging a spurious rejected-transition descriptor for a command
    # that the ACK correctly reports as accepted.
    if state in CHASE_STATES:
        fsm.apply(
            FsmEvent.STOP_CHASE_ACCEPTED,
            reason=ReasonCode.STOP_CHASE_ACCEPTED,
            now_ms=now_monotonic_ms(),
        )
    # The stop may have left BRAKE_REVERSE, whose saved objective is exactly the
    # chase this command just cancelled.
    engine.clear_brake_reverse_context(reset_attempts=True)
    engine.set_active_target_id(None)
    engine.clear_mission_home_freeze()
    engine.start_recording_postroll(now_monotonic_ms())
    engine.cancel_handoff()
    home = shared_state.get_home()
    if home.frozen_for_mission:
        shared_state.update_home(replace(home, frozen_for_mission=False))
    if home_store is not None:
        home_store.mark_frozen(False)
    shared_state.update_mission(engine.mission_state)
    return _ApplyResult(
        ack_type=AckType.COMMAND,
        status=AckStatus.ACCEPTED,
        state=fsm.state,
        reason=ReasonCode.STOP_CHASE_ACCEPTED,
        cause=None,
    )


def _apply_return_home(
    msg: CommandMessage,
    *,
    state: FsmState,
    shared_state: SharedState,
    fsm: "FSM",
    engine: "DecisionEngine",
    home_store: Optional[HomeStore] = None,
    geofence_polygon: Optional["GeofencePolygon"] = None,
) -> _ApplyResult:
    if state == FsmState.FAILSAFE:
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.FAILSAFE_ACTIVE,
        )

    mission = shared_state.get_mission()
    home = shared_state.get_home()
    range_ok = lidar_ok = _motion_sensors_healthy(shared_state)
    # Split health check for predicate: both must be healthy.
    now_ms = now_monotonic_ms()
    range_state = shared_state.get_range()
    lidar_state = shared_state.get_lidar_range()
    range_ok = (
        range_state.received_ms > 0
        and now_ms - range_state.received_ms <= RANGE_STALE_MS
        and range_state.distance_cm is not None
        and range_state.confidence > 0.0
    )
    lidar_ok = (
        lidar_state.received_ms > 0
        and now_ms - lidar_state.received_ms <= LIDAR_STALE_MS
        and lidar_state.distance_cm is not None
        and lidar_state.confidence > 0.0
    )
    ok, _reason = safe_return_possible(
        home=home,
        mission=mission,
        range_healthy=range_ok,
        lidar_healthy=lidar_ok,
        geofence=shared_state.get_geofence(),
        navigation=shared_state.get_navigation(),
    )
    if not ok:
        fsm.apply(
            FsmEvent.EMERGENCY_STOP_ACCEPTED,
            reason=ReasonCode.FAILSAFE_TRIGGERED,
            now_ms=now_monotonic_ms(),
        )
        engine.clear_brake_reverse_context(reset_attempts=True)
        engine.cancel_handoff()
        return _ApplyResult(
            ack_type=AckType.COMMAND,
            status=AckStatus.ACCEPTED,
            state=fsm.state,
            reason=ReasonCode.FAILSAFE_TRIGGERED,
            cause=None,
        )

    pose = frozen_home_pose(mission, home)
    assert pose is not None
    # Idempotent: already HOME within completion tolerance stays HOME.
    if state == FsmState.HOME:
        nav = shared_state.get_navigation()
        if nav.pose_received_ms > 0:
            xy_cm = (
                ((nav.pose_x_m - pose[4]) ** 2 + (nav.pose_y_m - pose[5]) ** 2)
                ** 0.5
            ) * 100.0
            if xy_cm <= 20.0:
                return _ApplyResult(
                    ack_type=AckType.COMMAND,
                    status=AckStatus.ACCEPTED,
                    state=state,
                    reason=ReasonCode.RETURN_HOME_ACCEPTED,
                    cause=None,
                )

    fsm.apply(
        FsmEvent.RETURN_HOME_ACCEPTED,
        reason=ReasonCode.RETURN_HOME_ACCEPTED,
        now_ms=now_monotonic_ms(),
    )
    # Interface Section 5.7: returning home from BRAKE_REVERSE cancels the saved
    # objective, so the reverse phase must not survive the transition.
    engine.clear_brake_reverse_context(reset_attempts=True)
    engine.cancel_handoff()
    return _ApplyResult(
        ack_type=AckType.COMMAND,
        status=AckStatus.ACCEPTED,
        state=fsm.state,
        reason=ReasonCode.RETURN_HOME_ACCEPTED,
        cause=None,
    )


def _apply_go_to(
    msg: CommandMessage,
    *,
    state: FsmState,
    shared_state: SharedState,
    fsm: "FSM",
    engine: "DecisionEngine",
    home_store: Optional[HomeStore] = None,
    geofence_polygon: Optional["GeofencePolygon"] = None,
) -> _ApplyResult:
    target_payload = msg.params.get("target")
    validation = _validate_yard_point_cm(target_payload, kind="target")
    if validation is not None:
        return _ApplyResult.rejected(
            state, ReasonCode.TRANSITION_REJECTED, validation
        )
    if state == FsmState.FAILSAFE:
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.FAILSAFE_ACTIVE,
        )
    # Interface contract section 5.6: GO_TO is accepted only from HOME or IDLE.
    if state not in {FsmState.HOME, FsmState.IDLE}:
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.INVALID_STATE,
        )
    if not shared_state.get_system().startup_ready:
        return _ApplyResult.rejected(
            state,
            ReasonCode.STARTUP_NOT_READY,
            RejectionCause.STARTUP_NOT_READY,
        )
    if not _motion_sensors_healthy(shared_state):
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.MOTION_UNSAFE,
        )
    if geofence_polygon is not None:
        x_cm, y_cm, _frame = _yard_point_cm(target_payload)
        if not point_in_polygon(
            x_cm / 100.0, y_cm / 100.0, geofence_polygon.vertices_m
        ):
            return _ApplyResult.rejected(
                state,
                ReasonCode.TRANSITION_REJECTED,
                RejectionCause.GEOFENCE_INVALID,
            )

    transition = fsm.apply(
        FsmEvent.GO_TO_ACCEPTED,
        reason=ReasonCode.GO_TO_ACCEPTED,
        now_ms=now_monotonic_ms(),
    )
    if not transition.accepted:
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.INVALID_STATE,
        )
    engine.set_goto_perception_flags(
        request_yolo=bool(msg.params.get("request_yolo", False)),
        request_recording=bool(msg.params.get("request_recording", False)),
    )
    shared_state.update_mission(engine.mission_state)
    return _ApplyResult(
        ack_type=AckType.COMMAND,
        status=AckStatus.ACCEPTED,
        state=fsm.state,
        reason=ReasonCode.GO_TO_ACCEPTED,
        cause=None,
    )


def _apply_emergency_stop(
    msg: CommandMessage,
    *,
    state: FsmState,
    shared_state: SharedState,
    fsm: "FSM",
    engine: "DecisionEngine",
    home_store: Optional[HomeStore] = None,
    geofence_polygon: Optional["GeofencePolygon"] = None,
) -> _ApplyResult:
    fsm.apply(
        FsmEvent.EMERGENCY_STOP_ACCEPTED,
        reason=ReasonCode.FAILSAFE_TRIGGERED,
        now_ms=now_monotonic_ms(),
    )
    engine.clear_brake_reverse_context(reset_attempts=True)
    engine.set_active_target_id(None)
    engine.clear_mission_home_freeze()
    engine.cancel_handoff()
    home = shared_state.get_home()
    if home.frozen_for_mission:
        shared_state.update_home(replace(home, frozen_for_mission=False))
    if home_store is not None:
        home_store.mark_frozen(False)
    shared_state.update_mission(engine.mission_state)
    return _ApplyResult(
        ack_type=AckType.COMMAND,
        status=AckStatus.ACCEPTED,
        state=fsm.state,
        reason=ReasonCode.FAILSAFE_TRIGGERED,
        cause=None,
    )


def _apply_clear_failsafe(
    msg: CommandMessage,
    *,
    state: FsmState,
    shared_state: SharedState,
    fsm: "FSM",
    engine: "DecisionEngine",
    home_store: Optional[HomeStore] = None,
    geofence_polygon: Optional["GeofencePolygon"] = None,
) -> _ApplyResult:
    confirmed = bool(msg.params.get("operator_confirmed", False))
    if not confirmed:
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.OPERATOR_CONFIRMATION_REQUIRED,
        )
    # Interface Section 5.8 / Target Redesign Section 7.6 also require
    # cause-specific clearance, stopped motor feedback, and a healthy control
    # loop/watchdog before FAILSAFE may clear. Those signals do not exist yet
    # (see open design questions), so only the fresh/valid lidar+ultrasonic
    # sub-condition -- already available via ``_motion_sensors_healthy`` -- is
    # enforced here.
    if not _motion_sensors_healthy(shared_state):
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.MOTION_UNSAFE,
        )
    transition = fsm.apply(
        FsmEvent.CLEAR_FAILSAFE_ACCEPTED,
        reason=ReasonCode.CLEAR_FAILSAFE_ACCEPTED,
        now_ms=now_monotonic_ms(),
    )
    # Only a latched FAILSAFE can be cleared; anywhere else this must not
    # report success, or an operator believes a failsafe was resolved.
    if not transition.accepted:
        return _ApplyResult.rejected(
            state,
            ReasonCode.TRANSITION_REJECTED,
            RejectionCause.INVALID_STATE,
        )
    engine.clear_brake_reverse_context(reset_attempts=True)
    engine.clear_mission_home_freeze()
    shared_state.update_mission(engine.mission_state)
    return _ApplyResult(
        ack_type=AckType.COMMAND,
        status=AckStatus.ACCEPTED,
        state=fsm.state,
        reason=ReasonCode.CLEAR_FAILSAFE_ACCEPTED,
        cause=None,
    )


def _coordinate_cm(payload: Dict[str, Any], axis: str) -> Any:
    """Return the raw yard coordinate for ``axis``.

    ``x_cm``/``y_cm`` is the contract spelling; the unsuffixed key is accepted
    from older senders and carries the same centimeter units.
    """

    key_cm = f"{axis}_cm"
    if key_cm in payload:
        return payload[key_cm]
    return payload.get(axis)


def _yard_point_cm(payload: Dict[str, Any]):
    """Return ``(x_cm, y_cm, frame_id)`` from a validated yard payload."""

    return (
        float(_coordinate_cm(payload, "x")),
        float(_coordinate_cm(payload, "y")),
        str(payload.get("frame_id", "yard")),
    )


def _validate_yard_point_cm(
    payload: Optional[Dict[str, Any]], *, kind: str = "home"
) -> Optional[RejectionCause]:
    """Validate a yard point expressed in centimeters."""

    missing = (
        RejectionCause.HOME_MISSING
        if kind == "home"
        else RejectionCause.TARGET_INVALID
    )
    invalid = (
        RejectionCause.HOME_INVALID
        if kind == "home"
        else RejectionCause.TARGET_INVALID
    )
    if payload is None or not isinstance(payload, dict):
        return missing
    if _coordinate_cm(payload, "x") is None or _coordinate_cm(payload, "y") is None:
        return invalid
    try:
        x_cm = float(_coordinate_cm(payload, "x"))
        y_cm = float(_coordinate_cm(payload, "y"))
        yaw_rad = float(payload.get("yaw_rad", 0.0))
    except (TypeError, ValueError):
        return invalid
    # NaN / infinity would propagate into the Nav2 goal and the durable home
    # record, so they are rejected here rather than at the ROS boundary.
    if not all(isfinite(value) for value in (x_cm, y_cm, yaw_rad)):
        return invalid
    if str(payload.get("frame_id", "yard")) != "yard":
        return invalid
    return None


__all__ = ["TransactionResult", "apply_pending_transaction"]
