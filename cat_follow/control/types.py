"""Shared control contract types for the target runtime architecture.

These types mirror the interface/data-contract specification.  They are kept
separate from the current prototype state machine so Milestone 1 can build the
new core without changing existing robot behavior.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple


class MessageType(str, Enum):
    TRACKING = "tracking"
    OVERHEAD_OBSERVATION = "overhead_observation"
    COMMAND = "command"
    MISSION_EVENT = "mission_event"
    ACK = "ack"


class CommandName(str, Enum):
    SET_HOME = "set_home"
    START_CHASE = "start_chase"
    STOP_CHASE = "stop_chase"
    RETURN_HOME = "return_home"
    GO_TO = "go_to"
    EMERGENCY_STOP = "emergency_stop"
    CLEAR_FAILSAFE = "clear_failsafe"


class AckStatus(str, Enum):
    ACCEPTED = "accepted"
    REJECTED = "rejected"


class AckType(str, Enum):
    COMMAND = "command"
    MISSION_EVENT = "mission_event"


class MissionEventName(str, Enum):
    PRIMARY_CAT_LEFT_PERIMETER = "PRIMARY_CAT_LEFT_PERIMETER"


class FsmState(str, Enum):
    HOME = "HOME"
    IDLE = "IDLE"
    GETTING_CLOSE = "GETTING_CLOSE"
    SEARCH = "SEARCH"
    CHASE = "CHASE"
    BRAKE_REVERSE = "BRAKE_REVERSE"
    GOTO = "GOTO"
    RETURN_HOME = "RETURN_HOME"
    FAILSAFE = "FAILSAFE"

    # Source compatibility for in-process V1 integrations.  Aliases serialize
    # with canonical values and can be removed after downstream migration.
    CHASE_A = GETTING_CLOSE
    TRACK_B = CHASE

    @classmethod
    def _missing_(cls, value):
        """Accept legacy V1 wire names while always emitting canonical names."""

        legacy_names = {
            "CHASE_A": cls.GETTING_CLOSE,
            "TRACK_B": cls.CHASE,
            "BRAKE": cls.BRAKE_REVERSE,
        }
        return legacy_names.get(value)


class FsmEvent(str, Enum):
    START_CHASE_ACCEPTED = "start_chase_accepted"
    STOP_CHASE_ACCEPTED = "stop_chase_accepted"
    RETURN_HOME_ACCEPTED = "return_home_accepted"
    GO_TO_ACCEPTED = "go_to_accepted"
    EMERGENCY_STOP_ACCEPTED = "emergency_stop_accepted"
    CLEAR_FAILSAFE_ACCEPTED = "clear_failsafe_accepted"
    CAT_VISIBLE_STABLE = "cat_visible_stable"
    SEARCH_ENTRY_READY = "search_entry_ready"
    LOCAL_TRACK_ACQUIRED = "local_track_acquired"
    CAT_LOST = "cat_lost"
    CAT_LOST_NEAR = "cat_lost_near"
    CAT_LOST_FAR = "cat_lost_far"
    TARGET_ID_CHANGED = "target_id_changed"
    OVERHEAD_RETENTION_EXPIRED = "overhead_retention_expired"
    SEARCH_EXHAUSTED = "search_exhausted"
    HANDOFF_TIMEOUT = "handoff_timeout"
    NAVIGATION_FAILURES_EXHAUSTED = "navigation_failures_exhausted"
    FINAL_APPROACH_READY = "final_approach_ready"
    BRAKE_ABORTED_CAT_MOVED = "brake_aborted_cat_moved"
    GO_TO_COMPLETE = "go_to_complete"
    RETURN_HOME_COMPLETE = "return_home_complete"
    FAILSAFE_TRIGGERED = "failsafe_triggered"
    OBSTACLE_TOO_CLOSE = "obstacle_too_close"
    BRAKE_REVERSE_TRIGGERED = "brake_reverse_triggered"
    BRAKE_REVERSE_CLEARED = "brake_reverse_cleared"
    PRIMARY_CAT_LEFT_PERIMETER = "primary_cat_left_perimeter"
    TRANSITION_REJECTED = "transition_rejected"


class RejectionCause(str, Enum):
    CAR_POSITION_INVALID = "car_position_invalid"
    CAT_POSITION_INVALID = "cat_position_invalid"
    TRACKING_STALE = "tracking_stale"
    HOME_MISSING = "home_missing"
    HOME_INVALID = "home_invalid"
    HOME_PERSIST_FAILED = "home_persist_failed"
    TARGET_INVALID = "target_invalid"
    MOTION_UNSAFE = "motion_unsafe"
    FAILSAFE_ACTIVE = "failsafe_active"
    OPERATOR_CONFIRMATION_REQUIRED = "operator_confirmation_required"
    INVALID_COMMAND = "invalid_command"
    INVALID_PARAMS = "invalid_params"
    WRONG_TARGET = "wrong_target"
    STALE_OBSERVATION = "stale_observation"
    DUPLICATE_SUPERSEDED = "duplicate_superseded"
    INVALID_STATE = "invalid_state"
    SAFETY_HEALTH_INVALID = "safety_health_invalid"
    GEOFENCE_INVALID = "geofence_invalid"
    LOCALIZATION_INVALID = "localization_invalid"
    CALIBRATION_MISMATCH = "calibration_mismatch"
    STARTUP_NOT_READY = "startup_not_ready"


class ReasonCode(str, Enum):
    START_CHASE_ACCEPTED = "start_chase_accepted"
    START_CHASE_REJECTED = "start_chase_rejected"
    GLOBAL_CHASE = "global_chase"
    LOCAL_TRACK = "local_track"
    SEARCH_ENTRY = "search_entry"
    SEARCH_ACQUIRING = "search_acquiring"
    SEARCH_RETRY = "search_retry"
    SEARCH_EXHAUSTED = "search_exhausted"
    FINAL_APPROACH = "final_approach"
    BRAKE_COMPLETE = "brake_complete"
    BRAKE_ABORTED_CAT_MOVED = "brake_aborted_cat_moved"
    CAT_LOST_FALLBACK = "cat_lost_fallback"
    CAT_LOST_NEAR = "cat_lost_near"
    CAT_LOST_FAR = "cat_lost_far"
    TARGET_ID_CHANGED = "target_id_changed"
    OVERHEAD_RETENTION = "overhead_retention"
    HANDOFF_WAIT = "handoff_wait"
    HANDOFF_TIMEOUT = "handoff_timeout"
    NAVIGATION_COMPLETE = "navigation_complete"
    NAVIGATION_FAILURE = "navigation_failure"
    NAVIGATION_FAILURES_EXHAUSTED = "navigation_failures_exhausted"
    NAVIGATION_PATH_BLOCKED = "navigation_path_blocked"
    STOP_CHASE_ACCEPTED = "stop_chase_accepted"
    RETURN_HOME_ACCEPTED = "return_home_accepted"
    RETURN_HOME_COMPLETE = "return_home_complete"
    GO_TO_ACCEPTED = "go_to_accepted"
    GO_TO_COMPLETE = "go_to_complete"
    OBSTACLE_TOO_CLOSE = "obstacle_too_close"
    OBSTACLE_VETO = "obstacle_veto"
    THERMAL_CRITICAL = "thermal_critical"
    SENSOR_HEALTH_HOLD = "sensor_health_hold"
    SENSOR_HEALTH_TIMEOUT = "sensor_health_timeout"
    BRAKE_REVERSE_TRIGGERED = "brake_reverse_triggered"
    BRAKE_REVERSE_ACTIVE = "brake_reverse_active"
    BRAKE_REVERSE_CLEAR = "brake_reverse_clear"
    BRAKE_REVERSE_EXHAUSTED = "brake_reverse_exhausted"
    OVERHEAD_STALE = "overhead_stale"
    OVERHEAD_EXPIRED = "overhead_expired"
    CAMERA_LOST = "camera_lost"
    TRACKING_INVALID = "tracking_invalid"
    HOME_MISSING = "home_missing"
    SET_HOME_ACCEPTED = "set_home_accepted"
    HOME_LOAD_FAILED = "home_load_failed"
    STARTUP_NOT_READY = "startup_not_ready"
    GEOFENCE_BREACH = "geofence_breach"
    GEOFENCE_UNOBSERVABLE = "geofence_unobservable"
    FAILSAFE_TRIGGERED = "failsafe_triggered"
    CLEAR_FAILSAFE_ACCEPTED = "clear_failsafe_accepted"
    TRANSITION_REJECTED = "transition_rejected"
    MANUAL_SEQUENCE = "manual_sequence"
    PRIMARY_TARGET_EXIT_HANDOFF = "primary_target_exit_handoff"
    INIT = "init"


class BrakeReversePhase(str, Enum):
    STOP_ENTRY = "stop_entry"
    CENTER = "center"
    SETTLE = "settle"
    REVERSE = "reverse"
    STOP_EXIT = "stop_exit"
    RECHECK = "recheck"


class TargetSource(str, Enum):
    CAT_GLOBAL = "cat_global"
    CAT_LOCAL = "cat_local"
    HOME = "home"
    GO_TO = "go_to"
    NONE = "none"


class NavigationObjectiveType(str, Enum):
    GETTING_CLOSE = "GETTING_CLOSE"
    SEARCH = "SEARCH"
    SEARCH_OBSERVATION = "SEARCH_OBSERVATION"
    CHASE = "CHASE"
    GOTO = "GOTO"
    RETURN_HOME = "RETURN_HOME"


class NavigationResultStatus(str, Enum):
    SUCCEEDED = "SUCCEEDED"
    ABORTED = "ABORTED"
    CANCELED = "CANCELED"
    UNKNOWN = "UNKNOWN"


class NavigationFailureClass(str, Enum):
    PLANNER_FAILURE = "PLANNER_FAILURE"
    CONTROLLER_FAILURE = "CONTROLLER_FAILURE"
    NO_PROGRESS = "NO_PROGRESS"
    PATH_BLOCKED = "PATH_BLOCKED"
    LOCALIZATION_LOST = "LOCALIZATION_LOST"
    PREEMPTED = "PREEMPTED"
    CORRELATION_MISMATCH = "CORRELATION_MISMATCH"


@dataclass(frozen=True)
class NavigationGoalIntent:
    goal_intent_id: str
    objective_type: NavigationObjectiveType
    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float = 0.0
    target_id: Optional[str] = None
    moving_goal: bool = False
    requested_at_ms: int = 0
    action_goal_id: Optional[str] = None
    refresh_count: int = 0
    last_refresh_ms: int = 0
    expected_replacement: bool = False


@dataclass(frozen=True)
class NavigationResult:
    goal_intent_id: str
    action_goal_id: str
    status: NavigationResultStatus
    result_code: Optional[int] = None
    terminal: bool = True
    failure_class: Optional[NavigationFailureClass] = None
    completed_at_ms: int = 0
    pose_qualified: bool = False
    dwell_qualified: bool = False


class RangeBackend(str, Enum):
    ULTRASONIC = "ultrasonic"
    LIDAR_C1 = "lidar_c1"
    # TMF8829 dToF integration is ON HOLD. The value is retained for
    # backward compatibility only. Production range sensing now uses the
    # Lidar C1 plus the ultrasonic hardware.
    TMF8829 = "tmf8829"


class ThermalState(str, Enum):
    UNKNOWN = "unknown"
    NORMAL = "normal"
    WARNING = "warning"
    SPEED_LIMITED = "speed_limited"
    CRITICAL = "critical"


class CameraHardwareState(str, Enum):
    CLOSED = "closed"
    READY_INACTIVE = "ready_inactive"
    ACTIVE = "active"
    FAULTED = "faulted"


class TelemetryEventType(str, Enum):
    CONFIGURATION = "configuration"
    STATE_TRANSITION = "state_transition"
    TRANSITION_REJECTED = "transition_rejected"
    DECISION = "decision"
    COMMAND_RECEIVED = "command_received"
    COMMAND_ACK = "command_ack"
    TRACKING_RECEIVED = "tracking_received"
    TRACKING_STALE = "tracking_stale"
    VISION_UPDATE = "vision_update"
    RANGE_UPDATE = "range_update"
    OBSTACLE_VETO = "obstacle_veto"
    FAILSAFE = "failsafe"
    THERMAL = "thermal"
    THREAD_HEALTH = "thread_health"
    MOTOR_COMMAND = "motor_command"
    NAVIGATION_GOAL = "navigation_goal"
    NAVIGATION_RESULT = "navigation_result"


class TelemetrySeverity(str, Enum):
    DEBUG = "debug"
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"


@dataclass(frozen=True)
class TrackingObjectState:
    x: float = 0.0
    y: float = 0.0
    confidence: float = 0.0
    target_id: Optional[str] = None
    inside_perimeter: bool = True


@dataclass(frozen=True)
class CarTrackingState(TrackingObjectState):
    heading: float = 0.0
    heading_valid: bool = False


@dataclass(frozen=True)
class OverheadState:
    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = False
    authority: str = "CommsManager"
    sequence: int = 0
    frame_id: str = "yard"
    perimeter_id: str = ""
    calibration_version: int = 0
    selected_target_id: Optional[str] = None
    car: CarTrackingState = field(default_factory=CarTrackingState)
    cat: TrackingObjectState = field(default_factory=TrackingObjectState)


@dataclass(frozen=True)
class HomeState:
    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = True
    authority: str = "HomeStore"
    set: bool = False
    valid: bool = False
    x: float = 0.0
    y: float = 0.0
    frame_id: str = "yard"
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    home_version: int = 0
    checksum: str = ""
    calibration_version: int = 0
    map_id: str = ""
    persisted_at_ms: int = 0
    source_command_id: Optional[str] = None
    frozen_for_mission: bool = False


@dataclass(frozen=True)
class GeofenceState:
    """Car containment polygon status (distinct from overhead cat perimeter)."""

    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = True
    authority: str = "Geofence"
    car_geofence_id: str = ""
    configured: bool = False
    car_inside: bool = True
    car_distance_to_boundary_cm: float = 0.0
    localization_valid_for_containment: bool = False
    breach_confirmed: bool = False
    breach_at_ms: Optional[int] = None


@dataclass(frozen=True)
class VisionState:
    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = False
    authority: str = "VisionTracker"
    cat_visible: bool = False
    cat_visible_stable: bool = False
    x_offset_norm: float = 0.0
    confidence: float = 0.0
    last_seen_ms: int = 0
    observation_sequence: int = 0
    associated_target_id: Optional[str] = None
    association_ambiguous: bool = False


@dataclass(frozen=True)
class RangeState:
    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = False
    authority: str = "RangeSafety"
    backend: RangeBackend = RangeBackend.ULTRASONIC
    distance_cm: Optional[float] = None
    confidence: float = 0.0
    obstacle_detected: bool = False
    obstacle_critical: bool = False
    obstacle_severity: float = 0.0
    zone: Optional[str] = None


@dataclass(frozen=True)
class NavigationState:
    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = False
    authority: str = "Navigation"
    heading: float = 0.0
    heading_valid: bool = False
    speed_limit: float = 0.0
    path_correction: float = 0.0
    no_progress: bool = False
    dead_end: bool = False
    healthy: bool = False
    path_viable: bool = False
    safe_steering_min: float = 0.0
    safe_steering_max: float = 0.0
    speed_cap_mps: float = 0.0
    pose_x_m: float = 0.0
    pose_y_m: float = 0.0
    pose_yaw_rad: float = 0.0
    pose_received_ms: int = 0
    goal_intent: Optional[NavigationGoalIntent] = None
    last_result: Optional[NavigationResult] = None
    completion_qualified: bool = False
    failures_exhausted: bool = False


@dataclass(frozen=True)
class ThreadHealthState:
    comms_alive: bool = False
    vision_alive: bool = False
    range_alive: bool = False
    navigation_alive: bool = False
    control_alive: bool = False


@dataclass(frozen=True)
class SystemState:
    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = True
    authority: str = "Runtime"
    thermal_c: Optional[float] = None
    thermal_state: ThermalState = ThermalState.UNKNOWN
    battery_voltage: Optional[float] = None
    brownout_detected: bool = False
    threads: ThreadHealthState = field(default_factory=ThreadHealthState)
    startup_ready: bool = True
    startup_seed_applied: bool = False
    startup_degraded_reason: Optional[str] = None


@dataclass(frozen=True)
class FSMSnapshot:
    """SharedState `fsm` group.

    Renamed from `FSMState` to avoid case-only collision with the `FsmState`
    enum.  The enum represents the logical state value; this dataclass holds
    the metadata (timestamps, last transition info) that other modules read
    from `SharedSnapshot.fsm`.
    """

    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = True
    authority: str = "FSM"
    state: FsmState = FsmState.IDLE
    previous_state: Optional[FsmState] = None
    last_transition_ms: int = 0
    last_transition_reason: ReasonCode = ReasonCode.INIT
    last_rejected_transition: Optional[str] = None


@dataclass(frozen=True)
class MissionState:
    active_target_id: Optional[str] = None
    last_event_observation_seq: int = -1
    blocked_target_id: Optional[str] = None
    blocked_through_observation_seq: int = -1
    handoff_deadline_ms: Optional[int] = None
    overhead_invalid_started_ms: Optional[int] = None
    search_stage: int = 0
    search_lock_observations: int = 0
    home_version_frozen: Optional[int] = None
    frozen_home_x: Optional[float] = None
    frozen_home_y: Optional[float] = None
    frozen_home_x_m: Optional[float] = None
    frozen_home_y_m: Optional[float] = None
    frozen_home_yaw_rad: float = 0.0
    frozen_home_frame_id: str = "yard"
    chase_recording_requested: bool = False
    recording_postroll_deadline_ms: Optional[int] = None
    goto_request_yolo: bool = False
    goto_request_recording: bool = False


@dataclass(frozen=True)
class ConsumerState:
    requested: bool = False
    active: bool = False
    consumer_refcount: int = 0
    reason: str = ""
    postroll_deadline_ms: Optional[int] = None
    degraded_reason: Optional[str] = None
    segment_path: Optional[str] = None


@dataclass(frozen=True)
class RecordingRuntimeState:
    """Actual recording writer feedback merged into lifecycle telemetry."""

    active: bool = False
    segment_path: Optional[str] = None
    degraded_reason: Optional[str] = None
    bytes_written: int = 0
    segments_finalized: int = 0


@dataclass(frozen=True)
class PerceptionLifecycleState:
    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = True
    authority: str = "PerceptionLifecycleManager"
    detector: ConsumerState = field(default_factory=ConsumerState)
    recording: ConsumerState = field(default_factory=ConsumerState)
    stream_requested_clients: int = 0
    stream_active_clients: int = 0
    stream_encoder_ready: bool = False
    stream_forced_off: bool = False
    stream_degraded_reason: Optional[str] = None
    camera_hardware_state: CameraHardwareState = CameraHardwareState.READY_INACTIVE
    camera_streamoff_capable: bool = True
    camera_last_revalidation_ms: int = 0
    camera_fatal_fault: bool = False
    capture_active: bool = False
    detector_mission_override: bool = False


@dataclass(frozen=True)
class CommandState:
    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = True
    authority: str = "CommsManager"
    last_command_id: Optional[str] = None
    last_command: Optional[CommandName] = None
    last_status: Optional[AckStatus] = None
    last_reason: Optional[ReasonCode] = None
    last_cause: Optional[RejectionCause] = None
    target_id: Optional[str] = None
    mission_event_id: Optional[str] = None
    mission_event_name: Optional[MissionEventName] = None
    mission_event_observation_sequence: Optional[int] = None
    # Yard-frame objective in centimeters; the metric conversion happens once,
    # at the Nav2 boundary in NavigationManager.
    objective_x_cm: Optional[float] = None
    objective_y_cm: Optional[float] = None
    objective_yaw_rad: float = 0.0
    objective_frame_id: Optional[str] = None
    request_yolo: bool = False
    request_recording: bool = False


@dataclass(frozen=True)
class DecisionState:
    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = True
    authority: str = "DecisionEngine"
    requested_state: FsmState = FsmState.IDLE
    speed: float = 0.0
    steering: float = 0.0
    brake: bool = False
    reason: ReasonCode = ReasonCode.INIT
    active_constraints: Tuple[str, ...] = ()


@dataclass(frozen=True)
class SharedSnapshot:
    overhead: OverheadState = field(default_factory=OverheadState)
    home: HomeState = field(default_factory=HomeState)
    vision: VisionState = field(default_factory=VisionState)
    range: RangeState = field(default_factory=RangeState)
    # Separate lidar range channel (backend=LIDAR_C1) so the C1 and the
    # ultrasonic sensor are fused in DecisionEngine rather than clobbering one
    # shared group.  Defaults stale (fresh=False) until the ros_bridge runs.
    lidar: RangeState = field(default_factory=RangeState)
    navigation: NavigationState = field(default_factory=NavigationState)
    system: SystemState = field(default_factory=SystemState)
    fsm: FSMSnapshot = field(default_factory=FSMSnapshot)
    command: CommandState = field(default_factory=CommandState)
    decision: DecisionState = field(default_factory=DecisionState)
    mission: MissionState = field(default_factory=MissionState)
    geofence: GeofenceState = field(default_factory=GeofenceState)
    perception_lifecycle: PerceptionLifecycleState = field(
        default_factory=PerceptionLifecycleState
    )


@dataclass(frozen=True)
class DecisionInput:
    now_ms: int
    overhead: OverheadState
    home: HomeState
    vision: VisionState
    # NOTE: `range` matches the contract field name from the interface spec.
    # It shadows the Python builtin only inside this dataclass scope.
    range: RangeState
    navigation: NavigationState
    system: SystemState
    fsm: FSMSnapshot
    command: CommandState
    # Lidar (C1) obstacle channel, fused with `range` in DecisionEngine.
    # Defaulted so existing construction sites remain valid.
    lidar: RangeState = field(default_factory=RangeState)
    mission: MissionState = field(default_factory=MissionState)
    geofence: GeofenceState = field(default_factory=GeofenceState)


@dataclass(frozen=True)
class DecisionOutput:
    timestamp_ms: int
    requested_state: FsmState
    speed: float
    steering: float
    brake: bool
    reason: ReasonCode
    active_constraints: Tuple[str, ...] = ()
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    target_source: TargetSource = TargetSource.NONE
    rejected_transition: bool = False
