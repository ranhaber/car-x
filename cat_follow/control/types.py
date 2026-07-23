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
    COMMAND = "command"
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


class FsmState(str, Enum):
    HOME = "HOME"
    IDLE = "IDLE"
    CHASE_A = "CHASE_A"
    TRACK_B = "TRACK_B"
    BRAKE = "BRAKE"
    GOTO = "GOTO"
    RETURN_HOME = "RETURN_HOME"
    FAILSAFE = "FAILSAFE"


class FsmEvent(str, Enum):
    START_CHASE_ACCEPTED = "start_chase_accepted"
    STOP_CHASE_ACCEPTED = "stop_chase_accepted"
    RETURN_HOME_ACCEPTED = "return_home_accepted"
    GO_TO_ACCEPTED = "go_to_accepted"
    EMERGENCY_STOP_ACCEPTED = "emergency_stop_accepted"
    CLEAR_FAILSAFE_ACCEPTED = "clear_failsafe_accepted"
    CAT_VISIBLE_STABLE = "cat_visible_stable"
    CAT_LOST = "cat_lost"
    FINAL_APPROACH_READY = "final_approach_ready"
    BRAKE_ABORTED_CAT_MOVED = "brake_aborted_cat_moved"
    GO_TO_COMPLETE = "go_to_complete"
    RETURN_HOME_COMPLETE = "return_home_complete"
    FAILSAFE_TRIGGERED = "failsafe_triggered"
    OBSTACLE_TOO_CLOSE = "obstacle_too_close"
    TRANSITION_REJECTED = "transition_rejected"


class RejectionCause(str, Enum):
    CAR_POSITION_INVALID = "car_position_invalid"
    CAT_POSITION_INVALID = "cat_position_invalid"
    TRACKING_STALE = "tracking_stale"
    HOME_MISSING = "home_missing"
    HOME_INVALID = "home_invalid"
    TARGET_INVALID = "target_invalid"
    MOTION_UNSAFE = "motion_unsafe"
    FAILSAFE_ACTIVE = "failsafe_active"
    OPERATOR_CONFIRMATION_REQUIRED = "operator_confirmation_required"
    INVALID_COMMAND = "invalid_command"
    INVALID_PARAMS = "invalid_params"


class ReasonCode(str, Enum):
    START_CHASE_ACCEPTED = "start_chase_accepted"
    START_CHASE_REJECTED = "start_chase_rejected"
    GLOBAL_CHASE = "global_chase"
    LOCAL_TRACK = "local_track"
    FINAL_APPROACH = "final_approach"
    BRAKE_COMPLETE = "brake_complete"
    BRAKE_ABORTED_CAT_MOVED = "brake_aborted_cat_moved"
    CAT_LOST_FALLBACK = "cat_lost_fallback"
    STOP_CHASE_ACCEPTED = "stop_chase_accepted"
    RETURN_HOME_ACCEPTED = "return_home_accepted"
    RETURN_HOME_COMPLETE = "return_home_complete"
    GO_TO_ACCEPTED = "go_to_accepted"
    GO_TO_COMPLETE = "go_to_complete"
    OBSTACLE_TOO_CLOSE = "obstacle_too_close"
    OBSTACLE_VETO = "obstacle_veto"
    OVERHEAD_STALE = "overhead_stale"
    OVERHEAD_EXPIRED = "overhead_expired"
    CAMERA_LOST = "camera_lost"
    TRACKING_INVALID = "tracking_invalid"
    HOME_MISSING = "home_missing"
    FAILSAFE_TRIGGERED = "failsafe_triggered"
    CLEAR_FAILSAFE_ACCEPTED = "clear_failsafe_accepted"
    TRANSITION_REJECTED = "transition_rejected"
    MANUAL_SEQUENCE = "manual_sequence"
    INIT = "init"


class TargetSource(str, Enum):
    CAT_GLOBAL = "cat_global"
    CAT_LOCAL = "cat_local"
    HOME = "home"
    GO_TO = "go_to"
    NONE = "none"


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


class TelemetryEventType(str, Enum):
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
    car: CarTrackingState = field(default_factory=CarTrackingState)
    cat: TrackingObjectState = field(default_factory=TrackingObjectState)


@dataclass(frozen=True)
class HomeState:
    timestamp_ms: int = 0
    received_ms: int = 0
    fresh: bool = True
    authority: str = "CommsManager"
    set: bool = False
    x: float = 0.0
    y: float = 0.0
    frame_id: str = "yard"
    source_command_id: Optional[str] = None


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
