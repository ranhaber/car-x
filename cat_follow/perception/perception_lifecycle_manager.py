"""FSM-driven perception consumer lifecycle (detector, recording, stream).

``PerceptionLifecycleManager`` is the sole owner of named consumer demand.
Camera hardware pause/resume is driven from the resulting intent; DecisionEngine
remains the sole drivetrain authority.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Optional

from cat_follow.control.types import (
    CameraHardwareState,
    ConsumerState,
    FsmState,
    MissionState,
    PerceptionLifecycleState,
    RecordingRuntimeState,
)
from cat_follow.runtime.shared_state import now_monotonic_ms
from cat_follow.target_config import TargetRuntimeConfig


@dataclass(frozen=True)
class LifecycleMissionContext:
    """Mission-side inputs that influence perception consumers."""

    chase_recording_requested: bool = False
    recording_postroll_deadline_ms: Optional[int] = None
    goto_request_yolo: bool = False
    goto_request_recording: bool = False
    brake_saved_detector: bool = False
    brake_saved_recording: bool = False
    brake_saved_state: Optional[FsmState] = None


def _recording_blocked(feedback: Optional[RecordingRuntimeState]) -> bool:
    """True when the writer reported it cannot currently persist a segment."""
    return feedback is not None and feedback.degraded_reason is not None


class PerceptionLifecycleManager:
    """Compute and publish detector/recording/stream consumer policy."""

    def __init__(
        self,
        config: Optional[TargetRuntimeConfig] = None,
        *,
        logger=None,
    ) -> None:
        self._config = config or TargetRuntimeConfig()
        self._logger = logger
        self._lock = threading.RLock()
        self._stream_clients = 0
        self._camera_state = CameraHardwareState.READY_INACTIVE
        self._camera_fatal = False
        self._camera_streamoff_capable = True
        self._last_revalidation_ms = 0
        self._last_state = PerceptionLifecycleState()

    @property
    def stream_clients(self) -> int:
        with self._lock:
            return self._stream_clients

    def register_stream_client(self) -> int:
        with self._lock:
            self._stream_clients += 1
            return self._stream_clients

    def unregister_stream_client(self) -> int:
        with self._lock:
            self._stream_clients = max(0, self._stream_clients - 1)
            return self._stream_clients

    def note_camera_fault(self, *, now_ms: Optional[int] = None) -> None:
        with self._lock:
            self._camera_fatal = True
            self._camera_state = CameraHardwareState.FAULTED
            if now_ms is not None:
                self._last_revalidation_ms = now_ms

    def note_camera_revalidated(self, *, now_ms: int) -> None:
        with self._lock:
            self._camera_fatal = False
            self._last_revalidation_ms = now_ms

    def tick(
        self,
        *,
        fsm_state: FsmState,
        mission: MissionState,
        context: LifecycleMissionContext,
        now_ms: Optional[int] = None,
        recording_feedback: Optional[RecordingRuntimeState] = None,
    ) -> PerceptionLifecycleState:
        """Return authoritative lifecycle snapshot for the current control tick."""

        now = now_ms if now_ms is not None else now_monotonic_ms()
        with self._lock:
            stream_clients = self._stream_clients
            camera_fatal = self._camera_fatal
            streamoff_capable = self._camera_streamoff_capable
            last_revalidation_ms = self._last_revalidation_ms

            detector_req, detector_reason = self._detector_policy(
                fsm_state, context
            )
            recording_req, recording_reason, postroll = self._recording_policy(
                fsm_state, mission, context, now
            )
            forced_off = fsm_state in {FsmState.HOME, FsmState.FAILSAFE}
            stream_active = 0 if forced_off else stream_clients
            recording_consumer = recording_req and not _recording_blocked(
                recording_feedback
            )
            capture_active = self._capture_active(
                forced_off=forced_off,
                camera_fatal=camera_fatal,
                detector_req=detector_req,
                recording_consumer=recording_consumer,
                stream_active=stream_active,
            )
            if camera_fatal:
                camera_state = CameraHardwareState.FAULTED
            elif forced_off or not capture_active:
                camera_state = CameraHardwareState.READY_INACTIVE
            else:
                camera_state = CameraHardwareState.ACTIVE
            self._camera_state = camera_state

            recording_active = recording_req and capture_active
            recording_degraded = None
            segment_path = None
            if recording_feedback is not None:
                recording_degraded = recording_feedback.degraded_reason
                segment_path = recording_feedback.segment_path
                # Actual writer activity wins when demand is present.
                recording_active = bool(
                    recording_req and recording_feedback.active
                )

            state = PerceptionLifecycleState(
                received_ms=now,
                detector=ConsumerState(
                    requested=detector_req,
                    active=detector_req and capture_active,
                    consumer_refcount=1 if detector_req else 0,
                    reason=detector_reason,
                ),
                recording=ConsumerState(
                    requested=recording_req,
                    active=recording_active,
                    consumer_refcount=1 if recording_consumer else 0,
                    reason=recording_reason,
                    postroll_deadline_ms=postroll,
                    degraded_reason=recording_degraded,
                    segment_path=segment_path,
                ),
                stream_requested_clients=stream_clients,
                stream_active_clients=stream_active,
                stream_encoder_ready=stream_active > 0 and capture_active,
                stream_forced_off=forced_off,
                stream_degraded_reason=(
                    "fsm_force_off" if forced_off else None
                ),
                camera_hardware_state=camera_state,
                camera_streamoff_capable=streamoff_capable,
                camera_last_revalidation_ms=last_revalidation_ms,
                camera_fatal_fault=camera_fatal,
                capture_active=capture_active,
                detector_mission_override=detector_req
                and fsm_state
                in {FsmState.SEARCH, FsmState.CHASE, FsmState.GOTO},
            )
            self._last_state = state
            return state

    def merge_recording_feedback(
        self, feedback: RecordingRuntimeState
    ) -> PerceptionLifecycleState:
        """Overlay writer actuals onto the last published lifecycle state."""

        with self._lock:
            previous = self._last_state
            recording_consumer = bool(
                previous.recording.requested and not _recording_blocked(feedback)
            )
            recording = ConsumerState(
                requested=previous.recording.requested,
                active=bool(
                    previous.recording.requested and feedback.active
                ),
                consumer_refcount=1 if recording_consumer else 0,
                reason=previous.recording.reason,
                postroll_deadline_ms=previous.recording.postroll_deadline_ms,
                degraded_reason=feedback.degraded_reason,
                segment_path=feedback.segment_path,
            )
            # A recorder that cannot write must stop holding the camera open,
            # otherwise capture keeps running for a consumer that is dead.
            capture_active = self._capture_active(
                forced_off=previous.stream_forced_off,
                camera_fatal=previous.camera_fatal_fault,
                detector_req=previous.detector.requested,
                recording_consumer=recording_consumer,
                stream_active=previous.stream_active_clients,
            )
            if previous.camera_fatal_fault:
                camera_state = CameraHardwareState.FAULTED
            elif capture_active:
                camera_state = CameraHardwareState.ACTIVE
            else:
                camera_state = CameraHardwareState.READY_INACTIVE
            self._camera_state = camera_state
            state = PerceptionLifecycleState(
                timestamp_ms=previous.timestamp_ms,
                received_ms=previous.received_ms,
                fresh=previous.fresh,
                authority=previous.authority,
                detector=previous.detector,
                recording=recording,
                stream_requested_clients=previous.stream_requested_clients,
                stream_active_clients=previous.stream_active_clients,
                stream_encoder_ready=previous.stream_encoder_ready,
                stream_forced_off=previous.stream_forced_off,
                stream_degraded_reason=previous.stream_degraded_reason,
                camera_hardware_state=camera_state,
                camera_streamoff_capable=previous.camera_streamoff_capable,
                camera_last_revalidation_ms=previous.camera_last_revalidation_ms,
                camera_fatal_fault=previous.camera_fatal_fault,
                capture_active=capture_active,
                detector_mission_override=previous.detector_mission_override,
            )
            self._last_state = state
            return state

    @staticmethod
    def _capture_active(
        *,
        forced_off: bool,
        camera_fatal: bool,
        detector_req: bool,
        recording_consumer: bool,
        stream_active: int,
    ) -> bool:
        return (
            not forced_off
            and not camera_fatal
            and (detector_req or recording_consumer or stream_active > 0)
        )

    def last_state(self) -> PerceptionLifecycleState:
        with self._lock:
            return self._last_state

    def _detector_policy(
        self, fsm_state: FsmState, context: LifecycleMissionContext
    ) -> tuple[bool, str]:
        if fsm_state in {FsmState.HOME, FsmState.FAILSAFE}:
            return False, "force_off"
        if fsm_state in {FsmState.SEARCH, FsmState.CHASE}:
            return True, f"{fsm_state.value}_required"
        if fsm_state == FsmState.GOTO:
            return bool(context.goto_request_yolo), "goto_request_yolo"
        if fsm_state == FsmState.BRAKE_REVERSE:
            return bool(context.brake_saved_detector), "brake_inherit"
        return False, "policy_off"

    def _recording_policy(
        self,
        fsm_state: FsmState,
        mission: MissionState,
        context: LifecycleMissionContext,
        now_ms: int,
    ) -> tuple[bool, str, Optional[int]]:
        if fsm_state in {FsmState.HOME, FsmState.FAILSAFE}:
            return False, "force_off", None

        deadline = context.recording_postroll_deadline_ms
        if deadline is None:
            deadline = mission.recording_postroll_deadline_ms
        postroll_active = deadline is not None and now_ms < int(deadline)

        if fsm_state == FsmState.IDLE:
            if postroll_active:
                return True, "postroll", deadline
            return False, "idle", None

        if fsm_state == FsmState.GOTO:
            return (
                bool(context.goto_request_recording),
                "goto_request_recording",
                deadline if postroll_active else None,
            )

        if fsm_state == FsmState.BRAKE_REVERSE:
            return (
                bool(context.brake_saved_recording),
                "brake_inherit",
                deadline if postroll_active else None,
            )

        if fsm_state in {
            FsmState.GETTING_CLOSE,
            FsmState.SEARCH,
            FsmState.CHASE,
        }:
            if context.chase_recording_requested or mission.chase_recording_requested:
                return True, "chase_mission", None
            return False, "chase_no_recording", None

        if fsm_state == FsmState.RETURN_HOME:
            if postroll_active or context.chase_recording_requested:
                return True, "return_home_retain", deadline
            return False, "return_home", None

        return False, "policy_off", None
