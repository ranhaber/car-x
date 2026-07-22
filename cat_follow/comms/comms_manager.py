"""In-process CommsManager for the contract-driven runtime.

Validates incoming :class:`TrackingMessage` and :class:`CommandMessage`
objects, publishes them into :class:`SharedState`, emits ACK messages, and
maintains the command idempotency cache.

Milestone 2 deliberately uses an in-process API: producers call
:py:meth:`submit_tracking` / :py:meth:`submit_command` directly.  A future
Milestone 3 transport (UDP) will adapt incoming bytes to these dataclasses
without changing this module.
"""

from __future__ import annotations

import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from cat_follow.comms.messages import (
    AckMessage,
    CommandMessage,
    TrackingMessage,
)
from cat_follow.control.types import (
    AckStatus,
    AckType,
    CarTrackingState,
    CommandName,
    CommandState,
    FsmState,
    HomeState,
    OverheadState,
    ReasonCode,
    RejectionCause,
    TelemetryEventType,
    TelemetrySeverity,
    TrackingObjectState,
)
from cat_follow.runtime.shared_state import SharedState, now_monotonic_ms
from cat_follow.telemetry.async_logger import AsyncLogger


# Default size of the command-idempotency cache.  Matches Interface spec
# section 12.7 / 13.14 (`COMMAND_ID_CACHE_SIZE = 100`).
DEFAULT_COMMAND_ID_CACHE_SIZE = 100


@dataclass(frozen=True)
class _CommandResult:
    """Stored outcome of a processed command, used for idempotent retries."""

    ack_type: AckType
    status: AckStatus
    state: FsmState
    reason: ReasonCode
    cause: Optional[RejectionCause]


class CommsManager:
    """Validates, dispatches, and acknowledges contract messages."""

    def __init__(
        self,
        shared_state: SharedState,
        ack_sink: Callable[[AckMessage], None],
        logger: Optional[AsyncLogger] = None,
        command_id_cache_size: int = DEFAULT_COMMAND_ID_CACHE_SIZE,
        source: str = "CommsManager",
        on_emergency_stop: Optional[Callable[[], None]] = None,
    ) -> None:
        self._ss = shared_state
        self._ack_sink = ack_sink
        self._logger = logger
        self._cache_size = command_id_cache_size
        self._source = source
        # Synchronous e-stop actuation hook: invoked the moment an
        # emergency_stop command is accepted, so motors stop immediately
        # instead of waiting for the next ControlLoop tick (which may be hung).
        self._on_emergency_stop = on_emergency_stop
        self._lock = threading.Lock()
        self._command_results: "OrderedDict[str, _CommandResult]" = OrderedDict()
        self._last_tracking_sequence: int = -1
        self._next_outbound_sequence: int = 1

    # ── tracking ────────────────────────────────────────────────────

    def submit_tracking(self, msg: TrackingMessage) -> bool:
        """Accept a tracking packet and update SharedState.overhead.

        Returns True if the packet was accepted, False if dropped as a
        duplicate or out-of-order retry.
        """

        with self._lock:
            if msg.sequence <= self._last_tracking_sequence:
                self._log_tracking_duplicate(msg)
                return False
            self._last_tracking_sequence = msg.sequence

        now_ms = now_monotonic_ms()
        new_overhead = OverheadState(
            timestamp_ms=msg.timestamp_ms,
            received_ms=now_ms,
            fresh=True,
            authority=self._source,
            sequence=msg.sequence,
            frame_id=msg.frame_id,
            car=CarTrackingState(
                x=msg.car.x,
                y=msg.car.y,
                heading=msg.car.heading,
                heading_valid=msg.car.heading_valid,
                confidence=msg.car.confidence,
            ),
            cat=TrackingObjectState(
                x=msg.cat.x,
                y=msg.cat.y,
                confidence=msg.cat.confidence,
            ),
        )
        self._ss.update_overhead(new_overhead)
        self._log_tracking_received(msg)
        return True

    # ── command ─────────────────────────────────────────────────────

    def submit_command(self, msg: CommandMessage) -> AckMessage:
        """Accept a command packet, validate it, update state, and return ACK.

        Idempotent: a retry with the same ``command_id`` re-uses the original
        accept/reject result without re-executing side effects.
        """

        with self._lock:
            cached = self._command_results.get(msg.command_id)
            if cached is not None:
                self._command_results.move_to_end(msg.command_id)
                ack = self._build_ack(msg, cached)
                self._dispatch_ack(ack, duplicate=True)
                return ack

            self._log_command_received(msg)
            result = self._validate_and_apply(msg)
            self._command_results[msg.command_id] = result
            while len(self._command_results) > self._cache_size:
                self._command_results.popitem(last=False)
            ack = self._build_ack(msg, result)
            self._update_command_state(msg, result)
            self._dispatch_ack(ack, duplicate=False)
            return ack

    # ── stats helpers (mostly for tests) ────────────────────────────

    def cached_command_ids(self) -> int:
        with self._lock:
            return len(self._command_results)

    # ── internals ───────────────────────────────────────────────────

    def _validate_and_apply(self, msg: CommandMessage) -> _CommandResult:
        fsm_state = self._ss.get_fsm().state

        if msg.command == CommandName.SET_HOME:
            return self._handle_set_home(msg, fsm_state)
        if msg.command == CommandName.START_CHASE:
            return self._handle_start_chase(msg, fsm_state)
        if msg.command == CommandName.STOP_CHASE:
            return self._handle_stop_chase(msg, fsm_state)
        if msg.command == CommandName.RETURN_HOME:
            return self._handle_return_home(msg, fsm_state)
        if msg.command == CommandName.GO_TO:
            return self._handle_go_to(msg, fsm_state)
        if msg.command == CommandName.EMERGENCY_STOP:
            return self._handle_emergency_stop(msg, fsm_state)
        if msg.command == CommandName.CLEAR_FAILSAFE:
            return self._handle_clear_failsafe(msg, fsm_state)

        return _CommandResult(
            ack_type=AckType.COMMAND,
            status=AckStatus.REJECTED,
            state=fsm_state,
            reason=ReasonCode.TRANSITION_REJECTED,
            cause=RejectionCause.INVALID_COMMAND,
        )

    # set_home ──────────────────────────────────────────────────────

    def _handle_set_home(self, msg: CommandMessage, state: FsmState) -> _CommandResult:
        home_payload = msg.params.get("home")
        validation = _validate_yard_point(home_payload)
        if validation is not None:
            return _reject(state, ReasonCode.TRANSITION_REJECTED, validation)

        x, y, frame_id = _yard_point(home_payload)
        self._ss.update_home(
            HomeState(
                timestamp_ms=msg.timestamp_ms,
                received_ms=now_monotonic_ms(),
                fresh=True,
                authority=self._source,
                set=True,
                x=x,
                y=y,
                frame_id=frame_id,
                source_command_id=msg.command_id,
            )
        )
        return _CommandResult(
            ack_type=AckType.COMMAND,
            status=AckStatus.ACCEPTED,
            state=state,
            reason=ReasonCode.INIT,
            cause=None,
        )

    # start_chase ───────────────────────────────────────────────────

    def _handle_start_chase(self, msg: CommandMessage, state: FsmState) -> _CommandResult:
        overhead = self._ss.get_overhead()
        if overhead.car.confidence < 1.0:
            return _CommandResult(
                ack_type=AckType.COMMAND,
                status=AckStatus.REJECTED,
                state=state,
                reason=ReasonCode.START_CHASE_REJECTED,
                cause=RejectionCause.CAR_POSITION_INVALID,
            )
        if overhead.cat.confidence < 1.0:
            return _CommandResult(
                ack_type=AckType.COMMAND,
                status=AckStatus.REJECTED,
                state=state,
                reason=ReasonCode.START_CHASE_REJECTED,
                cause=RejectionCause.CAT_POSITION_INVALID,
            )
        if overhead.received_ms <= 0:
            return _CommandResult(
                ack_type=AckType.COMMAND,
                status=AckStatus.REJECTED,
                state=state,
                reason=ReasonCode.START_CHASE_REJECTED,
                cause=RejectionCause.TRACKING_STALE,
            )
        return _CommandResult(
            ack_type=AckType.COMMAND,
            status=AckStatus.ACCEPTED,
            state=state,
            reason=ReasonCode.START_CHASE_ACCEPTED,
            cause=None,
        )

    # stop_chase ────────────────────────────────────────────────────

    def _handle_stop_chase(self, msg: CommandMessage, state: FsmState) -> _CommandResult:
        if state == FsmState.FAILSAFE:
            return _reject(
                state,
                ReasonCode.TRANSITION_REJECTED,
                RejectionCause.FAILSAFE_ACTIVE,
            )
        return _CommandResult(
            ack_type=AckType.COMMAND,
            status=AckStatus.ACCEPTED,
            state=state,
            reason=ReasonCode.STOP_CHASE_ACCEPTED,
            cause=None,
        )

    # return_home ───────────────────────────────────────────────────

    def _handle_return_home(self, msg: CommandMessage, state: FsmState) -> _CommandResult:
        home_payload = msg.params.get("home")
        validation = _validate_yard_point(home_payload)
        if validation is not None:
            return _reject(state, ReasonCode.TRANSITION_REJECTED, validation)
        if state == FsmState.FAILSAFE:
            return _reject(
                state,
                ReasonCode.TRANSITION_REJECTED,
                RejectionCause.FAILSAFE_ACTIVE,
            )

        x, y, frame_id = _yard_point(home_payload)
        self._ss.update_home(
            HomeState(
                timestamp_ms=msg.timestamp_ms,
                received_ms=now_monotonic_ms(),
                fresh=True,
                authority=self._source,
                set=True,
                x=x,
                y=y,
                frame_id=frame_id,
                source_command_id=msg.command_id,
            )
        )
        return _CommandResult(
            ack_type=AckType.COMMAND,
            status=AckStatus.ACCEPTED,
            state=state,
            reason=ReasonCode.RETURN_HOME_ACCEPTED,
            cause=None,
        )

    # go_to ─────────────────────────────────────────────────────────

    def _handle_go_to(self, msg: CommandMessage, state: FsmState) -> _CommandResult:
        target_payload = msg.params.get("target")
        validation = _validate_yard_point(target_payload, kind="target")
        if validation is not None:
            return _reject(state, ReasonCode.TRANSITION_REJECTED, validation)
        if state == FsmState.FAILSAFE:
            return _reject(
                state,
                ReasonCode.TRANSITION_REJECTED,
                RejectionCause.FAILSAFE_ACTIVE,
            )
        return _CommandResult(
            ack_type=AckType.COMMAND,
            status=AckStatus.ACCEPTED,
            state=state,
            reason=ReasonCode.GO_TO_ACCEPTED,
            cause=None,
        )

    # emergency_stop ────────────────────────────────────────────────

    def _handle_emergency_stop(self, msg: CommandMessage, state: FsmState) -> _CommandResult:
        # Actuate the stop synchronously (motor e-stop + FAILSAFE latch) before
        # ACKing.  The DecisionEngine still consumes the accepted command to keep
        # FSM state consistent, but safety must not wait for the control tick.
        if self._on_emergency_stop is not None:
            try:
                self._on_emergency_stop()
            except Exception:
                pass
        return _CommandResult(
            ack_type=AckType.COMMAND,
            status=AckStatus.ACCEPTED,
            state=state,
            reason=ReasonCode.FAILSAFE_TRIGGERED,
            cause=None,
        )

    # clear_failsafe ────────────────────────────────────────────────

    def _handle_clear_failsafe(self, msg: CommandMessage, state: FsmState) -> _CommandResult:
        confirmed = bool(msg.params.get("operator_confirmed", False))
        if not confirmed:
            return _reject(
                state,
                ReasonCode.TRANSITION_REJECTED,
                RejectionCause.OPERATOR_CONFIRMATION_REQUIRED,
            )
        return _CommandResult(
            ack_type=AckType.COMMAND,
            status=AckStatus.ACCEPTED,
            state=state,
            reason=ReasonCode.CLEAR_FAILSAFE_ACCEPTED,
            cause=None,
        )

    # ── ack / publish helpers ───────────────────────────────────────

    def _build_ack(
        self, msg: CommandMessage, result: _CommandResult
    ) -> AckMessage:
        return AckMessage(
            sequence=self._next_seq(),
            timestamp_ms=int(time.time() * 1000),
            ack_sequence=msg.sequence,
            ack_type=result.ack_type,
            command_id=msg.command_id,
            status=result.status,
            state=result.state,
            reason=result.reason,
            cause=result.cause,
        )

    def _next_seq(self) -> int:
        seq = self._next_outbound_sequence
        self._next_outbound_sequence += 1
        return seq

    def _update_command_state(
        self, msg: CommandMessage, result: _CommandResult
    ) -> None:
        self._ss.update_command(
            CommandState(
                timestamp_ms=msg.timestamp_ms,
                received_ms=now_monotonic_ms(),
                fresh=True,
                authority=self._source,
                last_command_id=msg.command_id,
                last_command=msg.command,
                last_status=result.status,
                last_reason=result.reason,
                last_cause=result.cause,
            )
        )

    def _dispatch_ack(self, ack: AckMessage, *, duplicate: bool) -> None:
        try:
            self._ack_sink(ack)
        except Exception:
            # Sink failures must never cascade; future work surfaces them
            # via thread_health telemetry.
            pass
        self._log_command_ack(ack, duplicate=duplicate)

    # ── telemetry helpers ───────────────────────────────────────────

    def _log_tracking_received(self, msg: TrackingMessage) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.TRACKING_RECEIVED,
            severity=TelemetrySeverity.DEBUG,
            source=self._source,
            state=self._ss.get_fsm().state,
            data={
                "sequence": msg.sequence,
                "car_confidence": msg.car.confidence,
                "cat_confidence": msg.cat.confidence,
            },
        )

    def _log_tracking_duplicate(self, msg: TrackingMessage) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.TRACKING_RECEIVED,
            severity=TelemetrySeverity.DEBUG,
            source=self._source,
            state=self._ss.get_fsm().state,
            data={
                "sequence": msg.sequence,
                "duplicate_or_out_of_order": True,
            },
        )

    def _log_command_received(self, msg: CommandMessage) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.COMMAND_RECEIVED,
            severity=TelemetrySeverity.INFO,
            source=self._source,
            state=self._ss.get_fsm().state,
            data={
                "sequence": msg.sequence,
                "command_id": msg.command_id,
                "command": msg.command.value,
            },
        )

    def _log_command_ack(self, ack: AckMessage, *, duplicate: bool) -> None:
        if self._logger is None:
            return
        severity = (
            TelemetrySeverity.WARNING
            if ack.status == AckStatus.REJECTED
            else TelemetrySeverity.INFO
        )
        self._logger.log(
            event_type=TelemetryEventType.COMMAND_ACK,
            severity=severity,
            source=self._source,
            state=ack.state,
            data={
                "ack_sequence": ack.ack_sequence,
                "command_id": ack.command_id,
                "status": ack.status.value,
                "reason": ack.reason.value,
                "cause": ack.cause.value if ack.cause is not None else None,
                "duplicate": duplicate,
            },
        )


# ── helpers ─────────────────────────────────────────────────────────


def _yard_point(payload: Dict[str, Any]):
    return (
        float(payload["x"]),
        float(payload["y"]),
        str(payload.get("frame_id", "yard")),
    )


def _validate_yard_point(
    payload: Optional[Dict[str, Any]], *, kind: str = "home"
) -> Optional[RejectionCause]:
    if payload is None or not isinstance(payload, dict):
        return (
            RejectionCause.HOME_MISSING
            if kind == "home"
            else RejectionCause.TARGET_INVALID
        )
    if "x" not in payload or "y" not in payload:
        return (
            RejectionCause.HOME_INVALID
            if kind == "home"
            else RejectionCause.TARGET_INVALID
        )
    try:
        float(payload["x"])
        float(payload["y"])
    except (TypeError, ValueError):
        return (
            RejectionCause.HOME_INVALID
            if kind == "home"
            else RejectionCause.TARGET_INVALID
        )
    if str(payload.get("frame_id", "yard")) != "yard":
        return (
            RejectionCause.HOME_INVALID
            if kind == "home"
            else RejectionCause.TARGET_INVALID
        )
    return None


def _reject(
    state: FsmState, reason: ReasonCode, cause: RejectionCause
) -> _CommandResult:
    return _CommandResult(
        ack_type=AckType.COMMAND,
        status=AckStatus.REJECTED,
        state=state,
        reason=reason,
        cause=cause,
    )


__all__ = [
    "CommsManager",
    "DEFAULT_COMMAND_ID_CACHE_SIZE",
]
