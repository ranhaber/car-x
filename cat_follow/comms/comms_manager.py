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
from collections import OrderedDict
from dataclasses import dataclass
from typing import Callable, Dict, Optional

from cat_follow.comms.messages import (
    AckMessage,
    CommandMessage,
    MissionEventMessage,
    PendingTransaction,
    TrackingMessage,
)
from cat_follow.comms.transaction_apply import (
    TransactionResult,
    apply_pending_transaction,
)
from cat_follow.control.types import (
    AckStatus,
    AckType,
    CarTrackingState,
    CommandName,
    FsmState,
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

# How long a submitter waits for the control loop to commit its transaction
# and publish the ACK before giving up.
DEFAULT_COMMIT_TIMEOUT_S = 2.0


@dataclass(frozen=True)
class _CommandResult:
    """Stored outcome of a processed command, used for idempotent retries."""

    ack_type: AckType
    status: AckStatus
    state: FsmState
    reason: ReasonCode
    cause: Optional[RejectionCause]
    applied_control_seq: Optional[int] = None


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
        on_start_chase: Optional[Callable[[], None]] = None,
        home_store=None,
        geofence_polygon=None,
        commit_timeout_s: float = DEFAULT_COMMIT_TIMEOUT_S,
    ) -> None:
        self._ss = shared_state
        self._ack_sink = ack_sink
        self._logger = logger
        self._cache_size = command_id_cache_size
        self._source = source
        self._commit_timeout_s = float(commit_timeout_s)
        # Synchronous e-stop actuation hook: invoked the moment an
        # emergency_stop command is accepted, so motors stop immediately
        # instead of waiting for the next ControlLoop tick (which may be hung).
        self._on_emergency_stop = on_emergency_stop
        # Lightweight edge-trigger hook used to ask the detector thread to
        # load/warm its lazily-unloaded RKNN model after START_CHASE is accepted.
        self._on_start_chase = on_start_chase
        self._home_store = home_store
        self._geofence_polygon = geofence_polygon
        self._lock = threading.Lock()
        # Serialize command semantics separately from the short-lived cache /
        # tracking lock. Hardware and ACK I/O never run while ``_lock`` is held,
        # so an emergency stop cannot block incoming tracking updates.
        self._command_lock = threading.Lock()
        # The outbound sequence counter has its own lock so ACKs can be built
        # without taking either of the locks above.
        self._sequence_lock = threading.Lock()
        self._command_results: "OrderedDict[str, _CommandResult]" = OrderedDict()
        self._event_results: "OrderedDict[str, _CommandResult]" = OrderedDict()
        self._ack_waiters: Dict[str, threading.Event] = {}
        self._completed_acks: Dict[str, AckMessage] = {}
        self._control_loop = None
        self._engine = None
        self._fsm = None
        self._last_tracking_sequence: int = -1
        self._next_outbound_sequence: int = 1

    def bind_runtime(self, *, control_loop=None, decision_engine=None, fsm=None) -> None:
        """Attach runtime components used for transactional commit."""

        self._control_loop = control_loop
        self._engine = decision_engine
        self._fsm = fsm

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
            perimeter_id=msg.perimeter_id,
            calibration_version=msg.calibration_version,
            selected_target_id=msg.selected_target_id,
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
                target_id=msg.cat.target_id,
                inside_perimeter=msg.cat.inside_perimeter,
            ),
        )
        self._ss.update_overhead(new_overhead)
        self._log_tracking_received(msg)
        return True

    # ── command ─────────────────────────────────────────────────────

    def submit_command(self, msg: CommandMessage) -> AckMessage:
        """Queue a command for control-loop commit and return its ACK."""

        # Emergency stop actuates directly: it must never queue behind another
        # command's critical section, and it never enqueues a transaction.
        if msg.command == CommandName.EMERGENCY_STOP:
            cached = self._cached_result(self._command_results, msg.command_id)
            if cached is not None:
                return self._emit_ack(self._build_ack(msg, cached), duplicate=True)
            self._log_command_received(msg)
            return self._commit_emergency_stop(msg)

        duplicate: Optional[_CommandResult] = None
        waiter: Optional[threading.Event] = None
        with self._command_lock:
            duplicate = self._cached_result(self._command_results, msg.command_id)
            if duplicate is None:
                self._log_command_received(msg)
                waiter = threading.Event()
                self._ack_waiters[msg.command_id] = waiter
                self._ss.enqueue_pending(PendingTransaction(command=msg))

        # ACK dispatch is network I/O and stays outside every lock.
        if duplicate is not None:
            return self._emit_ack(self._build_ack(msg, duplicate), duplicate=True)

        assert waiter is not None
        return self._await_commit(msg.command_id, waiter, kind="command_id")

    def submit_mission_event(self, msg: MissionEventMessage) -> AckMessage:
        """Queue a mission event for control-loop commit and return its ACK."""

        duplicate: Optional[_CommandResult] = None
        waiter: Optional[threading.Event] = None
        with self._command_lock:
            duplicate = self._cached_result(self._event_results, msg.event_id)
            if duplicate is None:
                waiter = threading.Event()
                self._ack_waiters[msg.event_id] = waiter
                self._ss.enqueue_pending(PendingTransaction(mission_event=msg))

        if duplicate is not None:
            return self._emit_ack(
                self._build_event_ack(msg, duplicate), duplicate=True
            )

        assert waiter is not None
        return self._await_commit(msg.event_id, waiter, kind="event_id")

    def _cached_result(
        self, cache: "OrderedDict[str, _CommandResult]", message_id: str
    ) -> Optional[_CommandResult]:
        """Return the stored outcome for ``message_id`` and mark it recent."""

        with self._lock:
            cached = cache.get(message_id)
            if cached is not None:
                cache.move_to_end(message_id)
            return cached

    def _await_commit(
        self, message_id: str, waiter: threading.Event, *, kind: str
    ) -> AckMessage:
        """Block until the queued transaction is committed and ACKed."""

        try:
            if self._control_loop is None:
                self._ensure_local_runtime()
                assert self._engine is not None
                assert self._fsm is not None
                self.apply_pending_transactions(
                    applied_control_seq=1,
                    decision_engine=self._engine,
                    fsm=self._fsm,
                )
            elif not waiter.wait(timeout=self._commit_timeout_s):
                raise TimeoutError(
                    f"timed out waiting for ACK on {kind}={message_id!r}"
                )
            with self._command_lock:
                return self._completed_acks.pop(message_id)
        finally:
            # Drop the waiter (and any ACK that landed after a timeout) so a
            # stalled control loop cannot grow these dicts without bound.
            with self._command_lock:
                self._ack_waiters.pop(message_id, None)
                self._completed_acks.pop(message_id, None)

    def apply_pending_transactions(
        self,
        *,
        applied_control_seq: int,
        decision_engine,
        fsm,
    ) -> None:
        """Apply queued ingress at a control-loop boundary and emit ACKs."""

        pending = self._ss.drain_pending()
        for txn in pending:
            result = apply_pending_transaction(
                txn,
                shared_state=self._ss,
                fsm=fsm,
                engine=decision_engine,
                applied_control_seq=applied_control_seq,
                authority=self._source,
                on_start_chase=self._on_start_chase,
                home_store=self._home_store,
                geofence_polygon=self._geofence_polygon,
            )
            self._commit_transaction(txn, result)

    def _ensure_local_runtime(self) -> None:
        """Provide a synchronous boundary when no ControlLoop is attached."""

        if self._engine is not None and self._fsm is not None:
            return
        from cat_follow.control.decision_engine import DecisionEngine
        from cat_follow.control.fsm import FSM

        self._fsm = FSM(initial_state=self._ss.get_fsm().state)
        self._engine = DecisionEngine(self._fsm)

    def _commit_emergency_stop(self, msg: CommandMessage) -> AckMessage:
        """Stop the vehicle now, holding no lock across motor or ACK I/O."""

        result: Optional[TransactionResult] = None
        if self._engine is not None and self._fsm is not None:
            result = apply_pending_transaction(
                PendingTransaction(command=msg),
                shared_state=self._ss,
                fsm=self._fsm,
                engine=self._engine,
                applied_control_seq=0,
                authority=self._source,
                home_store=self._home_store,
                geofence_polygon=self._geofence_polygon,
            )
        if self._on_emergency_stop is not None:
            try:
                self._on_emergency_stop()
            except Exception:
                pass
        # The hook latches FAILSAFE itself, so the ACK state is read after it
        # runs; reading it earlier advertises the pre-stop state.
        state = (
            self._fsm.state if self._fsm is not None else self._ss.get_fsm().state
        )
        if result is None:
            stored = _CommandResult(
                ack_type=AckType.COMMAND,
                status=AckStatus.ACCEPTED,
                state=state,
                reason=ReasonCode.FAILSAFE_TRIGGERED,
                cause=None,
                applied_control_seq=0,
            )
        else:
            stored = _CommandResult(
                ack_type=result.ack_type,
                status=result.status,
                state=state,
                reason=result.reason,
                cause=result.cause,
                applied_control_seq=result.applied_control_seq,
            )
        with self._lock:
            self._command_results[msg.command_id] = stored
            self._trim_cache(self._command_results)
        return self._emit_ack(self._build_ack(msg, stored), duplicate=False)

    def _commit_transaction(
        self, txn: PendingTransaction, result: TransactionResult
    ) -> None:
        stored = _CommandResult(
            ack_type=result.ack_type,
            status=result.status,
            state=result.state,
            reason=result.reason,
            cause=result.cause,
            applied_control_seq=result.applied_control_seq,
        )
        message_id: str
        if txn.command is not None:
            message_id = txn.command.command_id
            cache = self._command_results
            ack = self._build_ack(txn.command, stored)
        else:
            assert txn.mission_event is not None
            message_id = txn.mission_event.event_id
            cache = self._event_results
            ack = self._build_event_ack(txn.mission_event, stored)

        with self._lock:
            cache[message_id] = stored
            self._trim_cache(cache)

        self._dispatch_ack(ack, duplicate=False)
        with self._command_lock:
            waiter = self._ack_waiters.get(message_id)
            if waiter is not None:
                # Only hand the ACK over while a submitter is still waiting for
                # it; after a timeout nobody would ever collect it again.
                self._completed_acks[message_id] = ack
                waiter.set()

    def _trim_cache(self, cache: "OrderedDict[str, _CommandResult]") -> None:
        """Bound an idempotency cache; caller holds ``_lock``."""

        while len(cache) > self._cache_size:
            cache.popitem(last=False)

    def _emit_ack(self, ack: AckMessage, *, duplicate: bool) -> AckMessage:
        self._dispatch_ack(ack, duplicate=duplicate)
        return ack

    # ── stats helpers (mostly for tests) ────────────────────────────

    def cached_command_ids(self) -> int:
        with self._lock:
            return len(self._command_results)

    def pending_ack_waiters(self) -> int:
        """Number of submitters currently waiting for a commit ACK."""

        with self._command_lock:
            return len(self._ack_waiters)

    def uncollected_acks(self) -> int:
        """Number of committed ACKs not yet collected by their submitter."""

        with self._command_lock:
            return len(self._completed_acks)

    @property
    def active_target_id(self) -> Optional[str]:
        return self._ss.get_mission().active_target_id

    # ── ack / publish helpers ───────────────────────────────────────

    def _build_ack(
        self, msg: CommandMessage, result: _CommandResult
    ) -> AckMessage:
        return AckMessage(
            sequence=self._next_seq(),
            timestamp_ms=now_monotonic_ms(),
            ack_sequence=msg.sequence,
            ack_type=result.ack_type,
            command_id=msg.command_id,
            status=result.status,
            state=result.state,
            reason=result.reason,
            cause=result.cause,
            applied_control_sequence=result.applied_control_seq,
        )

    def _build_event_ack(
        self, msg: MissionEventMessage, result: _CommandResult
    ) -> AckMessage:
        return AckMessage(
            sequence=self._next_seq(),
            timestamp_ms=now_monotonic_ms(),
            ack_sequence=msg.observation_sequence,
            ack_type=AckType.MISSION_EVENT,
            command_id=msg.event_id,
            status=result.status,
            state=result.state,
            reason=result.reason,
            cause=result.cause,
            applied_control_sequence=result.applied_control_seq,
        )

    def _next_seq(self) -> int:
        with self._sequence_lock:
            seq = self._next_outbound_sequence
            self._next_outbound_sequence += 1
            return seq

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


__all__ = [
    "CommsManager",
    "DEFAULT_COMMAND_ID_CACHE_SIZE",
]
