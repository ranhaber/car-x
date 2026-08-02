"""Validated FSM for the target control architecture.

The FSM is the single canonical owner of the system mode.  Every transition
must be explicitly listed in the transition table or the pattern rules below
or it is rejected.  Rejected transitions are recorded so DecisionEngine and
telemetry can surface the violation.

Conformance reference: Interface and Data Contract Specification, section
10 (FSM States, Events, Transition Rules, and Reason Codes).
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from cat_follow.control.types import (
    FSMSnapshot,
    FsmEvent,
    FsmState,
    ReasonCode,
)
from cat_follow.runtime.shared_state import now_monotonic_ms


# ── Transition tables ───────────────────────────────────────────────

# Direct (from-state, event) -> to-state lookups.  Anything not listed here
# falls through to the pattern rules in :func:`_resolve_transition`.
_DIRECT_TRANSITIONS = {
    (FsmState.HOME, FsmEvent.START_CHASE_ACCEPTED): FsmState.GETTING_CLOSE,
    (FsmState.IDLE, FsmEvent.START_CHASE_ACCEPTED): FsmState.GETTING_CLOSE,
    (FsmState.GETTING_CLOSE, FsmEvent.SEARCH_ENTRY_READY): FsmState.SEARCH,
    (FsmState.SEARCH, FsmEvent.LOCAL_TRACK_ACQUIRED): FsmState.CHASE,
    (FsmState.CHASE, FsmEvent.CAT_LOST_NEAR): FsmState.SEARCH,
    (FsmState.CHASE, FsmEvent.CAT_LOST_FAR): FsmState.GETTING_CLOSE,
    (FsmState.GETTING_CLOSE, FsmEvent.TARGET_ID_CHANGED): FsmState.IDLE,
    (FsmState.SEARCH, FsmEvent.TARGET_ID_CHANGED): FsmState.IDLE,
    (FsmState.HOME, FsmEvent.GO_TO_ACCEPTED): FsmState.GOTO,
    (FsmState.IDLE, FsmEvent.GO_TO_ACCEPTED): FsmState.GOTO,
    (FsmState.GOTO, FsmEvent.GO_TO_COMPLETE): FsmState.IDLE,
    (
        FsmState.GOTO,
        FsmEvent.NAVIGATION_FAILURES_EXHAUSTED,
    ): FsmState.IDLE,
    (FsmState.RETURN_HOME, FsmEvent.RETURN_HOME_COMPLETE): FsmState.HOME,
    (
        FsmState.RETURN_HOME,
        FsmEvent.NAVIGATION_FAILURES_EXHAUSTED,
    ): FsmState.FAILSAFE,
    (FsmState.FAILSAFE, FsmEvent.CLEAR_FAILSAFE_ACCEPTED): FsmState.IDLE,
}

# Chase state set used by the ``stop_chase`` pattern rule.  Public so the
# DecisionEngine shares one definition instead of keeping a parallel copy that
# can silently drift from the transition rules.
CHASE_STATES = frozenset(
    {
        FsmState.GETTING_CLOSE,
        FsmState.SEARCH,
        FsmState.CHASE,
        FsmState.BRAKE_REVERSE,
    }
)

NORMAL_DRIVING_STATES = frozenset(
    {
        FsmState.GETTING_CLOSE,
        FsmState.SEARCH,
        FsmState.CHASE,
        FsmState.GOTO,
        FsmState.RETURN_HOME,
    }
)


def _resolve_transition(
    state: FsmState,
    event: FsmEvent,
    *,
    resume_state: Optional[FsmState] = None,
) -> Optional[FsmState]:
    """Return the target state for ``(state, event)`` or ``None`` if rejected."""

    target = _DIRECT_TRANSITIONS.get((state, event))
    if target is not None:
        return target

    if (
        event == FsmEvent.BRAKE_REVERSE_TRIGGERED
        and state in NORMAL_DRIVING_STATES
    ):
        return FsmState.BRAKE_REVERSE

    if (
        event == FsmEvent.BRAKE_REVERSE_CLEARED
        and state == FsmState.BRAKE_REVERSE
        and resume_state in NORMAL_DRIVING_STATES
    ):
        return resume_state

    # ``CHASE_STATES`` includes ``BRAKE_REVERSE``, where the matrix additionally
    # requires the interrupted objective to be a chase.  That predicate needs
    # the DecisionEngine's saved state, so it is enforced by the command/event
    # apply path rather than duplicated here.
    if (
        event == FsmEvent.PRIMARY_CAT_LEFT_PERIMETER
        and state in CHASE_STATES
    ):
        return FsmState.IDLE

    # Each event below is only ever raised by DecisionEngine from the states
    # listed here; the sets intentionally differ per event instead of sharing
    # one over-broad set, so the table cannot silently accept an event from a
    # state no caller actually uses it from.
    if (
        event == FsmEvent.OVERHEAD_RETENTION_EXPIRED
        and state
        in {FsmState.GETTING_CLOSE, FsmState.SEARCH, FsmState.CHASE}
    ):
        return FsmState.RETURN_HOME

    if event == FsmEvent.SEARCH_EXHAUSTED and state == FsmState.SEARCH:
        return FsmState.RETURN_HOME

    if event == FsmEvent.HANDOFF_TIMEOUT and state == FsmState.IDLE:
        return FsmState.RETURN_HOME

    if (
        event == FsmEvent.NAVIGATION_FAILURES_EXHAUSTED
        and state
        in {FsmState.GETTING_CLOSE, FsmState.SEARCH, FsmState.CHASE}
    ):
        return FsmState.RETURN_HOME

    # ``stop_chase_accepted`` from any chase state -> IDLE.
    if event == FsmEvent.STOP_CHASE_ACCEPTED and state in CHASE_STATES:
        return FsmState.IDLE

    # ``return_home_accepted`` from any non-FAILSAFE state -> RETURN_HOME.
    if (
        event == FsmEvent.RETURN_HOME_ACCEPTED
        and state != FsmState.FAILSAFE
    ):
        return FsmState.RETURN_HOME

    # Safety paths to FAILSAFE valid from any state.
    if event in {
        FsmEvent.OBSTACLE_TOO_CLOSE,
        FsmEvent.FAILSAFE_TRIGGERED,
        FsmEvent.EMERGENCY_STOP_ACCEPTED,
    }:
        return FsmState.FAILSAFE

    return None


def is_transition_allowed(
    state: FsmState,
    event: FsmEvent,
    *,
    resume_state: Optional[FsmState] = None,
) -> bool:
    """Return True if ``state`` can transition on ``event``.

    Useful for tests and for callers that want to query before applying.
    ``resume_state`` must be supplied for resume-dependent events such as
    ``BRAKE_REVERSE_CLEARED``, otherwise they always report disallowed even
    though :py:meth:`FSM.apply` would accept the restore.
    """

    return _resolve_transition(state, event, resume_state=resume_state) is not None


# ── FSM ─────────────────────────────────────────────────────────────


class TransitionResult:
    """Result of an :py:meth:`FSM.apply` call.

    A small structured object instead of returning a bare boolean so callers
    can inspect the rejected-transition descriptor for telemetry without
    re-deriving it.
    """

    __slots__ = ("accepted", "from_state", "to_state", "rejected_descriptor")

    def __init__(
        self,
        accepted: bool,
        from_state: FsmState,
        to_state: FsmState,
        rejected_descriptor: Optional[str],
    ) -> None:
        self.accepted = accepted
        self.from_state = from_state
        self.to_state = to_state
        self.rejected_descriptor = rejected_descriptor


class FSM:
    """Thread-safe validated state machine.

    Producers call :py:meth:`apply` with a triggering :class:`FsmEvent` and
    the :class:`ReasonCode` that explains why the transition was requested.
    Snapshot consumers call :py:meth:`snapshot` to obtain an immutable
    :class:`FSMSnapshot` suitable for publishing into ``SharedState.fsm``.
    """

    def __init__(
        self,
        initial_state: FsmState = FsmState.IDLE,
        initial_reason: ReasonCode = ReasonCode.INIT,
    ) -> None:
        self._lock = threading.Lock()
        self._state: FsmState = initial_state
        self._previous_state: Optional[FsmState] = None
        self._last_transition_ms: int = 0
        self._last_transition_reason: ReasonCode = initial_reason
        self._last_rejected_transition: Optional[str] = None

    # ── public state helpers ────────────────────────────────────────

    @property
    def state(self) -> FsmState:
        with self._lock:
            return self._state

    def apply(
        self,
        event: FsmEvent,
        *,
        reason: ReasonCode,
        now_ms: Optional[int] = None,
        resume_state: Optional[FsmState] = None,
    ) -> TransitionResult:
        """Attempt to apply ``event``.

        Returns a :class:`TransitionResult` describing whether the transition
        was accepted.  Rejected transitions update the ``last_rejected``
        descriptor and leave the current state unchanged.
        """

        if now_ms is None:
            now_ms = now_monotonic_ms()

        with self._lock:
            from_state = self._state
            target = _resolve_transition(
                from_state, event, resume_state=resume_state
            )
            if target is None:
                descriptor = (
                    f"{from_state.value} + {event.value} -> rejected"
                )
                self._last_rejected_transition = descriptor
                return TransitionResult(
                    accepted=False,
                    from_state=from_state,
                    to_state=from_state,
                    rejected_descriptor=descriptor,
                )

            self._previous_state = from_state
            self._state = target
            self._last_transition_ms = now_ms
            self._last_transition_reason = reason
            self._last_rejected_transition = None
            return TransitionResult(
                accepted=True,
                from_state=from_state,
                to_state=target,
                rejected_descriptor=None,
            )

    def snapshot(
        self,
        *,
        timestamp_ms: Optional[int] = None,
        received_ms: Optional[int] = None,
    ) -> FSMSnapshot:
        """Return an immutable snapshot suitable for SharedState publication."""

        if received_ms is None:
            received_ms = now_monotonic_ms()
        if timestamp_ms is None:
            timestamp_ms = int(time.time() * 1000)

        with self._lock:
            return FSMSnapshot(
                timestamp_ms=timestamp_ms,
                received_ms=received_ms,
                fresh=True,
                authority="FSM",
                state=self._state,
                previous_state=self._previous_state,
                last_transition_ms=self._last_transition_ms,
                last_transition_reason=self._last_transition_reason,
                last_rejected_transition=self._last_rejected_transition,
            )

    # ── force overrides (use with care) ─────────────────────────────

    def force_state(
        self,
        state: FsmState,
        *,
        reason: ReasonCode,
        now_ms: Optional[int] = None,
    ) -> FSMSnapshot:
        """Unconditionally set the state.

        Reserved for hard process-level safety paths and bring-up tests.  The
        normal control flow must use :py:meth:`apply` so transitions remain
        validated.
        """

        if now_ms is None:
            now_ms = now_monotonic_ms()

        with self._lock:
            self._previous_state = self._state
            self._state = state
            self._last_transition_ms = now_ms
            self._last_transition_reason = reason
            self._last_rejected_transition = None

        return self.snapshot()


__all__ = [
    "CHASE_STATES",
    "FSM",
    "NORMAL_DRIVING_STATES",
    "TransitionResult",
    "is_transition_allowed",
]
