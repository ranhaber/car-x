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
    (FsmState.HOME, FsmEvent.START_CHASE_ACCEPTED): FsmState.CHASE_A,
    (FsmState.IDLE, FsmEvent.START_CHASE_ACCEPTED): FsmState.CHASE_A,
    (FsmState.CHASE_A, FsmEvent.CAT_VISIBLE_STABLE): FsmState.TRACK_B,
    (FsmState.TRACK_B, FsmEvent.CAT_LOST): FsmState.CHASE_A,
    (FsmState.TRACK_B, FsmEvent.FINAL_APPROACH_READY): FsmState.BRAKE,
    (FsmState.BRAKE, FsmEvent.BRAKE_ABORTED_CAT_MOVED): FsmState.TRACK_B,
    (FsmState.HOME, FsmEvent.GO_TO_ACCEPTED): FsmState.GOTO,
    (FsmState.IDLE, FsmEvent.GO_TO_ACCEPTED): FsmState.GOTO,
    (FsmState.GOTO, FsmEvent.GO_TO_COMPLETE): FsmState.IDLE,
    (FsmState.RETURN_HOME, FsmEvent.RETURN_HOME_COMPLETE): FsmState.HOME,
    (FsmState.FAILSAFE, FsmEvent.CLEAR_FAILSAFE_ACCEPTED): FsmState.IDLE,
}

# Chase state set used by the ``stop_chase`` pattern rule.
_CHASE_STATES = frozenset(
    {FsmState.CHASE_A, FsmState.TRACK_B, FsmState.BRAKE}
)


def _resolve_transition(
    state: FsmState, event: FsmEvent
) -> Optional[FsmState]:
    """Return the target state for ``(state, event)`` or ``None`` if rejected."""

    target = _DIRECT_TRANSITIONS.get((state, event))
    if target is not None:
        return target

    # ``stop_chase_accepted`` from any chase state -> IDLE.
    if event == FsmEvent.STOP_CHASE_ACCEPTED and state in _CHASE_STATES:
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


def is_transition_allowed(state: FsmState, event: FsmEvent) -> bool:
    """Return True if ``state`` can transition on ``event``.

    Useful for tests and for callers that want to query before applying.
    """

    return _resolve_transition(state, event) is not None


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
            target = _resolve_transition(from_state, event)
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
    "FSM",
    "TransitionResult",
    "is_transition_allowed",
]
