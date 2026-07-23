"""Open-loop motion sequence executor for the Movement tab.

During contract runtime the executor is ticked from :class:`DecisionEngine`
so motor commands still flow through the 50 Hz safety stack
(``MotorInterface`` + obstacle veto).  The web UI must send heartbeats
while a sequence runs; stale heartbeats abort the sequence (dead-man) but
do not latch FAILSAFE.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence

from cat_follow.motion.action_plan import (
    DriveAction,
    SteerAction,
    StopAction,
    ValidatedAction,
    WaitAction,
    plan_to_public_dict,
    validate_plan,
)


DEFAULT_HEARTBEAT_TIMEOUT_MS = 500


class SequenceStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    COMPLETED = "completed"
    ABORTED = "aborted"


@dataclass(frozen=True)
class SequenceMotionCommand:
    speed: float
    steering: float
    brake: bool


@dataclass
class _RuntimeState:
    status: SequenceStatus = SequenceStatus.IDLE
    plan: List[ValidatedAction] | None = None
    action_index: int = 0
    action_started_ms: int = 0
    started_ms: int = 0
    last_heartbeat_ms: int = 0
    abort_reason: str = ""
    completed_actions: int = 0


class MotionSequenceExecutor:
    """Thread-safe executor for validated open-loop action plans."""

    def __init__(
        self,
        *,
        heartbeat_timeout_ms: int = DEFAULT_HEARTBEAT_TIMEOUT_MS,
    ) -> None:
        self._heartbeat_timeout_ms = heartbeat_timeout_ms
        self._lock = threading.Lock()
        self._state = _RuntimeState()

    # ── public API ──────────────────────────────────────────────────

    def validate(self, raw_actions: Any) -> tuple[list[ValidatedAction], list[str]]:
        return validate_plan(raw_actions)

    def start(self, actions: Sequence[ValidatedAction], *, now_ms: int) -> tuple[bool, str]:
        """Begin executing ``actions``.  Returns (ok, message)."""

        with self._lock:
            if self._state.status == SequenceStatus.RUNNING:
                return False, "sequence already running"
            self._state = _RuntimeState(
                status=SequenceStatus.RUNNING,
                plan=list(actions),
                action_index=0,
                action_started_ms=now_ms,
                started_ms=now_ms,
                last_heartbeat_ms=now_ms,
            )
            return True, "started"

    def stop(self, reason: str = "operator_stop") -> None:
        with self._lock:
            if self._state.status != SequenceStatus.RUNNING:
                return
            self._state.status = SequenceStatus.ABORTED
            self._state.abort_reason = reason
            self._state.plan = None

    def complete(self) -> None:
        """Mark a still-running sequence as successfully completed.

        Used by the prototype hardware runner when the full plan finishes
        without abort.  Contract mode reaches COMPLETED via :meth:`advance`.
        """

        with self._lock:
            if self._state.status != SequenceStatus.RUNNING:
                return
            plan_len = len(self._state.plan) if self._state.plan else 0
            if plan_len:
                self._state.completed_actions = plan_len
                self._state.action_index = plan_len
            self._state.status = SequenceStatus.COMPLETED
            self._state.abort_reason = ""
            self._state.plan = None

    def heartbeat(self, *, now_ms: int) -> None:
        with self._lock:
            if self._state.status == SequenceStatus.RUNNING:
                self._state.last_heartbeat_ms = now_ms

    def advance(self, now_ms: int) -> None:
        """Advance the internal action index (called each control tick)."""

        with self._lock:
            self._advance_locked(now_ms)

    def motion_command(self, now_ms: int) -> Optional[SequenceMotionCommand]:
        """Return the motor command for the current action, if any."""

        with self._lock:
            return self._motion_command_locked(now_ms)

    def status(self) -> Dict[str, Any]:
        with self._lock:
            plan_len = len(self._state.plan) if self._state.plan else 0
            current = None
            if (
                self._state.status == SequenceStatus.RUNNING
                and self._state.plan
                and 0 <= self._state.action_index < plan_len
            ):
                current = plan_to_public_dict([self._state.plan[self._state.action_index]])[0]
            return {
                "status": self._state.status.value,
                "action_index": self._state.action_index,
                "action_count": plan_len,
                "completed_actions": self._state.completed_actions,
                "current_action": current,
                "abort_reason": self._state.abort_reason or None,
                "heartbeat_timeout_ms": self._heartbeat_timeout_ms,
            }

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._state.status == SequenceStatus.RUNNING

    # ── internals ───────────────────────────────────────────────────

    def _advance_locked(self, now_ms: int) -> None:
        state = self._state
        if state.status != SequenceStatus.RUNNING or not state.plan:
            return

        if (now_ms - state.last_heartbeat_ms) > self._heartbeat_timeout_ms:
            state.status = SequenceStatus.ABORTED
            state.abort_reason = "heartbeat_timeout"
            state.plan = None
            return

        while state.action_index < len(state.plan):
            elapsed_ms = now_ms - state.action_started_ms
            action = state.plan[state.action_index]
            duration_ms = int(action.duration_s * 1000)
            if isinstance(action, WaitAction):
                if elapsed_ms < duration_ms:
                    return
            elif isinstance(action, StopAction):
                if elapsed_ms < duration_ms:
                    return
            else:
                if elapsed_ms < duration_ms:
                    return

            state.completed_actions += 1
            state.action_index += 1
            state.action_started_ms = now_ms
            if state.action_index >= len(state.plan):
                state.status = SequenceStatus.COMPLETED
                state.plan = None
                return

    def _motion_command_locked(self, now_ms: int) -> Optional[SequenceMotionCommand]:
        state = self._state
        if state.status != SequenceStatus.RUNNING or not state.plan:
            return None
        if state.action_index >= len(state.plan):
            return None

        action = state.plan[state.action_index]
        if isinstance(action, WaitAction):
            return SequenceMotionCommand(speed=0.0, steering=0.0, brake=False)
        if isinstance(action, StopAction):
            return SequenceMotionCommand(speed=0.0, steering=0.0, brake=True)
        if isinstance(action, DriveAction):
            return SequenceMotionCommand(
                speed=action.normalized_speed(),
                steering=0.0,
                brake=False,
            )
        if isinstance(action, SteerAction):
            return SequenceMotionCommand(
                speed=action.normalized_speed(),
                steering=action.normalized_steering(),
                brake=False,
            )
        return None
