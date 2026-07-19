"""Runtime SharedState for the target control architecture.

Owns one instance of each contract dataclass group from
``cat_follow.control.types`` and exposes single-writer ``update_*`` methods,
per-group readers, and a coherent snapshot API.

Concurrency model
-----------------
- Each group has its own lock.
- Writers replace the entire group object atomically (the dataclasses are
  frozen, so consumers always observe a consistent group).
- Readers either fetch a single group or call ``get_snapshot()`` which
  captures each group reference under its own lock without ever holding
  multiple locks simultaneously.  Because group instances are immutable,
  the resulting snapshot is safe to read concurrently with later writes.

Freshness model
---------------
- ``received_ms`` on each group is a local monotonic timestamp (in ms).
- ``is_fresh`` and ``now_monotonic_ms`` provide the helpers that callers use
  to decide whether a group is still usable for safety/control.  Wall-clock
  time (``timestamp_ms``) is for cross-device log correlation only.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from cat_follow.control.types import (
    CommandState,
    DecisionState,
    FSMSnapshot,
    HomeState,
    NavigationState,
    OverheadState,
    RangeState,
    SharedSnapshot,
    SystemState,
    VisionState,
)


def now_monotonic_ms() -> int:
    """Return PiCar-X local monotonic time in milliseconds."""

    return int(time.monotonic_ns() // 1_000_000)


def is_fresh(received_ms: int, max_age_ms: int, now_ms: Optional[int] = None) -> bool:
    """Return True if a group sampled at ``received_ms`` is still fresh.

    Freshness is computed against PiCar-X local monotonic time, never against
    sender wall-clock ``timestamp_ms``.  This matches the safety/freshness
    rules in the Interface and Data Contract Specification.
    """

    if now_ms is None:
        now_ms = now_monotonic_ms()
    return (now_ms - received_ms) <= max_age_ms


class SharedState:
    """Thread-safe holder for the contract shared-state groups.

    Writers call the ``update_*`` method that owns each group.  Readers either
    call the matching ``get_*`` method or ``get_snapshot()`` for a coherent
    multi-group view used by ``DecisionEngine``.
    """

    def __init__(self) -> None:
        self._lock_overhead = threading.Lock()
        self._lock_home = threading.Lock()
        self._lock_vision = threading.Lock()
        self._lock_range = threading.Lock()
        self._lock_navigation = threading.Lock()
        self._lock_system = threading.Lock()
        self._lock_fsm = threading.Lock()
        self._lock_command = threading.Lock()
        self._lock_decision = threading.Lock()

        self._overhead: OverheadState = OverheadState()
        self._home: HomeState = HomeState()
        self._vision: VisionState = VisionState()
        # ``_range`` matches the contract group name ``range``.  It shadows
        # the Python builtin only inside this attribute scope.
        self._range: RangeState = RangeState()
        self._navigation: NavigationState = NavigationState()
        self._system: SystemState = SystemState()
        self._fsm: FSMSnapshot = FSMSnapshot()
        self._command: CommandState = CommandState()
        self._decision: DecisionState = DecisionState()

    # ── overhead (CommsManager) ─────────────────────────────────────

    def update_overhead(self, new: OverheadState) -> None:
        with self._lock_overhead:
            self._overhead = new

    def get_overhead(self) -> OverheadState:
        with self._lock_overhead:
            return self._overhead

    # ── home (CommsManager) ─────────────────────────────────────────

    def update_home(self, new: HomeState) -> None:
        with self._lock_home:
            self._home = new

    def get_home(self) -> HomeState:
        with self._lock_home:
            return self._home

    # ── vision (VisionTracker) ──────────────────────────────────────

    def update_vision(self, new: VisionState) -> None:
        with self._lock_vision:
            self._vision = new

    def get_vision(self) -> VisionState:
        with self._lock_vision:
            return self._vision

    # ── range (RangeSafety) ─────────────────────────────────────────

    def update_range(self, new: RangeState) -> None:
        with self._lock_range:
            self._range = new

    def get_range(self) -> RangeState:
        with self._lock_range:
            return self._range

    # ── navigation (Navigation) ─────────────────────────────────────

    def update_navigation(self, new: NavigationState) -> None:
        with self._lock_navigation:
            self._navigation = new

    def get_navigation(self) -> NavigationState:
        with self._lock_navigation:
            return self._navigation

    # ── system (Runtime) ────────────────────────────────────────────

    def update_system(self, new: SystemState) -> None:
        with self._lock_system:
            self._system = new

    def get_system(self) -> SystemState:
        with self._lock_system:
            return self._system

    # ── fsm (FSM) ───────────────────────────────────────────────────

    def update_fsm(self, new: FSMSnapshot) -> None:
        with self._lock_fsm:
            self._fsm = new

    def get_fsm(self) -> FSMSnapshot:
        with self._lock_fsm:
            return self._fsm

    # ── command (CommsManager) ──────────────────────────────────────

    def update_command(self, new: CommandState) -> None:
        with self._lock_command:
            self._command = new

    def get_command(self) -> CommandState:
        with self._lock_command:
            return self._command

    # ── decision (DecisionEngine) ───────────────────────────────────

    def update_decision(self, new: DecisionState) -> None:
        with self._lock_decision:
            self._decision = new

    def get_decision(self) -> DecisionState:
        with self._lock_decision:
            return self._decision

    # ── snapshot ────────────────────────────────────────────────────

    def get_snapshot(self) -> SharedSnapshot:
        """Return a coherent multi-group snapshot.

        Each group is captured under its own lock.  Locks are released between
        groups to honor the no-multiple-locks rule from the architecture
        specification.  Because each group is an immutable frozen dataclass,
        the captured references remain stable for the snapshot's lifetime even
        if writers replace the underlying group afterwards.
        """

        with self._lock_overhead:
            overhead = self._overhead
        with self._lock_home:
            home = self._home
        with self._lock_vision:
            vision = self._vision
        with self._lock_range:
            range_ = self._range
        with self._lock_navigation:
            navigation = self._navigation
        with self._lock_system:
            system = self._system
        with self._lock_fsm:
            fsm = self._fsm
        with self._lock_command:
            command = self._command
        with self._lock_decision:
            decision = self._decision

        return SharedSnapshot(
            overhead=overhead,
            home=home,
            vision=vision,
            range=range_,
            navigation=navigation,
            system=system,
            fsm=fsm,
            command=command,
            decision=decision,
        )
