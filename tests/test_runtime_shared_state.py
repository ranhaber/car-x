"""Tests for the contract-driven runtime SharedState."""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.control.types import (  # noqa: E402
    DecisionState,
    FSMSnapshot,
    FsmState,
    HomeState,
    OverheadState,
    RangeState,
    ReasonCode,
    SharedSnapshot,
    VisionState,
)
from cat_follow.runtime.shared_state import (  # noqa: E402
    SharedState,
    is_fresh,
    now_monotonic_ms,
)


def test_default_snapshot_is_safe():
    state = SharedState()
    snapshot = state.get_snapshot()

    assert isinstance(snapshot, SharedSnapshot)
    assert snapshot.fsm.state == FsmState.IDLE
    assert snapshot.home.set is False
    assert snapshot.overhead.car.confidence == 0.0
    assert snapshot.decision.speed == 0.0
    assert snapshot.decision.steering == 0.0


def test_update_replaces_entire_group_atomically():
    state = SharedState()
    new_home = HomeState(
        timestamp_ms=10,
        received_ms=20,
        set=True,
        x=100.0,
        y=200.0,
        source_command_id="cmd-1",
    )

    state.update_home(new_home)

    assert state.get_home() is new_home
    assert state.get_snapshot().home is new_home


def test_get_snapshot_returns_coherent_groups():
    state = SharedState()

    overhead = OverheadState(timestamp_ms=1, received_ms=2, fresh=True, sequence=42)
    home = HomeState(timestamp_ms=3, received_ms=4, set=True, x=1.0, y=2.0)
    vision = VisionState(timestamp_ms=5, received_ms=6, fresh=True, cat_visible=True)
    range_state = RangeState(timestamp_ms=7, received_ms=8, fresh=True, distance_cm=50.0)
    fsm = FSMSnapshot(state=FsmState.CHASE_A, last_transition_reason=ReasonCode.GLOBAL_CHASE)
    decision = DecisionState(
        requested_state=FsmState.CHASE_A,
        speed=0.4,
        steering=-0.1,
        reason=ReasonCode.GLOBAL_CHASE,
    )

    state.update_overhead(overhead)
    state.update_home(home)
    state.update_vision(vision)
    state.update_range(range_state)
    state.update_fsm(fsm)
    state.update_decision(decision)

    snapshot = state.get_snapshot()

    assert snapshot.overhead is overhead
    assert snapshot.home is home
    assert snapshot.vision is vision
    assert snapshot.range is range_state
    assert snapshot.fsm is fsm
    assert snapshot.decision is decision


def test_snapshot_is_stable_against_later_writes():
    state = SharedState()
    first = OverheadState(received_ms=1, sequence=1)
    state.update_overhead(first)
    snapshot = state.get_snapshot()

    second = OverheadState(received_ms=99, sequence=2)
    state.update_overhead(second)

    # Snapshot keeps the original reference even after the writer moved on.
    assert snapshot.overhead is first
    assert state.get_overhead() is second


def test_now_monotonic_ms_is_monotonic_non_negative():
    a = now_monotonic_ms()
    time.sleep(0.001)
    b = now_monotonic_ms()
    assert a >= 0
    assert b >= a


def test_is_fresh_uses_local_monotonic_window():
    now = 1000
    assert is_fresh(received_ms=950, max_age_ms=100, now_ms=now) is True
    assert is_fresh(received_ms=900, max_age_ms=100, now_ms=now) is True
    assert is_fresh(received_ms=899, max_age_ms=100, now_ms=now) is False


def test_concurrent_writers_and_snapshot_reader_do_not_crash():
    state = SharedState()
    stop = threading.Event()

    def write_overhead():
        i = 0
        while not stop.is_set():
            state.update_overhead(OverheadState(received_ms=i, sequence=i))
            i += 1

    def write_vision():
        i = 0
        while not stop.is_set():
            state.update_vision(
                VisionState(received_ms=i, cat_visible=bool(i % 2))
            )
            i += 1

    workers = [
        threading.Thread(target=write_overhead, daemon=True),
        threading.Thread(target=write_vision, daemon=True),
    ]
    for worker in workers:
        worker.start()

    try:
        for _ in range(200):
            snap = state.get_snapshot()
            assert isinstance(snap, SharedSnapshot)
    finally:
        stop.set()
        for worker in workers:
            worker.join(timeout=1.0)
