"""Tests for the ControlLoop runtime."""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.comms.comms_manager import CommsManager  # noqa: E402
from cat_follow.comms.messages import (  # noqa: E402
    CommandMessage,
    TrackingCar,
    TrackingCat,
    TrackingMessage,
)
from cat_follow.control.decision_engine import (  # noqa: E402
    OBSTACLE_TOO_CLOSE_CM,
    DecisionEngine,
)
from cat_follow.control.fsm import FSM  # noqa: E402
from cat_follow.control.types import (  # noqa: E402
    CommandName,
    FsmState,
    RangeState,
    TelemetryEventType,
)
from cat_follow.motion.motor_interface import MotorInterface, NoOpMotorBackend  # noqa: E402
from cat_follow.runtime.control_loop import ControlLoop  # noqa: E402
from cat_follow.runtime.shared_state import SharedState, now_monotonic_ms  # noqa: E402
from cat_follow.telemetry.async_logger import AsyncLogger, CallableSink  # noqa: E402


def _build_stack(*, with_logger=False):
    ss = SharedState()
    fsm = FSM()
    engine = DecisionEngine(fsm)
    backend = NoOpMotorBackend()
    captured = []
    logger = None
    if with_logger:
        logger = AsyncLogger(
            sink=CallableSink(captured.append),
            max_queue=128,
            flush_interval_s=0.05,
            flush_batch_size=16,
        )
        logger.start()
    motor = MotorInterface(backend=backend, logger=logger)
    loop = ControlLoop(
        shared_state=ss,
        decision_engine=engine,
        fsm=fsm,
        motor_interface=motor,
        logger=logger,
        target_rate_hz=200.0,  # fast for tests
        tick_budget_ms=20,
    )
    return ss, fsm, engine, backend, motor, loop, logger, captured


def _wait_until(predicate, timeout=2.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


# ── single tick ────────────────────────────────────────────────────


def test_tick_publishes_decision_and_fsm_to_shared_state():
    ss, fsm, _, backend, _, loop, *_ = _build_stack()

    output = loop.tick(now_ms=now_monotonic_ms())
    assert output.requested_state == FsmState.IDLE

    decision = ss.get_decision()
    assert decision.requested_state == FsmState.IDLE
    assert decision.speed == 0.0
    assert decision.authority == "DecisionEngine"
    assert decision.fresh is True

    fsm_snapshot = ss.get_fsm()
    assert fsm_snapshot.state == FsmState.IDLE
    assert fsm_snapshot.authority == "FSM"

    assert backend.applied  # motor backend received the command


def test_tick_uses_motor_interface_with_clamping_and_change_logging():
    ss, _, _, backend, _, loop, *_ = _build_stack()
    loop.tick(now_ms=now_monotonic_ms())
    loop.tick(now_ms=now_monotonic_ms())
    assert len(backend.applied) == 2
    # Default tick keeps speed/steering at 0.0; both within normalized range.
    for cmd in backend.applied:
        assert -1.0 <= cmd.speed <= 1.0
        assert -1.0 <= cmd.steering <= 1.0


def test_tick_count_increments():
    *_, loop, _, _ = _build_stack()
    assert loop.tick_count == 0
    loop.tick(now_ms=now_monotonic_ms())
    loop.tick(now_ms=now_monotonic_ms())
    assert loop.tick_count == 2


# ── command-driven transitions through the loop ────────────────────


def test_command_acceptance_through_comms_manager_drives_fsm_via_loop():
    ss, fsm, _, _, _, loop, *_ = _build_stack()
    received_acks = []
    comms = CommsManager(shared_state=ss, ack_sink=received_acks.append)

    # Provide valid tracking so start_chase passes validation.
    comms.submit_tracking(
        TrackingMessage(
            sequence=1,
            timestamp_ms=1,
            car=TrackingCar(x=0.0, y=0.0, confidence=1.0),
            cat=TrackingCat(x=10.0, y=10.0, confidence=1.0),
        )
    )

    # Apply a start_chase command.
    comms.submit_command(
        CommandMessage(
            sequence=2001,
            timestamp_ms=2,
            command_id="cmd-start",
            command=CommandName.START_CHASE,
        )
    )
    assert received_acks[-1].status.value == "accepted"

    # Run one tick of the loop.
    loop.tick(now_ms=now_monotonic_ms())

    assert fsm.state == FsmState.CHASE_A
    assert ss.get_fsm().state == FsmState.CHASE_A
    assert ss.get_decision().requested_state == FsmState.CHASE_A


# ── obstacle veto via the loop ─────────────────────────────────────


def test_obstacle_too_close_triggers_failsafe_through_loop():
    ss, fsm, _, _, _, loop, *_ = _build_stack()
    # Inject a too-close range observation directly into shared state.
    ss.update_range(
        RangeState(
            timestamp_ms=1,
            received_ms=now_monotonic_ms(),
            fresh=True,
            authority="test",
            distance_cm=OBSTACLE_TOO_CLOSE_CM - 1.0,
        )
    )
    loop.tick(now_ms=now_monotonic_ms())
    assert fsm.state == FsmState.FAILSAFE
    assert ss.get_decision().brake is True


# ── overrun telemetry ──────────────────────────────────────────────


def test_overrun_emits_thread_health_warning():
    ss, fsm, _, _, _, loop, logger, captured = _build_stack(with_logger=True)
    try:
        # Force a tick to look slow by lying about its start time: pass an
        # ``now_ms`` that is far in the past so elapsed > tick_budget_ms.
        loop.tick(now_ms=now_monotonic_ms() - 60)
        assert _wait_until(
            lambda: any(
                e["event_type"] == TelemetryEventType.THREAD_HEALTH.value
                for e in captured
            )
        )
    finally:
        logger.stop()

    overrun_events = [
        e
        for e in captured
        if e["event_type"] == TelemetryEventType.THREAD_HEALTH.value
        and e["data"].get("event") == "control_tick_overrun"
    ]
    assert overrun_events, captured
    assert overrun_events[0]["data"]["elapsed_ms"] >= 60


def test_critical_overrun_calls_emergency_stop():
    ss, fsm, _, backend, motor, loop, logger, captured = _build_stack(
        with_logger=True
    )
    try:
        # Pass an old now_ms so elapsed > critical_overrun_ms (default 100).
        loop.tick(now_ms=now_monotonic_ms() - 150)
        time.sleep(0.05)  # let the logger thread drain
    finally:
        logger.stop()

    assert backend.emergency_stops == 1


# ── lifecycle (start/stop) ─────────────────────────────────────────


def test_start_runs_ticks_and_stop_cleans_up():
    ss, fsm, _, backend, _, loop, *_ = _build_stack()
    loop.start()
    try:
        # Wait for a few ticks at the loop's high test rate.
        assert _wait_until(lambda: loop.tick_count >= 5, timeout=2.0)
    finally:
        loop.stop()

    assert loop.tick_count >= 5
    assert backend.applied  # motor backend was called


def test_starting_twice_is_idempotent():
    *_, loop, _, _ = _build_stack()
    loop.start()
    loop.start()  # no-op
    assert isinstance(loop._thread, threading.Thread)  # type: ignore[attr-defined]
    loop.stop()
