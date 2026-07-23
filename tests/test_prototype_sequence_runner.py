"""Focused tests for prototype sequence runner safety/heartbeat behavior."""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

from cat_follow.motion.action_plan import DriveAction, WaitAction
from cat_follow.motion.prototype_sequence_runner import PrototypeSequenceRunner
from cat_follow.motion.sequence_executor import MotionSequenceExecutor, SequenceStatus


def _now_ms() -> int:
    return int(time.monotonic_ns() // 1_000_000)


class _FakePx:
    def __init__(self) -> None:
        self.events: list[tuple] = []
        self.lock = threading.Lock()

    def _record(self, *event) -> None:
        with self.lock:
            self.events.append(event)

    def forward(self, speed) -> None:
        self._record("forward", speed)

    def backward(self, speed) -> None:
        self._record("backward", speed)

    def stop(self) -> None:
        self._record("stop")

    def set_dir_servo_angle(self, angle) -> None:
        self._record("center" if angle == 0 else "steer", angle)


def test_prototype_runner_marks_completed_and_stops_hardware():
    px = _FakePx()
    executor = MotionSequenceExecutor(heartbeat_timeout_ms=5_000)
    runner = PrototypeSequenceRunner(
        picarx=px,
        hardware_lock=threading.Lock(),
        executor=executor,
    )
    ok, _ = runner.start(
        [WaitAction(duration_s=0.15)],
        now_ms=_now_ms(),
    )
    assert ok
    deadline = time.time() + 2.0
    while runner.is_running and time.time() < deadline:
        time.sleep(0.02)
    assert not runner.is_running
    status = executor.status()
    assert status["status"] == SequenceStatus.COMPLETED.value
    assert ("stop",) in px.events
    assert ("center", 0) in px.events


def test_prototype_runner_aborts_on_stale_heartbeat():
    px = _FakePx()
    executor = MotionSequenceExecutor(heartbeat_timeout_ms=80)
    runner = PrototypeSequenceRunner(
        picarx=px,
        hardware_lock=threading.Lock(),
        executor=executor,
    )
    ok, _ = runner.start(
        [DriveAction(direction="forward", speed_pct=10, duration_s=2.0)],
        now_ms=_now_ms(),
    )
    assert ok
    deadline = time.time() + 2.0
    while runner.is_running and time.time() < deadline:
        time.sleep(0.02)
    assert not runner.is_running
    status = executor.status()
    assert status["status"] == SequenceStatus.ABORTED.value
    assert status["abort_reason"] == "heartbeat_timeout"
    assert ("stop",) in px.events
    assert ("center", 0) in px.events


def test_prototype_runner_stops_hardware_on_exception():
    class _BoomPx(_FakePx):
        def forward(self, speed) -> None:
            self._record("forward", speed)
            raise RuntimeError("motor fault")

    px = _BoomPx()
    executor = MotionSequenceExecutor(heartbeat_timeout_ms=5_000)
    runner = PrototypeSequenceRunner(
        picarx=px,
        hardware_lock=threading.Lock(),
        executor=executor,
    )
    ok, _ = runner.start(
        [DriveAction(direction="forward", speed_pct=10, duration_s=1.0)],
        now_ms=_now_ms(),
    )
    assert ok
    deadline = time.time() + 2.0
    while runner.is_running and time.time() < deadline:
        time.sleep(0.02)
    status = executor.status()
    assert status["status"] == SequenceStatus.ABORTED.value
    assert status["abort_reason"] == "prototype_exception"
    assert ("stop",) in px.events
    assert ("center", 0) in px.events


def test_executor_complete_api():
    executor = MotionSequenceExecutor()
    executor.start([WaitAction(duration_s=1.0)], now_ms=0)
    executor.complete()
    status = executor.status()
    assert status["status"] == SequenceStatus.COMPLETED.value
    assert status["abort_reason"] is None
