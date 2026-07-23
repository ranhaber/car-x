"""Tests for the Movement tab sequence executor."""

from cat_follow.motion.action_plan import DriveAction, WaitAction
from cat_follow.motion.sequence_executor import MotionSequenceExecutor, SequenceStatus


def test_executor_runs_drive_action():
    executor = MotionSequenceExecutor(heartbeat_timeout_ms=1000)
    plan = [DriveAction(speed_pct=20, duration_s=0.2)]
    ok, _ = executor.start(plan, now_ms=0)
    assert ok

    cmd = executor.motion_command(now_ms=50)
    assert cmd is not None
    assert cmd.speed == 0.2
    assert cmd.brake is False

    executor.advance(now_ms=250)
    status = executor.status()
    assert status["status"] == SequenceStatus.COMPLETED.value


def test_executor_aborts_on_heartbeat_timeout():
    executor = MotionSequenceExecutor(heartbeat_timeout_ms=100)
    plan = [WaitAction(duration_s=5.0)]
    executor.start(plan, now_ms=0)
    executor.advance(now_ms=250)
    status = executor.status()
    assert status["status"] == SequenceStatus.ABORTED.value
    assert status["abort_reason"] == "heartbeat_timeout"


def test_executor_complete_is_idempotent():
    executor = MotionSequenceExecutor()
    plan = [WaitAction(duration_s=1.0)]
    executor.start(plan, now_ms=0)
    executor.complete()
    executor.complete()
    assert executor.status()["status"] == SequenceStatus.COMPLETED.value


def test_executor_stop_is_idempotent():
    executor = MotionSequenceExecutor()
    plan = [WaitAction(duration_s=1.0)]
    executor.start(plan, now_ms=0)
    executor.stop("test")
    executor.stop("test")
    assert executor.status()["status"] == SequenceStatus.ABORTED.value
