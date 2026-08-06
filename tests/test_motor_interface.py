"""Tests for MotorInterface and NoOpMotorBackend."""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.control.types import (  # noqa: E402
    DecisionOutput,
    FsmState,
    LookCommand,
    LookDriveMode,
    ReasonCode,
    TargetSource,
    TelemetryEventType,
    TelemetrySeverity,
)
from cat_follow.motion.motor_interface import (  # noqa: E402
    MotorInterface,
    NoOpMotorBackend,
)
from cat_follow.telemetry.async_logger import AsyncLogger, CallableSink  # noqa: E402


def _decision(
    *,
    speed=0.0,
    steering=0.0,
    brake=False,
    reason=ReasonCode.INIT,
    state=FsmState.IDLE,
    look=None,
):
    return DecisionOutput(
        timestamp_ms=1,
        requested_state=state,
        speed=speed,
        steering=steering,
        brake=brake,
        reason=reason,
        active_constraints=(),
        target_x=None,
        target_y=None,
        target_source=TargetSource.NONE,
        rejected_transition=False,
        look=look or LookCommand(),
    )


def test_no_op_backend_records_calls():
    backend = NoOpMotorBackend()
    iface = MotorInterface(backend=backend)
    iface.apply(_decision(speed=0.5, steering=-0.3))

    assert len(backend.applied) == 1
    cmd = backend.applied[0]
    assert cmd.speed == 0.5
    assert cmd.steering == -0.3
    assert cmd.brake is False


def test_apply_clamps_to_normalized_range():
    backend = NoOpMotorBackend()
    iface = MotorInterface(backend=backend)
    cmd = iface.apply(_decision(speed=2.5, steering=-9.0))

    assert cmd.speed == 1.0
    assert cmd.steering == -1.0
    assert backend.applied[0].speed == 1.0
    assert backend.applied[0].steering == -1.0


def test_logs_only_on_change_in_motor_command():
    captured = []
    sink = CallableSink(captured.append)
    logger = AsyncLogger(sink=sink, max_queue=64, flush_interval_s=0.05, flush_batch_size=8)
    logger.start()

    backend = NoOpMotorBackend()
    iface = MotorInterface(backend=backend, logger=logger)
    try:
        # First apply -> logged.
        iface.apply(_decision(speed=0.4, steering=0.1))
        # Same command -> NOT logged.
        iface.apply(_decision(speed=0.4, steering=0.1))
        # Changed -> logged.
        iface.apply(_decision(speed=0.5, steering=0.1))
        # Brake change -> logged.
        iface.apply(_decision(speed=0.0, steering=0.0, brake=True))

        # Allow the writer thread to drain.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and len(captured) < 3:
            time.sleep(0.02)
    finally:
        logger.stop()

    motor_events = [
        e
        for e in captured
        if e["event_type"] == TelemetryEventType.MOTOR_COMMAND.value
    ]
    assert len(motor_events) == 3
    # Backend still receives all four applies (backend is the source of truth).
    assert len(backend.applied) == 4


def test_brake_command_is_logged_at_info_severity():
    captured = []
    logger = AsyncLogger(sink=CallableSink(captured.append), flush_interval_s=0.05)
    logger.start()
    iface = MotorInterface(backend=NoOpMotorBackend(), logger=logger)
    try:
        iface.apply(_decision(speed=0.0, steering=0.0, brake=True))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not captured:
            time.sleep(0.02)
    finally:
        logger.stop()

    assert captured
    assert captured[0]["severity"] == TelemetrySeverity.INFO.value


def test_emergency_stop_logs_critical_and_resets_last_command():
    captured = []
    logger = AsyncLogger(sink=CallableSink(captured.append), flush_interval_s=0.05)
    logger.start()
    backend = NoOpMotorBackend()
    iface = MotorInterface(backend=backend, logger=logger)
    try:
        iface.apply(_decision(speed=0.5))
        iface.emergency_stop(reason="obstacle_too_close")
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and len(captured) < 2:
            time.sleep(0.02)
        # An apply equivalent to the post-emergency state should still log
        # because last_command was reset to (0,0,brake=True).
        iface.apply(_decision(speed=0.5))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and len(captured) < 3:
            time.sleep(0.02)
    finally:
        logger.stop()

    severities = [
        e["severity"]
        for e in captured
        if e["event_type"] == TelemetryEventType.MOTOR_COMMAND.value
    ]
    assert TelemetrySeverity.CRITICAL.value in severities
    assert backend.emergency_stops == 1


def test_motor_interface_is_safe_with_no_logger():
    backend = NoOpMotorBackend()
    iface = MotorInterface(backend=backend)
    # No logger passed -> applies and emergency_stop must not raise.
    iface.apply(_decision(speed=0.2))
    iface.emergency_stop()
    assert backend.emergency_stops == 1


def test_pan_only_change_is_logged():
    captured = []
    logger = AsyncLogger(
        sink=CallableSink(captured.append), flush_interval_s=0.05
    )
    logger.start()
    backend = NoOpMotorBackend()
    iface = MotorInterface(backend=backend, logger=logger)
    try:
        iface.apply(
            _decision(
                speed=0.2,
                look=LookCommand(mode=LookDriveMode.LOOK_AT, pan_deg=0.0),
            )
        )
        iface.apply(
            _decision(
                speed=0.2,
                look=LookCommand(mode=LookDriveMode.LOOK_AT, pan_deg=12.0),
            )
        )
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and len(captured) < 2:
            time.sleep(0.02)
    finally:
        logger.stop()

    motor_events = [
        e
        for e in captured
        if e["event_type"] == TelemetryEventType.MOTOR_COMMAND.value
    ]
    assert len(motor_events) >= 2
    look_only = [e for e in motor_events if e["data"].get("look_only")]
    assert look_only
    assert look_only[0]["data"]["pan_deg"] == 12.0
    assert backend.look_applied[-1] == 12.0


def test_emergency_stop_aligns_pan_cache_with_forward():
    backend = NoOpMotorBackend(pan_forward_deg=8.0)
    iface = MotorInterface(backend=backend, pan_forward_deg=8.0)
    iface.apply(
        _decision(
            speed=0.3,
            look=LookCommand(pan_deg=25.0, mode=LookDriveMode.LOOK_AT),
        )
    )
    assert backend.look_applied[-1] == 25.0
    iface.emergency_stop()
    assert backend.emergency_stops == 1
    assert backend.look_applied[-1] == 8.0
    # Dedup must not skip a re-apply of calibrated forward after e-stop.
    iface.apply(
        _decision(
            speed=0.0,
            brake=True,
            look=LookCommand(pan_deg=8.0, mode=LookDriveMode.HOLD),
        )
    )
    # Chassis changed (e-stop reset last_command), but pan equals cache — OK.
    assert iface._last_pan == 8.0
