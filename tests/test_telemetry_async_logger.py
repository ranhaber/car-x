"""Tests for the async telemetry logger."""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.control.types import (  # noqa: E402
    FsmState,
    TelemetryEventType,
    TelemetrySeverity,
)
from cat_follow.telemetry.async_logger import (  # noqa: E402
    AsyncLogger,
    CallableSink,
    default_jsonl_path,
)


class _CapturingSink:
    """Thread-safe sink that records batches and flush flags for inspection."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.events: list = []
        self.batches: list = []
        self.force_flush_count = 0
        self.closed = False

    def write_batch(self, events, force_flush: bool = False) -> None:
        with self._lock:
            batch = list(events)
            self.batches.append((batch, force_flush))
            self.events.extend(batch)
            if force_flush:
                self.force_flush_count += 1

    def close(self) -> None:
        with self._lock:
            self.closed = True

    def event_severities(self):
        with self._lock:
            return [e["severity"] for e in self.events]


def _make_logger(sink, **kwargs):
    defaults = dict(max_queue=8, flush_interval_s=0.05, flush_batch_size=4)
    defaults.update(kwargs)
    return AsyncLogger(sink=sink, **defaults)


def _wait_until(predicate, timeout=2.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def test_log_builds_event_envelope():
    captured = []
    sink = CallableSink(captured.append)
    logger = _make_logger(sink)
    logger.start()
    try:
        logger.log(
            event_type=TelemetryEventType.DECISION,
            severity=TelemetrySeverity.INFO,
            source="DecisionEngine",
            state=FsmState.CHASE_A,
            data={"speed": 0.5},
        )
        assert _wait_until(lambda: len(captured) == 1)
    finally:
        logger.stop()

    event = captured[0]
    assert event["schema_version"] == 1
    assert event["event_id"] == "evt-000001"
    assert event["event_type"] == "decision"
    assert event["severity"] == "info"
    assert event["state"] == "GETTING_CLOSE"
    assert event["source"] == "DecisionEngine"
    assert event["data"] == {"speed": 0.5}
    assert isinstance(event["timestamp_ms"], int)
    assert isinstance(event["monotonic_ms"], int)


def test_event_ids_are_sequential():
    captured = []
    sink = CallableSink(captured.append)
    logger = _make_logger(sink)
    logger.start()
    try:
        for _ in range(3):
            logger.log(
                event_type=TelemetryEventType.DECISION,
                severity=TelemetrySeverity.DEBUG,
                source="DecisionEngine",
            )
        assert _wait_until(lambda: len(captured) == 3)
    finally:
        logger.stop()

    assert [e["event_id"] for e in captured] == [
        "evt-000001",
        "evt-000002",
        "evt-000003",
    ]


def test_critical_event_triggers_force_flush():
    sink = _CapturingSink()
    logger = _make_logger(sink, flush_interval_s=10.0)  # long interval; only critical triggers wake
    logger.start()
    try:
        logger.log(
            event_type=TelemetryEventType.FAILSAFE,
            severity=TelemetrySeverity.CRITICAL,
            source="SafetySupervisor",
            data={"reason": "obstacle_too_close"},
        )
        assert _wait_until(lambda: sink.force_flush_count >= 1)
    finally:
        logger.stop()


def test_low_priority_events_dropped_when_full_with_critical_pending():
    sink = _CapturingSink()
    # Stop the writer from draining by giving it nothing to do until we say so.
    logger = AsyncLogger(sink=sink, max_queue=4, flush_interval_s=10.0, flush_batch_size=4)
    # Do not start the writer thread so the queue stays full deterministically.
    try:
        # Fill the queue with debug events.
        for _ in range(4):
            logger.log(
                event_type=TelemetryEventType.DECISION,
                severity=TelemetrySeverity.DEBUG,
                source="DecisionEngine",
            )

        assert logger.stats()["queued"] == 4

        # New debug event should be dropped (low priority + queue full).
        logger.log(
            event_type=TelemetryEventType.DECISION,
            severity=TelemetrySeverity.DEBUG,
            source="DecisionEngine",
        )
        stats = logger.stats()
        assert stats["queued"] == 4
        assert stats["dropped_low_priority"] == 1

        # Critical event should evict an existing debug and be enqueued.
        logger.log(
            event_type=TelemetryEventType.FAILSAFE,
            severity=TelemetrySeverity.CRITICAL,
            source="SafetySupervisor",
        )
        stats = logger.stats()
        assert stats["queued"] == 4
        assert stats["dropped_low_priority"] == 2

        # Drain by starting the writer briefly.
        logger.start()
        assert _wait_until(lambda: any(
            e["severity"] == "critical" for e in sink.events
        ))
    finally:
        logger.stop()


def test_default_jsonl_path_is_dated_jsonl_under_log_dir(tmp_path):
    path = default_jsonl_path(tmp_path)
    assert path.parent == tmp_path
    assert path.name.startswith("telemetry-")
    assert path.suffix == ".jsonl"


class _FlakySink:
    """Fails ``fail_times`` write_batch calls, then records events."""

    def __init__(self, fail_times: int) -> None:
        self._lock = threading.Lock()
        self._fail_times = fail_times
        self.events: list = []

    def write_batch(self, events, force_flush: bool = False) -> None:
        with self._lock:
            if self._fail_times > 0:
                self._fail_times -= 1
                raise RuntimeError("sink temporarily down")
            self.events.extend(list(events))

    def close(self) -> None:
        pass

    def severities(self):
        with self._lock:
            return [e["severity"] for e in self.events]


def test_critical_event_survives_transient_sink_failure():
    sink = _FlakySink(fail_times=2)
    logger = _make_logger(sink, flush_interval_s=0.02)
    logger.start()
    try:
        logger.log(
            event_type=TelemetryEventType.FAILSAFE,
            severity=TelemetrySeverity.CRITICAL,
            source="test",
        )
        # Despite two failed sink writes, the dequeued CRITICAL event must not
        # be lost -- it is re-buffered and retried until the sink recovers.
        assert _wait_until(
            lambda: "critical" in sink.severities(), timeout=2.0
        )
    finally:
        logger.stop()
    assert logger.stats()["sink_failures"] >= 2


def test_stop_drains_remaining_events():
    sink = _CapturingSink()
    logger = _make_logger(sink, flush_interval_s=10.0)
    logger.start()
    try:
        for _ in range(3):
            logger.log(
                event_type=TelemetryEventType.DECISION,
                severity=TelemetrySeverity.DEBUG,
                source="DecisionEngine",
            )
    finally:
        logger.stop()

    # All three events should be flushed by the final drain in stop().
    assert len(sink.events) == 3
    assert sink.closed is True
