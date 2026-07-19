"""Bounded async telemetry logger with priority-aware drop and batched flush.

Producers call :py:meth:`AsyncLogger.log` and never block.  A background
writer thread drains the queue in batches.  ``critical`` events trigger an
immediate flush so failsafe forensics survive crashes.

Drop policy
-----------
- If queue has spare capacity, append the event.
- If queue is full and the new event is ``debug`` or ``info``, drop the new
  event itself.
- If queue is full and the new event is ``warning``/``error``/``critical``,
  evict the lowest-rank event already in the queue and append the new one.
  If every queued event is at the same or higher rank, drop the new event.

Flush policy
------------
- Drain up to ``flush_batch_size`` events per writer wake-up.
- Wake up at most every ``flush_interval_s`` seconds, or sooner if a producer
  signals via the wake event (``critical`` events do this automatically).
- ``critical`` events in a batch cause the sink to perform a sync flush so
  the event is durable on disk before the next tick.
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Iterable, Optional, Union

from cat_follow.control.types import (
    FsmState,
    TelemetryEventType,
    TelemetrySeverity,
)
from cat_follow.runtime.shared_state import now_monotonic_ms


# Severity ranking used by the priority-aware drop policy.  Higher rank means
# more important; a higher-rank event may evict a lower-rank one when the
# queue is full.
_SEVERITY_RANK = {
    TelemetrySeverity.DEBUG: 0,
    TelemetrySeverity.INFO: 1,
    TelemetrySeverity.WARNING: 2,
    TelemetrySeverity.ERROR: 3,
    TelemetrySeverity.CRITICAL: 4,
}


def default_jsonl_path(log_dir: Union[str, Path] = "logs") -> Path:
    """Return today's default JSONL path: ``logs/telemetry-YYYYMMDD.jsonl``."""

    date = datetime.now(timezone.utc).strftime("%Y%m%d")
    return Path(log_dir) / f"telemetry-{date}.jsonl"


class CallableSink:
    """Forwards each event to a callable.  Useful for tests, UI, or stdout."""

    def __init__(self, fn: Callable[[dict], None]):
        self._fn = fn

    def write_batch(self, events: Iterable[dict], force_flush: bool = False) -> None:
        for event in events:
            self._fn(event)

    def close(self) -> None:
        pass


class JsonlFileSink:
    """Append-only JSONL sink with line buffering and optional fsync."""

    def __init__(self, path: Union[str, Path], fsync_on_flush: bool = False):
        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = self._path.open("a", buffering=1, encoding="utf-8")
        self._fsync_on_flush = fsync_on_flush

    def write_batch(self, events: Iterable[dict], force_flush: bool = False) -> None:
        for event in events:
            self._fp.write(json.dumps(event, separators=(",", ":")))
            self._fp.write("\n")
        if force_flush:
            self._fp.flush()
            if self._fsync_on_flush:
                os.fsync(self._fp.fileno())

    def close(self) -> None:
        try:
            self._fp.flush()
        finally:
            self._fp.close()


class AsyncLogger:
    """Bounded async logger with priority drop and batched flush."""

    def __init__(
        self,
        sink,
        max_queue: int = 1000,
        flush_interval_s: float = 1.0,
        flush_batch_size: int = 100,
    ) -> None:
        self._sink = sink
        self._max_queue = max_queue
        self._flush_interval_s = flush_interval_s
        self._flush_batch_size = flush_batch_size

        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._queue: deque = deque()
        self._next_event_id = 1
        self._dropped_low_priority = 0
        self._dropped_high_priority = 0
        self._writer: Optional[threading.Thread] = None

    # ── lifecycle ───────────────────────────────────────────────────

    def start(self) -> None:
        if self._writer is not None:
            return
        self._stop.clear()
        self._writer = threading.Thread(
            target=self._writer_loop,
            name="CatFollow-Log",
            daemon=True,
        )
        self._writer.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        self._wake.set()
        writer = self._writer
        self._writer = None
        if writer is not None:
            writer.join(timeout=timeout)
        # Final synchronous drain to flush anything left in the queue.
        self._drain_and_write(force_flush=True)
        try:
            self._sink.close()
        except Exception:
            pass

    # ── public log API ──────────────────────────────────────────────

    def log(
        self,
        *,
        event_type: TelemetryEventType,
        severity: TelemetrySeverity,
        source: str,
        state: Optional[FsmState] = None,
        data: Optional[dict] = None,
        timestamp_ms: Optional[int] = None,
    ) -> None:
        event = self._build_event(
            event_type=event_type,
            severity=severity,
            source=source,
            state=state,
            data=data,
            timestamp_ms=timestamp_ms,
        )
        self._enqueue(event, severity)
        if severity == TelemetrySeverity.CRITICAL:
            self._wake.set()

    def flush(self) -> None:
        """Wake the writer thread so it drains pending events promptly."""

        self._wake.set()

    def stats(self) -> dict:
        with self._lock:
            return {
                "queued": len(self._queue),
                "dropped_low_priority": self._dropped_low_priority,
                "dropped_high_priority": self._dropped_high_priority,
                "next_event_id": self._next_event_id,
            }

    # ── internals ───────────────────────────────────────────────────

    def _build_event(
        self,
        *,
        event_type: TelemetryEventType,
        severity: TelemetrySeverity,
        source: str,
        state: Optional[FsmState],
        data: Optional[dict],
        timestamp_ms: Optional[int],
    ) -> dict:
        with self._lock:
            event_id = f"evt-{self._next_event_id:06d}"
            self._next_event_id += 1

        if timestamp_ms is None:
            timestamp_ms = int(time.time() * 1000)

        return {
            "schema_version": 1,
            "event_id": event_id,
            "event_type": _enum_value(event_type),
            "timestamp_ms": timestamp_ms,
            "monotonic_ms": now_monotonic_ms(),
            "state": _enum_value(state) if state is not None else None,
            "source": source,
            "severity": _enum_value(severity),
            "data": data or {},
        }

    def _enqueue(self, event: dict, severity: TelemetrySeverity) -> None:
        with self._lock:
            if len(self._queue) < self._max_queue:
                self._queue.append(event)
                self._wake.set()
                return

            new_rank = _SEVERITY_RANK[severity]
            # debug / info: drop the new event itself
            if new_rank <= _SEVERITY_RANK[TelemetrySeverity.INFO]:
                self._dropped_low_priority += 1
                return

            # warning / error / critical: try to evict the lowest-rank event
            evict_idx = -1
            evict_rank = new_rank
            for idx, queued in enumerate(self._queue):
                queued_severity = TelemetrySeverity(queued["severity"])
                rank = _SEVERITY_RANK[queued_severity]
                if rank < evict_rank:
                    evict_idx = idx
                    evict_rank = rank
                    if rank == 0:
                        break

            if evict_idx >= 0:
                # Pop the chosen index by rotating it to the front and popping.
                self._queue.rotate(-evict_idx)
                self._queue.popleft()
                self._queue.rotate(evict_idx)
                self._queue.append(event)
                self._dropped_low_priority += 1
            else:
                self._dropped_high_priority += 1
            self._wake.set()

    def _writer_loop(self) -> None:
        while not self._stop.is_set():
            self._wake.wait(timeout=self._flush_interval_s)
            self._wake.clear()
            self._drain_and_write(force_flush=False)

    def _drain_and_write(self, *, force_flush: bool) -> None:
        events: list = []
        had_critical = False
        with self._lock:
            while self._queue and len(events) < self._flush_batch_size:
                event = self._queue.popleft()
                events.append(event)
                if event["severity"] == TelemetrySeverity.CRITICAL.value:
                    had_critical = True
        if not events:
            return
        try:
            self._sink.write_batch(
                events,
                force_flush=force_flush or had_critical,
            )
        except Exception:
            # Telemetry must never crash the control loop.  Sink errors are
            # swallowed; future work can surface them through a metrics
            # channel once thread_health telemetry is wired in.
            pass


def _enum_value(value) -> str:
    """Return the wire value of an enum, or stringify the value as fallback."""

    return value.value if hasattr(value, "value") else str(value)
