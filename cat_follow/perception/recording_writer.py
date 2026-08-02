"""Recording writer: demand → encoder → segmented store."""

from __future__ import annotations

import os
import threading
import time
from typing import Optional, Protocol

from cat_follow.control.types import (
    PerceptionLifecycleState,
    RecordingRuntimeState,
)
from cat_follow.logger import get_logger
from cat_follow.perception.recording_store import RecordingStore
from cat_follow.runtime.shared_state import now_monotonic_ms

log = get_logger("perception.recording")


class RecordingEncoderUnavailable(RuntimeError):
    """No encoder can produce real video for this deployment."""


class AccessUnitEncoder(Protocol):
    """Minimal encoder interface used by the recording writer.

    A container-producing encoder may additionally implement
    ``begin_segment() -> bool`` and ``end_segment() -> list[bytes]``. The
    writer calls them around every segment so each file carries its own
    headers and trailing bytes; without them a rotated segment would not be
    independently playable.
    """

    def start(self) -> bool: ...

    def stop(self) -> None: ...

    def encode_tick(self, *, now_ms: int) -> list[bytes]: ...


class StubH264Encoder:
    """Host CI encoder that emits deterministic fake Annex-B-like AUs.

    Never selected implicitly: production must either get a real hardware
    encoder or report recording as unavailable, because fake bytes would look
    like a healthy chase recording while producing unplayable files.
    """

    def __init__(self, *, au_size: int = 64) -> None:
        self._au_size = max(16, int(au_size))
        self._started = False
        self._seq = 0

    @staticmethod
    def available() -> bool:
        return True

    def start(self) -> bool:
        self._started = True
        self._seq = 0
        return True

    def stop(self) -> None:
        self._started = False

    def encode_tick(self, *, now_ms: int) -> list[bytes]:
        if not self._started:
            return []
        self._seq += 1
        # Fake start-code + payload for muxer-agnostic host tests.
        return [
            b"\x00\x00\x00\x01"
            + self._seq.to_bytes(4, "big")
            + now_ms.to_bytes(8, "big")
            + bytes(self._au_size - 16)
        ]


class RecordingWriter:
    """Turn lifecycle recording demand into store segments.

    Failures set ``degraded_reason`` and never raise into the control loop.

    Encode and disk work runs on a dedicated thread once :meth:`start` is
    called, so the control loop never waits on the encoder or the SD card
    between reading sensors and applying motor commands. Without that thread
    (tests and simple tools) ``tick`` performs the same work inline.
    """

    def __init__(
        self,
        store: RecordingStore,
        *,
        encoder: Optional[AccessUnitEncoder] = None,
        segment_duration_ms: int = 60_000,
        retention_interval_ms: int = 5_000,
    ) -> None:
        self.store = store
        # No implicit stub: an absent encoder means recording is unavailable.
        self.encoder = encoder
        self.segment_duration_ms = max(1_000, int(segment_duration_ms))
        self.retention_interval_ms = max(0, int(retention_interval_ms))
        # ``_io_lock`` serializes ticks (encoder + disk); ``_state_lock`` only
        # guards the published snapshot, so a slow filesystem can never block
        # a status read on the control/HTTP path.
        self._io_lock = threading.RLock()
        self._state_lock = threading.Lock()
        self._active = False
        self._segment_path: Optional[str] = None
        self._segment_started_ms: Optional[int] = None
        self._degraded_reason: Optional[str] = None
        self._bytes_written = 0
        self._segments_finalized = 0
        self._encoder_started = False
        self._last_retention_ms: Optional[int] = None
        self._snapshot = RecordingRuntimeState()
        # Latest-wins request handoff to the I/O worker.
        self._worker: Optional[threading.Thread] = None
        self._worker_stop = threading.Event()
        self._request_lock = threading.Lock()
        self._request: Optional[tuple[bool, int]] = None
        self._request_ready = threading.Event()

    def start(self) -> None:
        """Move encode and disk I/O onto a dedicated thread."""
        with self._request_lock:
            if self._worker is not None:
                return
            self._worker_stop.clear()
            self._worker = threading.Thread(
                target=self._worker_loop,
                name="CatFollow-Recording",
                daemon=True,
            )
            self._worker.start()

    def stop(self, *, timeout: float = 2.0) -> None:
        """Stop the I/O worker and finalize any in-progress segment."""
        with self._request_lock:
            worker = self._worker
            self._worker = None
        self._worker_stop.set()
        self._request_ready.set()
        if worker is not None:
            worker.join(timeout=timeout)
        with self._io_lock:
            self._stop(finalize=True)
            self._publish_snapshot()

    def _worker_loop(self) -> None:
        while not self._worker_stop.is_set():
            if not self._request_ready.wait(0.1):
                continue
            with self._request_lock:
                request = self._request
                self._request = None
                self._request_ready.clear()
            if request is None:
                continue
            requested, now_ms = request
            try:
                self._apply_tick(requested=requested, now_ms=now_ms)
            except Exception as exc:  # noqa: BLE001
                # _apply_tick already degrades on expected failures; anything
                # left here must not kill the worker and silently stop
                # reporting recording state.
                log.error("Recording writer tick failed: %s", exc)

    def runtime_state(self) -> RecordingRuntimeState:
        with self._state_lock:
            return self._snapshot

    def _publish_snapshot(self) -> RecordingRuntimeState:
        """Recompute the snapshot from writer state. Call under ``_io_lock``."""
        snapshot = RecordingRuntimeState(
            active=self._active and self._degraded_reason is None,
            segment_path=(
                self._segment_path[: -len(".part")]
                if self._segment_path and self._segment_path.endswith(".part")
                else self._segment_path
            ),
            degraded_reason=self._degraded_reason,
            bytes_written=self._bytes_written,
            segments_finalized=self._segments_finalized,
        )
        with self._state_lock:
            self._snapshot = snapshot
        return snapshot

    def tick(
        self,
        lifecycle: PerceptionLifecycleState,
        *,
        now_ms: Optional[int] = None,
    ) -> RecordingRuntimeState:
        now = now_ms if now_ms is not None else now_monotonic_ms()
        requested = bool(lifecycle.recording.requested)
        worker = self._worker
        if worker is None:
            return self._apply_tick(requested=requested, now_ms=now)
        if not worker.is_alive():
            # A dead worker would leave recording state frozen while the
            # lifecycle keeps the camera powered for it.
            with self._io_lock:
                self._degraded_reason = "writer_stopped"
                return self._publish_snapshot()
        with self._request_lock:
            self._request = (requested, now)
            self._request_ready.set()
        return self.runtime_state()

    def _apply_tick(
        self, *, requested: bool, now_ms: int
    ) -> RecordingRuntimeState:
        now = now_ms
        with self._io_lock:
            if not requested:
                self._stop(finalize=True)
                return self._publish_snapshot()

            if self._degraded_reason == "low_space":
                if self.store.space_available():
                    self._degraded_reason = None
                else:
                    return self._publish_snapshot()

            try:
                if not self._active:
                    self._start(now_ms=now)
                self._write_tick(now_ms=now)
                if self._should_rotate(now_ms=now):
                    self._rotate(now_ms=now)
                else:
                    self._enforce_retention_throttled(now_ms=now)
            except RecordingEncoderUnavailable:
                # Fail closed: report unavailable so the lifecycle stops
                # holding the camera for a recorder that cannot produce video.
                self._degraded_reason = "encoder_unavailable"
                self._stop(finalize=False)
            except OSError as exc:
                self._degraded_reason = (
                    "low_space"
                    if "reserve" in str(exc).lower() or "space" in str(exc).lower()
                    else "io_error"
                )
                self._stop(finalize=False)
            except Exception:  # noqa: BLE001
                self._degraded_reason = "encoder_error"
                self._stop(finalize=False)
            return self._publish_snapshot()

    def _start(self, *, now_ms: int) -> None:
        if self.encoder is None:
            raise RecordingEncoderUnavailable(
                "no recording encoder is available on this host"
            )
        if not self.store.space_available():
            raise OSError("recording storage reserve exhausted")
        if not self._encoder_started:
            if not self.encoder.start():
                raise RuntimeError("recording encoder failed to start")
            self._encoder_started = True
        begin_segment = getattr(self.encoder, "begin_segment", None)
        if begin_segment is not None and not begin_segment():
            raise RuntimeError("recording encoder failed to open a segment")
        # Segment names are operator-facing, so they use wall-clock time while
        # rotation/aging stays on the caller's monotonic ``now_ms``.
        path = self.store.begin_segment(wall_clock_ms=int(time.time() * 1000))
        self._segment_path = path
        self._segment_started_ms = now_ms
        self._active = True
        self._degraded_reason = None

    def _close_encoder_segment(self) -> None:
        """Append the bytes that make the current segment a complete file."""
        end_segment = getattr(self.encoder, "end_segment", None)
        if end_segment is None or self._segment_path is None:
            return
        for au in end_segment():
            self._bytes_written = self.store.append_bytes(
                self._segment_path, au
            )

    def _write_tick(self, *, now_ms: int) -> None:
        if self._segment_path is None or self.encoder is None:
            return
        # Re-check the reserve every tick: a long segment keeps appending long
        # after the start-of-segment check, and would otherwise fill the disk.
        if not self.store.space_available():
            raise OSError("recording storage reserve exhausted")
        for au in self.encoder.encode_tick(now_ms=now_ms):
            self._bytes_written = self.store.append_bytes(
                self._segment_path, au
            )

    def _should_rotate(self, *, now_ms: int) -> bool:
        if self._segment_started_ms is None:
            return False
        if now_ms - self._segment_started_ms >= self.segment_duration_ms:
            return True
        # Retention can only delete finalized segments, so an active segment
        # that alone breaches the quota must be closed early to be reclaimable.
        return self.store.active_segment_over_quota()

    def _enforce_retention_throttled(self, *, now_ms: int) -> None:
        """Prune finalized segments while the active one grows into the quota."""
        if self.store.quota_bytes is None:
            return
        last = self._last_retention_ms
        throttled = (
            last is not None and now_ms - last < self.retention_interval_ms
        )
        # Being over quota already is disk pressure, so it outranks the throttle
        # that only exists to keep steady-state index rewrites off every tick.
        if throttled and not self.store.over_quota():
            return
        self._last_retention_ms = now_ms
        self.store.enforce_retention()

    def _rotate(self, *, now_ms: int) -> None:
        if self._segment_path is None:
            return
        self._close_encoder_segment()
        self.store.finalize_segment(self._segment_path)
        self._segments_finalized += 1
        self._segment_path = None
        self._active = False
        self._start(now_ms=now_ms)

    def _stop(self, *, finalize: bool) -> None:
        if self._segment_path is not None:
            try:
                self._close_encoder_segment()
            except Exception as exc:  # noqa: BLE001
                # The segment stays on disk; it just loses its trailing bytes.
                log.warning("Recording segment could not be closed: %s", exc)
        path = self._segment_path
        self._segment_path = None
        self._segment_started_ms = None
        self._active = False
        if path is not None and finalize and os_path_exists(path):
            try:
                self.store.finalize_segment(path)
                self._segments_finalized += 1
            except Exception:  # noqa: BLE001
                # The ``.part`` stays on disk for startup recovery, but the
                # operator must see that this segment was never closed.
                self._degraded_reason = "finalize_error"
        if self._encoder_started and not finalize:
            # Keep encoder warm during transient degradation for fast resume.
            return
        if self._encoder_started and finalize and self.encoder is not None:
            try:
                self.encoder.stop()
            except Exception:  # noqa: BLE001
                pass
            self._encoder_started = False


def os_path_exists(path: str) -> bool:
    return os.path.exists(path)
