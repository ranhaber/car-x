"""
SharedState: thread-safe wrapper around MemoryPool.

One lock per logical resource.  Every get/set operates on the
pre-allocated buffers from pool.py — no new arrays are ever created
inside the get/set methods.
"""

import threading
import time
from typing import Optional, Tuple

import numpy as np

from cat_follow.memory.pool import MemoryPool, BBOX_LEN, ODOM_LEN
from cat_follow.memory.pool import FRAME_SHAPE


class FrameLease:
    """Pinned, read-only-by-contract view of a published frame-ring slot.

    The camera cannot reuse ``slot_idx`` until :meth:`release` is called.
    Consumers should use this as a context manager so exceptions cannot leak
    slot references.
    """

    __slots__ = (
        "_owner",
        "frame",
        "frame_seq",
        "capture_started_ns",
        "published_ns",
        "slot_generation",
        "slot_idx",
        "_released",
    )

    def __init__(
        self,
        owner: "SharedState",
        slot_idx: int,
        frame_seq: int,
        capture_started_ns: int,
        published_ns: int,
        slot_generation: int,
        frame: np.ndarray,
    ) -> None:
        self._owner = owner
        self.slot_idx = slot_idx
        self.frame_seq = frame_seq
        self.capture_started_ns = capture_started_ns
        self.published_ns = published_ns
        self.slot_generation = slot_generation
        self.frame = frame
        self._released = False

    def __enter__(self) -> "FrameLease":
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.release()

    @property
    def stale(self) -> bool:
        """Whether the slot generation changed while this lease was held."""
        return self._owner._frame_lease_stale(self.slot_idx, self.slot_generation)

    def release(self) -> None:
        """Release this reader's pin. Safe to call more than once."""
        if self._released:
            return
        self._owner._release_frame_slot(self.slot_idx)
        self._released = True


class SharedState:
    """Thread-safe accessor for every shared buffer.

    Parameters
    ----------
    pool : MemoryPool
        The pre-allocated buffer pool (from ``allocate_pool()``).
        SharedState does **not** allocate any new arrays.
    """

    def __init__(self, pool: MemoryPool) -> None:
        self._pool = pool

        # One lock per logical resource
        self._lock_frame = threading.Lock()
        self._lock_tracking = threading.Lock()
        self._lock_bbox_tracker = self._lock_tracking
        self._lock_bbox_detector = threading.Lock()
        self._lock_detector_detections = threading.Lock()
        self._lock_tracked_targets = self._lock_tracking
        self._lock_odometry = threading.Lock()
        self._lock_cat_injection = threading.Lock()

        # Legacy detector-model selection slot. Detection is now RKNN-only and
        # env-driven (CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH); this value is kept
        # only for backward-compatible web UI reads and is not used to pick a
        # model at runtime.
        self._lock_detector_model = threading.Lock()
        self._detector_model = "rknn"

        # Ring ownership. A single camera writer marks one slot as WRITING;
        # zero-copy readers pin published slots with a refcount. The writer
        # never reuses the latest slot or a pinned slot.
        self._ring_n = self._pool.frame_ring.shape[0]
        self._write_idx = 0
        self._latest_idx = -1
        self._active_write_idx: Optional[int] = None
        self._slot_refcounts = [0] * self._ring_n
        # Odd means write in progress; even means stable/published.
        self._slot_generations = [0] * self._ring_n
        self._slot_frame_seqs = [0] * self._ring_n
        self._slot_capture_started_ns = [0] * self._ring_n
        self._slot_published_ns = [0] * self._ring_n
        self._frame_seq = 0

        self._bbox_detector_gen = -1
        # Full post-NMS detection set consumed by PredictiveTracker. Tuples are
        # immutable snapshots, so readers never share mutable detector data.
        self._detector_detections = ()
        self._detector_detections_gen = -1
        # Role -> immutable target snapshot:
        # (track_id, x, y, w, h, confidence, frames_since_update, valid,
        #  detector_backed). The final field is optional for compatibility.
        # PRIMARY_CAT remains mirrored to bbox_tracker for behavior consumers.
        self._tracked_targets = {}
        # Monotonic generation for the tracker bbox, bumped on every publish.
        # The vision adapter keys stability/freshness off *changes* in this
        # generation so a frozen tracker (dead thread) ages out instead of the
        # adapter counting its own poll rate as "stable" frames.
        self._bbox_tracker_gen = 0

        # Optional hardware-scaled lores gray frame (motion source). Allocated
        # lazily on first publish so single-stream setups pay nothing.
        self._lock_lores = threading.Lock()
        self._lores_gray = None

        # Edge-triggered request from an accepted START_CHASE command. The
        # detector consumes it and loads/warmups the lazily-unloaded RKNN model
        # before the next detection.
        self._detector_warmup_requested = threading.Event()
        # Runtime-switchable live-frame test injection. The camera thread owns
        # the compositor; web/headless controls only update this flag.
        self._cat_injection_enabled = threading.Event()
        self._cat_injection_bbox = None

    # ── frame_latest ─────────────────────────────────────────────────

    def request_detector_warmup(self) -> None:
        """Request asynchronous RKNN load/warmup on the detector thread."""
        self._detector_warmup_requested.set()

    def consume_detector_warmup_request(self) -> bool:
        """Return and clear a pending detector warmup request."""
        if not self._detector_warmup_requested.is_set():
            return False
        self._detector_warmup_requested.clear()
        return True

    def set_cat_injection_enabled(self, enabled: bool) -> None:
        """Enable/disable live camera cat pixels (never synthetic detections)."""
        if enabled:
            self._cat_injection_enabled.set()
        else:
            self._cat_injection_enabled.clear()
            with self._lock_cat_injection:
                self._cat_injection_bbox = None

    def cat_injection_enabled(self) -> bool:
        return self._cat_injection_enabled.is_set()

    def set_cat_injection_bbox(self, bbox) -> None:
        """Publish the injected sprite's tight ``xyxy`` pixel bounds."""
        snapshot = None if bbox is None else tuple(int(value) for value in bbox)
        with self._lock_cat_injection:
            self._cat_injection_bbox = snapshot

    def get_cat_injection_status(self) -> dict:
        """Return injection state and ground-truth bbox for diagnostics only."""
        with self._lock_cat_injection:
            bbox = self._cat_injection_bbox
        return {
            "enabled": self._cat_injection_enabled.is_set(),
            "bbox": list(bbox) if bbox is not None else None,
            "detection_fallback": False,
        }

    def set_frame_latest(self, src: np.ndarray) -> None:
        """Copy *src* into the next write slot and publish it as latest.

        This is a convenience method for tests or simple producers. The
        high-performance camera loop should use the ``get_write_buffer()``
        and ``publish_latest_from_write()`` pair to avoid an extra
        copy if the camera driver can write directly into the shared
        buffer.
        """
        # Copy into current write buffer, then publish under lock.
        write_buf = self.try_get_write_buffer()
        if write_buf is None:
            return
        np.copyto(write_buf, src)
        self.publish_latest_from_write()

    def get_frame_latest(self, dst: np.ndarray) -> None:
        """Copy the currently published latest frame into *dst*.

        If no frame has been published yet, *dst* is zeroed.
        """
        # Basic shape checks to catch incorrect caller usage early.
        if dst.shape != FRAME_SHAPE:
            raise ValueError(f"dst has wrong shape {dst.shape}, expected {FRAME_SHAPE}")
        if dst.dtype != self._pool.frame_ring.dtype:
            raise ValueError(f"dst has wrong dtype {dst.dtype}, expected {self._pool.frame_ring.dtype}")

        with self._lock_frame:
            if self._latest_idx < 0:
                dst.fill(0)
            else:
                np.copyto(dst, self._pool.frame_ring[self._latest_idx])

    # ── ring helpers (camera use) ─────────────────────────────────────

    def try_get_write_buffer(self) -> Optional[np.ndarray]:
        """Reserve and return a free ring slot, or ``None`` if all are busy.

        This is non-blocking by design: camera capture is latest-wins and must
        drop a frame rather than wait behind detector or stream work.
        """
        with self._lock_frame:
            if self._active_write_idx is not None:
                raise RuntimeError("frame write already active")

            for offset in range(self._ring_n):
                idx = (self._write_idx + offset) % self._ring_n
                if idx == self._latest_idx or self._slot_refcounts[idx] != 0:
                    continue
                self._active_write_idx = idx
                self._slot_generations[idx] += 1  # even -> odd (WRITING)
                return self._pool.frame_ring[idx]
        return None

    def get_write_buffer(self) -> np.ndarray:
        """Return a writable view into the pool's current write slot.

        The caller (camera thread) may write the frame data into this
        buffer (in-place). After writing, call
        ``publish_latest_from_write()`` to make the frame visible to
        readers.
        """
        buf = self.try_get_write_buffer()
        if buf is None:
            raise RuntimeError("no free frame-ring write slot")
        if __debug__:
            assert buf.shape == FRAME_SHAPE, f"write buffer shape {buf.shape} != {FRAME_SHAPE}"
            assert buf.dtype == self._pool.frame_ring.dtype
        return buf

    def publish_latest_from_write(
        self, *, capture_started_ns: Optional[int] = None
    ) -> None:
        """Publish the active slot with capture timing and advance its sequence."""
        published_ns = time.monotonic_ns()
        if capture_started_ns is None:
            capture_started_ns = published_ns
        capture_started_ns = int(capture_started_ns)
        if capture_started_ns < 0 or capture_started_ns > published_ns:
            raise ValueError("capture_started_ns must be a past monotonic timestamp")
        with self._lock_frame:
            idx = self._active_write_idx
            if idx is None:
                raise RuntimeError("no active frame write to publish")
            if self._slot_generations[idx] % 2 != 1:
                raise RuntimeError("active frame slot is not marked WRITING")
            self._slot_generations[idx] += 1  # odd -> even (stable)
            self._frame_seq += 1
            self._slot_frame_seqs[idx] = self._frame_seq
            self._slot_capture_started_ns[idx] = capture_started_ns
            self._slot_published_ns[idx] = published_ns
            self._latest_idx = idx
            self._write_idx = (idx + 1) % self._ring_n
            self._active_write_idx = None

    def abort_frame_write(self) -> None:
        """Cancel an active write reservation without publishing its pixels."""
        with self._lock_frame:
            idx = self._active_write_idx
            if idx is None:
                return
            if self._slot_generations[idx] % 2 == 1:
                self._slot_generations[idx] += 1
            self._active_write_idx = None
            self._write_idx = (idx + 1) % self._ring_n

    def acquire_latest_frame(self) -> Optional[FrameLease]:
        """Pin and return the latest stable frame without copying pixels."""
        with self._lock_frame:
            idx = self._latest_idx
            if idx < 0:
                return None
            slot_generation = self._slot_generations[idx]
            if slot_generation % 2 != 0:
                return None
            self._slot_refcounts[idx] += 1
            frame_seq = self._slot_frame_seqs[idx]
            return FrameLease(
                self,
                idx,
                frame_seq,
                self._slot_capture_started_ns[idx],
                self._slot_published_ns[idx],
                slot_generation,
                self._pool.frame_ring[idx],
            )

    def latest_frame_generation(self) -> int:
        """Return the latest published capture sequence, or zero before startup."""
        with self._lock_frame:
            return self._frame_seq

    def _release_frame_slot(self, slot_idx: int) -> None:
        with self._lock_frame:
            if not 0 <= slot_idx < self._ring_n:
                raise ValueError(f"invalid frame-ring slot {slot_idx}")
            if self._slot_refcounts[slot_idx] <= 0:
                raise RuntimeError(f"frame-ring slot {slot_idx} is not acquired")
            self._slot_refcounts[slot_idx] -= 1

    def _frame_lease_stale(self, slot_idx: int, slot_generation: int) -> bool:
        with self._lock_frame:
            return self._slot_generations[slot_idx] != slot_generation

    # ── bbox_tracker ─────────────────────────────────────────────────

    def set_bbox_tracker(
        self,
        x: float,
        y: float,
        w: float,
        h: float,
        valid: float,
        *,
        refresh_generation: bool = True,
    ) -> None:
        """Write tracker bbox, optionally marking it as a fresh observation."""
        with self._lock_bbox_tracker:
            self._write_bbox_tracker(x, y, w, h, valid, refresh_generation)

    def _write_bbox_tracker(
        self,
        x: float,
        y: float,
        w: float,
        h: float,
        valid: float,
        refresh_generation: bool,
    ) -> None:
        buf = self._pool.bbox_tracker
        buf[0] = x
        buf[1] = y
        buf[2] = w
        buf[3] = h
        buf[4] = valid
        if refresh_generation:
            self._bbox_tracker_gen += 1

    def get_bbox_tracker(self) -> Tuple[float, float, float, float, float]:
        """Return a snapshot ``(x, y, w, h, valid)`` under lock.

        Returns a plain tuple (cheap, immutable) so the caller holds a
        consistent copy that won't change once the lock is released.
        """
        with self._lock_bbox_tracker:
            buf = self._pool.bbox_tracker
            return (float(buf[0]), float(buf[1]), float(buf[2]),
                    float(buf[3]), float(buf[4]))

    def get_bbox_tracker_with_gen(
        self,
    ) -> Tuple[float, float, float, float, float, int]:
        """Return ``(x, y, w, h, valid, gen)`` under lock.

        ``gen`` increments on every publish, so a consumer can tell a genuinely
        new tracker observation from a repeated poll of an unchanged buffer.
        """
        with self._lock_bbox_tracker:
            buf = self._pool.bbox_tracker
            return (float(buf[0]), float(buf[1]), float(buf[2]),
                    float(buf[3]), float(buf[4]), int(self._bbox_tracker_gen))

    # ── bbox_detector ────────────────────────────────────────────────

    def set_bbox_detector(
        self,
        x: float,
        y: float,
        w: float,
        h: float,
        valid: float,
        frame_gen: int = -1,
    ) -> None:
        """Write detector bbox into the pre-allocated array under lock.

        ``frame_gen`` is the capture sequence from the detector's frame lease;
        it lets the tracker match the bbox to the exact inferred pixels.
        """
        with self._lock_bbox_detector:
            buf = self._pool.bbox_detector
            buf[0] = x
            buf[1] = y
            buf[2] = w
            buf[3] = h
            buf[4] = valid
            self._bbox_detector_gen = int(frame_gen)

    def get_bbox_detector(self) -> Tuple[float, float, float, float, float]:
        """Return a snapshot ``(x, y, w, h, valid)`` under lock."""
        with self._lock_bbox_detector:
            buf = self._pool.bbox_detector
            return (float(buf[0]), float(buf[1]), float(buf[2]),
                    float(buf[3]), float(buf[4]))

    def get_bbox_detector_with_gen(
        self,
    ) -> Tuple[float, float, float, float, float, int]:
        """Return ``(x, y, w, h, valid, frame_gen)`` under lock."""
        with self._lock_bbox_detector:
            buf = self._pool.bbox_detector
            return (float(buf[0]), float(buf[1]), float(buf[2]),
                    float(buf[3]), float(buf[4]), int(self._bbox_detector_gen))

    def set_detector_detections(self, detections, frame_gen: int) -> None:
        """Publish all ``(x1,y1,x2,y2,confidence,class_id)`` detections."""
        snapshot = tuple(
            (
                float(det[0]),
                float(det[1]),
                float(det[2]),
                float(det[3]),
                float(det[4]),
                int(det[5]),
            )
            for det in detections
        )
        with self._lock_detector_detections:
            self._detector_detections = snapshot
            self._detector_detections_gen = int(frame_gen)

    def get_detector_detections_with_gen(self):
        """Return the immutable full detection snapshot and its frame generation."""
        with self._lock_detector_detections:
            return self._detector_detections, self._detector_detections_gen

    def set_tracked_targets(self, targets) -> None:
        """Publish immutable PRIMARY_CAT/SECONDARY_CAT tracking snapshots."""
        snapshot = self._tracked_targets_snapshot(targets)
        with self._lock_tracked_targets:
            self._tracked_targets = snapshot

    @staticmethod
    def _tracked_targets_snapshot(targets) -> dict:
        snapshot = {}
        for role, target in targets.items():
            values = (
                int(target[0]),
                float(target[1]),
                float(target[2]),
                float(target[3]),
                float(target[4]),
                float(target[5]),
                int(target[6]),
                float(target[7]),
            )
            if len(target) > 8:
                values += (float(target[8]),)
            snapshot[str(role)] = values
        return snapshot

    def publish_tracking_snapshot(
        self,
        targets,
        bbox: Tuple[float, float, float, float, float],
        *,
        detector_backed: bool,
    ) -> None:
        """Atomically publish role targets and the legacy primary bbox.

        Prediction-only/coasting publications update display coordinates but
        do not advance the observation generation consumed by VisionAdapter.
        """
        snapshot = self._tracked_targets_snapshot(targets)
        with self._lock_tracking:
            self._tracked_targets = snapshot
            self._write_bbox_tracker(*bbox, detector_backed)

    def get_tracking_snapshot(self):
        """Return role targets and generation-aware primary bbox atomically."""
        with self._lock_tracking:
            buf = self._pool.bbox_tracker
            bbox = (
                float(buf[0]),
                float(buf[1]),
                float(buf[2]),
                float(buf[3]),
                float(buf[4]),
                int(self._bbox_tracker_gen),
            )
            return dict(self._tracked_targets), bbox

    def get_tracked_targets(self) -> dict:
        """Return a detached role-to-target snapshot."""
        with self._lock_tracked_targets:
            return dict(self._tracked_targets)

    # ── odometry ─────────────────────────────────────────────────────

    def set_odometry(self, x: float, y: float, heading_deg: float) -> None:
        """Write odometry into the pre-allocated array under lock."""
        with self._lock_odometry:
            buf = self._pool.odometry
            buf[0] = x
            buf[1] = y
            buf[2] = heading_deg

    def get_odometry(self) -> Tuple[float, float, float]:
        """Return a snapshot ``(x, y, heading_deg)`` under lock."""
        with self._lock_odometry:
            buf = self._pool.odometry
            return (float(buf[0]), float(buf[1]), float(buf[2]))

    # ── lores gray frame (optional hardware-scaled motion source) ─────

    def set_lores_gray(self, gray: np.ndarray) -> None:
        """Publish a single-channel lores gray frame for motion detection.

        The backing buffer is allocated on first use (and reallocated if the
        lores geometry changes), then reused in-place for every later frame.
        """
        with self._lock_lores:
            if self._lores_gray is None or self._lores_gray.shape != gray.shape:
                self._lores_gray = np.empty(gray.shape, dtype=np.uint8)
            np.copyto(self._lores_gray, gray)

    def get_lores_gray(
        self, dst: "np.ndarray | None" = None
    ) -> "np.ndarray | None":
        """Copy lores gray into reusable *dst*, allocating only when needed."""
        with self._lock_lores:
            if self._lores_gray is None:
                return None
            if (
                dst is None
                or dst.shape != self._lores_gray.shape
                or dst.dtype != self._lores_gray.dtype
            ):
                dst = np.empty_like(self._lores_gray)
            np.copyto(dst, self._lores_gray)
            return dst

    def has_lores_gray(self) -> bool:
        """Return True if a lores gray frame has been published."""
        with self._lock_lores:
            return self._lores_gray is not None

    # ── detector model selection ──────────────────────────────────────
    def set_detector_model(self, model_key: str) -> None:
        """Set the active detector model key (currently ``rknn``)."""
        with self._lock_detector_model:
            self._detector_model = str(model_key)

    def get_detector_model(self) -> str:
        """Return the currently-selected detector model key."""
        with self._lock_detector_model:
            return str(self._detector_model)
