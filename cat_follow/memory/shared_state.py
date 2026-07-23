"""
SharedState: thread-safe wrapper around MemoryPool.

One lock per logical resource.  Every get/set operates on the
pre-allocated buffers from pool.py — no new arrays are ever created
inside the get/set methods.
"""

import threading
from typing import Tuple

import numpy as np

from cat_follow.memory.pool import MemoryPool, BBOX_LEN, ODOM_LEN
from cat_follow.memory.pool import FRAME_SHAPE


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

        # Legacy detector-model selection slot. Detection is now RKNN-only and
        # env-driven (CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH); this value is kept
        # only for backward-compatible web UI reads and is not used to pick a
        # model at runtime.
        self._lock_detector_model = threading.Lock()
        self._detector_model = "rknn"

        # Ring buffer indices for rotating frame buffers. The camera writes
        # into the slot returned by ``get_write_buffer()``, then calls
        # ``publish_latest_from_write()`` to atomically publish that slot
        # as the newest frame. Readers use ``get_frame_latest(dst)`` which
        # copies from the currently published index.
        self._ring_n = self._pool.frame_ring.shape[0]
        self._write_idx = 0
        self._latest_idx = -1

        # Monotonic generation counter for the detector snapshot frame.  Each
        # time a new frame is snapshotted for detection the generation is
        # bumped, and the detector tags the bbox it produces with that
        # generation (see ``set_bbox_detector``/``get_bbox_detector_with_gen``).
        # The tracker uses this to init/re-init on the *same* frame the detector
        # saw rather than a later ``frame_latest`` (avoids wrong-pixel init).
        self._detector_frame_gen = 0
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

    def set_frame_latest(self, src: np.ndarray) -> None:
        """Copy *src* into the next write slot and publish it as latest.

        This is a convenience method for tests or simple producers. The
        high-performance camera loop should use the ``get_write_buffer()``
        and ``publish_latest_from_write()`` pair to avoid an extra
        copy if the camera driver can write directly into the shared
        buffer.
        """
        # Copy into current write buffer, then publish under lock.
        write_buf = self.get_write_buffer()
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

    def copy_latest_to_detector_frame(self) -> None:
        """Copy ``frame_latest`` → ``frame_for_detector`` under lock.

        Called by the main thread every K frames so the detector has a
        stable snapshot to work with.
        """
        with self._lock_frame:
            if self._latest_idx < 0:
                # no frame yet
                self._pool.frame_for_detector.fill(0)
            else:
                np.copyto(self._pool.frame_for_detector, self._pool.frame_ring[self._latest_idx])

    def get_frame_for_detector(self, dst: np.ndarray) -> None:
        """Copy the current ``frame_for_detector`` into *dst* under lock."""
        # Validate dst shape and dtype to make misuse obvious.
        if dst.shape != FRAME_SHAPE:
            raise ValueError(f"dst has wrong shape {dst.shape}, expected {FRAME_SHAPE}")
        if dst.dtype != self._pool.frame_for_detector.dtype:
            raise ValueError(f"dst has wrong dtype {dst.dtype}, expected {self._pool.frame_for_detector.dtype}")

        with self._lock_frame:
            np.copyto(dst, self._pool.frame_for_detector)

    def snapshot_detector_frame(self, dst: np.ndarray) -> int:
        """Atomically copy ``frame_latest`` directly into *dst* and return the
        new detector-frame generation.

        This is one full-frame copy, rather than staging through
        ``frame_for_detector``. The legacy staging APIs remain available to
        callers that explicitly use them.
        """
        if dst.shape != FRAME_SHAPE:
            raise ValueError(f"dst has wrong shape {dst.shape}, expected {FRAME_SHAPE}")
        if dst.dtype != self._pool.frame_for_detector.dtype:
            raise ValueError(f"dst has wrong dtype {dst.dtype}, expected {self._pool.frame_for_detector.dtype}")

        with self._lock_frame:
            if self._latest_idx < 0:
                dst.fill(0)
            else:
                np.copyto(dst, self._pool.frame_ring[self._latest_idx])
            self._detector_frame_gen += 1
            gen = self._detector_frame_gen
        return gen

    def get_detector_frame_and_gen(self, dst: np.ndarray) -> int:
        """Copy the current ``frame_for_detector`` into *dst* and return its
        generation, under a single lock (for the tracker's frame-matched init).
        """
        if dst.shape != FRAME_SHAPE:
            raise ValueError(f"dst has wrong shape {dst.shape}, expected {FRAME_SHAPE}")
        if dst.dtype != self._pool.frame_for_detector.dtype:
            raise ValueError(f"dst has wrong dtype {dst.dtype}, expected {self._pool.frame_for_detector.dtype}")

        with self._lock_frame:
            np.copyto(dst, self._pool.frame_for_detector)
            return self._detector_frame_gen

    # ── ring helpers (camera use) ─────────────────────────────────────

    def get_write_buffer(self) -> np.ndarray:
        """Return a writable view into the pool's current write slot.

        The caller (camera thread) may write the frame data into this
        buffer (in-place). After writing, call
        ``publish_latest_from_write()`` to make the frame visible to
        readers.
        """
        # NOTE: This method intentionally does not acquire ``_lock_frame``.
        # It is safe only when a single writer (the camera thread) uses it.
        # Add a debug-time sanity check to catch accidental multi-writer use.
        buf = self._pool.frame_ring[self._write_idx]
        if __debug__:
            # shape/dtype guard
            assert buf.shape == FRAME_SHAPE, f"write buffer shape {buf.shape} != {FRAME_SHAPE}"
            assert buf.dtype == self._pool.frame_ring.dtype
        return buf

    def publish_latest_from_write(self) -> None:
        """Atomically publish the buffer at the current write index as
        the latest frame, then advance the write index.
        """
        with self._lock_frame:
            self._latest_idx = self._write_idx
            # advance write index for next frame
            self._write_idx = (self._write_idx + 1) % self._ring_n

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

        ``frame_gen`` is the detector-frame generation this bbox was produced
        from (see :meth:`snapshot_detector_frame`); it lets the tracker match
        the bbox to the exact frame the detector inferred on.
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
