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
        self._lock_bbox_tracker = threading.Lock()
        self._lock_bbox_detector = threading.Lock()
        self._lock_odometry = threading.Lock()

        # Detector model selection (string key). Web UI toggles this value
        # and the detector thread reads it to decide which .tflite to load.
        self._lock_detector_model = threading.Lock()
        # Default to SSD MobileNet V2 (key used by web UI and detector mapping)
        self._detector_model = "ssd_mobilenet_v2"

        # Ring buffer indices for rotating frame buffers. The camera writes
        # into the slot returned by ``get_write_buffer()``, then calls
        # ``publish_latest_from_write()`` to atomically publish that slot
        # as the newest frame. Readers use ``get_frame_latest(dst)`` which
        # copies from the currently published index.
        self._ring_n = self._pool.frame_ring.shape[0]
        self._write_idx = 0
        self._latest_idx = -1

    # ── frame_latest ─────────────────────────────────────────────────

    def set_frame_latest(self, src: np.ndarray) -> None:
        """Copy *src* into the next write slot and publish it as latest.

        This convenience method keeps backward compatibility: it copies
        into the current write buffer and then publishes that buffer as
        the latest frame (rotating the write index).
        """
        # Copy into current write buffer, then publish under lock.
        write_buf = self.get_write_buffer()
        np.copyto(write_buf, src)
        self.publish_latest_from_write()

    def get_frame_latest(self, dst: np.ndarray) -> None:
        """Copy the currently published latest frame into *dst*.

        If no frame has been published yet, *dst* is zeroed.
        """
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
        with self._lock_frame:
            np.copyto(dst, self._pool.frame_for_detector)

    # ── ring helpers (camera use) ─────────────────────────────────────

    def get_write_buffer(self) -> np.ndarray:
        """Return a writable view into the pool's current write slot.

        The caller (camera thread) may write the frame data into this
        buffer (in-place). After writing, call
        ``publish_latest_from_write()`` to make the frame visible to
        readers.
        """
        return self._pool.frame_ring[self._write_idx]

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
        self, x: float, y: float, w: float, h: float, valid: float
    ) -> None:
        """Write tracker bbox into the pre-allocated array under lock."""
        with self._lock_bbox_tracker:
            buf = self._pool.bbox_tracker
            buf[0] = x
            buf[1] = y
            buf[2] = w
            buf[3] = h
            buf[4] = valid

    def get_bbox_tracker(self) -> Tuple[float, float, float, float, float]:
        """Return a snapshot ``(x, y, w, h, valid)`` under lock.

        Returns a plain tuple (cheap, immutable) so the caller holds a
        consistent copy that won't change once the lock is released.
        """
        with self._lock_bbox_tracker:
            buf = self._pool.bbox_tracker
            return (float(buf[0]), float(buf[1]), float(buf[2]),
                    float(buf[3]), float(buf[4]))

    # ── bbox_detector ────────────────────────────────────────────────

    def set_bbox_detector(
        self, x: float, y: float, w: float, h: float, valid: float
    ) -> None:
        """Write detector bbox into the pre-allocated array under lock."""
        with self._lock_bbox_detector:
            buf = self._pool.bbox_detector
            buf[0] = x
            buf[1] = y
            buf[2] = w
            buf[3] = h
            buf[4] = valid

    def get_bbox_detector(self) -> Tuple[float, float, float, float, float]:
        """Return a snapshot ``(x, y, w, h, valid)`` under lock."""
        with self._lock_bbox_detector:
            buf = self._pool.bbox_detector
            return (float(buf[0]), float(buf[1]), float(buf[2]),
                    float(buf[3]), float(buf[4]))

    # ── odometry ─────────────────────────────────────────────────────

    def set_odometry(self, x: float, y: float, heading_deg: float) -> None:
        """Write odometry into the pre-allocated array under lock."""
        with self._lock_odometry:
            buf = self._pool.odometry_xyh
            buf[0] = x
            buf[1] = y
            buf[2] = heading_deg

    def get_odometry(self) -> Tuple[float, float, float]:
        """Return a snapshot ``(x, y, heading_deg)`` under lock."""
        with self._lock_odometry:
            buf = self._pool.odometry_xyh
            return (float(buf[0]), float(buf[1]), float(buf[2]))

    # ── detector model selection ──────────────────────────────────────
    def set_detector_model(self, model_key: str) -> None:
        """Set the active detector model key (e.g. 'ssd_mobilenet_v2')."""
        with self._lock_detector_model:
            self._detector_model = str(model_key)

    def get_detector_model(self) -> str:
        """Return the currently-selected detector model key."""
        with self._lock_detector_model:
            return str(self._detector_model)
