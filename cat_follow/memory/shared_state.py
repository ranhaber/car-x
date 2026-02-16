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

    # ── frame_latest ─────────────────────────────────────────────────

    def set_frame_latest(self, src: np.ndarray) -> None:
        """Copy *src* into the pre-allocated ``frame_latest`` buffer.

        The copy is done under ``_lock_frame`` so readers always see a
        consistent frame (not half old / half new).
        """
        with self._lock_frame:
            np.copyto(self._pool.frame_latest, src)

    def get_frame_latest(self, dst: np.ndarray) -> None:
        """Copy the current ``frame_latest`` into caller-supplied *dst*.

        *dst* must have the same shape and dtype as ``frame_latest``.
        Using a caller-supplied buffer avoids allocating a new array.
        """
        with self._lock_frame:
            np.copyto(dst, self._pool.frame_latest)

    def copy_latest_to_detector_frame(self) -> None:
        """Copy ``frame_latest`` → ``frame_for_detector`` under lock.

        Called by the main thread every K frames so the detector has a
        stable snapshot to work with.
        """
        with self._lock_frame:
            np.copyto(self._pool.frame_for_detector, self._pool.frame_latest)

    def get_frame_for_detector(self, dst: np.ndarray) -> None:
        """Copy the current ``frame_for_detector`` into *dst* under lock."""
        with self._lock_frame:
            np.copyto(dst, self._pool.frame_for_detector)

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
