"""
Unit tests for cat_follow.memory.pool — pre-allocated buffer pool.

Run:
    python -m pytest tests/test_memory_pool.py -v
or:
    python tests/test_memory_pool.py
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from cat_follow.memory.pool import (
    allocate_pool,
    MemoryPool,
    FRAME_H,
    FRAME_W,
    FRAME_C,
    FRAME_SHAPE,
    FRAME_NBYTES,
    BBOX_LEN,
    FRAME_RING_N,
    ODOM_LEN,
)


# ── helpers ──────────────────────────────────────────────────────────────

def _make_pool() -> MemoryPool:
    return allocate_pool()


# ── tests ────────────────────────────────────────────────────────────────

def test_constants_consistent():
    """FRAME_SHAPE and FRAME_NBYTES match the individual H/W/C constants."""
    assert FRAME_SHAPE == (FRAME_H, FRAME_W, FRAME_C)
    assert FRAME_NBYTES == FRAME_H * FRAME_W * FRAME_C


def test_frame_ring_shape_and_dtype():
    pool = _make_pool()
    assert pool.frame_ring.shape == (FRAME_RING_N, FRAME_H, FRAME_W, FRAME_C)
    assert pool.frame_ring.dtype == np.uint8


def test_frame_for_detector_shape_and_dtype():
    pool = _make_pool()
    assert pool.frame_for_detector.shape == FRAME_SHAPE
    assert pool.frame_for_detector.dtype == np.uint8


def test_frame_nbytes():
    pool = _make_pool()
    # Test one frame from the ring
    assert pool.frame_ring[0].nbytes == FRAME_NBYTES
    assert pool.frame_for_detector.nbytes == FRAME_NBYTES


def test_frames_are_separate_buffers():
    """frame_ring and frame_for_detector must be independent arrays."""
    pool = _make_pool()
    assert pool.frame_ring.base is not pool.frame_for_detector.base
    pool.frame_ring[0, :, :, :] = 42
    assert pool.frame_for_detector[0, 0, 0] == 0, (
        "Writing to frame_ring must not affect frame_for_detector"
    )


def test_bbox_tracker_length_and_dtype():
    pool = _make_pool()
    assert len(pool.bbox_tracker) == BBOX_LEN
    assert pool.bbox_tracker.dtype == np.float64


def test_bbox_detector_length_and_dtype():
    pool = _make_pool()
    assert len(pool.bbox_detector) == BBOX_LEN
    assert pool.bbox_detector.dtype == np.float64


def test_bboxes_are_separate_buffers():
    pool = _make_pool()
    assert pool.bbox_tracker is not pool.bbox_detector
    pool.bbox_tracker[0] = 99.0
    assert pool.bbox_detector[0] == 0.0


def test_odometry_length_and_dtype():
    pool = _make_pool()
    assert len(pool.odometry) == ODOM_LEN
    assert pool.odometry.dtype == np.float64


def test_write_read_frame():
    """Write a known value into a frame buffer, read it back."""
    pool = _make_pool()
    pool.frame_for_detector[:] = 128
    assert np.all(pool.frame_for_detector == 128)


def test_write_read_bbox_tracker():
    pool = _make_pool()
    pool.bbox_tracker[:] = [10.0, 20.0, 30.0, 40.0, 1.0]
    assert pool.bbox_tracker[0] == 10.0
    assert pool.bbox_tracker[4] == 1.0


def test_write_read_odometry():
    pool = _make_pool()
    pool.odometry[:] = [1.5, 2.5, 90.0]
    assert pool.odometry[0] == 1.5
    assert pool.odometry[2] == 90.0


def test_no_realloc_on_repeated_writes():
    """Repeated in-place writes must reuse the same underlying buffer."""
    pool = _make_pool()

    frame_id = id(pool.frame_for_detector)
    bbox_id = id(pool.bbox_tracker)
    odom_id = id(pool.odometry)

    for i in range(10):
        pool.frame_for_detector[:] = i
        pool.bbox_tracker[:] = [float(i)] * BBOX_LEN
        pool.odometry[:] = [float(i)] * ODOM_LEN

    assert id(pool.frame_for_detector) == frame_id, "frame_for_detector was reallocated"
    assert id(pool.bbox_tracker) == bbox_id, "bbox_tracker was reallocated"
    assert id(pool.odometry) == odom_id, "odometry was reallocated"


def test_all_buffers_start_at_zero():
    """Every buffer must be zero-initialized."""
    pool = _make_pool()
    assert np.all(pool.frame_ring == 0)
    assert np.all(pool.frame_for_detector == 0)
    assert np.all(pool.bbox_tracker == 0.0)
    assert np.all(pool.bbox_detector == 0.0)
    assert np.all(pool.odometry == 0.0)


# ── run as script ────────────────────────────────────────────────────────

if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"  PASS  {fn.__name__}")
    print(f"\nAll {len(tests)} tests passed.")
