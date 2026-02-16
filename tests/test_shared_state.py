"""
Unit tests for cat_follow.memory.shared_state — thread-safe SharedState.

Covers:
  - Single-thread get/set round-trips for every resource.
  - Concurrent writer + reader on bbox_tracker to verify no torn reads.
  - Frame copy helpers (set/get frame_latest, copy_latest_to_detector_frame).

Run:
    python -m pytest tests/test_shared_state.py -v
or:
    python tests/test_shared_state.py
"""

import sys
import os
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from cat_follow.memory.pool import allocate_pool, FRAME_SHAPE, BBOX_LEN
from cat_follow.memory.shared_state import SharedState


# ── helpers ──────────────────────────────────────────────────────────────

def _make_shared() -> SharedState:
    return SharedState(allocate_pool())


# ── single-thread tests ─────────────────────────────────────────────────

def test_bbox_tracker_set_get():
    shared = _make_shared()
    shared.set_bbox_tracker(10.0, 20.0, 30.0, 40.0, 1.0)
    result = shared.get_bbox_tracker()
    assert result == (10.0, 20.0, 30.0, 40.0, 1.0)


def test_bbox_tracker_default_zero():
    shared = _make_shared()
    result = shared.get_bbox_tracker()
    assert result == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_bbox_detector_set_get():
    shared = _make_shared()
    shared.set_bbox_detector(100.0, 200.0, 50.0, 60.0, 1.0)
    result = shared.get_bbox_detector()
    assert result == (100.0, 200.0, 50.0, 60.0, 1.0)


def test_bbox_detector_default_zero():
    shared = _make_shared()
    result = shared.get_bbox_detector()
    assert result == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_odometry_set_get():
    shared = _make_shared()
    shared.set_odometry(1.5, 2.5, 90.0)
    result = shared.get_odometry()
    assert result == (1.5, 2.5, 90.0)


def test_odometry_default_zero():
    shared = _make_shared()
    result = shared.get_odometry()
    assert result == (0.0, 0.0, 0.0)


def test_frame_latest_set_get():
    shared = _make_shared()
    src = np.full(FRAME_SHAPE, 42, dtype=np.uint8)
    shared.set_frame_latest(src)

    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_latest(dst)
    assert np.all(dst == 42)


def test_frame_latest_does_not_alias_src():
    """set_frame_latest must copy, not reference, the source array."""
    shared = _make_shared()
    src = np.full(FRAME_SHAPE, 99, dtype=np.uint8)
    shared.set_frame_latest(src)

    src[:] = 0  # mutate source after set

    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_latest(dst)
    assert np.all(dst == 99), "SharedState must hold a copy, not a reference"


def test_copy_latest_to_detector_frame():
    shared = _make_shared()
    src = np.full(FRAME_SHAPE, 77, dtype=np.uint8)
    shared.set_frame_latest(src)
    shared.copy_latest_to_detector_frame()

    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_for_detector(dst)
    assert np.all(dst == 77)


def test_detector_frame_independent_of_later_latest():
    """After copy, changing frame_latest must not affect frame_for_detector."""
    shared = _make_shared()

    src1 = np.full(FRAME_SHAPE, 55, dtype=np.uint8)
    shared.set_frame_latest(src1)
    shared.copy_latest_to_detector_frame()

    src2 = np.full(FRAME_SHAPE, 88, dtype=np.uint8)
    shared.set_frame_latest(src2)

    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_for_detector(dst)
    assert np.all(dst == 55), "Detector frame must keep old snapshot"


def test_bbox_tracker_overwrite():
    shared = _make_shared()
    shared.set_bbox_tracker(1.0, 2.0, 3.0, 4.0, 1.0)
    shared.set_bbox_tracker(5.0, 6.0, 7.0, 8.0, 0.0)
    result = shared.get_bbox_tracker()
    assert result == (5.0, 6.0, 7.0, 8.0, 0.0)


def test_odometry_overwrite():
    shared = _make_shared()
    shared.set_odometry(1.0, 2.0, 3.0)
    shared.set_odometry(10.0, 20.0, 30.0)
    assert shared.get_odometry() == (10.0, 20.0, 30.0)


# ── concurrent tests ────────────────────────────────────────────────────

def test_concurrent_bbox_tracker_no_torn_reads():
    """One writer, one reader on bbox_tracker for many iterations.

    The writer writes 5-tuples where all five values equal the iteration
    index (e.g. (7,7,7,7,7) at iteration 7).  The reader asserts that
    every read is such a "uniform" 5-tuple — i.e. all five values are the
    same, proving no partial/torn write was observed.
    """
    shared = _make_shared()
    iterations = 5_000
    errors: list = []
    stop = threading.Event()

    def writer():
        for i in range(iterations):
            v = float(i)
            shared.set_bbox_tracker(v, v, v, v, v)
        stop.set()

    def reader():
        while not stop.is_set():
            tup = shared.get_bbox_tracker()
            # All five values must be the same (from one write iteration)
            if len(set(tup)) != 1:
                errors.append(tup)
                break  # one failure is enough

    t_w = threading.Thread(target=writer, name="writer")
    t_r = threading.Thread(target=reader, name="reader")
    t_r.start()
    t_w.start()
    t_w.join()
    t_r.join()

    assert len(errors) == 0, f"Torn read detected: {errors[0]}"


def test_concurrent_odometry_no_torn_reads():
    """Same pattern for odometry (3 values)."""
    shared = _make_shared()
    iterations = 5_000
    errors: list = []
    stop = threading.Event()

    def writer():
        for i in range(iterations):
            v = float(i)
            shared.set_odometry(v, v, v)
        stop.set()

    def reader():
        while not stop.is_set():
            tup = shared.get_odometry()
            if len(set(tup)) != 1:
                errors.append(tup)
                break

    t_w = threading.Thread(target=writer, name="odom-writer")
    t_r = threading.Thread(target=reader, name="odom-reader")
    t_r.start()
    t_w.start()
    t_w.join()
    t_r.join()

    assert len(errors) == 0, f"Torn read detected: {errors[0]}"


# ── run as script ────────────────────────────────────────────────────────

if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"  PASS  {fn.__name__}")
    print(f"\nAll {len(tests)} tests passed.")
