"""
Integration tests for cat_follow.threads (stub camera, tracker, detector).

Start all three threads with SharedState, run for a short burst, stop,
and verify that shared state was written by each thread.

Run:
    python -m pytest tests/test_thread_stubs.py -v
or:
    python tests/test_thread_stubs.py
"""

import sys
import os
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from cat_follow.memory.pool import allocate_pool, FRAME_SHAPE
from cat_follow.memory.shared_state import SharedState
from cat_follow.threads.camera import run_camera_loop
from cat_follow.threads.tracker import run_tracker_loop
from cat_follow.threads.detector import run_detector_loop


# ── helpers ──────────────────────────────────────────────────────────────

def _run_threads_for(seconds: float = 1.0):
    """Start all three stub threads, run for *seconds*, stop, return shared."""
    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()

    threads = [
        threading.Thread(
            target=run_camera_loop, args=(shared, stop),
            name="camera-stub", daemon=True,
        ),
        threading.Thread(
            target=run_tracker_loop, args=(shared, stop),
            name="tracker-stub", daemon=True,
        ),
        threading.Thread(
            target=run_detector_loop, args=(shared, stop),
            name="detector-stub", daemon=True,
        ),
    ]
    for t in threads:
        t.start()

    time.sleep(seconds)
    stop.set()

    for t in threads:
        t.join(timeout=3.0)

    return shared, threads


# ── tests ────────────────────────────────────────────────────────────────

def test_all_threads_start_and_stop():
    """Threads start, run briefly, and join without hanging."""
    shared, threads = _run_threads_for(0.5)
    for t in threads:
        assert not t.is_alive(), f"Thread {t.name} did not stop"


def test_camera_writes_frame_latest():
    """After running, frame_latest must not be all zeros."""
    shared, _ = _run_threads_for(0.5)
    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_latest(dst)
    assert not np.all(dst == 0), "Camera stub should have written non-zero frames"


def test_camera_frame_has_pattern():
    """Camera stub fills frames with (frame_index % 256), so at least
    some pixel values should be non-zero after many frames."""
    shared, _ = _run_threads_for(0.5)
    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_latest(dst)
    # All pixels in one frame should be the same value (stub fills uniformly)
    val = dst[0, 0, 0]
    assert np.all(dst == val), "Stub camera should fill the entire frame with one value"


def test_tracker_writes_bbox():
    """Tracker stub should write bbox_tracker with valid=1."""
    shared, _ = _run_threads_for(0.5)
    bbox = shared.get_bbox_tracker()
    assert bbox == (100.0, 100.0, 80.0, 80.0, 1.0), (
        f"Expected tracker stub bbox (100,100,80,80,1), got {bbox}"
    )


def test_detector_writes_bbox():
    """Detector stub should write bbox_detector with valid=1."""
    shared, _ = _run_threads_for(0.5)

    # The detector needs frame_for_detector to be populated.
    # In a real app main would call copy_latest_to_detector_frame().
    # Our stub detector reads whatever is in the buffer (zeros is fine).
    bbox = shared.get_bbox_detector()
    assert bbox == (120.0, 120.0, 60.0, 60.0, 1.0), (
        f"Expected detector stub bbox (120,120,60,60,1), got {bbox}"
    )


def test_copy_latest_to_detector_during_run():
    """Simulate main copying frame_latest -> frame_for_detector while
    threads are running; verify detector still gets a valid frame."""
    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()

    threads = [
        threading.Thread(
            target=run_camera_loop, args=(shared, stop),
            name="camera-stub", daemon=True,
        ),
        threading.Thread(
            target=run_tracker_loop, args=(shared, stop),
            name="tracker-stub", daemon=True,
        ),
        threading.Thread(
            target=run_detector_loop, args=(shared, stop),
            name="detector-stub", daemon=True,
        ),
    ]
    for t in threads:
        t.start()

    # Simulate main loop copying every 0.1 s for 0.5 s
    for _ in range(5):
        time.sleep(0.1)
        shared.copy_latest_to_detector_frame()

    stop.set()
    for t in threads:
        t.join(timeout=3.0)

    # After copies, frame_for_detector should be non-zero
    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_for_detector(dst)
    assert not np.all(dst == 0), (
        "frame_for_detector should be non-zero after copies from camera"
    )


def test_no_exceptions_during_run():
    """Run threads; if any thread raised, it would have stopped early.
    We verify all threads were still alive right before stop."""
    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()
    alive_flags = {}

    threads = [
        threading.Thread(
            target=run_camera_loop, args=(shared, stop),
            name="camera-stub", daemon=True,
        ),
        threading.Thread(
            target=run_tracker_loop, args=(shared, stop),
            name="tracker-stub", daemon=True,
        ),
        threading.Thread(
            target=run_detector_loop, args=(shared, stop),
            name="detector-stub", daemon=True,
        ),
    ]
    for t in threads:
        t.start()

    time.sleep(0.5)

    # Snapshot alive status before stopping
    for t in threads:
        alive_flags[t.name] = t.is_alive()

    stop.set()
    for t in threads:
        t.join(timeout=3.0)

    for name, was_alive in alive_flags.items():
        assert was_alive, f"Thread {name} died before stop (likely exception)"


# ── run as script ────────────────────────────────────────────────────────

if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"  PASS  {fn.__name__}")
    print(f"\nAll {len(tests)} tests passed.")
