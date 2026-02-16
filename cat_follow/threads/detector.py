"""
Detector thread (stub).

Reads ``frame_for_detector`` every K iterations and writes a stub bbox
into ``bbox_detector``.  Replace with real TFLite inference later; the
interface stays the same.
"""

import threading
import time

import numpy as np

from cat_follow.memory.shared_state import SharedState
from cat_follow.memory.pool import FRAME_SHAPE


def run_detector_loop(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    target_fps: float = 30.0,
    detect_every_k: int = 10,
) -> None:
    """Detector loop — runs until *stop_event* is set.

    **Stub behaviour:**  Every *detect_every_k* iterations, reads
    ``frame_for_detector`` and writes a fixed bbox
    ``(120, 120, 60, 60, 1.0)`` into ``bbox_detector``.  On other
    iterations it just sleeps (simulating the tracker handling those
    frames).

    Parameters
    ----------
    shared : SharedState
        Thread-safe wrapper around the pre-allocated memory pool.
    stop_event : threading.Event
        Set this to signal the loop to exit.
    target_fps : float
        Desired frames per second (default 30).
    detect_every_k : int
        Run "detection" every K-th iteration (default 10).
    """
    tick = 1.0 / target_fps
    # Pre-allocate a buffer to receive the detector frame (no per-frame alloc)
    frame_buf = np.empty(FRAME_SHAPE, dtype=np.uint8)
    iteration = 0

    while not stop_event.is_set():
        t0 = time.monotonic()

        if iteration % detect_every_k == 0:
            # Read the detector-dedicated frame snapshot
            shared.get_frame_for_detector(frame_buf)

            # Stub: always report a fixed bbox with valid=1
            shared.set_bbox_detector(120.0, 120.0, 60.0, 60.0, 1.0)

        iteration += 1
        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, tick - elapsed))
