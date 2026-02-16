"""
Tracker thread (stub).

Reads ``frame_latest`` and writes a stub bbox into ``bbox_tracker``.
Replace with real OpenCV KCF / CSRT tracker later; the interface stays
the same.
"""

import threading
import time

import numpy as np

from cat_follow.memory.shared_state import SharedState
from cat_follow.memory.pool import FRAME_SHAPE


def run_tracker_loop(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    target_fps: float = 30.0,
) -> None:
    """Tracker loop — runs until *stop_event* is set.

    **Stub behaviour:**  Reads the first pixel of ``frame_latest`` to
    prove it is consuming the frame, then writes a fixed bbox
    ``(100, 100, 80, 80, 1.0)`` into ``bbox_tracker``.

    Parameters
    ----------
    shared : SharedState
        Thread-safe wrapper around the pre-allocated memory pool.
    stop_event : threading.Event
        Set this to signal the loop to exit.
    target_fps : float
        Desired frames per second (default 30).
    """
    tick = 1.0 / target_fps
    # Pre-allocate a buffer to receive the frame (no per-frame alloc)
    frame_buf = np.empty(FRAME_SHAPE, dtype=np.uint8)

    while not stop_event.is_set():
        t0 = time.monotonic()

        # Read the latest frame (proves we consume it)
        shared.get_frame_latest(frame_buf)

        # Stub: always report a fixed bbox with valid=1
        shared.set_bbox_tracker(100.0, 100.0, 80.0, 80.0, 1.0)

        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, tick - elapsed))
