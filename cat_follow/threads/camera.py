"""
Camera thread (stub).

Writes a deterministic pattern into ``frame_latest`` at ~30 FPS.
Replace the body with real picamera2 / vilib capture later;
the interface (shared, stop_event) stays the same.
"""

import threading
import time

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.memory.shared_state import SharedState
from cat_follow.memory.pool import FRAME_SHAPE

log = get_logger("thread.camera")

# Pre-allocate one write buffer so the loop never allocates per frame.
_write_buf: np.ndarray | None = None


def _get_write_buf() -> np.ndarray:
    """Lazily allocate (once) a local buffer for building each frame."""
    global _write_buf
    if _write_buf is None:
        _write_buf = np.empty(FRAME_SHAPE, dtype=np.uint8)
    return _write_buf


def run_camera_loop(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    target_fps: float = 30.0,
) -> None:
    """Capture loop — runs until *stop_event* is set.

    **Stub behaviour:**  Each frame is filled with ``frame_index % 256``
    so tests can verify that new frames are arriving and the value
    changes over time.

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
    buf = _get_write_buf()
    frame_index = 0
    log.info("Camera loop started (stub, %.0f FPS).", target_fps)

    while not stop_event.is_set():
        t0 = time.monotonic()

        # Stub: fill entire frame with a rolling value
        buf[:] = frame_index % 256
        shared.set_frame_latest(buf)

        frame_index += 1
        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, tick - elapsed))
