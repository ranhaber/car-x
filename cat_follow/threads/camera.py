"""
Camera thread (stub).

Writes a deterministic pattern into ``frame_latest`` at ~30 FPS.
Replace the body with real picamera2 / vilib capture later;
the interface (shared, stop_event) stays the same.
"""

import threading
import time

import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except Exception:
    _HAS_CV2 = False

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
    frame_index = 0

    if _HAS_CV2:
        # Use OpenCV VideoCapture as a robust fallback for direct capture.
        cap = cv2.VideoCapture(0)
        # Try to set desired resolution
        try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SHAPE[1])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SHAPE[0])
        except Exception:
            pass

        log.info("Camera loop started (cv2 capture, %.0f FPS).", target_fps)

        try:
            while not stop_event.is_set():
                t0 = time.monotonic()

                ret, frame = cap.read()
                if not ret or frame is None:
                    # camera read failed; sleep a bit and retry
                    time.sleep(0.01)
                    continue

                # Ensure shape matches FRAME_SHAPE; resize if necessary
                if frame.shape[:2] != (FRAME_SHAPE[0], FRAME_SHAPE[1]):
                    frame = cv2.resize(frame, (FRAME_SHAPE[1], FRAME_SHAPE[0]), interpolation=cv2.INTER_AREA)

                # Copy into the pool's current write buffer and publish index
                write_buf = shared.get_write_buffer()
                # OpenCV frames are BGR; we keep the raw layout as-is
                np.copyto(write_buf, frame)
                shared.publish_latest_from_write()

                frame_index += 1
                elapsed = time.monotonic() - t0
                time.sleep(max(0.0, tick - elapsed))
        finally:
            try:
                cap.release()
            except Exception:
                pass
    else:
        # Fallback stub behavior (no cv2 available)
        buf = _get_write_buf()
        log.info("Camera loop started (stub, %.0f FPS). cv2 not available.", target_fps)
        while not stop_event.is_set():
            t0 = time.monotonic()

            # Stub: fill entire frame with a rolling value
            buf[:] = frame_index % 256
            # copy into the ring write slot and publish
            write_buf = shared.get_write_buffer()
            np.copyto(write_buf, buf)
            shared.publish_latest_from_write()

            frame_index += 1
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, tick - elapsed))
