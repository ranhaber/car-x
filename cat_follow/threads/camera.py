"""Camera capture thread with a deterministic no-OpenCV fallback."""

import threading
import time

import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except Exception:
    _HAS_CV2 = False

from cat_follow.camera_config import CameraConfig, load_camera_config
from cat_follow.logger import get_logger
from cat_follow.memory.pool import FRAME_SHAPE
from cat_follow.memory.shared_state import SharedState

log = get_logger("thread.camera")


def _open_capture(config: CameraConfig):
    if config.backend == "v4l2":
        cap = cv2.VideoCapture(config.source, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(config.source)

    if config.pixel_format:
        fourcc = cv2.VideoWriter_fourcc(*config.pixel_format)
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
    cap.set(cv2.CAP_PROP_FPS, config.fps)
    return cap


def _prepare_frame(frame: np.ndarray, config: CameraConfig) -> np.ndarray:
    """Convert a raw NV12 frame when needed, then resize to the pool shape."""
    if frame.ndim == 2 and config.pixel_format == "NV12":
        expected_size = config.width * config.height * 3 // 2
        if frame.size != expected_size:
            raise ValueError(
                f"NV12 frame has {frame.size} bytes, expected {expected_size}"
            )
        nv12 = frame.reshape((config.height * 3 // 2, config.width))
        frame = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)

    if frame.ndim != 3 or frame.shape[2] != FRAME_SHAPE[2]:
        raise ValueError(f"unsupported camera frame shape {frame.shape}")

    if frame.shape[:2] != (FRAME_SHAPE[0], FRAME_SHAPE[1]):
        frame = cv2.resize(
            frame,
            (FRAME_SHAPE[1], FRAME_SHAPE[0]),
            interpolation=cv2.INTER_AREA,
        )
    return frame


def run_camera_loop(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    target_fps: float | None = None,
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
    target_fps : float, optional
        Desired processing rate. The camera environment setting is used when
        omitted.
    """
    config = load_camera_config()
    if target_fps is None:
        target_fps = config.fps
    tick = 1.0 / target_fps
    frame_index = 0

    if _HAS_CV2:
        cap = None
        failed_reads = 0
        log.info(
            "Camera loop started (device=%s, %dx%d %s, backend=%s, %.0f FPS).",
            config.device,
            config.width,
            config.height,
            config.pixel_format or "driver-default",
            config.backend,
            target_fps,
        )

        try:
            while not stop_event.is_set():
                if cap is None:
                    cap = _open_capture(config)
                    if not cap.isOpened():
                        log.error(
                            "Unable to open camera %s; retrying in 1 second.",
                            config.device,
                        )
                        cap.release()
                        cap = None
                        stop_event.wait(1.0)
                        continue

                t0 = time.monotonic()

                ret, frame = cap.read()
                if not ret or frame is None:
                    failed_reads += 1
                    if failed_reads >= 30:
                        log.warning(
                            "Camera %s failed 30 consecutive reads; reopening.",
                            config.device,
                        )
                        cap.release()
                        cap = None
                        failed_reads = 0
                    time.sleep(0.01)
                    continue
                failed_reads = 0

                try:
                    frame = _prepare_frame(frame, config)
                except ValueError as exc:
                    log.error("Dropping camera frame: %s", exc)
                    stop_event.wait(0.1)
                    continue

                # Copy into the pool's current write buffer and publish index
                write_buf = shared.get_write_buffer()
                # OpenCV frames are BGR; we keep the raw layout as-is
                np.copyto(write_buf, frame)
                shared.publish_latest_from_write()

                frame_index += 1
                elapsed = time.monotonic() - t0
                time.sleep(max(0.0, tick - elapsed))
        finally:
            if cap is not None:
                cap.release()
    else:
        # Fallback stub behavior (no cv2 available)
        log.info("Camera loop started (stub, %.0f FPS). cv2 not available.", target_fps)
        while not stop_event.is_set():
            t0 = time.monotonic()

            # Get the next write buffer from the ring, fill it, and publish.
            write_buf = shared.get_write_buffer()
            # Stub: fill entire frame with a rolling value
            write_buf[:] = frame_index % 256
            shared.publish_latest_from_write()

            frame_index += 1
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, tick - elapsed))
