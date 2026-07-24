"""Camera capture thread with a deterministic no-OpenCV fallback."""

import threading
import time
from pathlib import Path

import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except Exception:
    _HAS_CV2 = False

from cat_follow.camera_config import CameraConfig, load_camera_config
from cat_follow.logger import get_logger
from cat_follow.memory.pool import FRAME_BGR_SHAPE, FRAME_H, FRAME_NV12_SHAPE, FRAME_W
from cat_follow.memory.shared_state import SharedState
from cat_follow.perception.live_cat_injector import LiveCatInjector
from cat_follow.perception.tuning import apply_affinity
from cat_follow.perception_config import load_perception_config
from cat_follow.vision.nv12_utils import (
    bgr_to_nv12,
    nv12_to_bgr,
    validate_nv12,
    y_plane,
)

log = get_logger("thread.camera")


def _open_capture(config: CameraConfig):
    if config.capture_backend == "gst_nv12":
        from cat_follow.vision.gst_nv12_capture import GstV4l2Nv12Capture, IO_MODE_MMAP

        if config.pixel_format != "NV12":
            raise ValueError("gst_nv12 capture requires NV12 pixel format")
        cap = GstV4l2Nv12Capture(
            config.device,
            config.width,
            config.height,
            config.fps,
            io_mode=IO_MODE_MMAP,
        )
        if cap.start():
            return cap
        log.warning(
            "GStreamer NV12 capture unavailable; falling back to OpenCV."
        )

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
    if config.pixel_format == "NV12":
        # Ask OpenCV to expose the driver's packed NV12 bytes instead of
        # eagerly converting every capture to BGR.
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    return cap


def _open_lores_capture(config: CameraConfig):
    """Open the optional hardware-scaled lores stream (RKISP self-path)."""
    backend = cv2.CAP_V4L2 if config.backend == "v4l2" else cv2.CAP_ANY
    cap = cv2.VideoCapture(config.lores_source, backend)
    if config.lores_pixel_format:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.lores_pixel_format))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.lores_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.lores_height)
    cap.set(cv2.CAP_PROP_FPS, config.fps)
    return cap


def _lores_to_gray(frame: np.ndarray, config: CameraConfig) -> np.ndarray:
    """Extract a single-channel gray/luma image from a lores frame.

    For NV12 the luma (Y) plane is the top ``height`` rows, so no color
    conversion is needed — the cheapest possible motion source.
    """
    if frame.ndim == 2 and config.lores_pixel_format == "NV12":
        return frame[: config.lores_height, : config.lores_width]
    if frame.ndim == 3:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return frame


def _prepare_frame(
    frame: np.ndarray,
    config: CameraConfig,
    *,
    dst: np.ndarray | None = None,
) -> np.ndarray:
    """Pack a camera frame as 640x480 NV12, preferably into *dst*."""
    if dst is None:
        dst = np.empty(FRAME_NV12_SHAPE, dtype=np.uint8)
    elif dst.shape != FRAME_NV12_SHAPE or dst.dtype != np.uint8:
        raise ValueError(
            f"NV12 destination must be uint8 {FRAME_NV12_SHAPE}, "
            f"got {dst.dtype} {dst.shape}"
        )

    if frame.ndim == 2 and config.pixel_format == "NV12":
        nv12 = validate_nv12(frame, config.width, config.height)
        if (config.width, config.height) == (FRAME_W, FRAME_H):
            np.copyto(dst, nv12)
            return dst
        # Non-production geometries require a BGR resize before repacking.
        frame = nv12_to_bgr(nv12, config.width, config.height)

    if frame.ndim != 3 or frame.shape[2] != FRAME_BGR_SHAPE[2]:
        raise ValueError(f"unsupported camera frame shape {frame.shape}")

    if frame.shape[:2] != (FRAME_H, FRAME_W):
        frame = cv2.resize(
            frame,
            (FRAME_W, FRAME_H),
            interpolation=cv2.INTER_AREA,
        )
    return bgr_to_nv12(frame, dst=dst)


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

    perception = None
    try:
        perception = load_perception_config()
        if perception.affinity_enabled:
            apply_affinity(perception.camera_cores)
    except Exception as exc:  # noqa: BLE001 - tuning must never be fatal
        log.debug("camera affinity/tuning skipped: %s", exc)

    injector = None
    inject_bgr = None
    if perception is not None:
        image_path = Path(perception.inject_cat_image)
        if not image_path.is_absolute():
            image_path = Path(__file__).resolve().parents[2] / image_path
        injector = LiveCatInjector(
            str(image_path), speed_px_s=perception.inject_cat_speed_px_s
        )
        inject_bgr = np.empty(FRAME_BGR_SHAPE, dtype=np.uint8)
        if perception.inject_cat_enabled:
            shared.set_cat_injection_enabled(True)

    if _HAS_CV2:
        cap = None
        lores_cap = None
        failed_reads = 0
        log.info(
            "Camera loop started (device=%s, %dx%d %s, backend=%s/%s, "
            "%.0f FPS, lores=%s).",
            config.device,
            config.width,
            config.height,
            config.pixel_format or "driver-default",
            config.capture_backend,
            config.backend,
            target_fps,
            config.lores_device or "off",
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

                if config.lores_enabled and lores_cap is None:
                    lores_cap = _open_lores_capture(config)
                    if not lores_cap.isOpened():
                        log.warning(
                            "Unable to open lores stream %s; motion will use the "
                            "main frame.",
                            config.lores_device,
                        )
                        lores_cap.release()
                        lores_cap = None

                t0 = time.monotonic()

                # GStreamer can map/pack directly into the reserved ring slot.
                # OpenCV owns its read buffer, so that path reserves afterward.
                write_buf = None
                direct_capture = bool(
                    getattr(cap, "writes_nv12_destination", False)
                )
                if direct_capture:
                    write_buf = shared.try_get_write_buffer()
                    if write_buf is None:
                        frame_index += 1
                        stop_event.wait(tick)
                        continue
                    capture_started_ns = time.monotonic_ns()
                    ret, frame = cap.read(dst=write_buf)
                else:
                    capture_started_ns = time.monotonic_ns()
                    ret, frame = cap.read()
                if not ret or frame is None:
                    if write_buf is not None:
                        shared.abort_frame_write()
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

                # Reserve a free ring slot. Capture is latest-wins: if every
                # slot is pinned by a zero-copy reader, drop this camera frame
                # instead of blocking the producer behind inference/encoding.
                if write_buf is None:
                    write_buf = shared.try_get_write_buffer()
                    if write_buf is None:
                        frame_index += 1
                        elapsed = time.monotonic() - t0
                        stop_event.wait(max(0.0, tick - elapsed))
                        continue

                    try:
                        _prepare_frame(frame, config, dst=write_buf)
                    except ValueError as exc:
                        shared.abort_frame_write()
                        log.error("Dropping camera frame: %s", exc)
                        stop_event.wait(0.1)
                        continue

                injected = False
                bbox = None
                if injector is not None:
                    enabled = shared.cat_injection_enabled()
                    try:
                        injector.set_enabled(enabled)
                        if enabled:
                            assert inject_bgr is not None
                            nv12_to_bgr(
                                write_buf, FRAME_W, FRAME_H, dst=inject_bgr
                            )
                            bbox = injector.apply(inject_bgr)
                            if bbox is not None:
                                bgr_to_nv12(inject_bgr, dst=write_buf)
                                injected = True
                    except Exception as exc:  # noqa: BLE001
                        # Injection is a diagnostic feature; a missing/bad
                        # sprite must not take down the real camera pipeline.
                        log.error("Disabling live cat injection: %s", exc)
                        injector.set_enabled(False)
                        shared.set_cat_injection_enabled(False)

                shared.publish_latest_from_write(
                    capture_started_ns=capture_started_ns
                )
                if injector is not None:
                    # Publish diagnostics only after the matching pixels become
                    # visible; a dropped camera frame must not advance its bbox.
                    shared.set_cat_injection_bbox(bbox)

                # Publish a hardware-scaled lores gray frame for motion.
                if lores_cap is not None:
                    lret, lframe = lores_cap.read()
                    try:
                        if injected:
                            # The hardware lores stream does not contain the
                            # composited sprite. Derive its motion frame from
                            # the injected main image so motion gating sees the
                            # exact pixels subsequently sent to RKNN.
                            gray = y_plane(write_buf, FRAME_W, FRAME_H)
                            gray = cv2.resize(
                                gray,
                                (config.lores_width, config.lores_height),
                                interpolation=cv2.INTER_AREA,
                            )
                            shared.set_lores_gray(gray)
                        elif lret and lframe is not None:
                            shared.set_lores_gray(_lores_to_gray(lframe, config))
                    except Exception as exc:  # noqa: BLE001
                        log.debug("lores gray publish skipped: %s", exc)

                frame_index += 1
                elapsed = time.monotonic() - t0
                time.sleep(max(0.0, tick - elapsed))
        finally:
            if cap is not None:
                cap.release()
            if lores_cap is not None:
                lores_cap.release()
    else:
        # Fallback stub behavior (no cv2 available)
        log.info("Camera loop started (stub, %.0f FPS). cv2 not available.", target_fps)
        while not stop_event.is_set():
            t0 = time.monotonic()

            # Get the next free write slot. Stub capture follows the same
            # non-blocking latest-wins policy as the real camera.
            write_buf = shared.try_get_write_buffer()
            if write_buf is None:
                elapsed = time.monotonic() - t0
                stop_event.wait(max(0.0, tick - elapsed))
                continue
            # Stub: valid neutral-chroma NV12 with rolling luma.
            capture_started_ns = time.monotonic_ns()
            write_buf[:FRAME_H].fill(frame_index % 256)
            write_buf[FRAME_H:].fill(128)
            shared.publish_latest_from_write(
                capture_started_ns=capture_started_ns
            )

            frame_index += 1
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, tick - elapsed))
