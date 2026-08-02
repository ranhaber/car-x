"""Camera capture thread with a deterministic no-OpenCV fallback."""

import threading
import time
from pathlib import Path
from typing import Callable, Optional

import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except Exception:
    _HAS_CV2 = False

from cat_follow.camera_config import CameraConfig, load_camera_config
from cat_follow.logger import get_logger
from cat_follow.memory.pool import FRAME_BGR_SHAPE, FRAME_H, FRAME_NV12_SHAPE, FRAME_W
from cat_follow.memory.shared_state import DmabufRequeueError, SharedState
from cat_follow.perception.live_cat_injector import LiveCatInjector
from cat_follow.perception.tuning import apply_affinity
from cat_follow.perception_config import PerceptionConfig, load_perception_config
from cat_follow.vision.nv12_utils import (
    bgr_to_nv12,
    nv12_to_bgr,
    validate_nv12,
    y_plane,
)

log = get_logger("thread.camera")


def _wait_for_capture_active(
    shared: SharedState,
    stop_event: threading.Event,
    last_publish_s: float,
) -> tuple[bool, float]:
    """Block while lifecycle says capture is inactive (no busy-loop).

    Returns ``(running, last_publish_s)``; ``running`` is False when
    ``stop_event`` is set. The publish deadline restarts after an intentional
    pause, because an idle HOME/FAILSAFE gap is not a stalled camera and must
    not make the first frame after resume look overdue.
    """

    paused = False
    while not stop_event.is_set():
        if not hasattr(shared, "capture_active") or shared.capture_active():
            return True, (time.monotonic() if paused else last_publish_s)
        paused = True
        stop_event.wait(0.05)
    return False, last_publish_s


class CameraHandshake:
    """Startup health channel used by the runtime supervisor."""

    def __init__(self) -> None:
        self._ready = threading.Event()
        self._error: Optional[BaseException] = None

    def set_ready(self) -> None:
        self._ready.set()

    def set_failed(self, exc: BaseException) -> None:
        self._error = exc
        self._ready.set()

    def wait_ready(self, timeout: float = 30.0) -> None:
        if not self._ready.wait(timeout):
            raise RuntimeError(
                f"camera worker did not report readiness within {timeout:.0f}s"
            )
        if self._error is not None:
            raise RuntimeError(
                f"camera worker failed to start: {self._error}"
            ) from self._error


class CameraFatalHook:
    """One-shot, bufferable fatal camera escalation callback."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cb: Optional[Callable[[str], None]] = None
        self._pending: Optional[str] = None
        self.fired = False

    def set_handler(self, cb: Callable[[str], None]) -> None:
        with self._lock:
            self._cb = cb
            pending = self._pending
            self._pending = None
        if pending is not None:
            cb(pending)

    def fire(self, message: str) -> None:
        with self._lock:
            if self.fired:
                return
            self.fired = True
            cb = self._cb
            if cb is None:
                self._pending = message
                return
        cb(message)


def _check_publish_deadline(last_publish_s: float, config: CameraConfig) -> None:
    elapsed = time.monotonic() - last_publish_s
    if elapsed >= config.no_publish_timeout_sec:
        raise RuntimeError(
            f"camera published no frame for {elapsed:.1f}s "
            f"(limit {config.no_publish_timeout_sec:.1f}s)"
        )


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


def _run_zerocopy_camera_loop(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    config: CameraConfig,
    perception: PerceptionConfig,
    target_fps: float,
    tick: float,
    handshake: CameraHandshake | None = None,
    on_fatal: CameraFatalHook | None = None,
) -> None:
    from cat_follow.vision.zerocopy_backend import ZerocopySession

    repo_root = Path(__file__).resolve().parents[2]
    model_path = Path(perception.rknn_model_path)
    if not model_path.is_absolute():
        model_path = repo_root / model_path

    session = None
    for attempt in range(1, config.open_failure_limit + 1):
        session = ZerocopySession.open(
            device=config.device,
            model_path=str(model_path),
            src_w=config.width,
            src_h=config.height,
            crop_w=perception.rknn_input_size[0],
            crop_h=perception.rknn_input_size[1],
            animal_mode=perception.animal_mode,
        )
        if session is not None:
            break
        log.error(
            "Zerocopy camera open failed (attempt %d/%d)",
            attempt,
            config.open_failure_limit,
        )
        if stop_event.wait(1.0):
            raise RuntimeError("camera startup stopped before zerocopy opened")
    if session is None:
        raise RuntimeError("persistent zerocopy camera open failure")

    # Validate dequeue -> RGA -> RKNN -> QBUF before exposing the session to
    # detector/H.264 consumers or reporting camera readiness.
    try:
        session.self_test()
    except BaseException:
        session.close()
        raise
    def _requeue_or_fatal(buffer_index: int) -> bool:
        """Requeue one V4L2 buffer; escalate and report failure to the ring.

        Returning False (rather than raising) lets ``SharedState`` finish its
        own bookkeeping and raise :class:`DmabufRequeueError`, which tells the
        caller whether the frame was published before the failure.
        """
        if session.requeue(buffer_index):
            return True
        message = (
            f"zerocopy QBUF/requeue failed for buffer {buffer_index}: "
            f"{session.last_error}"
        )
        log.error("%s", message)
        if on_fatal is not None:
            on_fatal.fire(message)
        stop_event.set()
        return False

    shared.attach_zerocopy_session(session, requeue_cb=_requeue_or_fatal)
    if handshake is not None:
        handshake.set_ready()
    log.info(
        "Camera zerocopy loop started (device=%s, H.264=dmabuf, "
        "numpy_pack=inject-only).",
        config.device,
    )

    frame_index = 0
    dequeue_failures = 0
    last_publish_s = time.monotonic()
    lores_cap = None
    inject_rejected = False
    try:
        if config.lores_enabled:
            lores_cap = _open_lores_capture(config)
            if not lores_cap.isOpened():
                log.warning(
                    "Unable to open lores stream %s; dmabuf motion gating "
                    "will remain inactive.",
                    config.lores_device,
                )
                lores_cap.release()
                lores_cap = None
        while not stop_event.is_set():
            running, last_publish_s = _wait_for_capture_active(
                shared, stop_event, last_publish_s
            )
            if not running:
                break
            loop_started_s = time.monotonic()
            write_buf = shared.try_get_write_buffer()
            if write_buf is None:
                frame_index += 1
                _check_publish_deadline(last_publish_s, config)
                stop_event.wait(0.001)
                continue

            capture_started_ns = time.monotonic_ns()
            zc_frame = session.dequeue(timeout_ms=int(max(1000, tick * 1000)))
            if zc_frame is None:
                shared.abort_frame_write()
                dequeue_failures += 1
                log.debug("zerocopy dequeue failed: %s", session.last_error)
                if dequeue_failures >= config.dequeue_failure_limit:
                    raise RuntimeError(
                        "persistent zerocopy dequeue failure "
                        f"({dequeue_failures} attempts): {session.last_error}"
                    )
                _check_publish_deadline(last_publish_s, config)
                stop_event.wait(0.01)
                continue
            dequeue_failures = 0

            handed_off = False
            try:
                # Inject composites CPU pixels into the NumPy ring, but native
                # detection and H.264 read the camera DMA-BUF, so an injected
                # frame would only ever be half-applied. The camera owns the
                # compositor, so it is also the component that refuses it.
                if shared.cat_injection_enabled():
                    if not inject_rejected:
                        log.error(
                            "Live cat inject is not supported with "
                            "ZEROCOPY=dmabuf (detector and H.264 read the "
                            "camera DMA-BUF); disabling inject."
                        )
                        inject_rejected = True
                    shared.set_cat_injection_enabled(False)
                    shared.set_cat_injection_bbox(None)

                # A DMA-BUF fd alone is not evidence of motion, so motion
                # gating needs the real RKISP lores luma stream.
                if lores_cap is not None:
                    lret, lframe = lores_cap.read()
                    if lret and lframe is not None:
                        shared.set_lores_gray(_lores_to_gray(lframe, config))

                try:
                    shared.publish_dmabuf_from_write(
                        capture_started_ns=capture_started_ns,
                        dmabuf_fd=zc_frame.cam_fd,
                        buffer_index=zc_frame.buffer_index,
                        image_size=zc_frame.image_size,
                        stride=zc_frame.stride,
                    )
                except DmabufRequeueError:
                    # The slot was published before the superseded buffer
                    # failed to requeue; this frame's buffer now belongs to the
                    # ring and must not be requeued a second time here.
                    handed_off = True
                    raise
                handed_off = True
                last_publish_s = time.monotonic()
            finally:
                # Before publication the camera still owns the dequeued V4L2
                # buffer. Every exit path must QBUF it; after publication the
                # frame lease/ring supersession path owns requeue.
                if not handed_off:
                    shared.abort_frame_write()
                    _requeue_or_fatal(zc_frame.buffer_index)

            frame_index += 1
            # Pace to the configured rate; without this the loop free-runs at
            # whatever the driver will hand over whenever a slot is free.
            stop_event.wait(max(0.0, tick - (time.monotonic() - loop_started_s)))
    finally:
        shared.attach_zerocopy_session(None, requeue_cb=lambda _idx: True)
        if lores_cap is not None:
            lores_cap.release()
        session.close()


def _run_camera_loop_impl(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    target_fps: float | None = None,
    handshake: CameraHandshake | None = None,
    on_fatal: CameraFatalHook | None = None,
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

    use_zerocopy = (
        perception is not None
        and perception.effective_zerocopy() == "dmabuf"
    )
    if use_zerocopy:
        from cat_follow.vision.zerocopy_backend import runtime_available

        if not runtime_available():
            raise RuntimeError(
                "CAT_FOLLOW_PERCEPTION_ZEROCOPY=dmabuf but native zerocopy "
                "runtime is unavailable (libcat_follow_zerocopy / /dev/rga). "
                "Set ZEROCOPY=numpy to use the Option A capture path."
            )
        if perception is None:
            raise RuntimeError("zerocopy capture requires a perception config")
        _run_zerocopy_camera_loop(
            shared,
            stop_event,
            config=config,
            perception=perception,
            target_fps=target_fps,
            tick=tick,
            handshake=handshake,
            on_fatal=on_fatal,
        )
        return

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
        open_failures = 0
        last_publish_s = time.monotonic()
        ready_reported = False
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
                running, last_publish_s = _wait_for_capture_active(
                    shared, stop_event, last_publish_s
                )
                if not running:
                    break
                loop_started_s = time.monotonic()
                if cap is None:
                    try:
                        cap = _open_capture(config)
                    except Exception as exc:  # noqa: BLE001
                        open_failures += 1
                        log.warning(
                            "Camera open raised (attempt %d/%d): %s",
                            open_failures,
                            config.open_failure_limit,
                            exc,
                        )
                        if open_failures >= config.open_failure_limit:
                            raise RuntimeError(
                                f"persistent camera open failure for "
                                f"{config.device}: {exc}"
                            ) from exc
                        stop_event.wait(1.0)
                        continue
                    if not cap.isOpened():
                        open_failures += 1
                        log.error(
                            "Unable to open camera %s (attempt %d/%d).",
                            config.device,
                            open_failures,
                            config.open_failure_limit,
                        )
                        cap.release()
                        cap = None
                        if open_failures >= config.open_failure_limit:
                            raise RuntimeError(
                                f"persistent camera open failure for {config.device}"
                            )
                        stop_event.wait(1.0)
                        continue
                    open_failures = 0

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
                        _check_publish_deadline(last_publish_s, config)
                        stop_event.wait(0.001)
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
                    if failed_reads >= config.dequeue_failure_limit:
                        raise RuntimeError(
                            f"persistent camera dequeue/read failure for "
                            f"{config.device} ({failed_reads} attempts)"
                        )
                    _check_publish_deadline(last_publish_s, config)
                    stop_event.wait(0.01)
                    continue
                failed_reads = 0

                # Reserve a free ring slot. Capture is latest-wins: if every
                # slot is pinned by a zero-copy reader, drop this camera frame
                # instead of blocking the producer behind inference/encoding.
                if write_buf is None:
                    write_buf = shared.try_get_write_buffer()
                    if write_buf is None:
                        frame_index += 1
                        _check_publish_deadline(last_publish_s, config)
                        stop_event.wait(0.001)
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
                last_publish_s = time.monotonic()
                if handshake is not None and not ready_reported:
                    handshake.set_ready()
                    ready_reported = True
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
                # Pace to the configured rate instead of free-running whenever
                # a ring slot happens to be available.
                stop_event.wait(
                    max(0.0, tick - (time.monotonic() - loop_started_s))
                )
        finally:
            if cap is not None:
                cap.release()
            if lores_cap is not None:
                lores_cap.release()
    else:
        # Fallback stub behavior (no cv2 available)
        log.info("Camera loop started (stub, %.0f FPS). cv2 not available.", target_fps)
        ready_reported = False
        last_publish_s = time.monotonic()
        while not stop_event.is_set():
            running, last_publish_s = _wait_for_capture_active(
                shared, stop_event, last_publish_s
            )
            if not running:
                break
            t0 = time.monotonic()

            # Get the next free write slot. Stub capture follows the same
            # non-blocking latest-wins policy as the real camera.
            write_buf = shared.try_get_write_buffer()
            if write_buf is None:
                _check_publish_deadline(last_publish_s, config)
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
            if handshake is not None and not ready_reported:
                handshake.set_ready()
                ready_reported = True
            last_publish_s = time.monotonic()

            frame_index += 1
            elapsed = time.monotonic() - t0
            stop_event.wait(max(0.0, tick - elapsed))


def run_camera_loop(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    target_fps: float | None = None,
    handshake: CameraHandshake | None = None,
    on_fatal: CameraFatalHook | None = None,
) -> None:
    """Run capture and escalate any persistent or uncaught failure exactly once."""
    try:
        _run_camera_loop_impl(
            shared,
            stop_event,
            target_fps=target_fps,
            handshake=handshake,
            on_fatal=on_fatal,
        )
    except BaseException as exc:
        if handshake is not None:
            handshake.set_failed(exc)
        message = str(exc) or type(exc).__name__
        log.exception("Camera fatal error: %s", message)
        if on_fatal is not None:
            on_fatal.fire(message)
        stop_event.set()
        if handshake is None and on_fatal is None:
            raise
