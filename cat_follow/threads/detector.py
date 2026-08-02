"""Detector thread (motion-gated, lazy-loaded, RKNN NPU only).

Resource-optimization design (scaled for the ROCK 4D):

- **Motion gating**: a cheap frame-difference motion detector runs every tick;
  the expensive model is only invoked when motion is present (or at a reduced
  cadence while a target is locked), driven by a small perception phase FSM.
- **Lazy load + idle unload**: the RK3576 NPU (RKNN) backend is loaded on first
  need and unloaded after an idle period so its worker threads stop
  busy-waiting.  An optional boot warmup JITs kernels then unloads immediately.
- **Adaptive OpenCV threads + CPU affinity**: single-threaded OpenCV while
  idle, wider pool while active; the thread pins itself to configured cores.
- **Zero-copy frame handoff**: pins a ring slot only while motion fallback or
  RKNN reads it; the camera drops rather than overwrites a pinned slot.

There is no software inference fallback. When the RKNN runtime is present but the
model is missing / fails to load, the loop hard-fails with a ``RuntimeError``.
The deterministic no-NPU stub (periodically publishing a center bbox, for unit
tests) is only used when the RKNN runtime is entirely absent *and*
``CAT_FOLLOW_PERCEPTION_ALLOW_STUB`` is explicitly enabled -- in production a
missing runtime is a hard error.

The worker validates its own backend and reports the outcome to the
main-thread supervisor through a :class:`DetectorHandshake`, so a failed
initialization aborts startup instead of silently killing the daemon thread.
Fatal runtime errors (e.g. a failed NPU reload after idle-unload, or repeated
inference failures) set the shared ``stop_event`` so the supervisor tears the
app down rather than running blind.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import numpy as np

from cat_follow.camera_config import load_camera_config
from cat_follow.logger import get_logger
from cat_follow.memory.pool import FRAME_H, FRAME_W, CROP_RING_N
from cat_follow.memory.shared_state import SharedState
from cat_follow.perception.motion_detector import MotionDetector
from cat_follow.perception.phase import Phase, PhaseMachine
from cat_follow.perception.status import update_perception_diagnostics
from cat_follow.perception.tuning import apply_affinity, set_opencv_threads
from cat_follow.perception_config import PerceptionConfig, load_perception_config
from cat_follow.vision.backends import create_backend
from cat_follow.vision.rga_crop import crop_center_bottom_nv12, crop_backend_name
from cat_follow.vision.nv12_utils import (
    nv12_to_bgr,
    y_plane,
)
from cat_follow.vision.zerocopy_backend import ZerocopySession, runtime_available

log = get_logger("thread.detector")

# Default time the supervisor waits for the detector worker to validate the
# backend (model load + one strict inference can take a few seconds on the NPU).
DETECTOR_READY_TIMEOUT_S = 30.0


class DetectorHandshake:
    """Startup + runtime health channel between the detector worker and the
    main-thread supervisor.

    The worker validates the backend *itself* (single initialization, no
    separate preflight double-init) and reports the outcome here.  The
    supervisor blocks on :meth:`wait_ready` and aborts startup if validation
    failed or timed out.  The same object also carries fatal *runtime* errors
    reported after startup.
    """

    def __init__(self) -> None:
        self._ready = threading.Event()
        self._ok = False
        self._error: Optional[BaseException] = None
        self.stub_mode = False

    def set_ready(self, *, stub_mode: bool = False) -> None:
        self.stub_mode = stub_mode
        self._ok = True
        self._ready.set()

    def set_failed(self, exc: BaseException) -> None:
        self._error = exc
        self._ok = False
        self._ready.set()

    @property
    def error(self) -> Optional[BaseException]:
        return self._error

    def wait_ready(self, timeout: float = DETECTOR_READY_TIMEOUT_S) -> bool:
        """Block until the worker reports readiness.

        Returns ``True`` when a real NPU backend is ready, ``False`` when the
        worker is running the dev/CI stub.  Raises ``RuntimeError`` on
        validation failure or timeout.
        """
        if not self._ready.wait(timeout):
            raise RuntimeError(
                f"detector worker did not report readiness within {timeout:.0f}s"
            )
        if not self._ok:
            raise RuntimeError(
                f"detector worker failed to start: {self._error}"
            ) from self._error
        return not self.stub_mode


class DetectorFatalHook:
    """Settable escalation channel from the detector worker to the supervisor.

    The perception threads are created before the motor/FSM exist, so the
    supervisor wires the real handler (motor e-stop + FAILSAFE + app stop)
    later via :meth:`set_handler`.  A fatal error fired before the handler is
    wired is buffered and delivered as soon as the handler is set, so an
    escalation can never be dropped.
    """

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


def _resolve_model_path(config: PerceptionConfig, explicit: Optional[str]) -> str:
    """The RKNN model path (env-driven), or an explicit override for tests."""
    return explicit if explicit is not None else config.rknn_model_path


def _center_bottom_crop(
    frame_nv12: np.ndarray,
    input_size: tuple[int, int],
    *,
    dst: np.ndarray,
    src_w: int = FRAME_W,
    src_h: int = FRAME_H,
) -> tuple[np.ndarray, int, int]:
    """Extract a model-sized center-bottom packed NV12 crop."""
    crop_w, crop_h = input_size
    return crop_center_bottom_nv12(
        frame_nv12,
        src_w,
        src_h,
        crop_w,
        crop_h,
        dst=dst,
    )


def build_validated_backend(
    config: PerceptionConfig, model_path: Optional[str] = None
):
    """Build and validate the RKNN backend, applying the no-fallback policy.

    - RKNN runtime absent + ``allow_stub``       -> return ``None`` (the caller
      runs the deterministic stub; dev/CI only).
    - RKNN runtime absent + not ``allow_stub``   -> raise ``RuntimeError``
      (production: a missing/broken rknnlite must NOT masquerade as valid
      detection).
    - runtime present, model loads + passes a strict inference self-test
                                                 -> return the validated backend.
    - runtime present, model missing / corrupt / wrong I/O contract
                                                 -> raise ``RuntimeError``.

    File existence alone is *not* accepted: a strict ``self_test()`` runs one
    real inference and validates the output contract, so corrupt, truncated,
    wrong-target, or wrong-dimension models fail loudly at startup instead of
    silently returning empty detections.  After validation the model is left
    resident when ``warmup_on_start`` is set (first real inference is instant),
    otherwise it is unloaded to free NPU worker threads until first use.
    """
    path = _resolve_model_path(config, model_path)
    b = create_backend(
        path,
        input_size=config.rknn_input_size,
        animal_mode=config.animal_mode,
        input_format=config.rknn_input_format,
    )
    if not b.runtime_available():
        if config.allow_stub:
            log.warning(
                "RKNN runtime not available; using the deterministic stub "
                "because CAT_FOLLOW_PERCEPTION_ALLOW_STUB is set (dev/CI only)."
            )
            return None
        raise RuntimeError(
            "RKNN runtime (rknnlite) not available and the stub is disabled. "
            "Install the NPU runtime on the robot, or set "
            "CAT_FOLLOW_PERCEPTION_ALLOW_STUB=1 for development/CI. Production "
            "must not run the fake-detection stub."
        )
    if not b.available():
        raise RuntimeError(
            f"RKNN model file not found: {path!r}. Set "
            "CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH to a valid .rknn file "
            "(build one with scripts/convert_yolo_to_rknn.py)."
        )
    try:
        b.self_test()
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            f"RKNN model failed strict validation: {path!r} ({exc}). It may be "
            "corrupt, built for a different target, or have an input/output "
            "contract that does not match the YOLOv8n 9-tensor model-zoo head. "
            "Rebuild it with scripts/convert_yolo_to_rknn.py."
        ) from exc
    if not config.warmup_on_start:
        # Release NPU worker threads until the first real inference.
        b.unload()
    return b


def preflight_perception(
    config: Optional[PerceptionConfig] = None, model_path: Optional[str] = None
) -> bool:
    """Standalone one-shot validation of the detection backend.

    The supervisors now rely on the :class:`DetectorHandshake` (the worker
    validates its own backend and reports readiness), so this is no longer on
    the startup path -- it remains a convenience for tooling/tests that want to
    check the backend without starting the loop.

    Returns ``True`` when a usable NPU backend is present, ``False`` when the
    RKNN runtime is entirely absent and the stub is allowed (dev/CI).  Raises
    ``RuntimeError`` when the runtime is present but the model is unusable, or
    when the runtime is absent and the stub is disabled (production).
    """
    config = config or load_perception_config()
    backend = build_validated_backend(config, model_path)
    if backend is not None:
        backend.unload()
        return True
    return False


def run_detector_loop(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    model_path: Optional[str] = None,
    score_threshold: Optional[float] = None,
    target_fps: Optional[float] = None,
    config: Optional[PerceptionConfig] = None,
    handshake: Optional[DetectorHandshake] = None,
    on_fatal: Optional[DetectorFatalHook] = None,
):
    """Run the detector loop until *stop_event* is set.

    When *handshake* is provided, the worker reports its startup validation
    result there (so the supervisor can wait for readiness and abort on
    failure) and escalates fatal runtime errors by setting *stop_event*.

    When *on_fatal* is provided, a fatal error (startup validation failure or a
    runtime escalation) also notifies the supervisor so it can command a motor
    emergency-stop, latch FAILSAFE, and tear the app down -- setting the
    perception ``stop_event`` alone does not stop the control loop.
    """
    config = config or load_perception_config()
    if score_threshold is None:
        score_threshold = config.score_threshold
    if not np.isfinite(score_threshold) or not 0.0 <= score_threshold <= 1.0:
        raise ValueError("score_threshold must be finite and within [0, 1]")
    if target_fps is None:
        target_fps = config.detect_fps
    tick = 1.0 / target_fps
    camera_fps = load_camera_config().fps
    rate_cap_enabled = target_fps < camera_fps * 0.99
    frame_h, frame_w = FRAME_H, FRAME_W
    crop_w, crop_h = config.rknn_input_size
    if crop_w > frame_w or crop_h > frame_h:
        raise ValueError(
            f"RKNN input {crop_w}x{crop_h} exceeds frame {frame_w}x{frame_h}"
        )

    if config.affinity_enabled:
        apply_affinity(config.detector_cores)

    motion = MotionDetector(
        frame_w,
        frame_h,
        scale=config.motion_scale,
        threshold=config.motion_threshold,
        min_area=config.motion_min_area,
    )
    phase = PhaseMachine(
        tracking_interval=config.detect_interval_tracking,
        watch_interval=config.detect_interval_tracking,
    )

    last_detect_s = time.monotonic()
    crop_logged = False
    zerocopy_active = config.effective_zerocopy() == "dmabuf"
    zerocopy_session: ZerocopySession | None = None
    zerocopy_offsets = (0, 0)

    # Validate the backend HERE, in the worker, and report to the supervisor.
    # This is the single point of initialization (no preflight double-init): if
    # it fails, the supervisor waiting on the handshake aborts startup.
    try:
        if zerocopy_active:
            if not runtime_available():
                raise RuntimeError(
                    "CAT_FOLLOW_PERCEPTION_ZEROCOPY=dmabuf but native zerocopy "
                    "runtime is unavailable"
                )
            for _ in range(100):
                zerocopy_session = shared.zerocopy_session()
                if zerocopy_session is not None:
                    break
                if stop_event.wait(0.05):
                    break
            if zerocopy_session is None:
                raise RuntimeError(
                    "zerocopy requested but camera did not attach a session"
                )
            zerocopy_offsets = (
                zerocopy_session.offset_x,
                zerocopy_session.offset_y,
            )
            if not config.warmup_on_start and not zerocopy_session.unload_model():
                raise RuntimeError(
                    "failed to unload native RKNN model after startup "
                    f"validation: {zerocopy_session.last_error}"
                )
            backend = None
            log.info(
                "Detector using zerocopy fd path (crop offset=%s).",
                zerocopy_offsets,
            )
        else:
            backend = build_validated_backend(config, model_path)
    except Exception as exc:  # noqa: BLE001
        log.error("Detector backend validation failed: %s", exc)
        update_perception_diagnostics(
            backend="rknn", model_loaded=False, error=str(exc)
        )
        stop_event.set()
        if on_fatal is not None:
            on_fatal.fire(str(exc))
        if handshake is not None:
            # The supervisor learns of the failure via the handshake and aborts
            # startup; the worker exits quietly rather than re-raising into a
            # dead daemon thread.
            handshake.set_failed(exc)
            return
        raise
    # The production RKNN backend consumes NV12 directly. Allocate BGR scratch
    # only for legacy test doubles/backends that still require that contract.
    direct_nv12_input = bool(
        backend is not None and hasattr(backend, "infer_all_nv12")
    )
    crop_bgr = (
        None
        if backend is None or direct_nv12_input
        else np.empty((crop_h, crop_w, 3), dtype=np.uint8)
    )
    if handshake is not None:
        handshake.set_ready(stub_mode=backend is None and zerocopy_session is None)

    def _escalate(message: str) -> None:
        """Fatal runtime failure: notify the supervisor and stop the app."""
        log.error("Detector fatal error: %s", message)
        update_perception_diagnostics(
            backend="rknn", model_loaded=False, error=message
        )
        if handshake is not None:
            handshake.set_failed(RuntimeError(message))
        # Notify the supervisor to e-stop + FAILSAFE + tear down: setting the
        # perception stop_event alone would not stop the control loop.
        if on_fatal is not None:
            on_fatal.fire(message)
        stop_event.set()

    log.info(
        "Detector loop started (backend=%s, crop=%s, motion_gating=%s, target %.1f FPS).",
        "zerocopy"
        if zerocopy_session is not None
        else ("stub" if backend is None else "rknn"),
        "native-fd" if zerocopy_session is not None else crop_backend_name(),
        config.motion_gating,
        target_fps,
    )

    frame_index = 0
    stub_cycle = 0
    lores_gray = None
    last_frame_gen = 0
    next_run_at = 0.0
    # Latches once the loop proves motion gating has no luma source to read.
    motion_gating_inert = False
    while not stop_event.is_set():
        wake_gen = shared.wait_for_new_frame(
            last_frame_gen, stop_event, timeout_s=0.1
        )
        if stop_event.is_set():
            break
        if wake_gen <= last_frame_gen:
            # The dev/CI stub has no camera producer, so retain a deadline wake
            # solely for synthetic output. Real backends require a new frame.
            if backend is not None or zerocopy_session is not None:
                continue

        # A lower configured detector rate remains a cap, but the wake source
        # is always camera publication. While waiting for the cap deadline,
        # coalesce further camera notifications and process only the newest gen.
        while (
            rate_cap_enabled
            and time.monotonic() < next_run_at
            and not stop_event.is_set()
        ):
            newer_gen = shared.wait_for_new_frame(
                wake_gen,
                stop_event,
                timeout_s=max(0.0, next_run_at - time.monotonic()),
            )
            wake_gen = max(wake_gen, newer_gen)
        if stop_event.is_set():
            break

        last_frame_gen = wake_gen
        t0 = time.monotonic()
        next_run_at = t0 + tick if rate_cap_enabled else 0.0
        frame_index += 1
        tick_lease = None
        lease_request_ns = None
        lease_acquire_ms = 0.0
        published_frame_gen = None

        try:
            warmup_requested = shared.consume_detector_warmup_request()
            # START_CHASE asks this owner thread to load and warm the model. Keeping
            # RKNN ownership here avoids cross-thread runtime calls while ensuring
            # the first chase detection does not pay load/initialization latency.
            if (
                backend is not None
                and warmup_requested
            ):
                warmup_start = time.perf_counter()
                try:
                    backend.self_test()
                    last_detect_s = t0
                    log.info(
                        "[RKNN-WARMUP] chase request ready in %.2f ms",
                        (time.perf_counter() - warmup_start) * 1000.0,
                    )
                except Exception as exc:  # noqa: BLE001
                    _escalate(f"RKNN chase-start warmup failed: {exc}")
                    break

            # Cheap motion gate. Prefer hardware lores. When unavailable, pin one
            # ring slot for the whole tick (motion fallback + optional RKNN).
            if config.motion_gating:
                lores_gray = shared.get_lores_gray(lores_gray)
                if lores_gray is not None:
                    motion_result = motion.detect(lores_gray, gray_input=True)
                    has_motion = motion_result.motion
                else:
                    lease_request_ns = time.monotonic_ns()
                    tick_lease = shared.acquire_latest_frame()
                    lease_acquire_ms = (
                        time.monotonic_ns() - lease_request_ns
                    ) / 1_000_000.0
                    if tick_lease is None:
                        has_motion = False
                    elif tick_lease.frame is not None:
                        motion_result = motion.detect(
                            y_plane(tick_lease.frame, frame_w, frame_h),
                            gray_input=True,
                        )
                        has_motion = motion_result.motion
                    else:
                        # A DMA-BUF descriptor contains no motion evidence.
                        # Require a real lores/luma sample (or disable motion
                        # gating explicitly) instead of treating every fd-only
                        # frame as moving.
                        has_motion = False
                        if not motion_gating_inert:
                            motion_gating_inert = True
                            log.warning(
                                "Motion gating is inert: frames are DMA-BUF "
                                "only and no lores luma stream is available. "
                                "Detection now runs on the phase cadence and "
                                "mission demand only."
                            )
            else:
                has_motion = True
            lores_active = lores_gray is not None

            # Live inject moves a sprite too slowly at the production detect cadence
            # (5 FPS) to reliably exceed the motion area threshold. Force detection
            # while inject is active so end-to-end pipeline tests exercise real RKNN.
            if shared.cat_injection_enabled():
                has_motion = True
                if zerocopy_session is not None:
                    # Inject needs CPU NV12 blend; it is incompatible with the
                    # dmabuf-only detection path. Disable inject rather than
                    # silently falling back to NumPy/RKNNLite.
                    log.error(
                        "Live cat inject is incompatible with ZEROCOPY=dmabuf; "
                        "disabling inject for this run."
                    )
                    shared.set_cat_injection_enabled(False)

            # Decide whether to run the model this tick (phase + cadence).
            # Mission override (SEARCH/CHASE/GOTO+yolo) cannot be suppressed by
            # PhaseMachine IDLE or motion gating.
            detected_last = False
            detector_force_off = False
            detector_required = False
            mission_override = False
            if hasattr(shared, "get_perception_intent"):
                intent = shared.get_perception_intent()
                detector_force_off = bool(intent.get("detector_force_off"))
                detector_required = bool(intent.get("detector_required"))
                mission_override = bool(intent.get("detector_mission_override"))
            if detector_force_off:
                want_detect = False
            elif mission_override or detector_required:
                want_detect = True
            else:
                want_detect = (
                    (not config.motion_gating)
                    or phase.should_detect(frame_index)
                    or has_motion
                )

            if zerocopy_session is not None and warmup_requested:
                # The camera thread is actively dequeuing. Never dequeue here:
                # warm the NPU against a published buffer while its ring lease
                # prevents QBUF/reuse.
                if tick_lease is None:
                    lease_request_ns = time.monotonic_ns()
                    tick_lease = shared.acquire_latest_frame()
                    lease_acquire_ms = (
                        time.monotonic_ns() - lease_request_ns
                    ) / 1_000_000.0
                if (
                    tick_lease is None
                    or tick_lease.stale
                    or tick_lease.dmabuf_buffer_index < 0
                ):
                    shared.request_detector_warmup()
                    log.debug("Deferring zerocopy warmup until a valid frame lease")
                else:
                    warmup_start = time.perf_counter()
                    try:
                        zerocopy_session.warmup(
                            tick_lease.dmabuf_buffer_index,
                            score_threshold=score_threshold,
                        )
                    except Exception as exc:  # noqa: BLE001
                        _escalate(f"zerocopy chase-start warmup failed: {exc}")
                        break
                    last_detect_s = t0
                    # Warmup already ran inference; avoid running the same
                    # leased frame twice. The next camera frame performs the
                    # first published chase detection without reload latency.
                    want_detect = False
                    log.info(
                        "[RKNN-WARMUP] zerocopy chase request ready in %.2f ms",
                        (time.perf_counter() - warmup_start) * 1000.0,
                    )

            det: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)
            if want_detect and (backend is not None or zerocopy_session is not None):
                set_opencv_threads(config.opencv_threads_active)
                if zerocopy_session is None and backend is not None:
                    # Explicit (re)load so a failed reload after idle-unload is treated
                    # as fatal rather than silently running blind.
                    if not backend.loaded and not backend.load():
                        _escalate(
                            "RKNN reload failed after idle unload: "
                            f"{config.rknn_model_path}"
                        )
                        break
                if tick_lease is None:
                    lease_request_ns = time.monotonic_ns()
                    tick_lease = shared.acquire_latest_frame()
                    lease_acquire_ms = (
                        time.monotonic_ns() - lease_request_ns
                    ) / 1_000_000.0
                if tick_lease is None:
                    set_opencv_threads(config.opencv_threads_idle)
                elif tick_lease.stale:
                    log.debug(
                        "Detector lease stale before infer on slot %d; skipping tick",
                        tick_lease.slot_idx,
                    )
                    set_opencv_threads(config.opencv_threads_idle)
                elif (
                    zerocopy_session is not None
                    and tick_lease.dmabuf_buffer_index >= 0
                ):
                    if (
                        not zerocopy_session.model_loaded
                        and not zerocopy_session.load_model()
                    ):
                        _escalate(
                            "zerocopy RKNN reload failed after idle unload: "
                            f"{zerocopy_session.last_error}"
                        )
                        break
                    frame_gen = tick_lease.frame_seq
                    crop_start_ns = time.monotonic_ns()
                    offset_x, offset_y = zerocopy_offsets
                    infer_start_ns = crop_start_ns
                    failures_before = zerocopy_session.consecutive_failures
                    local_detections = zerocopy_session.infer(
                        tick_lease.dmabuf_buffer_index,
                        score_threshold,
                    )
                    infer_done_ns = time.monotonic_ns()
                    crop_done_ns = infer_done_ns
                    convert_done_ns = infer_done_ns
                    failures = zerocopy_session.consecutive_failures
                    if failures >= config.max_infer_failures:
                        _escalate(
                            f"zerocopy infer failed {failures} times "
                            f"({zerocopy_session.last_error})"
                        )
                        break
                    # A failed inference means "no information", not "no cat".
                    # Publishing its empty result would hand the FSM a
                    # confident negative, so let the last observation age out
                    # on freshness instead.
                    publish_results = failures <= failures_before
                    if not publish_results:
                        log.debug(
                            "zerocopy infer error; withholding result: %s",
                            zerocopy_session.last_error,
                        )
                    detections = [
                        (
                            item[0] + offset_x,
                            item[1] + offset_y,
                            item[2] + offset_x,
                            item[3] + offset_y,
                            item[4],
                            item[5],
                        )
                        for item in local_detections
                    ]
                    if detections:
                        best = max(detections, key=lambda item: item[4])
                        det = (
                            float(best[0]),
                            float(best[1]),
                            float(best[2] - best[0]),
                            float(best[3] - best[1]),
                            float(best[4]),
                        )
                    else:
                        det = (0.0, 0.0, 0.0, 0.0, 0.0)
                    detected_last = det[4] > 0
                    if detected_last:
                        last_detect_s = t0
                    result_publish_start_ns = time.monotonic_ns()
                    if publish_results:
                        published_frame_gen = frame_gen
                        shared.set_detector_detections(detections, frame_gen)
                        shared.set_bbox_detector(
                            det[0],
                            det[1],
                            det[2],
                            det[3],
                            det[4],
                            frame_gen=frame_gen,
                        )
                    result_publish_done_ns = time.monotonic_ns()
                    perf = zerocopy_session.last_perf
                    capture_ms = (
                        tick_lease.published_ns - tick_lease.capture_started_ns
                    ) / 1_000_000.0
                    queue_ms = (
                        (lease_request_ns or crop_start_ns) - tick_lease.published_ns
                    ) / 1_000_000.0
                    crop_ms = float(perf.get("pre", 0.0))
                    convert_ms = 0.0
                    infer_call_ms = (
                        infer_done_ns - infer_start_ns
                    ) / 1_000_000.0
                    result_publish_ms = (
                        result_publish_done_ns - result_publish_start_ns
                    ) / 1_000_000.0
                    detector_ms = (
                        result_publish_done_ns
                        - (lease_request_ns or crop_start_ns)
                    ) / 1_000_000.0
                    end_to_end_ms = (
                        result_publish_done_ns - tick_lease.capture_started_ns
                    ) / 1_000_000.0
                    log.info(
                        "[DETECT-PERF] gen=%d capture=%.2fms queue=%.2fms "
                        "lease=%.3fms crop=%.2fms nv12_rgb=%.2fms "
                        "npu=%.2fms post=%.2fms lifecycle_wait=%.2fms "
                        "native=%.2fms infer_call=%.2fms publish=%.2fms "
                        "detector=%.2fms end_to_end=%.2fms path=zerocopy",
                        frame_gen,
                        capture_ms,
                        max(0.0, queue_ms),
                        lease_acquire_ms,
                        crop_ms,
                        convert_ms,
                        float(perf.get("invoke", 0.0)),
                        float(perf.get("post", 0.0)),
                        float(perf.get("lifecycle_wait_ms", 0.0)),
                        float(perf.get("native_ms", 0.0)),
                        infer_call_ms,
                        result_publish_ms,
                        detector_ms,
                        end_to_end_ms,
                    )
                elif backend is not None:
                    frame_gen = tick_lease.frame_seq
                    if tick_lease.frame is None:
                        set_opencv_threads(config.opencv_threads_idle)
                        continue
                    crop_start_ns = time.monotonic_ns()
                    slot_idx = tick_lease.slot_idx % CROP_RING_N
                    crop_buf = shared.crop_buffer_for_slot(slot_idx)
                    crop, offset_x, offset_y = _center_bottom_crop(
                        tick_lease.frame,
                        config.rknn_input_size,
                        dst=crop_buf,
                    )
                    if not crop_logged:
                        log.info(
                            "Detector crop path: %s (Cam[i]->Crop[i], slot=%d)",
                            crop_backend_name(),
                            slot_idx,
                        )
                        crop_logged = True
                    crop_done_ns = time.monotonic_ns()
                    failures_before = getattr(backend, "consecutive_failures", 0)
                    if direct_nv12_input:
                        convert_done_ns = crop_done_ns
                        infer_start_ns = crop_done_ns
                        local_detections = backend.infer_all_nv12(
                            crop, score_threshold
                        )
                        infer_done_ns = time.monotonic_ns()
                    elif hasattr(backend, "infer_all"):
                        assert crop_bgr is not None
                        nv12_to_bgr(crop, crop_w, crop_h, dst=crop_bgr)
                        convert_done_ns = time.monotonic_ns()
                        infer_start_ns = convert_done_ns
                        local_detections = backend.infer_all(
                            crop_bgr, score_threshold
                        )
                        infer_done_ns = time.monotonic_ns()
                    else:
                        assert crop_bgr is not None
                        nv12_to_bgr(crop, crop_w, crop_h, dst=crop_bgr)
                        convert_done_ns = time.monotonic_ns()
                        infer_start_ns = convert_done_ns
                        # Compatibility for simple test doubles.
                        local_det = backend.infer(crop_bgr, score_threshold)
                        infer_done_ns = time.monotonic_ns()
                        det = (
                            local_det[0] + offset_x,
                            local_det[1] + offset_y,
                            local_det[2],
                            local_det[3],
                            local_det[4],
                        )
                        detections = (
                            [(
                                det[0],
                                det[1],
                                det[0] + det[2],
                                det[1] + det[3],
                                float(det[4]),
                                17,
                            )]
                            if det[4] > 0
                            else []
                        )

                    if direct_nv12_input or hasattr(backend, "infer_all"):
                        detections = [
                            (
                                item[0] + offset_x,
                                item[1] + offset_y,
                                item[2] + offset_x,
                                item[3] + offset_y,
                                item[4],
                                item[5],
                            )
                            for item in local_detections
                        ]
                        if detections:
                            best = max(detections, key=lambda item: item[4])
                            det = (
                                float(best[0]),
                                float(best[1]),
                                float(best[2] - best[0]),
                                float(best[3] - best[1]),
                                float(best[4]),
                            )

                    failures = getattr(backend, "consecutive_failures", 0)
                    if failures >= config.max_infer_failures:
                        _escalate(
                            f"RKNN inference failed {failures} times consecutively "
                            f"(last error: {getattr(backend, 'last_error', None)})"
                        )
                        break
                    # See the zerocopy branch: an errored inference must not be
                    # published as a confident "cat absent".
                    publish_results = failures <= failures_before
                    if not publish_results:
                        log.debug(
                            "RKNN infer error; withholding result: %s",
                            getattr(backend, "last_error", None),
                        )
                    detected_last = det[4] > 0
                    if detected_last:
                        last_detect_s = t0
                    result_publish_start_ns = time.monotonic_ns()
                    if publish_results:
                        published_frame_gen = frame_gen
                        shared.set_detector_detections(detections, frame_gen)
                        shared.set_bbox_detector(
                            det[0],
                            det[1],
                            det[2],
                            det[3],
                            det[4],
                            frame_gen=frame_gen,
                        )
                    result_publish_done_ns = time.monotonic_ns()

                    perf = getattr(backend, "last_perf", {})
                    capture_ms = (
                        tick_lease.published_ns
                        - tick_lease.capture_started_ns
                    ) / 1_000_000.0
                    queue_ms = (
                        (lease_request_ns or crop_start_ns)
                        - tick_lease.published_ns
                    ) / 1_000_000.0
                    crop_ms = (crop_done_ns - crop_start_ns) / 1_000_000.0
                    convert_ms = (
                        convert_done_ns - crop_done_ns
                    ) / 1_000_000.0
                    input_ms = convert_ms + float(perf.get("pre", 0.0))
                    infer_call_ms = (
                        infer_done_ns - infer_start_ns
                    ) / 1_000_000.0
                    result_publish_ms = (
                        result_publish_done_ns - result_publish_start_ns
                    ) / 1_000_000.0
                    detector_ms = (
                        result_publish_done_ns
                        - (lease_request_ns or crop_start_ns)
                    ) / 1_000_000.0
                    end_to_end_ms = (
                        result_publish_done_ns
                        - tick_lease.capture_started_ns
                    ) / 1_000_000.0
                    log.info(
                        "[DETECT-PERF] gen=%d capture=%.2fms queue=%.2fms "
                        "lease=%.3fms crop=%.2fms nv12_rgb=%.2fms "
                        "npu=%.2fms post=%.2fms "
                        "infer_call=%.2fms publish=%.3fms detector=%.2fms "
                        "end_to_end=%.2fms",
                        frame_gen,
                        capture_ms,
                        max(0.0, queue_ms),
                        lease_acquire_ms,
                        crop_ms,
                        input_ms,
                        float(perf.get("invoke", 0.0)),
                        float(perf.get("post", 0.0)),
                        infer_call_ms,
                        result_publish_ms,
                        detector_ms,
                        end_to_end_ms,
                    )
            elif backend is not None or zerocopy_session is not None:
                # Not invoking the model this tick; keep the previous bbox.
                set_opencv_threads(config.opencv_threads_idle)
            else:
                # Stub fallback (no backend available at all).
                set_opencv_threads(config.opencv_threads_idle)
                stub_gen = shared.latest_frame_generation()
                if stub_cycle % int(max(1, target_fps)) == 0:
                    w = frame_w // 6
                    h = frame_h // 6
                    x = frame_w // 2 - w // 2
                    y = frame_h // 2 - h // 2
                    published_frame_gen = stub_gen
                    shared.set_bbox_detector(
                        float(x), float(y), float(w), float(h), 1.0, frame_gen=stub_gen
                    )
                    shared.set_detector_detections(
                        [(x, y, x + w, y + h, 1.0, 17)], stub_gen
                    )
                    detected_last = True
                else:
                    published_frame_gen = stub_gen
                    shared.set_bbox_detector(
                        0.0, 0.0, 0.0, 0.0, 0.0, frame_gen=stub_gen
                    )
                    shared.set_detector_detections([], stub_gen)
                stub_cycle += 1

            # Advance the perception phase machine.
            phase.update(now_s=t0, motion=has_motion, detected=detected_last)

            # Clear the sticky detector bbox on IDLE only when this tick actually
            # published detector output (infer/stub). Skipped acquires must not
            # advance tracker-visible generations.
            if (
                phase.phase is Phase.IDLE
                and not detected_last
                and published_frame_gen is not None
            ):
                shared.set_bbox_detector(
                    0.0, 0.0, 0.0, 0.0, 0.0, frame_gen=published_frame_gen
                )
                shared.set_detector_detections([], published_frame_gen)

            # Idle unload: release the interpreter after a quiet period.
            # Never unload while mission policy requires the detector.
            # Force-off (HOME/FAILSAFE) is a definitive "detector must not run",
            # so it releases the NPU immediately: yard motion would otherwise
            # keep the phase machine out of IDLE and pin the model forever.
            can_idle_unload = (
                config.idle_unload_sec > 0
                and not detector_required
                and not mission_override
                and (
                    detector_force_off
                    or (
                        phase.phase is Phase.IDLE
                        and (t0 - last_detect_s) >= config.idle_unload_sec
                    )
                )
            )
            if (
                backend is not None
                and backend.loaded
                and can_idle_unload
            ):
                backend.unload()
                set_opencv_threads(config.opencv_threads_idle)
            elif (
                zerocopy_session is not None
                and zerocopy_session.model_loaded
                and can_idle_unload
            ):
                if not zerocopy_session.unload_model():
                    _escalate(
                        "zerocopy RKNN idle unload failed: "
                        f"{zerocopy_session.last_error}"
                    )
                    break
                set_opencv_threads(config.opencv_threads_idle)

            model_loaded = bool(
                zerocopy_session.model_loaded
                if zerocopy_session is not None
                else (backend is not None and backend.loaded)
            )
            update_perception_diagnostics(
                phase=phase.phase.value,
                backend="zerocopy" if zerocopy_session is not None else "rknn",
                model_loaded=model_loaded,
                lores_active=lores_active,
                motion=has_motion,
                motion_gating=config.motion_gating,
            )
        finally:
            if tick_lease is not None:
                last_frame_gen = max(last_frame_gen, tick_lease.frame_seq)
                tick_lease.release()
