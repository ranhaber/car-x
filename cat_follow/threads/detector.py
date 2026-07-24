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

from cat_follow.logger import get_logger
from cat_follow.memory.pool import FRAME_H, FRAME_W
from cat_follow.memory.shared_state import SharedState
from cat_follow.perception.motion_detector import MotionDetector
from cat_follow.perception.phase import Phase, PhaseMachine
from cat_follow.perception.status import update_perception_diagnostics
from cat_follow.perception.tuning import apply_affinity, set_opencv_threads
from cat_follow.perception_config import PerceptionConfig, load_perception_config
from cat_follow.vision.backends import create_backend
from cat_follow.vision.nv12_utils import (
    center_bottom_nv12_region,
    extract_nv12_crop,
    nv12_shape,
    nv12_to_bgr,
    y_plane,
)

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
    dst: Optional[np.ndarray] = None,
) -> tuple[np.ndarray, int, int]:
    """Extract a model-sized center-bottom packed NV12 crop."""
    crop_w, crop_h = input_size
    if crop_w > FRAME_W or crop_h > FRAME_H:
        raise ValueError(
            f"RKNN input {crop_w}x{crop_h} exceeds frame {FRAME_W}x{FRAME_H}"
        )
    region = center_bottom_nv12_region(FRAME_W, FRAME_H, crop_w, crop_h)
    crop = extract_nv12_crop(
        frame_nv12, FRAME_W, FRAME_H, region, dst=dst
    )
    return crop, region[0], region[1]


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
        path, input_size=config.rknn_input_size, animal_mode=config.animal_mode
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
    crop_nv12 = np.empty(nv12_shape(crop_w, crop_h), dtype=np.uint8)

    # Validate the backend HERE, in the worker, and report to the supervisor.
    # This is the single point of initialization (no preflight double-init): if
    # it fails, the supervisor waiting on the handshake aborts startup.
    try:
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
        handshake.set_ready(stub_mode=backend is None)

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
        "Detector loop started (backend=%s, motion_gating=%s, target %.1f FPS).",
        "stub" if backend is None else "rknn",
        config.motion_gating,
        target_fps,
    )

    frame_index = 0
    stub_cycle = 0
    lores_gray = None
    while not stop_event.is_set():
        t0 = time.monotonic()
        frame_index += 1
        tick_lease = None
        lease_request_ns = None
        lease_acquire_ms = 0.0
        published_frame_gen = None

        try:
            # START_CHASE asks this owner thread to load and warm the model. Keeping
            # RKNN ownership here avoids cross-thread runtime calls while ensuring
            # the first chase detection does not pay load/initialization latency.
            if (
                backend is not None
                and shared.consume_detector_warmup_request()
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
                    else:
                        motion_result = motion.detect(
                            y_plane(tick_lease.frame, frame_w, frame_h),
                            gray_input=True,
                        )
                        has_motion = motion_result.motion
            else:
                has_motion = True
            lores_active = lores_gray is not None

            # Live inject moves a sprite too slowly at the production detect cadence
            # (5 FPS) to reliably exceed the motion area threshold. Force detection
            # while inject is active so end-to-end pipeline tests exercise real RKNN.
            if shared.cat_injection_enabled():
                has_motion = True

            # Decide whether to run the model this tick (phase + cadence).
            detected_last = False
            want_detect = (
                (not config.motion_gating)
                or phase.should_detect(frame_index)
                or has_motion
            )

            det: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)
            if want_detect and backend is not None:
                set_opencv_threads(config.opencv_threads_active)
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
                    # Camera has not published yet, or every slot is pinned.
                    # Do not infer on synthetic pixels or replace prior results.
                    set_opencv_threads(config.opencv_threads_idle)
                elif tick_lease.stale:
                    log.debug(
                        "Detector lease stale before infer on slot %d; skipping tick",
                        tick_lease.slot_idx,
                    )
                    set_opencv_threads(config.opencv_threads_idle)
                else:
                    frame_gen = tick_lease.frame_seq
                    crop_start_ns = time.monotonic_ns()
                    crop, offset_x, offset_y = _center_bottom_crop(
                        tick_lease.frame,
                        config.rknn_input_size,
                        dst=crop_nv12,
                    )
                    crop_done_ns = time.monotonic_ns()
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
                    detected_last = det[4] > 0
                    if detected_last:
                        last_detect_s = t0
                    published_frame_gen = frame_gen
                    result_publish_start_ns = time.monotonic_ns()
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
            elif backend is not None:
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
            if (
                backend is not None
                and backend.loaded
                and config.idle_unload_sec > 0
                and phase.phase is Phase.IDLE
                and (t0 - last_detect_s) >= config.idle_unload_sec
            ):
                backend.unload()
                set_opencv_threads(config.opencv_threads_idle)

            update_perception_diagnostics(
                phase=phase.phase.value,
                backend="rknn",
                model_loaded=bool(backend is not None and backend.loaded),
                lores_active=lores_active,
                motion=has_motion,
                motion_gating=config.motion_gating,
            )
        finally:
            if tick_lease is not None:
                tick_lease.release()

        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, tick - elapsed))
