"""Detector thread (motion-gated, lazy-loaded, backend-pluggable).

Resource-optimization design (ported from cat_ball_tracker and scaled for the
ROCK 4D):

- **Motion gating**: a cheap frame-difference motion detector runs every tick;
  the expensive model is only invoked when motion is present (or at a reduced
  cadence while a target is locked), driven by a small perception phase FSM.
- **Lazy load + idle unload**: the detection backend (CPU TFLite or RK3576
  NPU) is loaded on first need and unloaded after an idle period so its worker
  threads stop busy-waiting.  An optional boot warmup JITs kernels then
  unloads immediately.
- **Adaptive OpenCV threads + CPU affinity**: single-threaded OpenCV while
  idle, wider pool while active; the thread pins itself to configured cores.
- **Zero per-frame allocation**: reuses the pre-allocated detector snapshot
  buffer and the motion detector's internal buffers.

If no model / OpenCV / TFLite is available the loop falls back to a
deterministic stub useful for unit tests (periodically publishing a center
bbox), preserving the previous behaviour.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.memory.pool import FRAME_SHAPE
from cat_follow.memory.shared_state import SharedState
from cat_follow.perception.motion_detector import MotionDetector
from cat_follow.perception.phase import Phase, PhaseMachine
from cat_follow.perception.tuning import apply_affinity, set_opencv_threads
from cat_follow.perception_config import PerceptionConfig, load_perception_config
from cat_follow.vision.backends import create_backend

log = get_logger("thread.detector")

# Logical model keys (used by the web UI) -> filesystem paths.
MODEL_MAP = {
    "ssd_mobilenet_v2": "models/ssd_mobilenet_v2_320x320.tflite",
    "efficientdet_lite0": "models/efficientdet_lite0.tflite",
}


def _resolve_model_path(
    choice: Optional[str], config: PerceptionConfig, explicit: Optional[str]
) -> str:
    if config.uses_rknn:
        return config.rknn_model_path
    if explicit is not None:
        return explicit
    return MODEL_MAP.get(choice or "ssd_mobilenet_v2", MODEL_MAP["ssd_mobilenet_v2"])


def run_detector_loop(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    model_path: Optional[str] = None,
    score_threshold: float = 0.5,
    target_fps: Optional[float] = None,
    config: Optional[PerceptionConfig] = None,
):
    """Run the detector loop until *stop_event* is set."""
    config = config or load_perception_config()
    if target_fps is None:
        target_fps = config.detect_fps
    tick = 1.0 / target_fps
    frame_h, frame_w = FRAME_SHAPE[0], FRAME_SHAPE[1]
    tmp = np.empty(FRAME_SHAPE, dtype=np.uint8)

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

    last_choice: Optional[str] = None
    backend = None
    last_detect_s = time.monotonic()

    def _make_backend(choice: Optional[str]):
        path = _resolve_model_path(choice, config, model_path)
        b = create_backend(path, backend=config.backend)
        if not b.available():
            # No usable model/runtime -> run the deterministic stub instead.
            return None
        if config.warmup_on_start:
            b.warmup()
        return b

    log.info(
        "Detector loop started (backend=%s, motion_gating=%s, target %.1f FPS).",
        config.backend,
        config.motion_gating,
        target_fps,
    )

    frame_index = 0
    stub_cycle = 0
    while not stop_event.is_set():
        t0 = time.monotonic()
        frame_index += 1

        # Refresh the stable detector snapshot from the latest camera frame so
        # detection works headless even without a driving main loop.
        shared.copy_latest_to_detector_frame()
        shared.get_frame_for_detector(tmp)

        # Handle UI-driven model changes.
        try:
            choice = shared.get_detector_model()
        except Exception:
            choice = None
        choice = choice or "ssd_mobilenet_v2"
        if choice != last_choice:
            if backend is not None:
                backend.unload()
            backend = _make_backend(choice)
            last_choice = choice

        # Cheap motion gate. Prefer the hardware-scaled lores gray frame when
        # the camera publishes one (zero software downscale); otherwise fall
        # back to the full detector snapshot.
        if config.motion_gating:
            lores_gray = shared.get_lores_gray()
            if lores_gray is not None:
                motion_result = motion.detect(lores_gray, gray_input=True)
            else:
                motion_result = motion.detect(tmp)
            has_motion = motion_result.motion
        else:
            has_motion = True

        # Decide whether to run the model this tick (phase + cadence).
        detected_last = False
        want_detect = (not config.motion_gating) or phase.should_detect(frame_index) or has_motion

        det: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)
        if want_detect and backend is not None:
            set_opencv_threads(config.opencv_threads_active)
            det = backend.infer(tmp, score_threshold)
            detected_last = det[4] > 0
            if detected_last:
                last_detect_s = t0
            shared.set_bbox_detector(det[0], det[1], det[2], det[3], det[4])
        elif backend is not None:
            # Not invoking the model this tick; keep the previous bbox.
            set_opencv_threads(config.opencv_threads_idle)
        else:
            # Stub fallback (no backend available at all).
            set_opencv_threads(config.opencv_threads_idle)
            if stub_cycle % int(max(1, target_fps)) == 0:
                w = frame_w // 6
                h = frame_h // 6
                x = frame_w // 2 - w // 2
                y = frame_h // 2 - h // 2
                shared.set_bbox_detector(float(x), float(y), float(w), float(h), 1.0)
                detected_last = True
            else:
                shared.set_bbox_detector(0.0, 0.0, 0.0, 0.0, 0.0)
            stub_cycle += 1

        # Advance the perception phase machine.
        phase.update(now_s=t0, motion=has_motion, detected=detected_last)

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

        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, tick - elapsed))
