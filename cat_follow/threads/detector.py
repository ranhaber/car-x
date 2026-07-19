"""Detector thread (TFLite-capable).

Attempts to load a TFLite interpreter from `tflite_runtime` or `tensorflow`.
If *model_path* is None or the interpreter can't be created, the loop falls
back to a deterministic stub useful for tests.

The detector reads the stable snapshot `frame_for_detector` (via
`SharedState.get_frame_for_detector`) and writes the best detection into
`SharedState.set_bbox_detector(x,y,w,h,valid)`.
"""

import threading
import time
import logging
from typing import Optional

import numpy as np

import cv2

from cat_follow.logger import get_logger
from cat_follow.memory.shared_state import SharedState
from cat_follow.memory.pool import FRAME_SHAPE
from cat_follow.vision.tflite_common import make_interpreter, parse_tflite_outputs

log = get_logger("thread.detector")


def run_detector_loop(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    model_path: Optional[str] = None,
    score_threshold: float = 0.5,
    target_fps: float = 5.0,
):
    """Run detector loop until *stop_event* set.

    If *model_path* is None the loop uses a deterministic stub useful for
    unit tests (periodically publishing a center bbox).
    """
    tick = 1.0 / target_fps
    frame_h, frame_w = FRAME_SHAPE[0], FRAME_SHAPE[1]
    tmp = np.empty(FRAME_SHAPE, dtype=np.uint8)

    interp = None
    input_shape = None
    input_index = None

    # Map logical model keys (used by the UI) to filesystem paths under
    # a `models/` directory. Files may be absent; detector will fall back
    # to stub behavior if the interpreter cannot be created.
    MODEL_MAP = {
        "ssd_mobilenet_v2": "models/ssd_mobilenet_v2_320x320.tflite",
        "efficientdet_lite0": "models/efficientdet_lite0.tflite",
    }

    # Last chosen model key; if it changes we attempt to reload the interpreter
    last_choice = None

    # If the caller supplied an explicit model_path, prefer that initially
    if model_path is not None:
        interp = make_interpreter(model_path)
        if interp is None:
            log.warning("Failed to create TFLite interpreter for %s", model_path)
        else:
            idet = interp.get_input_details()[0]
            input_index = idet["index"]
            input_shape = idet["shape"]
            log.info("TFLite detector loaded: %s", model_path)

    log.info("Detector loop started (target %.1f FPS). model=%s", target_fps, str(model_path))

    stub_cycle = 0
    while not stop_event.is_set():
        t0 = time.monotonic()

        # Read the stable snapshot built by main thread
        shared.get_frame_for_detector(tmp)

        # Check UI-selected model and reload interpreter if selection changed
        try:
            choice = shared.get_detector_model()
        except Exception:
            choice = None
        if choice is None:
            choice = "ssd_mobilenet_v2"

        if choice != last_choice:
            # Attempt to load the interpreter for the new choice
            mp = MODEL_MAP.get(choice)
            if mp is not None:
                new_interp = make_interpreter(mp)
                if new_interp is not None:
                    interp = new_interp
                    idet = interp.get_input_details()[0]
                    input_index = idet["index"]
                    input_shape = idet["shape"]
                    log.info("Loaded detector model '%s' -> %s", choice, mp)
                else:
                    log.warning("Requested model '%s' not available: %s", choice, mp)
                    interp = None
            else:
                log.warning("Unknown detector model key requested: %s", choice)
                interp = None
            last_choice = choice

        if interp is not None:
            try:
                # Preprocess: resize to model input
                in_h = int(input_shape[1]) if input_shape is not None and input_shape.shape[0] >= 3 else frame_h
                in_w = int(input_shape[2]) if input_shape is not None and input_shape.shape[0] >= 3 else frame_w
                resized = cv2.resize(tmp, (in_w, in_h), interpolation=cv2.INTER_LINEAR)
                # Convert BGR->RGB if model expects RGB (common)
                img = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                # Many TFLite models expect uint8 or float32; try to set directly
                try:
                    interp.set_tensor(input_index, np.expand_dims(img, axis=0))
                except Exception:
                    # try float32 normalization
                    arr = np.expand_dims(img.astype(np.float32) / 255.0, axis=0)
                    interp.set_tensor(input_index, arr)

                interp.invoke()
                outputs = [interp.get_tensor(o["index"]) for o in interp.get_output_details()]
                det = parse_tflite_outputs(outputs, frame_h, frame_w, score_threshold)
                shared.set_bbox_detector(det[0], det[1], det[2], det[3], det[4])
            except Exception as e:
                log.warning("Detector inference failed: %s", e)
                shared.set_bbox_detector(0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            # stub: every second publish a center bbox, otherwise invalid
            if stub_cycle % int(max(1, target_fps)) == 0:
                cx = frame_w // 2
                cy = frame_h // 2
                w = frame_w // 6
                h = frame_h // 6
                x = cx - w // 2
                y = cy - h // 2
                shared.set_bbox_detector(float(x), float(y), float(w), float(h), 1.0)
            else:
                shared.set_bbox_detector(0.0, 0.0, 0.0, 0.0, 0.0)
            stub_cycle += 1

        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, tick - elapsed))
