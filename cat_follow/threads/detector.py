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

try:
    from tflite_runtime.interpreter import Interpreter as _TFLiteInterpreter
    _HAS_TFLITE = True
except Exception:
    try:
        from tensorflow.lite import Interpreter as _TFLiteInterpreter
        _HAS_TFLITE = True
    except Exception:
        _HAS_TFLITE = False

import cv2

from cat_follow.logger import get_logger
from cat_follow.memory.shared_state import SharedState
from cat_follow.memory.pool import FRAME_SHAPE

log = get_logger("thread.detector")


def _make_interpreter(model_path: str):
    if not _HAS_TFLITE:
        return None
    try:
        interp = _TFLiteInterpreter(model_path)
        interp.allocate_tensors()
        return interp
    except Exception:
        return None


def _parse_tflite_outputs(outputs, frame_h, frame_w, score_thresh: float = 0.5):
    # Try common SSD-style outputs: boxes, classes, scores, num
    # boxes: [1, N, 4] (ymin, xmin, ymax, xmax) normalized
    if len(outputs) >= 4:
        boxes = outputs[0]
        scores = outputs[2]
        if isinstance(boxes, np.ndarray):
            boxes = np.squeeze(boxes)
        if isinstance(scores, np.ndarray):
            scores = np.squeeze(scores)
        if boxes.ndim == 2 and scores.ndim == 1:
            best_idx = int(np.argmax(scores))
            if float(scores[best_idx]) >= score_thresh:
                bymin, bxmin, bymax, bxmax = boxes[best_idx]
                # normalized -> pixel coords
                xmin = int(bxmin * frame_w)
                ymin = int(bymin * frame_h)
                xmax = int(bxmax * frame_w)
                ymax = int(bymax * frame_h)
                w = max(0, xmax - xmin)
                h = max(0, ymax - ymin)
                return (float(xmin), float(ymin), float(w), float(h), 1.0)
    # Fallback: single-box output length 4
    for out in outputs:
        arr = np.array(out).squeeze()
        if arr.size == 4:
            # assume either normalized or pixel coords
            a0, a1, a2, a3 = arr.tolist()
            if max(arr) <= 1.01:
                # normalized ymin,xmin,ymax,xmax
                xmin = int(a1 * frame_w)
                ymin = int(a0 * frame_h)
                xmax = int(a3 * frame_w)
                ymax = int(a2 * frame_h)
                w = max(0, xmax - xmin)
                h = max(0, ymax - ymin)
                return (float(xmin), float(ymin), float(w), float(h), 1.0)
            else:
                # already pixels, convert to x,y,w,h
                xmin = int(min(a0, a2))
                ymin = int(min(a1, a3))
                xmax = int(max(a0, a2))
                ymax = int(max(a1, a3))
                w = max(0, xmax - xmin)
                h = max(0, ymax - ymin)
                return (float(xmin), float(ymin), float(w), float(h), 1.0)
    return (0.0, 0.0, 0.0, 0.0, 0.0)


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
        interp = _make_interpreter(model_path)
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
                new_interp = _make_interpreter(mp)
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
                det = _parse_tflite_outputs(outputs, frame_h, frame_w, score_threshold)
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
"""
Detector thread (stub).

Reads ``frame_for_detector`` every K iterations and writes a stub bbox
into ``bbox_detector``.  Replace with real TFLite inference later; the
interface stays the same.
"""

import threading
import time

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.memory.shared_state import SharedState
from cat_follow.memory.pool import FRAME_SHAPE

log = get_logger("thread.detector")


def run_detector_loop(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    target_fps: float = 30.0,
    detect_every_k: int = 10,
) -> None:
    """Detector loop — runs until *stop_event* is set.

    **Stub behaviour:**  Every *detect_every_k* iterations, reads
    ``frame_for_detector`` and writes a fixed bbox
    ``(120, 120, 60, 60, 1.0)`` into ``bbox_detector``.  On other
    iterations it just sleeps (simulating the tracker handling those
    frames).

    Parameters
    ----------
    shared : SharedState
        Thread-safe wrapper around the pre-allocated memory pool.
    stop_event : threading.Event
        Set this to signal the loop to exit.
    target_fps : float
        Desired frames per second (default 30).
    detect_every_k : int
        Run "detection" every K-th iteration (default 10).
    """
    tick = 1.0 / target_fps
    # Pre-allocate a buffer to receive the detector frame (no per-frame alloc)
    frame_buf = np.empty(FRAME_SHAPE, dtype=np.uint8)
    iteration = 0
    log.info("Detector loop started (stub, every %d frames).", detect_every_k)

    while not stop_event.is_set():
        t0 = time.monotonic()

        if iteration % detect_every_k == 0:
            # Read the detector-dedicated frame snapshot
            shared.get_frame_for_detector(frame_buf)

            # Stub: always report a fixed bbox with valid=1
            shared.set_bbox_detector(120.0, 120.0, 60.0, 60.0, 1.0)

        iteration += 1
        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, tick - elapsed))
