"""
Shared TFLite inference helpers for cat detection.

Used by vision.detector.get_cat_bbox() and threads.detector.run_detector_loop.
"""

from typing import Any, List, Optional, Tuple

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


def has_tflite() -> bool:
    """Return True if TFLite (tflite_runtime or tensorflow.lite) is available."""
    return _HAS_TFLITE


def make_interpreter(model_path: str) -> Optional[Any]:
    """Create and allocate a TFLite interpreter for the given model path. Returns None on failure."""
    if not _HAS_TFLITE:
        return None
    try:
        interp = _TFLiteInterpreter(model_path)
        interp.allocate_tensors()
        return interp
    except Exception:
        return None


def parse_tflite_outputs(
    outputs: List[Any],
    frame_h: int,
    frame_w: int,
    score_thresh: float = 0.5,
) -> Tuple[float, float, float, float, float]:
    """
    Parse TFLite detection outputs to (x, y, w, h, valid).
    valid is 1.0 if a detection above score_thresh was found, else 0.0.
    """
    # SSD-style: boxes [1,N,4] (ymin,xmin,ymax,xmax) normalized, scores [1,N]
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
            a0, a1, a2, a3 = arr.tolist()
            if max(arr) <= 1.01:
                xmin = int(a1 * frame_w)
                ymin = int(a0 * frame_h)
                xmax = int(a3 * frame_w)
                ymax = int(a2 * frame_h)
                w = max(0, xmax - xmin)
                h = max(0, ymax - ymin)
                return (float(xmin), float(ymin), float(w), float(h), 1.0)
            else:
                xmin = int(min(a0, a2))
                ymin = int(min(a1, a3))
                xmax = int(max(a0, a2))
                ymax = int(max(a1, a3))
                w = max(0, xmax - xmin)
                h = max(0, ymax - ymin)
                return (float(xmin), float(ymin), float(w), float(h), 1.0)
    return (0.0, 0.0, 0.0, 0.0, 0.0)
