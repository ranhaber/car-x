"""
Cat detector: returns best cat bbox or None.

Uses TFLite when an image is provided and a model is available; otherwise returns None.
"""

import os
from typing import Optional, Tuple

import numpy as np

from cat_follow.vision.tflite_common import (
    has_tflite,
    make_interpreter,
    parse_tflite_outputs,
)

# Default model path (relative to project root or cwd)
_DEFAULT_MODEL = "models/ssd_mobilenet_v2_320x320.tflite"
_interpreter = None
_input_index = None
_input_shape = None


def _get_interpreter(model_path: Optional[str] = None):
    """Lazy-load TFLite interpreter. Returns (interp, input_index, input_shape) or (None, None, None)."""
    global _interpreter, _input_index, _input_shape
    path = model_path or _DEFAULT_MODEL
    if not os.path.isabs(path):
        path = os.path.abspath(path)
    if _interpreter is not None and getattr(_get_interpreter, "_path", None) == path:
        return _interpreter, _input_index, _input_shape
    if not has_tflite():
        return None, None, None
    interp = make_interpreter(path)
    if interp is None:
        return None, None, None
    _get_interpreter._path = path
    _interpreter = interp
    idet = interp.get_input_details()[0]
    _input_index = idet["index"]
    _input_shape = idet["shape"]
    return _interpreter, _input_index, _input_shape


def get_cat_bbox(
    image=None,
    image_width: int = 640,
    image_height: int = 480,
    model_path: Optional[str] = None,
    score_threshold: float = 0.5,
) -> Optional[Tuple[float, float, float, float]]:
    """
    Return (x, y, w, h) in pixels for the best detection in the image, or None.

    Uses TFLite when image is provided and a model is available. If image is None
    or TFLite is not available, returns None.
    """
    if image is None:
        return None
    try:
        import cv2
    except ImportError:
        return None
    interp, input_index, input_shape = _get_interpreter(model_path)
    if interp is None or input_index is None:
        return None
    if not hasattr(image, "shape") or len(image.shape) != 3:
        return None
    frame_h, frame_w = image.shape[0], image.shape[1]
    in_h = int(input_shape[1]) if input_shape is not None and len(input_shape) >= 3 else frame_h
    in_w = int(input_shape[2]) if input_shape is not None and len(input_shape) >= 3 else frame_w
    resized = cv2.resize(image, (in_w, in_h), interpolation=cv2.INTER_LINEAR)
    img = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    try:
        interp.set_tensor(input_index, np.expand_dims(img, axis=0))
    except Exception:
        arr = np.expand_dims(img.astype(np.float32) / 255.0, axis=0)
        interp.set_tensor(input_index, arr)
    interp.invoke()
    outputs = [interp.get_tensor(o["index"]) for o in interp.get_output_details()]
    x, y, w, h, valid = parse_tflite_outputs(outputs, frame_h, frame_w, score_threshold)
    if valid <= 0:
        return None
    return (x, y, w, h)
