"""Pluggable detection backends with lazy load and idle unload.

Both the CPU (`TFLiteBackend`) and NPU (`RknnBackend`, see
:mod:`cat_follow.vision.rknn_backend`) implementations share the same small
interface so the detector thread is backend-agnostic:

- ``loaded``      -> whether the model is currently resident
- ``load()``      -> create the interpreter (returns True on success)
- ``unload()``    -> release the interpreter + worker threads, reclaim memory
- ``warmup()``    -> load, run one dummy inference, unload (warms page cache /
                     JITs kernels without holding the model resident)
- ``infer(bgr)``  -> ``(x, y, w, h, valid)`` in full-frame pixels

Keeping the model *unloaded* while idle matters because TFLite's XNNPACK
worker threads busy-wait even with no work queued, burning a core; unloading
stops them.  The same applies to the RKNN runtime context.
"""

from __future__ import annotations

import os
from typing import Optional, Protocol, Tuple

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.perception.memory import reclaim_memory
from cat_follow.vision.tflite_common import (
    has_tflite,
    make_interpreter,
    parse_tflite_outputs,
)

log = get_logger("vision.backend")

Detection = Tuple[float, float, float, float, float]


class DetectionBackend(Protocol):
    """Common interface implemented by every detection backend."""

    @property
    def loaded(self) -> bool: ...

    def available(self) -> bool: ...

    def load(self) -> bool: ...

    def unload(self) -> None: ...

    def warmup(self) -> None: ...

    def infer(self, frame_bgr: np.ndarray, score_threshold: float) -> Detection: ...


class TFLiteBackend:
    """CPU TFLite backend (tflite_runtime or tensorflow.lite)."""

    def __init__(self, model_path: str) -> None:
        self._model_path = model_path
        self._interp = None
        self._input_index: Optional[int] = None
        self._input_shape = None
        self._cv2 = None

    @property
    def loaded(self) -> bool:
        return self._interp is not None

    def available(self) -> bool:
        """True only when TFLite is importable *and* the model file exists."""
        return has_tflite() and os.path.exists(self._model_path)

    def load(self) -> bool:
        if self._interp is not None:
            return True
        interp = make_interpreter(self._model_path)
        if interp is None:
            return False
        idet = interp.get_input_details()[0]
        self._interp = interp
        self._input_index = idet["index"]
        self._input_shape = idet["shape"]
        log.info("TFLite backend loaded: %s", self._model_path)
        return True

    def unload(self) -> None:
        if self._interp is None:
            return
        self._interp = None
        self._input_index = None
        self._input_shape = None
        reclaim_memory()
        log.info("TFLite backend unloaded: %s", self._model_path)

    def warmup(self) -> None:
        if not self.load():
            return
        try:
            in_h, in_w = self._input_hw()
            dummy = np.zeros((in_h, in_w, 3), dtype=np.uint8)
            self._invoke(dummy)
        except Exception as exc:  # noqa: BLE001
            log.debug("TFLite warmup inference skipped: %s", exc)
        finally:
            self.unload()

    def infer(self, frame_bgr: np.ndarray, score_threshold: float) -> Detection:
        if self._interp is None and not self.load():
            return (0.0, 0.0, 0.0, 0.0, 0.0)
        try:
            frame_h, frame_w = frame_bgr.shape[0], frame_bgr.shape[1]
            outputs = self._invoke(frame_bgr)
            return parse_tflite_outputs(outputs, frame_h, frame_w, score_threshold)
        except Exception as exc:  # noqa: BLE001
            log.warning("TFLite inference failed: %s", exc)
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    # ── internals ────────────────────────────────────────────────────

    def _cv(self):
        if self._cv2 is None:
            import cv2

            self._cv2 = cv2
        return self._cv2

    def _input_hw(self) -> Tuple[int, int]:
        shape = self._input_shape
        if shape is not None and len(shape) >= 3:
            return int(shape[1]), int(shape[2])
        return 300, 300

    def _invoke(self, frame_bgr: np.ndarray):
        cv2 = self._cv()
        in_h, in_w = self._input_hw()
        if frame_bgr.shape[0] != in_h or frame_bgr.shape[1] != in_w:
            resized = cv2.resize(
                frame_bgr, (in_w, in_h), interpolation=cv2.INTER_LINEAR
            )
        else:
            resized = frame_bgr
        img = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        try:
            self._interp.set_tensor(self._input_index, np.expand_dims(img, axis=0))
        except Exception:
            arr = np.expand_dims(img.astype(np.float32) / 255.0, axis=0)
            self._interp.set_tensor(self._input_index, arr)
        self._interp.invoke()
        return [
            self._interp.get_tensor(o["index"])
            for o in self._interp.get_output_details()
        ]


def create_backend(model_path: str, *, backend: str = "tflite") -> DetectionBackend:
    """Return a detection backend, falling back to TFLite when RKNN is absent."""
    if backend == "rknn":
        try:
            from cat_follow.vision.rknn_backend import RknnBackend

            rknn = RknnBackend(model_path)
            if rknn.available():
                return rknn
            log.warning(
                "RKNN backend requested but unavailable; falling back to TFLite"
            )
        except Exception as exc:  # noqa: BLE001
            log.warning("RKNN backend import failed (%s); using TFLite", exc)
    return TFLiteBackend(model_path)


__all__ = ["DetectionBackend", "TFLiteBackend", "create_backend", "Detection"]
