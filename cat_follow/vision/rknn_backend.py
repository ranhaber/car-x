"""RK3576 NPU detection backend (RKNN).

Implements the same interface as :class:`cat_follow.vision.backends.TFLiteBackend`
so the detector thread is backend-agnostic.  It uses ``rknnlite`` (the
RKNN-Toolkit-Lite2 runtime) which ships as ``from rknnlite.api import RKNNLite``
on Rockchip vendor images.

The backend is selected with ``CAT_FOLLOW_PERCEPTION_BACKEND=rknn`` and a
``CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH`` pointing at a converted ``.rknn``
model.  If the runtime or the model file is missing, ``available()`` returns
False and :func:`cat_follow.vision.backends.create_backend` transparently
falls back to the CPU TFLite backend.

Model conversion (run once on a workstation with rknn-toolkit2)::

    from rknn.api import RKNN
    rknn = RKNN()
    rknn.config(target_platform="rk3576",
                mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]])
    rknn.load_tflite(model="models/ssd_mobilenet_v2_320x320.tflite")
    rknn.build(do_quantization=True, dataset="dataset.txt")
    rknn.export_rknn("models/ssd_mobilenet_v2.rknn")

See ``cat_follow/docs/Software_Integration_*.md`` for the full NPU milestone.
"""

from __future__ import annotations

import os
from typing import Optional, Tuple

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.perception.memory import reclaim_memory
from cat_follow.vision.tflite_common import parse_tflite_outputs

log = get_logger("vision.rknn")

try:
    from rknnlite.api import RKNNLite  # type: ignore

    _HAS_RKNN = True
except Exception:  # pragma: no cover - only present on Rockchip images
    RKNNLite = None  # type: ignore
    _HAS_RKNN = False

Detection = Tuple[float, float, float, float, float]

# Default model input geometry (SSD MobileNet family). Overridable via env.
_DEFAULT_INPUT = (300, 300)


class RknnBackend:
    """NPU-accelerated detection backend using RKNNLite."""

    def __init__(
        self,
        model_path: str,
        *,
        input_size: Tuple[int, int] = _DEFAULT_INPUT,
    ) -> None:
        self._model_path = model_path
        self._in_h, self._in_w = input_size
        self._rknn = None
        self._cv2 = None

    @property
    def loaded(self) -> bool:
        return self._rknn is not None

    def available(self) -> bool:
        return _HAS_RKNN and os.path.exists(self._model_path)

    def load(self) -> bool:
        if self._rknn is not None:
            return True
        if not self.available():
            return False
        rknn = RKNNLite()
        if rknn.load_rknn(self._model_path) != 0:
            log.warning("RKNN load_rknn failed: %s", self._model_path)
            return False
        # NPU_CORE_AUTO lets the runtime pick a free core on the RK3576.
        core_mask = getattr(RKNNLite, "NPU_CORE_AUTO", 0)
        if rknn.init_runtime(core_mask=core_mask) != 0:
            log.warning("RKNN init_runtime failed: %s", self._model_path)
            rknn.release()
            return False
        self._rknn = rknn
        log.info("RKNN backend loaded on NPU: %s", self._model_path)
        return True

    def unload(self) -> None:
        if self._rknn is None:
            return
        try:
            self._rknn.release()
        except Exception:  # noqa: BLE001
            pass
        self._rknn = None
        reclaim_memory()
        log.info("RKNN backend unloaded: %s", self._model_path)

    def warmup(self) -> None:
        if not self.load():
            return
        try:
            dummy = np.zeros((self._in_h, self._in_w, 3), dtype=np.uint8)
            self._rknn.inference(inputs=[np.expand_dims(dummy, 0)])
        except Exception as exc:  # noqa: BLE001
            log.debug("RKNN warmup inference skipped: %s", exc)
        finally:
            self.unload()

    def infer(self, frame_bgr: np.ndarray, score_threshold: float) -> Detection:
        if self._rknn is None and not self.load():
            return (0.0, 0.0, 0.0, 0.0, 0.0)
        try:
            cv2 = self._cv()
            frame_h, frame_w = frame_bgr.shape[0], frame_bgr.shape[1]
            resized = cv2.resize(
                frame_bgr, (self._in_w, self._in_h), interpolation=cv2.INTER_LINEAR
            )
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            outputs = self._rknn.inference(inputs=[np.expand_dims(rgb, 0)])
            return parse_tflite_outputs(outputs, frame_h, frame_w, score_threshold)
        except Exception as exc:  # noqa: BLE001
            log.warning("RKNN inference failed: %s", exc)
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    def _cv(self):
        if self._cv2 is None:
            import cv2

            self._cv2 = cv2
        return self._cv2


__all__ = ["RknnBackend"]
