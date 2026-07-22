"""RK3576 NPU detection backend (RKNN) -- the project's sole detection backend.

Implements :class:`cat_follow.vision.backends.DetectionBackend` so the detector
thread is backend-agnostic.  It uses ``rknnlite`` (the RKNN-Toolkit-Lite2
runtime) which ships as ``from rknnlite.api import RKNNLite`` on Rockchip
vendor images.

The model path is configured with ``CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH``
(pointing at a converted ``.rknn`` model) and the input geometry with
``CAT_FOLLOW_PERCEPTION_RKNN_INPUT`` (e.g. ``320,320``).

Failure policy (there is intentionally NO CPU/TFLite fallback):

- ``runtime_available()`` -> whether ``rknnlite`` is importable at all.  On the
  ROCK 4D this is True; on a dev laptop / CI it is False.
- ``available()``          -> runtime importable *and* the ``.rknn`` model file
  exists.
- The detector thread hard-fails when the runtime is present but the model is
  missing / fails to load, and only runs a deterministic stub when the runtime
  is entirely absent (dev/CI).

The runtime feeds raw ``uint8`` NHWC RGB (no normalization), so the quantized
model must be converted with a pass-through input transform (``mean=0/std=1``)
to avoid double normalization -- see ``scripts/convert_to_rknn.py``.

See ``cat_follow/docs/Software_Integration_*.md`` for the full NPU milestone.
"""

from __future__ import annotations

import os
from typing import Optional, Tuple

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.perception.memory import reclaim_memory
from cat_follow.vision.ssd_postprocess import (
    parse_ssd_outputs,
    validate_ssd_output_contract,
)

log = get_logger("vision.rknn")

try:
    from rknnlite.api import RKNNLite  # type: ignore

    _HAS_RKNN = True
except Exception:  # pragma: no cover - only present on Rockchip images
    RKNNLite = None  # type: ignore
    _HAS_RKNN = False

Detection = Tuple[float, float, float, float, float]

# Default model input geometry (W, H) for the documented SSD MobileNet V2
# 320x320 model.  Must match the converted .rknn; override with
# CAT_FOLLOW_PERCEPTION_RKNN_INPUT when using a different model.
_DEFAULT_INPUT = (320, 320)


class RknnBackend:
    """NPU-accelerated detection backend using RKNNLite."""

    def __init__(
        self,
        model_path: str,
        *,
        input_size: Tuple[int, int] = _DEFAULT_INPUT,
    ) -> None:
        self._model_path = model_path
        self._in_w, self._in_h = input_size
        self._rknn = None
        self._cv2 = None
        # Runtime health: consecutive inference failures and the last error so
        # the detector can escalate instead of silently returning empty results.
        self.consecutive_failures = 0
        self.last_error: Optional[str] = None

    @property
    def loaded(self) -> bool:
        return self._rknn is not None

    def runtime_available(self) -> bool:
        """True when the RKNN runtime (``rknnlite``) is importable.

        Independent of whether the model file exists.  Used by the detector to
        distinguish "we are on the NPU platform" (a missing model is a hard
        error) from "dev/CI machine" (fall back to the deterministic stub).
        """
        return _HAS_RKNN

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

    def self_test(self) -> None:
        """Strict startup validation. Loads, runs ONE real inference on a dummy
        frame, and validates the output contract -- raising on any failure.

        This deliberately does NOT suppress errors (unlike :meth:`infer`): it
        catches wrong input dimensions, incompatible models, and undecoded
        output layouts at preflight instead of silently returning empty
        detections at runtime. Leaves the model loaded (kernels warmed).
        """
        if not self.load():
            raise RuntimeError(f"RKNN model failed to load/init: {self._model_path}")
        dummy = np.zeros((self._in_h, self._in_w, 3), dtype=np.uint8)
        outputs = self._raw_infer(dummy)
        validate_ssd_output_contract(outputs)

    def infer(self, frame_bgr: np.ndarray, score_threshold: float) -> Detection:
        # A failed (re)load is a failure, not "no detection": record it so the
        # detector can escalate rather than run blind after an idle unload.
        if self._rknn is None and not self.load():
            self.consecutive_failures += 1
            self.last_error = f"RKNN (re)load failed: {self._model_path}"
            log.warning("%s (consecutive failures=%d)",
                        self.last_error, self.consecutive_failures)
            return (0.0, 0.0, 0.0, 0.0, 0.0)
        try:
            frame_h, frame_w = frame_bgr.shape[0], frame_bgr.shape[1]
            outputs = self._raw_infer(frame_bgr)
            result = parse_ssd_outputs(outputs, frame_h, frame_w, score_threshold)
            self.consecutive_failures = 0
            self.last_error = None
            return result
        except Exception as exc:  # noqa: BLE001
            self.consecutive_failures += 1
            self.last_error = str(exc)
            log.warning("RKNN inference failed: %s (consecutive failures=%d)",
                        exc, self.consecutive_failures)
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    def _raw_infer(self, frame_bgr: np.ndarray):
        """Preprocess + run inference, returning raw output tensors.

        Does NOT suppress exceptions; used by :meth:`self_test` and the strict
        benchmark path.  Feeds raw uint8 NHWC RGB (no normalization) to match
        the quantized model's input contract.
        """
        if self._rknn is None and not self.load():
            raise RuntimeError(f"RKNN backend not loaded: {self._model_path}")
        cv2 = self._cv()
        resized = cv2.resize(
            frame_bgr, (self._in_w, self._in_h), interpolation=cv2.INTER_LINEAR
        )
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        return self._rknn.inference(inputs=[np.expand_dims(rgb, 0)])

    def _cv(self):
        if self._cv2 is None:
            import cv2

            self._cv2 = cv2
        return self._cv2


__all__ = ["RknnBackend"]
