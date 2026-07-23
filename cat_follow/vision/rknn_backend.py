"""YOLOv8n COCO RK3576 NPU detection backend.

Implements :class:`cat_follow.vision.backends.DetectionBackend` so the detector
thread is backend-agnostic.  It uses ``rknnlite`` (the RKNN-Toolkit-Lite2
runtime) which ships as ``from rknnlite.api import RKNNLite`` on Rockchip
vendor images.

The model path is configured with ``CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH``
(pointing at a converted ``.rknn`` model) and the input geometry with
``CAT_FOLLOW_PERCEPTION_RKNN_INPUT`` (e.g. ``320,320``).

Failure policy (there is intentionally no software inference fallback):

- ``runtime_available()`` -> whether ``rknnlite`` is importable at all.  On the
  ROCK 4D this is True; on a dev laptop / CI it is False.
- ``available()``          -> runtime importable *and* the ``.rknn`` model file
  exists.
- The detector thread hard-fails when the runtime is present but the model is
  missing / fails to load, and only runs a deterministic stub when the runtime
  is entirely absent (dev/CI).

The model is the airockchip nine-output model-zoo graph. It receives RGB uint8;
the RKNN graph applies its baked ``mean=0/std=255`` transform. ``infer_all``
returns every post-NMS cat for multi-target tracking; ``infer`` retains the
legacy best-box contract for compatibility.

See ``cat_follow/docs/Software_Integration_*.md`` for the full NPU milestone.
"""

from __future__ import annotations

import os
import time
from typing import Optional, Tuple

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.perception.memory import reclaim_memory
from cat_follow.perception.timing import perf_ms_from
from cat_follow.vision.yolo_postprocess import (
    decode_yolov8_outputs,
    validate_yolo_output_contract,
)

log = get_logger("vision.rknn")

try:
    from rknnlite.api import RKNNLite  # type: ignore

    _HAS_RKNN = True
except Exception:  # pragma: no cover - only present on Rockchip images
    RKNNLite = None  # type: ignore
    _HAS_RKNN = False

Detection = Tuple[float, float, float, float, float]
MultiDetection = Tuple[int, int, int, int, float, int]

# Default model input geometry (W, H) for YOLOv8n COCO 320x320. Must match
# the converted .rknn; override with
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
        self._input_buf: Optional[np.ndarray] = None
        self.last_perf: dict[str, float] = {}
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
        self._input_buf = None
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
        outputs, _meta = self._raw_infer(dummy)
        validate_yolo_output_contract(outputs)

    def infer_all(
        self, frame_bgr: np.ndarray, score_threshold: float
    ) -> list[MultiDetection]:
        """Return all post-NMS cats as ``(x1, y1, x2, y2, conf, class_id)``."""
        # A failed (re)load is a failure, not "no detection": record it so the
        # detector can escalate rather than run blind after an idle unload.
        if self._rknn is None and not self.load():
            self.consecutive_failures += 1
            self.last_error = f"RKNN (re)load failed: {self._model_path}"
            log.warning("%s (consecutive failures=%d)",
                        self.last_error, self.consecutive_failures)
            return []
        try:
            post_start = time.perf_counter()
            outputs, meta = self._raw_infer(frame_bgr)
            frame_w, frame_h, scale, pad_x, pad_y = meta
            detections = decode_yolov8_outputs(
                outputs,
                input_w=self._in_w,
                input_h=self._in_h,
                frame_w=frame_w,
                frame_h=frame_h,
                scale=scale,
                pad_x=pad_x,
                pad_y=pad_y,
                score_threshold=score_threshold,
                target_class_ids=frozenset({17}),
            )
            post_ms = (time.perf_counter() - post_start) * 1000.0
            self.last_perf["post"] = post_ms
            self.last_perf["total"] = (
                self.last_perf.get("pre", 0.0)
                + self.last_perf.get("invoke", 0.0)
                + post_ms
            )
            log.info(
                "[RKNN-PERF] t=%.2fms pre=%.2f inv=%.2f post=%.2f total=%.2f",
                self.last_perf.get("t_ms", 0.0),
                self.last_perf.get("pre", 0.0),
                self.last_perf.get("invoke", 0.0),
                post_ms,
                self.last_perf["total"],
            )
            self.consecutive_failures = 0
            self.last_error = None
            return detections
        except Exception as exc:  # noqa: BLE001
            self.consecutive_failures += 1
            self.last_error = str(exc)
            log.warning("RKNN inference failed: %s (consecutive failures=%d)",
                        exc, self.consecutive_failures)
            return []

    def infer(self, frame_bgr: np.ndarray, score_threshold: float) -> Detection:
        """Return the highest-confidence cat in the legacy ``xywhv`` format."""
        detections = self.infer_all(frame_bgr, score_threshold)
        if not detections:
            return (0.0, 0.0, 0.0, 0.0, 0.0)
        x1, y1, x2, y2, _confidence, _class_id = max(
            detections, key=lambda item: item[4]
        )
        return (
            float(x1),
            float(y1),
            float(x2 - x1),
            float(y2 - y1),
            1.0,
        )

    def _raw_infer(self, frame_bgr: np.ndarray):
        """Preprocess + run inference, returning raw output tensors.

        Does NOT suppress exceptions; used by :meth:`self_test` and the strict
        benchmark path. Uses aspect-preserving letterbox padding and RGB uint8.
        """
        if self._rknn is None and not self.load():
            raise RuntimeError(f"RKNN backend not loaded: {self._model_path}")
        cv2 = self._cv()
        frame_h, frame_w = frame_bgr.shape[:2]
        t0 = time.perf_counter()
        if self._input_buf is None:
            self._input_buf = np.empty(
                (1, self._in_h, self._in_w, 3), dtype=np.uint8
            )

        if frame_w == self._in_w and frame_h == self._in_h:
            scale, pad_x, pad_y = 1.0, 0, 0
            cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB, dst=self._input_buf[0])
        else:
            scale = min(self._in_w / frame_w, self._in_h / frame_h)
            new_w = max(1, int(round(frame_w * scale)))
            new_h = max(1, int(round(frame_h * scale)))
            pad_x = (self._in_w - new_w) // 2
            pad_y = (self._in_h - new_h) // 2
            self._input_buf.fill(114)
            resized = cv2.resize(
                frame_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR
            )
            cv2.cvtColor(
                resized,
                cv2.COLOR_BGR2RGB,
                dst=self._input_buf[
                    0, pad_y : pad_y + new_h, pad_x : pad_x + new_w
                ],
            )
        t_invoke = time.perf_counter()
        outputs = self._rknn.inference(inputs=[self._input_buf])
        done = time.perf_counter()
        self.last_perf = {
            "t_ms": perf_ms_from(t0),
            "pre": (t_invoke - t0) * 1000.0,
            "invoke": (done - t_invoke) * 1000.0,
        }
        return outputs, (frame_w, frame_h, scale, pad_x, pad_y)

    def _cv(self):
        if self._cv2 is None:
            import cv2

            self._cv2 = cv2
        return self._cv2


__all__ = ["RknnBackend"]
