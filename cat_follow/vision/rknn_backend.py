"""YOLOv8n COCO RK3576 NPU detection backend.

Implements :class:`cat_follow.vision.backends.DetectionBackend` so the detector
thread is backend-agnostic.  It uses ``rknnlite`` (the RKNN-Toolkit-Lite2
runtime) which ships as ``from rknnlite.api import RKNNLite`` on Rockchip
vendor images.

The model path is configured with ``CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH``
(pointing at a converted ``.rknn`` model) and the input geometry with
``CAT_FOLLOW_PERCEPTION_RKNN_INPUT`` (e.g. ``320,320``).

Input format is selected with ``CAT_FOLLOW_PERCEPTION_RKNN_INPUT_FORMAT``:

- ``rgb`` (default): RGB uint8 path (CPU NV12→RGB before inference).
- ``nv12``: packed 320×320 NV12 crop fed directly to the NPU tensor.

A model filename may declare its layout with an ``_rgb`` or ``_nv12`` token.
Requesting ``nv12`` for a model that declares neither is rejected, because
feeding an RGB model an NV12 tensor produces plausible-looking but wrong
detections rather than an error. Set
``CAT_FOLLOW_PERCEPTION_RKNN_ALLOW_UNTAGGED_NV12=1`` to override during model
bring-up.

Failure policy (there is intentionally no software inference fallback):

- ``runtime_available()`` -> whether ``rknnlite`` is importable at all.  On the
  ROCK 4D this is True; on a dev laptop / CI it is False.
- ``available()``          -> runtime importable *and* the ``.rknn`` model file
  exists.
- The detector thread hard-fails when the runtime is present but the model is
  missing / fails to load, and only runs a deterministic stub when the runtime
  is entirely absent (dev/CI).

See ``cat_follow/docs/Software_Integration_*.md`` for the full NPU milestone.
"""

from __future__ import annotations

import os
import time
from typing import Literal, Optional, Tuple

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.perception.memory import reclaim_memory
from cat_follow.perception.timing import perf_ms_from
from cat_follow.vision.nv12_utils import nv12_shape, nv12_to_rgb, validate_nv12
from cat_follow.vision.yolo_postprocess import (
    ANIMAL_CLASS_IDS_0IDX,
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

InputFormat = Literal["nv12", "rgb"]

# Default model input geometry (W, H) for YOLOv8n COCO 320x320. Must match
# the converted .rknn; override with
# CAT_FOLLOW_PERCEPTION_RKNN_INPUT when using a different model.
_DEFAULT_INPUT = (320, 320)


def _normalize_input_format(value: str) -> InputFormat:
    fmt = (value or "rgb").strip().lower()
    if fmt not in ("nv12", "rgb"):
        raise ValueError(
            f"RKNN input format must be 'nv12' or 'rgb', got {value!r}"
        )
    return fmt  # type: ignore[return-value]


def model_format_token(model_path: str) -> Optional[InputFormat]:
    """Return the input format declared by the model filename, if it declares one."""
    name = os.path.basename(model_path).lower()
    if "_nv12" in name:
        return "nv12"
    if "_rgb" in name:
        return "rgb"
    return None


def infer_input_format_from_model_path(model_path: str) -> InputFormat:
    """Guess NV12 vs RGB from the model filename, defaulting to RGB."""
    return model_format_token(model_path) or "rgb"


def allow_untagged_nv12() -> bool:
    """Whether an operator opted into NV12 on a model that declares no layout."""
    raw = os.getenv("CAT_FOLLOW_PERCEPTION_RKNN_ALLOW_UNTAGGED_NV12", "")
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def resolve_input_format(
    model_path: str, requested: Optional[str]
) -> InputFormat:
    """Resolve the input layout, refusing combinations that fail silently.

    A filename token is ground truth when present: it must match an explicit
    request. Requesting NV12 for an untagged model is refused unless the
    operator sets the override, because a wrong layout yields wrong boxes
    instead of an error.
    """
    declared = model_format_token(model_path)
    if requested is None:
        return declared or "rgb"
    fmt = _normalize_input_format(requested)
    if declared is not None and declared != fmt:
        raise ValueError(
            f"RKNN input format {fmt!r} contradicts the model filename "
            f"{os.path.basename(model_path)!r}, which declares {declared!r}"
        )
    if declared is None and fmt == "nv12" and not allow_untagged_nv12():
        raise ValueError(
            f"RKNN input format 'nv12' requested for "
            f"{os.path.basename(model_path)!r}, which declares no layout. "
            "Rename the model with an _nv12 token or set "
            "CAT_FOLLOW_PERCEPTION_RKNN_ALLOW_UNTAGGED_NV12=1."
        )
    return fmt


class RknnBackend:
    """NPU-accelerated detection backend using RKNNLite."""

    def __init__(
        self,
        model_path: str,
        *,
        input_size: Tuple[int, int] = _DEFAULT_INPUT,
        animal_mode: bool = False,
        input_format: InputFormat | None = None,
    ) -> None:
        self._model_path = model_path
        self._in_w, self._in_h = input_size
        self._input_format: InputFormat = resolve_input_format(
            model_path, input_format or None
        )
        self._animal_class_ids_0idx = (
            ANIMAL_CLASS_IDS_0IDX if animal_mode else frozenset()
        )
        self._rknn = None
        self._cv2 = None
        self._input_buf: Optional[np.ndarray] = None
        self.last_perf: dict[str, float] = {}
        self.consecutive_failures = 0
        self.last_error: Optional[str] = None

    @property
    def input_format(self) -> InputFormat:
        return self._input_format

    @property
    def loaded(self) -> bool:
        return self._rknn is not None

    def runtime_available(self) -> bool:
        return _HAS_RKNN

    def _cv(self):
        """Load OpenCV lazily so importing the backend stays CI-safe."""
        if self._cv2 is None:
            import cv2

            self._cv2 = cv2
        return self._cv2

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
        core_mask = getattr(RKNNLite, "NPU_CORE_AUTO", 0)
        if rknn.init_runtime(core_mask=core_mask) != 0:
            log.warning("RKNN init_runtime failed: %s", self._model_path)
            rknn.release()
            return False
        self._rknn = rknn
        log.info(
            "RKNN backend loaded on NPU: %s (input=%s)",
            self._model_path,
            self._input_format,
        )
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
        if not self.load():
            raise RuntimeError(f"RKNN model failed to load/init: {self._model_path}")
        # Probe the loaded model with a dummy tensor in the configured layout.
        # RKNNLite exposes no input-attribute query, so a rejected inference is
        # the only ground-truth signal that the layout is wrong; without this
        # the filename/env guard is the sole protection.
        try:
            if self._input_format == "nv12":
                dummy = np.zeros(
                    nv12_shape(self._in_w, self._in_h), dtype=np.uint8
                )
                outputs, _meta = self._raw_infer_nv12_native(dummy)
            else:
                dummy = np.zeros((self._in_h, self._in_w, 3), dtype=np.uint8)
                outputs, _meta = self._raw_infer(dummy)
        except Exception as exc:
            raise RuntimeError(
                f"RKNN model {self._model_path} rejected a "
                f"{self._input_format!r} {self._in_w}x{self._in_h} input "
                f"({exc}); the configured input format most likely does not "
                "match the converted model"
            ) from exc
        validate_yolo_output_contract(outputs)

    def infer_all(
        self, frame_bgr: np.ndarray, score_threshold: float
    ) -> list[MultiDetection]:
        return self._infer_all(frame_bgr, score_threshold, self._raw_infer)

    def infer_all_nv12(
        self, frame_nv12: np.ndarray, score_threshold: float
    ) -> list[MultiDetection]:
        if self._input_format == "nv12":
            return self._infer_all(
                frame_nv12, score_threshold, self._raw_infer_nv12_native
            )
        return self._infer_all(
            frame_nv12, score_threshold, self._raw_infer_nv12_rgb
        )

    def _infer_all(
        self, frame: np.ndarray, score_threshold: float, raw_infer
    ) -> list[MultiDetection]:
        self.last_perf = {}
        if self._rknn is None and not self.load():
            self.consecutive_failures += 1
            self.last_error = f"RKNN (re)load failed: {self._model_path}"
            log.warning(
                "%s (consecutive failures=%d)",
                self.last_error,
                self.consecutive_failures,
            )
            return []
        try:
            outputs, meta = raw_infer(frame)
            frame_w, frame_h, scale, pad_x, pad_y = meta
            post_start = time.perf_counter()
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
                animal_class_ids_0idx=self._animal_class_ids_0idx,
            )
            post_ms = (time.perf_counter() - post_start) * 1000.0
            self.last_perf["post"] = post_ms
            self.last_perf["total"] = (
                self.last_perf.get("pre", 0.0)
                + self.last_perf.get("invoke", 0.0)
                + post_ms
            )
            log.debug(
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
            log.warning(
                "RKNN inference failed: %s (consecutive failures=%d)",
                exc,
                self.consecutive_failures,
            )
            return []

    def infer(self, frame_bgr: np.ndarray, score_threshold: float) -> Detection:
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

    def _ensure_input_buf(self) -> np.ndarray:
        if self._input_buf is None:
            if self._input_format == "nv12":
                packed_shape = nv12_shape(self._in_w, self._in_h)
                self._input_buf = np.empty(
                    (1, packed_shape[0], packed_shape[1], 1), dtype=np.uint8
                )
            else:
                self._input_buf = np.empty(
                    (1, self._in_h, self._in_w, 3), dtype=np.uint8
                )
        return self._input_buf

    def _raw_infer(self, frame_bgr: np.ndarray):
        if self._rknn is None and not self.load():
            raise RuntimeError(f"RKNN backend not loaded: {self._model_path}")
        cv2 = self._cv()
        frame_h, frame_w = frame_bgr.shape[:2]
        t0 = time.perf_counter()
        buf = self._ensure_input_buf()

        if frame_w == self._in_w and frame_h == self._in_h:
            scale, pad_x, pad_y = 1.0, 0, 0
            cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB, dst=buf[0])
        else:
            scale = min(self._in_w / frame_w, self._in_h / frame_h)
            new_w = max(1, int(round(frame_w * scale)))
            new_h = max(1, int(round(frame_h * scale)))
            pad_x = (self._in_w - new_w) // 2
            pad_y = (self._in_h - new_h) // 2
            buf.fill(114)
            resized = cv2.resize(
                frame_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR
            )
            cv2.cvtColor(
                resized,
                cv2.COLOR_BGR2RGB,
                dst=buf[0, pad_y : pad_y + new_h, pad_x : pad_x + new_w],
            )
        t_invoke = time.perf_counter()
        outputs = self._rknn.inference(inputs=[buf])
        done = time.perf_counter()
        self.last_perf = {
            "t_ms": perf_ms_from(t0),
            "pre": (t_invoke - t0) * 1000.0,
            "invoke": (done - t_invoke) * 1000.0,
        }
        return outputs, (frame_w, frame_h, scale, pad_x, pad_y)

    def _raw_infer_nv12_native(self, frame_nv12: np.ndarray):
        """Feed packed NV12 directly to the NPU input tensor."""
        if self._rknn is None and not self.load():
            raise RuntimeError(f"RKNN backend not loaded: {self._model_path}")
        t0 = time.perf_counter()
        buf = self._ensure_input_buf()
        packed = validate_nv12(frame_nv12, self._in_w, self._in_h)
        np.copyto(buf[0, :, :, 0], packed)
        t_invoke = time.perf_counter()
        outputs = self._rknn.inference(inputs=[buf])
        done = time.perf_counter()
        self.last_perf = {
            "t_ms": perf_ms_from(t0),
            "pre": (t_invoke - t0) * 1000.0,
            "invoke": (done - t_invoke) * 1000.0,
        }
        return outputs, (self._in_w, self._in_h, 1.0, 0, 0)

    def _raw_infer_nv12_rgb(self, frame_nv12: np.ndarray):
        """Legacy RGB model path: CPU NV12→RGB before inference."""
        if self._rknn is None and not self.load():
            raise RuntimeError(f"RKNN backend not loaded: {self._model_path}")
        t0 = time.perf_counter()
        buf = self._ensure_input_buf()
        if buf.shape[-1] != 3:
            self._input_buf = np.empty(
                (1, self._in_h, self._in_w, 3), dtype=np.uint8
            )
            buf = self._input_buf
        nv12_to_rgb(
            frame_nv12,
            self._in_w,
            self._in_h,
            dst=buf[0],
        )
        t_invoke = time.perf_counter()
        outputs = self._rknn.inference(inputs=[buf])
        done = time.perf_counter()
        self.last_perf = {
            "t_ms": perf_ms_from(t0),
            "pre": (t_invoke - t0) * 1000.0,
            "invoke": (done - t_invoke) * 1000.0,
        }
        return outputs, (self._in_w, self._in_h, 1.0, 0, 0)


__all__ = [
    "RknnBackend",
    "InputFormat",
    "infer_input_format_from_model_path",
    "model_format_token",
    "_normalize_input_format",
]
