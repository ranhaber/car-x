"""Environment-driven perception/resource configuration.

Mirrors :mod:`cat_follow.camera_config` but governs the *processing* side of
the perception pipeline: the RK3576 NPU (RKNN) detection model, how
aggressively to gate the detector behind motion, model lifecycle (lazy load +
idle unload), OpenCV thread parallelism, CPU affinity, and the optional
hardware-scaled lores stream device.

All settings are read once from ``CAT_FOLLOW_PERCEPTION_*`` variables so the
same code runs unchanged on a laptop (defaults) and on the ROCK 4D (env file).
The defaults preserve the previous behaviour as closely as possible so the
system stays runnable if nothing is configured.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from typing import Tuple


_PREFIX = "CAT_FOLLOW_PERCEPTION_"


def _raw(name: str) -> str | None:
    value = os.getenv(f"{_PREFIX}{name}")
    if value is None:
        return None
    value = value.strip()
    return value or None


def _int(name: str, default: int, *, minimum: int | None = None) -> int:
    raw = _raw(name)
    if raw is None:
        return default
    value = int(raw)
    if minimum is not None and value < minimum:
        raise ValueError(f"{_PREFIX}{name} must be >= {minimum}")
    return value


def _float(name: str, default: float, *, minimum: float | None = None) -> float:
    raw = _raw(name)
    if raw is None:
        return default
    value = float(raw)
    if minimum is not None and value < minimum:
        raise ValueError(f"{_PREFIX}{name} must be >= {minimum}")
    return value


def _bool(name: str, default: bool) -> bool:
    raw = _raw(name)
    if raw is None:
        return default
    return raw.lower() in {"1", "true", "yes", "on"}


def _str(name: str, default: str) -> str:
    raw = _raw(name)
    return default if raw is None else raw


def _core_set(name: str, default: Tuple[int, ...]) -> Tuple[int, ...]:
    """Parse a comma-separated CPU core list, e.g. ``"1,2,3"``."""
    raw = _raw(name)
    if raw is None:
        return default
    cores = []
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        core = int(token)
        if core < 0:
            raise ValueError(f"{_PREFIX}{name} core indexes must be >= 0")
        cores.append(core)
    return tuple(cores)


def _size_pair(name: str, default: Tuple[int, int]) -> Tuple[int, int]:
    """Parse a ``"W,H"`` (or ``"N"`` for square) model input size."""
    raw = _raw(name)
    if raw is None:
        return default
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    if len(parts) == 1:
        n = int(parts[0])
        parts = [parts[0], parts[0]]
    if len(parts) != 2:
        raise ValueError(f"{_PREFIX}{name} must be 'W,H' or 'N'")
    w, h = int(parts[0]), int(parts[1])
    if w < 1 or h < 1:
        raise ValueError(f"{_PREFIX}{name} dimensions must be >= 1")
    return (w, h)


@dataclass(frozen=True)
class PerceptionConfig:
    """Processing-side settings loaded once when perception threads start.

    Detection runs exclusively on the RK3576 NPU via RKNN; there is no
    software inference fallback.
    """

    # Motion gating: only invoke the detector when motion is present, and at a
    # reduced cadence while a lock is held.  Disable to always run the model.
    motion_gating: bool = True
    motion_scale: float = 0.35
    motion_threshold: int = 25
    motion_min_area: int = 500

    # Detector cadence.  ``detect_interval_tracking`` is how many tracker
    # frames elapse between detector invocations once a target is locked.
    detect_fps: float = 5.0
    detect_interval_tracking: int = 2
    score_threshold: float = 0.5

    # Model lifecycle. Unload RKNN after this many idle seconds (0 disables).
    # Startup always validates once; keeping it resident is opt-in. An accepted
    # START_CHASE asynchronously loads/warms it on the detector owner thread.
    idle_unload_sec: float = 10.0
    warmup_on_start: bool = False

    # Allow the deterministic no-NPU stub. This must be explicitly enabled for
    # development / CI; in production (default False) a missing RKNN runtime is
    # a hard error so a broken/uninstalled rknnlite never masquerades as valid
    # detection.
    allow_stub: bool = False

    # Runtime health: escalate (stop the app) after this many consecutive NPU
    # inference failures so a wedged/failed-reload NPU cannot silently return
    # empty detections forever.
    max_infer_failures: int = 15

    # OpenCV parallelism: single-threaded when idle, wider when tracking.
    opencv_threads_idle: int = 1
    opencv_threads_active: int = 4

    # CPU affinity (Linux only).  Pin the camera/motion producer and the
    # detector consumer to different cores on the RK3576's 8 cores.
    affinity_enabled: bool = False
    camera_cores: Tuple[int, ...] = field(default_factory=tuple)
    detector_cores: Tuple[int, ...] = field(default_factory=tuple)

    # RKNN model path and input geometry (W, H) for the NPU backend.  The input
    # size must match the converted .rknn (YOLOv8n COCO 320x320 for rk3576).
    rknn_model_path: str = "models/yolov8n_coco_320_rk3576.rknn"
    rknn_input_size: Tuple[int, int] = (320, 320)

    def __post_init__(self) -> None:
        if not math.isfinite(self.score_threshold) or not (
            0.0 <= self.score_threshold <= 1.0
        ):
            raise ValueError("score_threshold must be finite and within [0, 1]")


def load_perception_config() -> PerceptionConfig:
    """Load perception settings from ``CAT_FOLLOW_PERCEPTION_*`` variables."""
    return PerceptionConfig(
        motion_gating=_bool("MOTION_GATING", True),
        motion_scale=_float("MOTION_SCALE", 0.35, minimum=0.05),
        motion_threshold=_int("MOTION_THRESHOLD", 25, minimum=1),
        motion_min_area=_int("MOTION_MIN_AREA", 500, minimum=1),
        detect_fps=_float("DETECT_FPS", 5.0, minimum=0.1),
        detect_interval_tracking=_int("DETECT_INTERVAL_TRACKING", 2, minimum=1),
        score_threshold=_float("SCORE_THRESHOLD", 0.5),
        idle_unload_sec=_float("IDLE_UNLOAD_SEC", 10.0, minimum=0.0),
        warmup_on_start=_bool("WARMUP_ON_START", False),
        allow_stub=_bool("ALLOW_STUB", False),
        max_infer_failures=_int("MAX_INFER_FAILURES", 15, minimum=1),
        opencv_threads_idle=_int("OPENCV_THREADS_IDLE", 1, minimum=1),
        opencv_threads_active=_int("OPENCV_THREADS_ACTIVE", 4, minimum=1),
        affinity_enabled=_bool("AFFINITY_ENABLED", False),
        camera_cores=_core_set("CAMERA_CORES", ()),
        detector_cores=_core_set("DETECTOR_CORES", ()),
        rknn_model_path=_str(
            "RKNN_MODEL_PATH", "models/yolov8n_coco_320_rk3576.rknn"
        ),
        rknn_input_size=_size_pair("RKNN_INPUT", (320, 320)),
    )
