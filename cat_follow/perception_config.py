"""Environment-driven perception/resource configuration.

Mirrors :mod:`cat_follow.camera_config` but governs the *processing* side of
the perception pipeline: which detection backend to use (CPU TFLite or the
RK3576 NPU via RKNN), how aggressively to gate the detector behind motion,
model lifecycle (lazy load + idle unload), OpenCV thread parallelism, CPU
affinity, and the optional hardware-scaled lores stream device.

All settings are read once from ``CAT_FOLLOW_PERCEPTION_*`` variables so the
same code runs unchanged on a laptop (defaults) and on the ROCK 4D (env file).
The defaults preserve the previous behaviour as closely as possible so the
system stays runnable if nothing is configured.
"""

from __future__ import annotations

from dataclasses import dataclass, field
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


@dataclass(frozen=True)
class PerceptionConfig:
    """Processing-side settings loaded once when perception threads start."""

    # Detection backend: "tflite" (CPU) or "rknn" (RK3576 NPU).
    backend: str = "tflite"

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

    # Model lifecycle.  Unload the interpreter after this many idle seconds to
    # release the XNNPACK / NPU worker threads (0 disables idle unloading).
    idle_unload_sec: float = 10.0
    warmup_on_start: bool = True

    # OpenCV parallelism: single-threaded when idle, wider when tracking.
    opencv_threads_idle: int = 1
    opencv_threads_active: int = 4

    # CPU affinity (Linux only).  Pin the camera/motion producer and the
    # detector consumer to different cores on the RK3576's 8 cores.
    affinity_enabled: bool = False
    camera_cores: Tuple[int, ...] = field(default_factory=tuple)
    detector_cores: Tuple[int, ...] = field(default_factory=tuple)

    # RKNN model path (used only when backend == "rknn").
    rknn_model_path: str = "models/ssd_mobilenet_v2.rknn"

    @property
    def uses_rknn(self) -> bool:
        return self.backend == "rknn"


def load_perception_config() -> PerceptionConfig:
    """Load perception settings from ``CAT_FOLLOW_PERCEPTION_*`` variables."""
    backend = _str("BACKEND", "tflite").lower()
    if backend not in {"tflite", "rknn"}:
        raise ValueError(
            f"{_PREFIX}BACKEND must be 'tflite' or 'rknn', got {backend!r}"
        )

    return PerceptionConfig(
        backend=backend,
        motion_gating=_bool("MOTION_GATING", True),
        motion_scale=_float("MOTION_SCALE", 0.35, minimum=0.05),
        motion_threshold=_int("MOTION_THRESHOLD", 25, minimum=1),
        motion_min_area=_int("MOTION_MIN_AREA", 500, minimum=1),
        detect_fps=_float("DETECT_FPS", 5.0, minimum=0.1),
        detect_interval_tracking=_int("DETECT_INTERVAL_TRACKING", 2, minimum=1),
        idle_unload_sec=_float("IDLE_UNLOAD_SEC", 10.0, minimum=0.0),
        warmup_on_start=_bool("WARMUP_ON_START", True),
        opencv_threads_idle=_int("OPENCV_THREADS_IDLE", 1, minimum=1),
        opencv_threads_active=_int("OPENCV_THREADS_ACTIVE", 4, minimum=1),
        affinity_enabled=_bool("AFFINITY_ENABLED", False),
        camera_cores=_core_set("CAMERA_CORES", ()),
        detector_cores=_core_set("DETECTOR_CORES", ()),
        rknn_model_path=_str("RKNN_MODEL_PATH", "models/ssd_mobilenet_v2.rknn"),
    )
