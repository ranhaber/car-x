"""Detection backend interface and factory (RKNN NPU only).

The project runs a single detection backend: the RK3576 NPU via
:class:`cat_follow.vision.rknn_backend.RknnBackend`.  There is intentionally
no software inference fallback -- the detector thread hard-fails when the RKNN runtime
is present but the model is missing, and only runs a deterministic stub on
machines that lack the runtime entirely (dev/CI).

Every backend implements the same small interface so the detector thread stays
backend-agnostic:

- ``loaded``      -> whether the model is currently resident
- ``load()``      -> create the runtime context (returns True on success)
- ``unload()``    -> release the context + worker threads, reclaim memory
- ``self_test()`` -> strict: load, run one inference, validate the output
                     contract; raise on any failure (used at preflight)
- ``infer(bgr)``  -> ``(x, y, w, h, valid)`` in full-frame pixels

Keeping the model *unloaded* while idle matters because the runtime's worker
threads busy-wait even with no work queued, burning a core; unloading stops
them.
"""

from __future__ import annotations

from typing import Protocol, Tuple

import numpy as np

from cat_follow.vision.rknn_backend import RknnBackend

Detection = Tuple[float, float, float, float, float]
MultiDetection = Tuple[int, int, int, int, float, int]


class DetectionBackend(Protocol):
    """Common interface implemented by every detection backend."""

    @property
    def loaded(self) -> bool: ...

    def runtime_available(self) -> bool: ...

    def available(self) -> bool: ...

    def load(self) -> bool: ...

    def unload(self) -> None: ...

    def self_test(self) -> None: ...

    def infer_all(
        self, frame_bgr: np.ndarray, score_threshold: float
    ) -> list[MultiDetection]: ...

    def infer(self, frame_bgr: np.ndarray, score_threshold: float) -> Detection: ...


def create_backend(
    model_path: str, *, input_size: Tuple[int, int] = (320, 320)
) -> RknnBackend:
    """Return the RKNN NPU detection backend (the only backend)."""
    return RknnBackend(model_path, input_size=input_size)


__all__ = [
    "DetectionBackend",
    "RknnBackend",
    "create_backend",
    "Detection",
    "MultiDetection",
]
