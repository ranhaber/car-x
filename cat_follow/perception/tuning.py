"""Adaptive CPU tuning helpers for perception threads.

Two knobs, both best-effort and safe on any platform:

- ``set_opencv_threads(n)``: OpenCV spins up a parallel worker pool; a single
  thread is best while idle (avoids spin-wait), a wider pool helps during
  active resize/motion work.  We only call ``cv2.setNumThreads`` when the
  value actually changes to avoid churn.
- ``apply_affinity(cores)``: pin the *calling* thread to a CPU core set via
  ``os.sched_setaffinity`` (Linux only).  On the RK3576's 8 cores this lets us
  keep the camera/motion producer and the detector consumer off each other.
"""

from __future__ import annotations

import os
from typing import Iterable, Optional

from cat_follow.logger import get_logger

log = get_logger("perception.tuning")

_last_opencv_threads: Optional[int] = None


def set_opencv_threads(n: int) -> None:
    """Set the OpenCV worker-thread count if it changed since last call."""
    global _last_opencv_threads
    if n == _last_opencv_threads:
        return
    try:
        import cv2

        cv2.setNumThreads(int(n))
        _last_opencv_threads = n
    except Exception as exc:  # noqa: BLE001 - cv2 optional / headless builds
        log.debug("setNumThreads(%s) skipped: %s", n, exc)


def apply_affinity(cores: Iterable[int]) -> bool:
    """Pin the calling thread to *cores*.  Returns True on success."""
    core_set = {int(c) for c in cores}
    if not core_set:
        return False
    setaffinity = getattr(os, "sched_setaffinity", None)
    if setaffinity is None:
        return False
    try:
        setaffinity(0, core_set)
        log.info("Pinned thread to cores %s", sorted(core_set))
        return True
    except Exception as exc:  # noqa: BLE001 - not fatal
        log.debug("sched_setaffinity(%s) failed: %s", sorted(core_set), exc)
        return False


__all__ = ["set_opencv_threads", "apply_affinity"]
