"""Lightweight motion detector used to gate expensive AI inference.

Ported from the cat_ball_tracker reference and adapted to cat_follow's
pre-allocated buffer discipline.  The detector downscales the incoming frame
(hardware ISP lores or software resize), maintains a running-average
background, and reports whether motion exceeds a configured area threshold.

Two hot-path optimizations from the reference are preserved:

- **Pre-allocated buffers**: every intermediate array (`_small`, `_gray`,
  `_bg`, `_delta`, `_thresh`) is allocated once and reused via ``dst=`` so a
  steady-state frame does not allocate.
- **Y-plane fast path**: when the caller already has a single-channel
  luma/gray image (e.g. the NV12 Y plane straight from RKISP), pass
  ``gray_input=True`` to skip the BGR to gray conversion entirely.

The detector degrades gracefully to a no-op (always "motion") when OpenCV is
unavailable, so the perception pipeline still runs on machines without cv2.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import cv2

    _HAS_CV2 = True
except Exception:  # pragma: no cover - exercised on cv2-less machines
    _HAS_CV2 = False


@dataclass(frozen=True)
class MotionResult:
    """Outcome of a single motion evaluation."""

    motion: bool
    area: int
    bbox: Optional[Tuple[int, int, int, int]] = None  # x, y, w, h in full-frame px


class MotionDetector:
    """Frame-differencing motion detector with a running-average background."""

    def __init__(
        self,
        frame_width: int,
        frame_height: int,
        *,
        scale: float = 0.35,
        threshold: int = 25,
        min_area: int = 500,
        background_alpha: float = 0.1,
    ) -> None:
        self._frame_w = int(frame_width)
        self._frame_h = int(frame_height)
        self._scale = float(scale)
        self._threshold = int(threshold)
        self._min_area = int(min_area)
        self._alpha = float(background_alpha)

        self._small_w = max(1, int(round(self._frame_w * self._scale)))
        self._small_h = max(1, int(round(self._frame_h * self._scale)))

        # Ratio to map scaled-space coordinates back to full-frame pixels.
        self._sx = self._frame_w / self._small_w
        self._sy = self._frame_h / self._small_h

        # Pre-allocated working buffers (allocated once, reused every frame).
        self._small = np.empty((self._small_h, self._small_w, 3), dtype=np.uint8)
        self._gray = np.empty((self._small_h, self._small_w), dtype=np.uint8)
        self._bg = None  # float32 running-average background, lazy-initialised
        self._delta = np.empty((self._small_h, self._small_w), dtype=np.uint8)
        self._bg_u8 = np.empty((self._small_h, self._small_w), dtype=np.uint8)

    # ── public API ───────────────────────────────────────────────────

    def reset(self) -> None:
        """Forget the learned background (e.g. after a scene change)."""
        self._bg = None

    def detect(self, frame: np.ndarray, *, gray_input: bool = False) -> MotionResult:
        """Return the motion result for *frame*.

        Parameters
        ----------
        frame:
            Either a full-resolution BGR frame, or a single-channel gray/luma
            frame when ``gray_input`` is True (e.g. an NV12 Y plane).
        gray_input:
            Skip the color conversion and treat *frame* as gray.
        """
        if not _HAS_CV2:
            # Without OpenCV we cannot compute motion; fail open so the
            # detector still runs (never silently blind the pipeline).
            return MotionResult(motion=True, area=self._min_area, bbox=None)

        gray = self._to_scaled_gray(frame, gray_input=gray_input)

        if self._bg is None:
            self._bg = gray.astype(np.float32)
            return MotionResult(motion=False, area=0, bbox=None)

        # delta = |gray - background|
        cv2.absdiff(gray, cv2.convertScaleAbs(self._bg, dst=self._bg_u8), dst=self._delta)
        _, thresh = cv2.threshold(
            self._delta, self._threshold, 255, cv2.THRESH_BINARY
        )

        # Update running-average background *after* the diff so fast movers
        # remain visible for at least one frame.
        cv2.accumulateWeighted(gray, self._bg, self._alpha)

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        best_area = 0
        best_bbox: Optional[Tuple[int, int, int, int]] = None
        for contour in contours:
            area = int(cv2.contourArea(contour))
            if area < self._min_area or area <= best_area:
                continue
            best_area = area
            bx, by, bw, bh = cv2.boundingRect(contour)
            best_bbox = (
                int(bx * self._sx),
                int(by * self._sy),
                int(bw * self._sx),
                int(bh * self._sy),
            )

        return MotionResult(
            motion=best_bbox is not None,
            area=best_area,
            bbox=best_bbox,
        )

    # ── internals ────────────────────────────────────────────────────

    def _to_scaled_gray(self, frame: np.ndarray, *, gray_input: bool) -> np.ndarray:
        if gray_input:
            source = frame
            if source.ndim != 2:
                source = source.reshape(source.shape[0], -1)
            if source.shape != (self._small_h, self._small_w):
                cv2.resize(
                    source,
                    (self._small_w, self._small_h),
                    dst=self._gray,
                    interpolation=cv2.INTER_AREA,
                )
                return self._gray
            return source

        # BGR path: downscale then convert to gray, both into pre-alloc bufs.
        if frame.shape[:2] != (self._small_h, self._small_w):
            cv2.resize(
                frame,
                (self._small_w, self._small_h),
                dst=self._small,
                interpolation=cv2.INTER_AREA,
            )
            src = self._small
        else:
            src = frame
        cv2.cvtColor(src, cv2.COLOR_BGR2GRAY, dst=self._gray)
        return self._gray


__all__ = ["MotionDetector", "MotionResult"]
