"""Live cat-sprite injection for end-to-end perception validation.

The injector alpha-composites a real cat cutout onto each live BGR camera
frame.  It deliberately creates pixels only: it never publishes a synthetic
detection, so any resulting track must have passed through motion gating,
RKNN inference, YOLO post-processing, and PredictiveTracker.
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Optional

import numpy as np

from cat_follow.logger import get_logger

log = get_logger("perception.live_inject")

_ALPHA_THRESHOLD = 8


def _opaque_content_rect(bgra: np.ndarray) -> tuple[int, int, int, int]:
    """Return tight ``x1,y1,x2,y2`` bounds for non-transparent pixels."""
    height, width = bgra.shape[:2]
    if bgra.ndim != 3 or bgra.shape[2] < 4:
        return (0, 0, width, height)
    mask = bgra[:, :, 3] > _ALPHA_THRESHOLD
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return (0, 0, width, height)
    return (
        int(xs.min()),
        int(ys.min()),
        int(xs.max()) + 1,
        int(ys.max()) + 1,
    )


def _blend_bgra_onto_bgr(
    destination: np.ndarray, x: int, y: int, sprite: np.ndarray
) -> None:
    """Alpha-composite an OpenCV BGRA sprite into a BGR frame in place."""
    height, width = sprite.shape[:2]
    region = destination[y : y + height, x : x + width]
    if region.shape[:2] != (height, width):
        return
    if sprite.ndim == 3 and sprite.shape[2] >= 4:
        alpha = sprite[:, :, 3:4].astype(np.float32) * (1.0 / 255.0)
        source = sprite[:, :, :3].astype(np.float32)
        np.multiply(source, alpha, out=source)
        background = region.astype(np.float32)
        background *= 1.0 - alpha
        source += background
        region[:] = source.astype(np.uint8)
    else:
        region[:] = sprite[:, :, :3]


class LiveCatInjector:
    """Move and paste one cat sprite across live camera frames.

    Instances are owned by the camera thread; no internal locking is needed.
    Movement is time-based rather than frame-based so speed remains stable when
    camera FPS changes.
    """

    def __init__(self, image_path: str, *, speed_px_s: float = 60.0) -> None:
        self.image_path = str(image_path)
        self.speed_px_s = max(0.0, float(speed_px_s))
        self._sprite: Optional[np.ndarray] = None
        self._opaque_rect = (0, 0, 0, 0)
        self._enabled = False
        self._x = 0.0
        self._direction = 1.0
        self._last_at: Optional[float] = None
        self.bbox: Optional[tuple[int, int, int, int]] = None

    @property
    def loaded(self) -> bool:
        return self._sprite is not None

    def set_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self._enabled:
            return
        self._enabled = enabled
        self._last_at = None
        self.bbox = None
        if enabled:
            self._load()
            log.info("Live cat injection enabled: %s", self.image_path)
        else:
            log.info("Live cat injection disabled")

    def _load(self) -> None:
        if self._sprite is not None:
            return
        import cv2

        path = Path(self.image_path)
        sprite = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if sprite is None:
            raise RuntimeError(f"cat injection image could not be loaded: {path}")
        if sprite.ndim != 3 or sprite.shape[2] not in (3, 4):
            raise RuntimeError(
                f"cat injection image must be BGR/BGRA, got shape {sprite.shape}"
            )
        self._sprite = sprite
        self._opaque_rect = _opaque_content_rect(sprite)
        log.info(
            "Loaded live cat sprite %s (%dx%d, alpha=%s)",
            path,
            sprite.shape[1],
            sprite.shape[0],
            sprite.shape[2] == 4,
        )

    def apply(
        self, frame_bgr: np.ndarray, *, now: Optional[float] = None
    ) -> Optional[tuple[int, int, int, int]]:
        """Mutate *frame_bgr* and return the tight injected ``xyxy`` bbox."""
        if not self._enabled:
            self.bbox = None
            return None
        self._load()
        assert self._sprite is not None

        frame_h, frame_w = frame_bgr.shape[:2]
        sprite_h, sprite_w = self._sprite.shape[:2]
        if sprite_w > frame_w or sprite_h > frame_h:
            raise RuntimeError(
                f"cat sprite {sprite_w}x{sprite_h} exceeds frame "
                f"{frame_w}x{frame_h}"
            )

        current = time.monotonic() if now is None else float(now)
        if self._last_at is not None:
            elapsed = min(0.25, max(0.0, current - self._last_at))
            self._x += self._direction * self.speed_px_s * elapsed
        self._last_at = current

        max_x = float(frame_w - sprite_w)
        if self._x >= max_x:
            self._x = max_x
            self._direction = -1.0
        elif self._x <= 0.0:
            self._x = 0.0
            self._direction = 1.0

        x = int(round(self._x))
        y = max(0, (frame_h - sprite_h) // 2)
        _blend_bgra_onto_bgr(frame_bgr, x, y, self._sprite)

        ox1, oy1, ox2, oy2 = self._opaque_rect
        self.bbox = (x + ox1, y + oy1, x + ox2, y + oy2)
        return self.bbox


__all__ = ["LiveCatInjector"]
