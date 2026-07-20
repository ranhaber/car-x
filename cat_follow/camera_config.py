"""Environment-driven camera capture configuration."""

from __future__ import annotations

from dataclasses import dataclass
import os


_PREFIX = "CAT_FOLLOW_CAMERA_"


def _positive_int(name: str, default: int) -> int:
    raw = os.getenv(f"{_PREFIX}{name}")
    if raw in (None, ""):
        return default
    value = int(raw)
    if value <= 0:
        raise ValueError(f"{_PREFIX}{name} must be greater than zero")
    return value


def _positive_float(name: str, default: float) -> float:
    raw = os.getenv(f"{_PREFIX}{name}")
    if raw in (None, ""):
        return default
    value = float(raw)
    if value <= 0:
        raise ValueError(f"{_PREFIX}{name} must be greater than zero")
    return value


@dataclass(frozen=True)
class CameraConfig:
    """Capture settings loaded once when the camera thread starts."""

    device: str = "0"
    width: int = 640
    height: int = 480
    pixel_format: str = ""
    backend: str = "default"
    fps: float = 30.0

    @property
    def source(self) -> int | str:
        """Return numeric OpenCV indexes as integers and paths unchanged."""
        return int(self.device) if self.device.isdecimal() else self.device


def load_camera_config() -> CameraConfig:
    """Load camera settings from ``CAT_FOLLOW_CAMERA_*`` variables."""
    backend = os.getenv(f"{_PREFIX}BACKEND", "default").strip().lower()
    if backend not in {"default", "v4l2"}:
        raise ValueError(
            f"{_PREFIX}BACKEND must be 'default' or 'v4l2', got {backend!r}"
        )

    pixel_format = os.getenv(f"{_PREFIX}PIXEL_FORMAT", "").strip().upper()
    if pixel_format and len(pixel_format) != 4:
        raise ValueError(f"{_PREFIX}PIXEL_FORMAT must be a four-character code")

    device = os.getenv(f"{_PREFIX}DEVICE", "0").strip()
    if not device:
        device = "0"

    return CameraConfig(
        device=device,
        width=_positive_int("WIDTH", 640),
        height=_positive_int("HEIGHT", 480),
        pixel_format=pixel_format,
        backend=backend,
        fps=_positive_float("FPS", 30.0),
    )
