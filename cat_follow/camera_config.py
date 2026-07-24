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
    capture_backend: str = "opencv"
    fps: float = 30.0

    # Optional second (lores) stream from a hardware ISP self-path. When set,
    # the camera opens it in addition to the main stream and publishes a
    # hardware-scaled gray frame for motion detection, so the CPU never has to
    # downscale full frames. Empty ``lores_device`` keeps single-stream mode.
    lores_device: str = ""
    lores_width: int = 320
    lores_height: int = 240
    lores_pixel_format: str = ""

    @property
    def source(self) -> int | str:
        """Return numeric OpenCV indexes as integers and paths unchanged."""
        return int(self.device) if self.device.isdecimal() else self.device

    @property
    def lores_source(self) -> int | str:
        return (
            int(self.lores_device)
            if self.lores_device.isdecimal()
            else self.lores_device
        )

    @property
    def lores_enabled(self) -> bool:
        return bool(self.lores_device)


def load_camera_config() -> CameraConfig:
    """Load camera settings from ``CAT_FOLLOW_CAMERA_*`` variables."""
    backend = os.getenv(f"{_PREFIX}BACKEND", "default").strip().lower()
    if backend not in {"default", "v4l2"}:
        raise ValueError(
            f"{_PREFIX}BACKEND must be 'default' or 'v4l2', got {backend!r}"
        )

    capture_backend = os.getenv(
        f"{_PREFIX}CAPTURE_BACKEND", "opencv"
    ).strip().lower()
    if capture_backend not in {"opencv", "gst_nv12"}:
        raise ValueError(
            f"{_PREFIX}CAPTURE_BACKEND must be 'opencv' or 'gst_nv12', "
            f"got {capture_backend!r}"
        )

    pixel_format = os.getenv(f"{_PREFIX}PIXEL_FORMAT", "").strip().upper()
    if pixel_format and len(pixel_format) != 4:
        raise ValueError(f"{_PREFIX}PIXEL_FORMAT must be a four-character code")

    lores_pixel_format = os.getenv(f"{_PREFIX}LORES_PIXEL_FORMAT", "").strip().upper()
    if lores_pixel_format and len(lores_pixel_format) != 4:
        raise ValueError(f"{_PREFIX}LORES_PIXEL_FORMAT must be a four-character code")

    device = os.getenv(f"{_PREFIX}DEVICE", "0").strip()
    if not device:
        device = "0"

    lores_device = os.getenv(f"{_PREFIX}LORES_DEVICE", "").strip()

    return CameraConfig(
        device=device,
        width=_positive_int("WIDTH", 640),
        height=_positive_int("HEIGHT", 480),
        pixel_format=pixel_format,
        backend=backend,
        capture_backend=capture_backend,
        fps=_positive_float("FPS", 30.0),
        lores_device=lores_device,
        lores_width=_positive_int("LORES_WIDTH", 320),
        lores_height=_positive_int("LORES_HEIGHT", 240),
        lores_pixel_format=lores_pixel_format,
    )
