"""Tests for environment-driven camera capture configuration."""

import numpy as np
import pytest

from cat_follow.camera_config import CameraConfig, load_camera_config
import cat_follow.threads.camera as camera_module


_ENV_NAMES = (
    "DEVICE",
    "WIDTH",
    "HEIGHT",
    "PIXEL_FORMAT",
    "BACKEND",
    "CAPTURE_BACKEND",
    "FPS",
)


@pytest.fixture(autouse=True)
def _clear_camera_env(monkeypatch):
    for name in _ENV_NAMES:
        monkeypatch.delenv(f"CAT_FOLLOW_CAMERA_{name}", raising=False)


def test_default_camera_config_preserves_legacy_index_zero():
    config = load_camera_config()

    assert config.source == 0
    assert (config.width, config.height) == (640, 480)
    assert config.pixel_format == ""
    assert config.backend == "default"
    assert config.capture_backend == "opencv"
    assert config.fps == 30.0


def test_rock4d_camera_config_from_environment(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_DEVICE", "/dev/video11")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_WIDTH", "1920")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_HEIGHT", "1080")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_PIXEL_FORMAT", "nv12")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_BACKEND", "V4L2")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_CAPTURE_BACKEND", "gst_nv12")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_FPS", "30")

    config = load_camera_config()

    assert config.source == "/dev/video11"
    assert (config.width, config.height) == (1920, 1080)
    assert config.pixel_format == "NV12"
    assert config.backend == "v4l2"
    assert config.capture_backend == "gst_nv12"
    assert config.fps == 30.0


@pytest.mark.parametrize(
    ("name", "value"),
    (("WIDTH", "0"), ("HEIGHT", "-1"), ("FPS", "0")),
)
def test_non_positive_numeric_settings_are_rejected(monkeypatch, name, value):
    monkeypatch.setenv(f"CAT_FOLLOW_CAMERA_{name}", value)

    with pytest.raises(ValueError):
        load_camera_config()


def test_unknown_capture_backend_is_rejected(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_CAPTURE_BACKEND", "mppjpeg")
    with pytest.raises(ValueError, match="CAPTURE_BACKEND"):
        load_camera_config()


def test_raw_nv12_is_packed_without_full_frame_color_conversion():
    config = CameraConfig(
        device="/dev/video11",
        width=640,
        height=480,
        pixel_format="NV12",
        backend="v4l2",
    )
    raw = np.arange(480 * 640 * 3 // 2, dtype=np.uint8).reshape(720, 640)

    frame = camera_module._prepare_frame(raw, config)

    assert frame.shape == (720, 640)
    assert np.array_equal(frame, raw)
    assert not np.shares_memory(frame, raw)
