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
    assert config.fps == 30.0


def test_rock4d_camera_config_from_environment(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_DEVICE", "/dev/video11")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_WIDTH", "1920")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_HEIGHT", "1080")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_PIXEL_FORMAT", "nv12")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_BACKEND", "V4L2")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_FPS", "30")

    config = load_camera_config()

    assert config.source == "/dev/video11"
    assert (config.width, config.height) == (1920, 1080)
    assert config.pixel_format == "NV12"
    assert config.backend == "v4l2"
    assert config.fps == 30.0


@pytest.mark.parametrize(
    ("name", "value"),
    (("WIDTH", "0"), ("HEIGHT", "-1"), ("FPS", "0")),
)
def test_non_positive_numeric_settings_are_rejected(monkeypatch, name, value):
    monkeypatch.setenv(f"CAT_FOLLOW_CAMERA_{name}", value)

    with pytest.raises(ValueError):
        load_camera_config()


def test_raw_nv12_is_converted_and_resized(monkeypatch):
    calls = []

    class FakeCv2:
        COLOR_YUV2BGR_NV12 = 1
        INTER_AREA = 2

        @staticmethod
        def cvtColor(frame, conversion):
            calls.append(("convert", frame.shape, conversion))
            return np.zeros((4, 8, 3), dtype=np.uint8)

        @staticmethod
        def resize(frame, size, interpolation):
            calls.append(("resize", size, interpolation))
            return np.zeros((size[1], size[0], 3), dtype=np.uint8)

    monkeypatch.setattr(camera_module, "cv2", FakeCv2)
    config = CameraConfig(
        device="/dev/video11",
        width=8,
        height=4,
        pixel_format="NV12",
        backend="v4l2",
    )
    raw = np.zeros((6, 8), dtype=np.uint8)

    frame = camera_module._prepare_frame(raw, config)

    assert frame.shape == (480, 640, 3)
    assert calls == [
        ("convert", (6, 8), FakeCv2.COLOR_YUV2BGR_NV12),
        ("resize", (640, 480), FakeCv2.INTER_AREA),
    ]
