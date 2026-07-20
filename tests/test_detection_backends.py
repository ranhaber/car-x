"""Tests for the pluggable detection backends and camera lores config."""

import numpy as np

from cat_follow.camera_config import CameraConfig, load_camera_config
from cat_follow.vision.backends import TFLiteBackend, create_backend


def test_create_backend_falls_back_to_tflite_when_rknn_absent():
    # On dev machines RKNN is unavailable, so requesting it must fall back.
    backend = create_backend("models/missing.tflite", backend="rknn")
    assert isinstance(backend, TFLiteBackend)


def test_tflite_backend_unavailable_without_model(tmp_path):
    backend = TFLiteBackend(str(tmp_path / "nope.tflite"))
    # Missing model file => not available => detector uses the stub path.
    assert backend.available() is False
    assert backend.loaded is False
    # infer on an unavailable backend must not raise and returns invalid bbox.
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert backend.infer(frame, 0.5) == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_camera_lores_config_defaults(monkeypatch):
    for name in ("LORES_DEVICE", "LORES_WIDTH", "LORES_HEIGHT", "LORES_PIXEL_FORMAT"):
        monkeypatch.delenv(f"CAT_FOLLOW_CAMERA_{name}", raising=False)
    config = load_camera_config()
    assert config.lores_enabled is False
    assert config.lores_device == ""


def test_camera_lores_config_from_env(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_LORES_DEVICE", "/dev/video12")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_LORES_WIDTH", "320")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_LORES_HEIGHT", "240")
    monkeypatch.setenv("CAT_FOLLOW_CAMERA_LORES_PIXEL_FORMAT", "nv12")
    config = load_camera_config()
    assert config.lores_enabled is True
    assert config.lores_source == "/dev/video12"
    assert (config.lores_width, config.lores_height) == (320, 240)
    assert config.lores_pixel_format == "NV12"


def test_camera_config_defaults_unaffected_by_lores():
    # Backward-compatibility: the main-stream fields keep their defaults.
    config = CameraConfig()
    assert (config.width, config.height) == (640, 480)
    assert config.lores_enabled is False
