"""Tests for the RKNN-only detection backend and camera lores config."""

import numpy as np
import pytest

import cat_follow.vision.rknn_backend as rknn_module
from cat_follow.camera_config import CameraConfig, load_camera_config
from cat_follow.vision.backends import RknnBackend, create_backend
from cat_follow.vision.ssd_postprocess import validate_ssd_output_contract


def test_create_backend_returns_rknn():
    backend = create_backend("models/ssd_mobilenet_v2.rknn")
    assert isinstance(backend, RknnBackend)


def test_create_backend_honours_input_size():
    backend = create_backend("models/foo.rknn", input_size=(320, 240))
    assert backend._in_w == 320
    assert backend._in_h == 240


def test_rknn_backend_unavailable_without_runtime(tmp_path, monkeypatch):
    # Force "no RKNN runtime" so the test is deterministic on any host,
    # including the ROCK 4D where rknnlite is actually importable.
    monkeypatch.setattr(rknn_module, "_HAS_RKNN", False)
    backend = RknnBackend(str(tmp_path / "nope.rknn"))
    assert backend.runtime_available() is False
    assert backend.available() is False
    assert backend.loaded is False
    # infer on an unavailable backend must not raise and returns invalid bbox.
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert backend.infer(frame, 0.5) == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_ssd_output_contract_accepts_four_output_ssd():
    # boxes[1,N,4], classes[1,N], scores[1,N], count[1]
    boxes = np.zeros((1, 10, 4), dtype=np.float32)
    classes = np.zeros((1, 10), dtype=np.float32)
    scores = np.zeros((1, 10), dtype=np.float32)
    count = np.array([10.0], dtype=np.float32)
    validate_ssd_output_contract([boxes, classes, scores, count])  # no raise


def test_ssd_output_contract_rejects_undecoded_heads():
    # Raw SSD heads (undecoded, wrong shape) must be rejected loudly.
    raw = np.zeros((1, 1917, 91), dtype=np.float32)
    with pytest.raises(ValueError):
        validate_ssd_output_contract([raw])
    with pytest.raises(ValueError):
        validate_ssd_output_contract([])


def test_ssd_output_contract_rejects_misaligned_scores():
    # scores N must match boxes N (tensor alignment / ordering check).
    boxes = np.zeros((1, 10, 4), dtype=np.float32)
    classes = np.zeros((1, 10), dtype=np.float32)
    scores = np.zeros((1, 7), dtype=np.float32)  # wrong N
    count = np.array([10.0], dtype=np.float32)
    with pytest.raises(ValueError):
        validate_ssd_output_contract([boxes, classes, scores, count])


def test_ssd_output_contract_rejects_out_of_range_scores():
    # Undecoded logits (scores outside [0, 1.5]) must be rejected: this catches
    # a model whose output ordering / decoding does not match the contract.
    boxes = np.zeros((1, 10, 4), dtype=np.float32)
    classes = np.zeros((1, 10), dtype=np.float32)
    scores = np.full((1, 10), 42.0, dtype=np.float32)
    count = np.array([10.0], dtype=np.float32)
    with pytest.raises(ValueError):
        validate_ssd_output_contract([boxes, classes, scores, count])


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
