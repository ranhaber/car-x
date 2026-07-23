"""Tests for the YOLO RKNN backend and camera lores config."""

import numpy as np
import pytest

import cat_follow.vision.rknn_backend as rknn_module
from cat_follow.camera_config import CameraConfig, load_camera_config
from cat_follow.vision.backends import RknnBackend, create_backend
from cat_follow.vision.yolo_postprocess import (
    _decode_cells_dfl,
    decode_yolov8_outputs,
    validate_yolo_output_contract,
)


def test_create_backend_returns_rknn():
    backend = create_backend("models/yolov8n_coco_320_rk3576.rknn")
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


def _empty_yolo_outputs():
    outputs = []
    for grid in (40, 20, 10):
        outputs.extend(
            [
                np.zeros((1, 64, grid, grid), dtype=np.float32),
                np.zeros((1, 80, grid, grid), dtype=np.float32),
                np.zeros((1, 1, grid, grid), dtype=np.float32),
            ]
        )
    return outputs


def test_yolo_output_contract_accepts_nine_tensor_head():
    validate_yolo_output_contract(_empty_yolo_outputs())


def test_yolo_output_contract_rejects_wrong_count_and_shapes():
    with pytest.raises(ValueError):
        validate_yolo_output_contract([])
    outputs = _empty_yolo_outputs()
    outputs[0] = np.zeros((1, 4, 40, 40), dtype=np.float32)
    with pytest.raises(ValueError):
        validate_yolo_output_contract(outputs)


def test_yolo_decoder_filters_to_cat_and_unletterboxes():
    outputs = _empty_yolo_outputs()
    # Flat DFL logits decode to a large box around this cell; only the class and
    # contract matter here. YOLO class index 15 maps to official COCO cat 17.
    outputs[1][0, 15, 20, 20] = 0.9
    outputs[2][0, 0, 20, 20] = 0.9
    detections = decode_yolov8_outputs(
        outputs,
        input_w=320,
        input_h=320,
        frame_w=640,
        frame_h=480,
        scale=0.5,
        pad_x=0,
        pad_y=40,
        score_threshold=0.3,
    )
    assert len(detections) == 1
    assert detections[0][4] == pytest.approx(0.9)
    assert detections[0][5] == 17


def test_yolo_decoder_uses_independent_non_square_strides():
    # One DFL bin decodes every side distance to zero. Cell (row=1, col=2)
    # on a 4x2 grid in a 320x240 input is centered at x=200, y=180.
    branch = np.zeros((1, 4, 2, 4), dtype=np.float32)
    decoded = _decode_cells_dfl(
        branch,
        np.asarray([6]),
        grid_h=2,
        grid_w=4,
        input_w=320,
        input_h=240,
    )
    assert decoded[0] == pytest.approx((200.0, 180.0, 200.0, 180.0))


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
