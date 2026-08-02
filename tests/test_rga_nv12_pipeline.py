"""Tests for crop pool, RGA wrapper, and NV12 RKNN backend wiring."""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from cat_follow.memory.pool import CROP_NV12_SHAPE, CROP_RING_N, allocate_pool
from cat_follow.memory.shared_state import SharedState
from cat_follow.vision.nv12_utils import center_bottom_nv12_region, extract_nv12_crop
from cat_follow.vision.rga_crop import crop_center_bottom_nv12, crop_backend_name
import cat_follow.vision.rga_crop as rga_crop_module
from cat_follow.vision.rknn_backend import infer_input_format_from_model_path
import cat_follow.vision.rknn_backend as rknn_module
from cat_follow.vision.backends import create_backend


def test_crop_ring_allocated_with_capture_ring():
    pool = allocate_pool()
    assert pool.crop_ring.shape == (CROP_RING_N, *CROP_NV12_SHAPE)


def test_crop_buffer_pairs_with_slot_index():
    shared = SharedState(allocate_pool())
    buf0 = shared.crop_buffer_for_slot(0)
    buf3 = shared.crop_buffer_for_slot(3)
    assert buf0.shape == CROP_NV12_SHAPE
    assert buf3.shape == CROP_NV12_SHAPE
    assert buf0 is not buf3
    assert np.shares_memory(buf3, shared._pool.crop_ring[3])


def test_cpu_crop_matches_extract_nv12_crop(monkeypatch):
    monkeypatch.setattr(rga_crop_module, "rga_available", lambda: False)
    pool = allocate_pool()
    slot = pool.frame_ring[0]
    slot[:480].fill(42)
    slot[480:].fill(128)
    dst = np.empty(CROP_NV12_SHAPE, dtype=np.uint8)
    region = center_bottom_nv12_region(640, 480, 320, 320)
    expected = np.empty(CROP_NV12_SHAPE, dtype=np.uint8)
    extract_nv12_crop(slot, 640, 480, region, dst=expected)
    crop_center_bottom_nv12(slot, 640, 480, 320, 320, dst=dst)
    assert np.array_equal(dst, expected)


class _FakeBuffer:
    def __init__(self, size, data=None):
        self.data = bytearray(size) if data is None else bytearray(data)

    def fill(self, offset, data):
        payload = bytes(data)
        self.data[offset : offset + len(payload)] = payload
        return len(payload)

    def map(self, _flags):
        info = SimpleNamespace(data=self.data, size=len(self.data))
        return True, info

    def unmap(self, _info):
        return None


class _FakePipeline:
    def __init__(self, gst):
        self._gst = gst
        self.src = SimpleNamespace(emit=self._src_emit)
        self.sink = SimpleNamespace(emit=self._sink_emit)

    def get_by_name(self, name):
        return self.src if name == "src" else self.sink if name == "sink" else None

    def set_state(self, state):
        if state == self._gst.State.NULL:
            self._gst.null_count += 1
        return self._gst.StateChangeReturn.SUCCESS

    def _src_emit(self, signal, _buffer):
        assert signal == "push-buffer"
        if self._gst.fail_next_push:
            self._gst.fail_next_push = False
            return self._gst.FlowReturn.ERROR
        return self._gst.FlowReturn.OK

    def _sink_emit(self, signal, _timeout):
        assert signal == "try-pull-sample"
        output = _FakeBuffer(self._gst.output_size)
        return SimpleNamespace(get_buffer=lambda: output)


class _FakeGst:
    SECOND = 1
    State = SimpleNamespace(PLAYING="playing", NULL="null")
    StateChangeReturn = SimpleNamespace(SUCCESS="success", FAILURE="failure")
    FlowReturn = SimpleNamespace(OK="ok", ERROR="error")
    MapFlags = SimpleNamespace(READ="read")

    def __init__(self, output_size):
        self.output_size = output_size
        self.parse_count = 0
        self.null_count = 0
        self.fail_next_push = False
        self.Buffer = SimpleNamespace(
            new_allocate=lambda _allocator, size, _params: _FakeBuffer(size)
        )

    def parse_launch(self, _description):
        self.parse_count += 1
        return _FakePipeline(self)


def test_rga_pipeline_reused_and_recreated_for_config_change(monkeypatch):
    gst = _FakeGst(output_size=12)
    backend = rga_crop_module._RgaCropBackend()
    monkeypatch.setattr(backend, "_load_gst", lambda: gst)
    frame = np.arange(48, dtype=np.uint8).reshape(6, 8)
    dst = np.empty((3, 4), dtype=np.uint8)

    backend.crop(frame, 8, 4, 4, 2, dst=dst, region=(2, 2, 4, 2))
    backend.crop(frame, 8, 4, 4, 2, dst=dst, region=(2, 2, 4, 2))
    assert gst.parse_count == 1
    assert gst.null_count == 0

    backend.crop(frame, 8, 4, 4, 2, dst=dst, region=(0, 2, 4, 2))
    assert gst.parse_count == 2
    assert gst.null_count == 1

    backend.close()
    assert gst.null_count == 2


def test_rga_pipeline_recovers_after_stream_error(monkeypatch):
    gst = _FakeGst(output_size=12)
    backend = rga_crop_module._RgaCropBackend()
    monkeypatch.setattr(backend, "_load_gst", lambda: gst)
    frame = np.arange(48, dtype=np.uint8).reshape(6, 8)
    dst = np.empty((3, 4), dtype=np.uint8)
    gst.fail_next_push = True

    with pytest.raises(RuntimeError, match="input push failed"):
        backend.crop(frame, 8, 4, 4, 2, dst=dst, region=(2, 2, 4, 2))
    assert gst.null_count == 1

    backend.crop(frame, 8, 4, 4, 2, dst=dst, region=(2, 2, 4, 2))
    assert gst.parse_count == 2


def test_rga_close_does_not_wait_for_a_blocked_crop(monkeypatch):
    """``close()`` must be able to tear down a pipeline stuck in push-buffer."""
    gst = _FakeGst(output_size=12)
    backend = rga_crop_module._RgaCropBackend()
    monkeypatch.setattr(backend, "_load_gst", lambda: gst)
    frame = np.arange(48, dtype=np.uint8).reshape(6, 8)
    dst = np.empty((3, 4), dtype=np.uint8)
    backend.crop(frame, 8, 4, 4, 2, dst=dst, region=(2, 2, 4, 2))

    pushing = threading.Event()
    unblock = threading.Event()
    pipeline = backend._pipeline

    def _blocking_push(signal, _buffer):
        assert signal == "push-buffer"
        pushing.set()
        unblock.wait(2.0)
        return gst.FlowReturn.OK

    pipeline.src.emit = _blocking_push
    worker = threading.Thread(
        target=backend.crop,
        args=(frame, 8, 4, 4, 2),
        kwargs={"dst": np.empty((3, 4), dtype=np.uint8), "region": (2, 2, 4, 2)},
    )
    worker.start()
    assert pushing.wait(1.0)

    started = time.perf_counter()
    backend.close()
    elapsed = time.perf_counter() - started
    unblock.set()
    worker.join(3.0)

    assert elapsed < 0.5


def test_rga_failure_falls_back_to_cpu(monkeypatch):
    monkeypatch.setattr(rga_crop_module, "rga_available", lambda: True)
    monkeypatch.setattr(
        rga_crop_module,
        "_rga_crop_center_bottom",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("RGA failed")),
    )
    frame = np.arange(48, dtype=np.uint8).reshape(6, 8)
    dst = np.empty((3, 4), dtype=np.uint8)
    expected = np.empty_like(dst)
    region = (2, 2, 4, 2)
    extract_nv12_crop(frame, 8, 4, region, dst=expected)

    result, x, y = crop_center_bottom_nv12(
        frame, 8, 4, 4, 2, dst=dst, region=region
    )

    assert result is dst
    assert (x, y) == (2, 2)
    assert np.array_equal(dst, expected)


def test_infer_input_format_from_model_path():
    assert infer_input_format_from_model_path("models/yolov8n_nv12_int8.rknn") == "nv12"
    assert infer_input_format_from_model_path("models/yolov8n_int8.rknn") == "rgb"


def test_create_backend_honours_input_format():
    backend = create_backend(
        "models/yolov8n_coco_320_rk3576_nv12_int8.rknn",
        input_format="nv12",
    )
    assert backend.input_format == "nv12"


def test_create_backend_rejects_format_conflicting_with_model_name():
    """The RGB default must not silently mis-feed an NV12 model."""
    with pytest.raises(ValueError, match="declares 'nv12'"):
        create_backend(
            "models/yolov8n_coco_320_rk3576_nv12_int8.rknn",
            input_format="rgb",
        )


def test_create_backend_rejects_nv12_for_an_untagged_model(monkeypatch):
    """The production model name declares no layout, so NV12 is a guess that
    would silently produce wrong boxes."""
    monkeypatch.delenv(
        "CAT_FOLLOW_PERCEPTION_RKNN_ALLOW_UNTAGGED_NV12", raising=False
    )
    with pytest.raises(ValueError, match="declares no layout"):
        create_backend(
            "models/yolov8n_coco_320_rk3576_int8.rknn", input_format="nv12"
        )


def test_untagged_nv12_is_allowed_with_an_explicit_override(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_RKNN_ALLOW_UNTAGGED_NV12", "1")
    backend = create_backend(
        "models/yolov8n_coco_320_rk3576_int8.rknn", input_format="nv12"
    )
    assert backend.input_format == "nv12"


def test_untagged_model_still_defaults_to_rgb(monkeypatch):
    monkeypatch.delenv(
        "CAT_FOLLOW_PERCEPTION_RKNN_ALLOW_UNTAGGED_NV12", raising=False
    )
    backend = create_backend("models/yolov8n_coco_320_rk3576_int8.rknn")
    assert backend.input_format == "rgb"


def test_self_test_reports_a_probable_input_format_mismatch(monkeypatch):
    """The model itself is the only ground truth for its input layout."""
    backend = create_backend("models/yolov8n_coco_320_rk3576_int8.rknn")
    monkeypatch.setattr(backend, "load", lambda: True)

    def _reject(_frame):
        raise RuntimeError("rknn inference returned -1")

    monkeypatch.setattr(backend, "_raw_infer", _reject)

    with pytest.raises(RuntimeError, match="does not match the converted model"):
        backend.self_test()


def test_rga_close_during_rebuild_does_not_orphan_a_playing_pipeline(monkeypatch):
    """A pipeline built across a close() must be torn down, not published."""
    gst = _FakeGst(output_size=12)
    backend = rga_crop_module._RgaCropBackend()
    building = threading.Event()
    closed = threading.Event()
    real_parse = gst.parse_launch

    def _slow_parse(description):
        pipeline = real_parse(description)
        building.set()
        closed.wait(2.0)
        return pipeline

    gst.parse_launch = _slow_parse
    monkeypatch.setattr(backend, "_load_gst", lambda: gst)

    frame = np.arange(48, dtype=np.uint8).reshape(6, 8)
    errors = []

    def _crop():
        try:
            backend.crop(
                frame,
                8,
                4,
                4,
                2,
                dst=np.empty((3, 4), dtype=np.uint8),
                region=(2, 2, 4, 2),
            )
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)

    worker = threading.Thread(target=_crop)
    worker.start()
    assert building.wait(2.0)

    backend.close()
    closed.set()
    worker.join(3.0)

    assert backend._pipeline is None
    assert errors and "closed during pipeline setup" in str(errors[0])
    # The orphan was stopped rather than left PLAYING.
    assert gst.null_count >= 1


def test_rknn_nv12_native_input_buf_shape():
    backend = create_backend(
        "models/foo_nv12.rknn",
        input_size=(320, 320),
        input_format="nv12",
    )
    backend._ensure_input_buf()
    assert backend._input_buf is not None
    assert backend._input_buf.shape == (1, 480, 320, 1)


def test_crop_backend_name_is_cpu_or_rga():
    assert crop_backend_name() in ("cpu", "rga")
