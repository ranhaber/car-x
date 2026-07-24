from cat_follow.vision.gst_nv12_capture import (
    GstV4l2Nv12Capture,
    IO_MODE_DMABUF,
    IO_MODE_MMAP,
)
import pytest


def test_gst_nv12_pipeline_uses_mmap_and_latest_frame_sink():
    capture = GstV4l2Nv12Capture("/dev/video11", 640, 480, 30.0)
    pipeline = capture._pipeline_description()

    assert f"v4l2src device=/dev/video11 io-mode={IO_MODE_MMAP}" in pipeline
    assert "format=NV12,width=640,height=480,framerate=30/1" in pipeline
    assert "max-buffers=1 drop=true" in pipeline


def test_gst_nv12_rejects_dmabuf_io_mode_for_cpu_pack():
    with pytest.raises(ValueError, match="io-mode=4"):
        GstV4l2Nv12Capture("/dev/video11", 640, 480, 30.0, io_mode=IO_MODE_DMABUF)