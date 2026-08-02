"""H.264 DMA-BUF ownership: pending cap and drain/release behavior."""

from unittest.mock import MagicMock

import pytest

from cat_follow.perception import h264_encoder as h264_module
from cat_follow.perception.h264_encoder import MppH264Encoder


@pytest.fixture(autouse=True)
def _mock_gst(monkeypatch):
    gst = MagicMock()
    gst.MapFlags.READ = 1
    gst.CLOCK_TIME_NONE = 18446744073709551615
    monkeypatch.setattr(h264_module, "Gst", gst)
    return gst


class _Lease:
    def __init__(self) -> None:
        self.released = False
        self.dmabuf_fd = 42
        self.dmabuf_stride = 640
        self.dmabuf_size = 460800

    @property
    def dmabuf(self) -> bool:
        return True

    def release(self) -> None:
        self.released = True


def test_encode_dmabuf_drops_when_pending_cap_reached(monkeypatch):
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")
    encoder._appsrc = MagicMock()
    encoder._appsink = MagicMock()
    encoder._dmabuf_allocator = MagicMock()
    encoder._pending_leases = {1: _Lease()}
    encoder._drain_encoded_locked = MagicMock(return_value=[])  # type: ignore[method-assign]

    lease = _Lease()

    result = encoder.submit_dmabuf(lease)

    assert result == []
    assert lease.released is True
    assert len(encoder._pending_leases) == 1


def test_drain_encoded_releases_pending_leases():
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")
    lease = _Lease()
    encoder._pending_leases = {1000: lease}

    mapinfo = MagicMock()
    mapinfo.data = b"\x00\x00\x00\x01\x67"

    gst_buf = MagicMock()
    gst_buf.pts = 1000
    gst_buf.map.return_value = (True, mapinfo)

    sample = MagicMock()
    sample.get_buffer.return_value = gst_buf

    encoder._appsink = MagicMock()
    encoder._appsink.emit.side_effect = [sample, None]

    chunks = encoder.poll()

    assert chunks == [b"\x00\x00\x00\x01\x67"]
    assert lease.released is True
    assert encoder._pending_leases == {}


def test_drain_encoded_returns_newest_when_multiple_ready():
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")
    lease_a = _Lease()
    lease_b = _Lease()
    encoder._pending_leases = {1000: lease_a, 2000: lease_b}

    def _sample(data: bytes, pts: int):
        mapinfo = MagicMock()
        mapinfo.data = data
        gst_buf = MagicMock()
        gst_buf.pts = pts
        gst_buf.map.return_value = (True, mapinfo)
        sample = MagicMock()
        sample.get_buffer.return_value = gst_buf
        return sample

    encoder._appsink = MagicMock()
    encoder._appsink.emit.side_effect = [
        _sample(b"first", 1000),
        _sample(b"second", 2000),
        None,
    ]

    chunks = encoder.poll()

    assert chunks == [b"first", b"second"]
    assert lease_a.released is True
    assert lease_b.released is True
    assert encoder._pending_leases == {}


def test_encode_dmabuf_pre_drains_before_cap_check():
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")
    encoder._appsrc = MagicMock()
    encoder._appsink = MagicMock()
    encoder._dmabuf_allocator = MagicMock()
    encoder._pending_leases = {1000: _Lease()}

    drain_calls: list[str] = []

    def _drain() -> list[bytes]:
        drain_calls.append("drain")
        return [b"stale"] if len(drain_calls) == 1 else []

    encoder._drain_encoded_locked = _drain  # type: ignore[method-assign]

    lease = _Lease()
    result = encoder.submit_dmabuf(lease)

    assert drain_calls == ["drain"]
    assert result == [b"stale"]
    assert lease.released is True


def test_unknown_output_pts_releases_oldest_pending_fifo():
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")
    lease = _Lease()
    encoder._pending_leases = {1000: lease}

    mapinfo = MagicMock()
    mapinfo.data = b"unmatched"
    gst_buf = MagicMock()
    gst_buf.pts = 2000
    gst_buf.map.return_value = (True, mapinfo)
    sample = MagicMock()
    sample.get_buffer.return_value = gst_buf
    encoder._appsink = MagicMock()
    encoder._appsink.emit.side_effect = [sample, None]

    assert encoder.poll() == [b"unmatched"]
    assert lease.released is True
    assert encoder._pending_leases == {}


def test_invalid_output_pts_does_not_release_without_pending():
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")

    mapinfo = MagicMock()
    mapinfo.data = b"codec-header"
    gst_buf = MagicMock()
    gst_buf.pts = 18446744073709551615  # GST_CLOCK_TIME_NONE on 64-bit
    gst_buf.map.return_value = (True, mapinfo)
    sample = MagicMock()
    sample.get_buffer.return_value = gst_buf
    encoder._appsink = MagicMock()
    encoder._appsink.emit.side_effect = [sample, None]

    assert encoder.poll() == [b"codec-header"]
    assert encoder._pending_leases == {}


def _mock_submit_dmabuf_gst_chain(monkeypatch):
    allocators = MagicMock()
    allocators.DmaBufAllocator.alloc.return_value = MagicMock()
    monkeypatch.setattr(h264_module, "GstAllocators", allocators, raising=False)
    monkeypatch.setattr(h264_module, "GstVideo", MagicMock(), raising=False)
    h264_module.Gst.Buffer.new = MagicMock(return_value=MagicMock())
    monkeypatch.setattr(h264_module.Gst, "SECOND", 1_000_000_000)
    monkeypatch.setattr(h264_module.Gst, "MSECOND", 1_000_000)
    monkeypatch.setattr(h264_module.Gst, "FlowReturn", MagicMock(OK=0))
    monkeypatch.setattr(h264_module.os, "dup", lambda _fd: 99)
    monkeypatch.setattr(h264_module.os, "close", lambda _fd: None)


def test_submit_dmabuf_registers_lease_only_after_push_ok(monkeypatch):
    _mock_submit_dmabuf_gst_chain(monkeypatch)
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")
    encoder._appsrc = MagicMock()
    encoder._appsink = MagicMock()
    encoder._dmabuf_allocator = MagicMock()
    encoder._drain_encoded_locked = MagicMock(return_value=[])  # type: ignore[method-assign]

    pending_during_push: list[int] = []

    def _push(_signal, _buf):  # noqa: ANN001
        pending_during_push.append(len(encoder._pending_leases))
        return h264_module.Gst.FlowReturn.OK

    encoder._appsrc.emit.side_effect = _push

    lease = _Lease()
    encoder.submit_dmabuf(lease)

    assert pending_during_push == [0]
    assert len(encoder._pending_leases) == 1


def test_submit_dmabuf_push_failure_releases_lease_without_pending(monkeypatch):
    _mock_submit_dmabuf_gst_chain(monkeypatch)
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")
    encoder._appsrc = MagicMock()
    encoder._appsink = MagicMock()
    encoder._dmabuf_allocator = MagicMock()
    encoder._drain_encoded_locked = MagicMock(return_value=[])  # type: ignore[method-assign]

    class _FailFlow:
        pass

    encoder._appsrc.emit.return_value = _FailFlow()

    lease = _Lease()
    assert encoder.submit_dmabuf(lease) == []
    assert lease.released is True
    assert encoder._pending_leases == {}


def test_submit_dmabuf_keeps_lease_pinned_when_post_push_drain_fails(monkeypatch):
    _mock_submit_dmabuf_gst_chain(monkeypatch)
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")
    encoder._appsrc = MagicMock()
    encoder._appsink = MagicMock()
    encoder._dmabuf_allocator = MagicMock()
    encoder._appsrc.emit.return_value = h264_module.Gst.FlowReturn.OK

    drains = {"count": 0}

    def _drain(*, wait_ns=0):  # noqa: ANN001
        drains["count"] += 1
        if drains["count"] == 1:
            return []
        raise RuntimeError("appsink pull exploded")

    encoder._drain_encoded_locked = _drain  # type: ignore[method-assign]

    lease = _Lease()
    assert encoder.submit_dmabuf(lease) == []

    # MPP already accepted the DMA-BUF, so V4L2 must not be allowed to requeue
    # it; the lease stays pinned until an output or stop() flush releases it.
    assert lease.released is False
    assert list(encoder._pending_leases.values()) == [lease]


def test_stop_flushes_pipeline_before_releasing_leases():
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")
    lease = _Lease()
    lease.release = MagicMock()  # type: ignore[method-assign]
    encoder._pending_leases = {1000: lease}
    encoder._appsrc = MagicMock()
    encoder._appsink = MagicMock()
    pipeline = MagicMock()
    encoder._pipeline = pipeline
    encoder._flush_pipeline_locked = MagicMock()  # type: ignore[method-assign]

    encoder.stop()
    encoder.stop()

    encoder._flush_pipeline_locked.assert_called_once()
    pipeline.set_state.assert_called_once()
    pipeline.get_state.assert_called_once()
    lease.release.assert_called_once_with()
