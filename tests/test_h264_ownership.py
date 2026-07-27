"""H.264 DMA-BUF ownership: pending cap and drain/release behavior."""

from unittest.mock import MagicMock

import pytest

from cat_follow.perception import h264_encoder as h264_module
from cat_follow.perception.h264_encoder import MppH264Encoder


@pytest.fixture(autouse=True)
def _mock_gst(monkeypatch):
    gst = MagicMock()
    gst.MapFlags.READ = 1
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


def test_stop_releases_each_remaining_lease_once():
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")
    lease = _Lease()
    lease.release = MagicMock()  # type: ignore[method-assign]
    encoder._pending_leases = {1000: lease}

    encoder.stop()
    encoder.stop()

    lease.release.assert_called_once_with()
