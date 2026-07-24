"""Optional GStreamer V4L2 capture for multiplanar RKISP NV12 nodes."""

from __future__ import annotations

from typing import Optional

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.vision.nv12_utils import pack_nv12_from_buffer

log = get_logger("vision.gst_nv12_capture")

# V4L2 memory modes from GStreamer v4l2src. Only mmap (2) is safe for CPU pack
# into the ring. dmabuf export (4) is for fd-only hardware consumers later.
IO_MODE_MMAP = 2
IO_MODE_DMABUF = 4

try:  # pragma: no cover - imports are board-only
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst, GstApp  # noqa: F401

    Gst.init(None)
    _HAS_GST = True
except Exception:  # pragma: no cover - normal on development hosts
    Gst = None  # type: ignore
    _HAS_GST = False


class GstV4l2Nv12Capture:
    """Pull tight NV12 frames from ``v4l2src`` into caller-owned storage."""

    writes_nv12_destination = True

    def __init__(
        self,
        device: str,
        width: int,
        height: int,
        fps: float,
        *,
        io_mode: int = IO_MODE_MMAP,
    ) -> None:
        self._device = str(device)
        self._width = int(width)
        self._height = int(height)
        self._fps = max(1, int(round(fps)))
        if int(io_mode) == IO_MODE_DMABUF:
            raise ValueError(
                "io-mode=4 (dmabuf) is fd-only and must not be CPU-mapped; "
                "use IO_MODE_MMAP (2) for ring packing"
            )
        self._io_mode = int(io_mode)
        self._pipeline = None
        self._appsink = None
        self._logged_first_frame = False

    @staticmethod
    def available() -> bool:
        return bool(
            _HAS_GST
            and Gst.ElementFactory.find("v4l2src") is not None
            and Gst.ElementFactory.find("appsink") is not None
        )

    def _pipeline_description(self) -> str:
        return (
            f"v4l2src device={self._device} io-mode={self._io_mode} "
            f"! video/x-raw,format=NV12,width={self._width},"
            f"height={self._height},framerate={self._fps}/1 "
            f"! appsink name=sink emit-signals=false sync=false "
            f"max-buffers=1 drop=true"
        )

    def start(self) -> bool:
        if not self.available():
            return False
        try:
            self._pipeline = Gst.parse_launch(self._pipeline_description())
            self._appsink = self._pipeline.get_by_name("sink")
            result = self._pipeline.set_state(Gst.State.PLAYING)
            if result == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("GStreamer pipeline refused PLAYING state")
            log.info(
                "GStreamer NV12 capture started (%s, %dx%d @ %d, io-mode=%d)",
                self._device,
                self._width,
                self._height,
                self._fps,
                self._io_mode,
            )
            return True
        except Exception as exc:  # noqa: BLE001
            log.warning("GStreamer NV12 capture failed to start: %s", exc)
            self.release()
            return False

    def isOpened(self) -> bool:  # noqa: N802 - OpenCV-compatible API
        return self._pipeline is not None and self._appsink is not None

    def read(
        self, *, dst: Optional[np.ndarray] = None
    ) -> tuple[bool, Optional[np.ndarray]]:
        """Pull one sample and pack it into *dst*."""
        if self._appsink is None:
            return False, None
        timeout_ns = int(2.0 * Gst.SECOND)
        sample = self._appsink.emit("try-pull-sample", timeout_ns)
        if sample is None:
            return False, None

        buffer = sample.get_buffer()
        caps = sample.get_caps()
        try:
            # Avoid GstVideo.VideoInfo.from_caps() on this board: the 1.24 Python
            # typelib can corrupt the allocator when parsing multiplanar caps.
            # extract_dup() returns owned bytes safe to expose to NumPy.
            buffer_size = int(buffer.get_size())
            data = buffer.extract_dup(0, buffer_size)
            expected_size = self._width * self._height * 3 // 2
            if len(data) != expected_size:
                raise ValueError(
                    f"GStreamer NV12 sample has {len(data)} bytes; "
                    f"expected tight {expected_size}"
                )
            if not self._logged_first_frame:
                self._logged_first_frame = True
                log.info(
                    "[GST-NV12] first frame caps=%s buffer_size=%d io-mode=%d",
                    caps.to_string() if caps is not None else "unknown",
                    buffer_size,
                    self._io_mode,
                )
            frame = pack_nv12_from_buffer(
                data,
                self._width,
                self._height,
                self._width,
                self._width,
                self._width * self._height,
                mapped_size=len(data),
                dst=dst,
            )
            return True, frame
        except Exception as exc:  # noqa: BLE001
            log.error("Could not pack GStreamer NV12 sample: %s", exc)
            return False, None

    def release(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.set_state(Gst.State.NULL)
            except Exception:  # noqa: BLE001
                pass
        self._pipeline = None
        self._appsink = None


__all__ = ["GstV4l2Nv12Capture", "IO_MODE_DMABUF", "IO_MODE_MMAP"]
