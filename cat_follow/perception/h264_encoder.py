"""Hardware H.264 encoder using Rockchip MPP via GStreamer.

The ROCK 4D exposes the Media Process Platform at ``/dev/mpp_service`` and the
``mpph264enc`` GStreamer element performs H.264 encode on the VPU. Production
imports the camera's DMA-BUF fd into a :class:`Gst.Buffer`, so raw NV12 pixels
flow camera -> MPP without a CPU copy. The NumPy/BGR methods remain available
for explicit Option A compatibility. Web overlays are sent separately as JSON.

Everything is optional and guarded: if PyGObject/GStreamer or ``mpph264enc``
are unavailable, :meth:`MppH264Encoder.available` returns False. The web UI
requires H.264 when ``CAT_FOLLOW_WEB_REQUIRE_H264=1`` (no MJPEG fallback).
"""

from __future__ import annotations

import os
import threading
from typing import Optional

import numpy as np

from cat_follow.logger import get_logger

log = get_logger("perception.h264")

# At most one camera DMA-BUF may remain dequeued for MPP. Additional stream
# frames are dropped so capture/detector never exhaust the V4L2 pool.
_MAX_PENDING_CAMERA_LEASES = 1

try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    gi.require_version("GstAllocators", "1.0")
    gi.require_version("GstVideo", "1.0")
    from gi.repository import Gst, GstAllocators, GstApp, GstVideo  # noqa: F401

    Gst.init(None)
    _HAS_GST = True
except Exception:  # pragma: no cover - only on boards with GStreamer
    Gst = None  # type: ignore
    _HAS_GST = False


class MppH264Encoder:
    """Encode packed NV12 or BGR frames via the Rockchip VPU."""

    def __init__(
        self,
        width: int,
        height: int,
        fps: int = 15,
        bitrate_kbps: int = 4000,
        *,
        pixel_format: str = "NV12",
    ) -> None:
        self._w = int(width)
        self._h = int(height)
        self._fps = int(fps)
        self._bitrate = int(bitrate_kbps)
        self._pixel_format = str(pixel_format).upper()
        if self._pixel_format not in {"NV12", "BGR"}:
            raise ValueError(
                f"unsupported H.264 input format {pixel_format!r}; "
                "expected NV12 or BGR"
            )
        self._pipeline = None
        self._appsrc = None
        self._appsink = None
        self._dmabuf_allocator = None
        self._input_seq = 0
        self._pending_leases: dict[int, object] = {}
        # RLock lets failure cleanup remove a just-registered lease while the
        # atomic submit transaction still owns the lock.
        self._io_lock = threading.RLock()

    @staticmethod
    def available() -> bool:
        if not _HAS_GST:
            return False
        return all(
            Gst.ElementFactory.find(element) is not None
            for element in ("mpph264enc", "h264parse")
        )

    def start(self) -> bool:
        if not self.available():
            return False
        desc = self._pipeline_description()
        try:
            self._pipeline = Gst.parse_launch(desc)
            self._appsrc = self._pipeline.get_by_name("src")
            self._appsink = self._pipeline.get_by_name("sink")
            self._dmabuf_allocator = GstAllocators.DmaBufAllocator.new()
            self._pipeline.set_state(Gst.State.PLAYING)
            log.info(
                "MPP H.264 encoder started (%s %dx%d @ %d fps)",
                self._pixel_format,
                self._w,
                self._h,
                self._fps,
            )
            return True
        except Exception as exc:  # noqa: BLE001
            log.warning("MPP H.264 pipeline failed to start: %s", exc)
            self.stop()
            return False

    def _pipeline_description(self) -> str:
        """Build the GStreamer pipeline without touching board-only APIs."""
        convert = " ! videoconvert" if self._pixel_format == "BGR" else ""
        # Baseline + Annex-B byte-stream matches Chrome WebCodecs reliably.
        return (
            f"appsrc name=src is-live=true format=time do-timestamp=true "
            f"caps=video/x-raw,format={self._pixel_format},"
            f"width={self._w},height={self._h},"
            f"framerate={self._fps}/1 "
            f"{convert} ! mpph264enc bps={self._bitrate * 1000} "
            f"profile=baseline level=40 "
            f"! h264parse config-interval=-1 "
            f"! video/x-h264,stream-format=byte-stream,alignment=au "
            f"! appsink name=sink emit-signals=false sync=false "
            f"max-buffers=4 drop=false"
        )

    def submit(self, frame: np.ndarray) -> list[bytes]:
        """Push one CPU frame and return every access unit ready in FIFO order."""
        if self._appsrc is None or self._appsink is None:
            return []
        chunks: list[bytes] = []
        try:
            expected_shape = (
                (self._h * 3 // 2, self._w)
                if self._pixel_format == "NV12"
                else (self._h, self._w, 3)
            )
            if frame.shape != expected_shape or frame.dtype != np.uint8:
                raise ValueError(
                    f"{self._pixel_format} frame must be uint8 {expected_shape}, "
                    f"got {frame.dtype} {frame.shape}"
                )
            data = np.ascontiguousarray(frame).tobytes()
            buf = Gst.Buffer.new_wrapped(data)
            with self._io_lock:
                chunks.extend(self._drain_encoded_locked())
                flow = self._appsrc.emit("push-buffer", buf)
                if flow != Gst.FlowReturn.OK:
                    raise RuntimeError(f"appsrc push-buffer failed: {flow}")
                chunks.extend(
                    self._drain_encoded_locked(wait_ns=10 * Gst.MSECOND)
                )
                return chunks
        except Exception as exc:  # noqa: BLE001
            log.debug("MPP H.264 encode step failed: %s", exc)
            return chunks

    def submit_dmabuf(self, lease) -> list[bytes]:  # noqa: ANN001
        """Push a camera DMA-BUF and return all ready access units in order.

        Ownership of *lease* transfers to this method. It remains pinned until
        the encoder emits the output carrying the matching PTS, ensuring V4L2
        cannot requeue/overwrite the camera buffer while MPP reads it.
        """
        if (
            self._appsrc is None
            or self._appsink is None
            or self._dmabuf_allocator is None
        ):
            lease.release()
            return []
        if not getattr(lease, "dmabuf", False):
            lease.release()
            raise ValueError("submit_dmabuf requires a DMA-BUF frame lease")

        chunks: list[bytes] = []
        # Keep the entire cap decision and submit transaction under one lock.
        # Buffer import is intentionally inside it: releasing the lock between
        # the cap check and registration would let another submit overbook.
        with self._io_lock:
            dup_fd = -1
            pending_pts = None
            try:
                chunks.extend(self._drain_encoded_locked())
                if len(self._pending_leases) >= _MAX_PENDING_CAMERA_LEASES:
                    lease.release()
                    return chunks

                stride = int(lease.dmabuf_stride or self._w)
                size = int(lease.dmabuf_size)
                minimum_size = stride * self._h * 3 // 2
                if size < minimum_size:
                    raise ValueError(
                        f"DMA-BUF size {size} is smaller than NV12 layout "
                        f"{minimum_size} (stride={stride})"
                    )

                dup_fd = os.dup(int(lease.dmabuf_fd))
                # This GI binding exposes alloc as a static-style method and
                # still requires the allocator argument explicitly.
                memory = GstAllocators.DmaBufAllocator.alloc(
                    self._dmabuf_allocator, dup_fd, size
                )
                if memory is None:
                    raise RuntimeError("GstDmaBufAllocator failed to import fd")
                # GstMemory owns dup_fd after a successful alloc().
                dup_fd = -1

                buf = Gst.Buffer.new()
                buf.append_memory(memory)
                GstVideo.buffer_add_video_meta_full(
                    buf,
                    GstVideo.VideoFrameFlags.NONE,
                    GstVideo.VideoFormat.NV12,
                    self._w,
                    self._h,
                    2,
                    [0, stride * self._h, 0, 0],
                    [stride, stride, 0, 0],
                )

                self._input_seq += 1
                pending_pts = self._input_seq * Gst.SECOND // self._fps
                buf.pts = pending_pts
                buf.dts = pending_pts
                buf.duration = Gst.SECOND // self._fps
                self._pending_leases[pending_pts] = lease
                flow = self._appsrc.emit("push-buffer", buf)
                if flow != Gst.FlowReturn.OK:
                    raise RuntimeError(f"appsrc push-buffer failed: {flow}")
                chunks.extend(
                    self._drain_encoded_locked(wait_ns=10 * Gst.MSECOND)
                )
                return chunks
            except Exception as exc:  # noqa: BLE001
                if dup_fd >= 0:
                    os.close(dup_fd)
                if pending_pts is not None:
                    pending = self._pending_leases.pop(pending_pts, None)
                    if pending is not None:
                        pending.release()
                else:
                    lease.release()
                log.warning("MPP DMA-BUF encode step failed: %s", exc)
                return chunks

    def poll(self, *, wait_ns: int = 0) -> list[bytes]:
        """Return delayed access units even when no newer frame is submitted."""
        with self._io_lock:
            return self._drain_encoded_locked(wait_ns=wait_ns)

    def _drain_encoded_locked(self, *, wait_ns: int = 0) -> list[bytes]:
        """Pull every ready encoded sample and release matching input leases."""
        if self._appsink is None:
            return []
        chunks: list[bytes] = []
        while True:
            sample = self._appsink.emit("try-pull-sample", 0)
            if sample is None:
                break
            gst_buf = sample.get_buffer()
            self._release_input_for_pts(int(gst_buf.pts))
            ok, mapinfo = gst_buf.map(Gst.MapFlags.READ)
            if not ok:
                continue
            try:
                chunks.append(bytes(mapinfo.data))
            finally:
                gst_buf.unmap(mapinfo)
        if wait_ns > 0:
            sample = self._appsink.emit("try-pull-sample", int(wait_ns))
            if sample is not None:
                gst_buf = sample.get_buffer()
                self._release_input_for_pts(int(gst_buf.pts))
                ok, mapinfo = gst_buf.map(Gst.MapFlags.READ)
                if ok:
                    try:
                        chunks.append(bytes(mapinfo.data))
                    finally:
                        gst_buf.unmap(mapinfo)
        return chunks

    def _release_input_for_pts(self, pts: int) -> None:
        lease = self._pending_leases.pop(pts, None)
        if lease is None and self._pending_leases:
            # Baseline H.264 has no B-frame reordering. Some Rockchip MPP
            # builds do not preserve PTS; release FIFO so the sole pending
            # camera lease cannot pin V4L2 indefinitely.
            oldest_pts = next(iter(self._pending_leases))
            lease = self._pending_leases.pop(oldest_pts)
        if lease is not None:
            lease.release()

    def encode(self, frame: np.ndarray) -> Optional[bytes]:
        """Compatibility wrapper; prefer :meth:`submit` for lossless streaming."""
        chunks = self.submit(frame)
        return chunks[-1] if chunks else None

    def encode_dmabuf(self, lease) -> Optional[bytes]:  # noqa: ANN001
        """Compatibility wrapper; prefer :meth:`submit_dmabuf`."""
        chunks = self.submit_dmabuf(lease)
        return chunks[-1] if chunks else None

    def stop(self) -> None:
        with self._io_lock:
            if self._pipeline is not None:
                try:
                    self._pipeline.set_state(Gst.State.NULL)
                except Exception:  # noqa: BLE001
                    pass
            self._pipeline = None
            self._appsrc = None
            self._appsink = None
            self._dmabuf_allocator = None
            pending = tuple(self._pending_leases.values())
            self._pending_leases.clear()
        for lease in pending:
            lease.release()


__all__ = ["MppH264Encoder"]
