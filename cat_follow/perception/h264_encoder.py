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


def _release_lease(lease) -> None:  # noqa: ANN001
    """Release one camera lease without aborting the rest of a drain.

    Releasing a lease can requeue a V4L2 buffer, and a failed QBUF raises after
    it has already escalated to the supervisor. Letting that propagate here
    would strand the remaining pending leases pinned.
    """
    try:
        lease.release()
    except Exception as exc:  # noqa: BLE001
        log.error("Camera lease release failed during H.264 drain: %s", exc)


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
        container: str = "annexb",
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
        self._container = str(container).lower()
        if self._container not in {"annexb", "matroska"}:
            raise ValueError(
                f"unsupported H.264 container {container!r}; "
                "expected annexb or matroska"
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
        if self._container == "matroska":
            # ``streamable`` writes headers up front: an appsink cannot seek
            # back to patch duration/cues, and a recording must stay playable
            # even if power is lost mid-segment. ``drop=false`` because losing
            # a muxed chunk corrupts the file, unlike a dropped live frame.
            tail = (
                "! h264parse config-interval=-1 "
                "! matroskamux streamable=true "
                "! appsink name=sink emit-signals=false sync=false "
                "max-buffers=8 drop=false"
            )
        else:
            # Baseline + Annex-B byte-stream matches Chrome WebCodecs reliably.
            tail = (
                "! h264parse config-interval=-1 "
                "! video/x-h264,stream-format=byte-stream,alignment=au "
                "! appsink name=sink emit-signals=false sync=false "
                "max-buffers=2 drop=true"
            )
        return (
            f"appsrc name=src is-live=true format=time do-timestamp=true "
            f"caps=video/x-raw,format={self._pixel_format},"
            f"width={self._w},height={self._h},"
            f"framerate={self._fps}/1 "
            f"{convert} ! mpph264enc bps={self._bitrate * 1000} "
            f"profile=baseline level=40 "
            f"{tail}"
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
            _release_lease(lease)
            return []
        if not getattr(lease, "dmabuf", False):
            _release_lease(lease)
            raise ValueError("submit_dmabuf requires a DMA-BUF frame lease")

        chunks: list[bytes] = []
        # Keep the entire cap decision and submit transaction under one lock.
        # Buffer import is intentionally inside it: releasing the lock between
        # the cap check and registration would let another submit overbook.
        with self._io_lock:
            dup_fd = -1
            pending_pts = None
            submitted = False
            try:
                chunks.extend(self._drain_encoded_locked())
                if len(self._pending_leases) >= _MAX_PENDING_CAMERA_LEASES:
                    _release_lease(lease)
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
                flow = self._appsrc.emit("push-buffer", buf)
                if flow != Gst.FlowReturn.OK:
                    raise RuntimeError(f"appsrc push-buffer failed: {flow}")
                # Pin the camera lease only after MPP accepted the buffer.
                # If push blocks or fails, the lease must not enter pending.
                self._pending_leases[pending_pts] = lease
                submitted = True
                chunks.extend(
                    self._drain_encoded_locked(wait_ns=10 * Gst.MSECOND)
                )
                return chunks
            except Exception as exc:  # noqa: BLE001
                if dup_fd >= 0:
                    os.close(dup_fd)
                if not submitted:
                    _release_lease(lease)
                # Once MPP has accepted the buffer it may still be reading the
                # camera DMA-BUF, so a later failure (e.g. drain/map) must leave
                # the lease pinned. stop()/poll() release it after the flush.
                log.warning("MPP DMA-BUF encode step failed: %s", exc)
                return chunks

    def poll(self, *, wait_ns: int = 0) -> list[bytes]:
        """Return delayed access units even when no newer frame is submitted."""
        with self._io_lock:
            return self._drain_encoded_locked(wait_ns=wait_ns)

    def finish(self, *, budget_ms: int = 500) -> list[bytes]:
        """End the stream and return every remaining encoded chunk.

        A muxed container is only a valid file once its trailing bytes are
        written, so callers recording to disk must persist what this returns
        before closing the segment.
        """
        chunks: list[bytes] = []
        with self._io_lock:
            if self._appsrc is None or self._appsink is None:
                return chunks
            try:
                self._appsrc.emit("end-of-stream")
            except Exception as exc:  # noqa: BLE001
                log.warning("H.264 end-of-stream signal failed: %s", exc)
            slice_ns = int(20 * Gst.MSECOND)
            remaining_ms = max(0, int(budget_ms))
            idle_ms = 0
            while remaining_ms > 0 and idle_ms < 100:
                batch = self._drain_encoded_locked(wait_ns=slice_ns)
                remaining_ms -= 20
                if batch:
                    chunks.extend(batch)
                    idle_ms = 0
                else:
                    idle_ms += 20
        return chunks

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
            self._release_input_for_pts(gst_buf.pts)
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
                self._release_input_for_pts(gst_buf.pts)
                ok, mapinfo = gst_buf.map(Gst.MapFlags.READ)
                if ok:
                    try:
                        chunks.append(bytes(mapinfo.data))
                    finally:
                        gst_buf.unmap(mapinfo)
        return chunks

    def _normalize_output_pts(self, pts) -> Optional[int]:  # noqa: ANN001
        if pts is None:
            return None
        try:
            value = int(pts)
        except (TypeError, ValueError):
            return None
        if value < 0:
            return None
        none_value = getattr(Gst, "CLOCK_TIME_NONE", None)
        if none_value is not None and value == int(none_value):
            return None
        return value

    def _release_input_for_pts(self, pts) -> None:  # noqa: ANN001
        normalized = self._normalize_output_pts(pts)
        lease = None
        if normalized is not None:
            lease = self._pending_leases.pop(normalized, None)
        if lease is None and len(self._pending_leases) == 1:
            # Baseline H.264 has no B-frame reordering. Some Rockchip MPP
            # builds do not preserve PTS; with cap=1, any completed output
            # corresponds to the sole pending input.
            oldest_pts = next(iter(self._pending_leases))
            lease = self._pending_leases.pop(oldest_pts)
        if lease is not None:
            _release_lease(lease)

    def _flush_pipeline_locked(self) -> None:
        """Drain encoded output before tearing down in-flight DMA-BUF reads."""
        if self._appsrc is not None:
            try:
                self._appsrc.emit("end-of-stream")
            except Exception:  # noqa: BLE001
                pass
        deadline_ns = int(250 * Gst.MSECOND)
        while deadline_ns > 0:
            before = len(self._pending_leases)
            self._drain_encoded_locked(wait_ns=min(deadline_ns, int(10 * Gst.MSECOND)))
            if not self._pending_leases:
                break
            if len(self._pending_leases) == before:
                deadline_ns -= int(10 * Gst.MSECOND)
            else:
                deadline_ns = int(250 * Gst.MSECOND)

    def encode(self, frame: np.ndarray) -> Optional[bytes]:
        """Compatibility wrapper; prefer :meth:`submit` for lossless streaming."""
        chunks = self.submit(frame)
        return chunks[-1] if chunks else None

    def encode_dmabuf(self, lease) -> Optional[bytes]:  # noqa: ANN001
        """Compatibility wrapper; prefer :meth:`submit_dmabuf`."""
        chunks = self.submit_dmabuf(lease)
        return chunks[-1] if chunks else None

    def stop(self) -> None:
        pipeline = None
        with self._io_lock:
            pipeline = self._pipeline
            if pipeline is not None:
                try:
                    self._flush_pipeline_locked()
                except Exception:  # noqa: BLE001
                    pass
                try:
                    pipeline.set_state(Gst.State.NULL)
                    pipeline.get_state(int(Gst.SECOND))
                except Exception:  # noqa: BLE001
                    pass
            self._pipeline = None
            self._appsrc = None
            self._appsink = None
            self._dmabuf_allocator = None
            pending = tuple(self._pending_leases.values())
            self._pending_leases.clear()
        for lease in pending:
            _release_lease(lease)


__all__ = ["MppH264Encoder"]
