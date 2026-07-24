"""Hardware H.264 encoder using Rockchip MPP via GStreamer.

The ROCK 4D exposes the Media Process Platform at ``/dev/mpp_service`` and the
``mpph264enc`` GStreamer element performs H.264 encode on the VPU with almost
no CPU cost. The production path feeds packed NV12 ring frames directly into
``appsrc -> mpph264enc``; BGR remains available for compatibility and inserts
``videoconvert``. The WebSocket route sends overlays separately as JSON.

Everything is optional and guarded: if PyGObject/GStreamer or ``mpph264enc``
are unavailable, :meth:`MppH264Encoder.available` returns False and callers
fall back to MJPEG.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from cat_follow.logger import get_logger

log = get_logger("perception.h264")

try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst, GstApp  # noqa: F401

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

    @staticmethod
    def available() -> bool:
        if not _HAS_GST:
            return False
        return Gst.ElementFactory.find("mpph264enc") is not None

    def start(self) -> bool:
        if not self.available():
            return False
        desc = self._pipeline_description()
        try:
            self._pipeline = Gst.parse_launch(desc)
            self._appsrc = self._pipeline.get_by_name("src")
            self._appsink = self._pipeline.get_by_name("sink")
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
        return (
            f"appsrc name=src is-live=true format=time "
            f"caps=video/x-raw,format={self._pixel_format},"
            f"width={self._w},height={self._h},"
            f"framerate={self._fps}/1 "
            f"{convert} ! mpph264enc bps={self._bitrate * 1000} "
            f"! h264parse config-interval=1 "
            f"! appsink name=sink emit-signals=false sync=false max-buffers=2 drop=true"
        )

    def encode(self, frame: np.ndarray) -> Optional[bytes]:
        """Push one frame and pull the next available encoded chunk (or None)."""
        if self._appsrc is None or self._appsink is None:
            return None
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
            self._appsrc.emit("push-buffer", buf)
            sample = self._appsink.emit("try-pull-sample", 0)
            if sample is None:
                return None
            gst_buf = sample.get_buffer()
            ok, mapinfo = gst_buf.map(Gst.MapFlags.READ)
            if not ok:
                return None
            try:
                return bytes(mapinfo.data)
            finally:
                gst_buf.unmap(mapinfo)
        except Exception as exc:  # noqa: BLE001
            log.debug("MPP H.264 encode step failed: %s", exc)
            return None

    def stop(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.set_state(Gst.State.NULL)
            except Exception:  # noqa: BLE001
                pass
        self._pipeline = None
        self._appsrc = None
        self._appsink = None


__all__ = ["MppH264Encoder"]
