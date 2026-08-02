"""Center-bottom NV12 crop via Rockchip RGA when available, else CPU."""

from __future__ import annotations

import atexit
import threading
from typing import Any, Optional, Tuple

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.vision.nv12_utils import (
    center_bottom_nv12_region,
    extract_nv12_crop,
    validate_nv12,
)

log = get_logger("vision.rga_crop")

Region = Tuple[int, int, int, int]

_rga_probe_lock = threading.Lock()
_rga_available: Optional[bool] = None
_rga_logged = False


class _RgaCropBackend:
    """Thread-safe, reusable GStreamer RGA crop pipeline."""

    def __init__(self) -> None:
        # ``_crop_lock`` serializes crops; ``_state_lock`` only guards the
        # element references, so ``close()`` never waits behind a blocked push
        # and can tear the pipeline down to unblock it.
        self._crop_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._gst: Any = None
        self._pipeline: Any = None
        self._appsrc: Any = None
        self._appsink: Any = None
        self._config: Optional[tuple[int, int, int, int, Region]] = None
        # Bumped by close(). A rebuild that spans a close must not publish its
        # pipeline, or shutdown would leave a PLAYING pipeline nobody stops.
        self._epoch = 0

    @staticmethod
    def _load_gst():
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        return Gst

    def _detach(self, *, closing: bool = False) -> Any:
        """Clear the element references and return the pipeline to stop."""
        with self._state_lock:
            pipeline = self._pipeline
            self._pipeline = None
            self._appsrc = None
            self._appsink = None
            self._config = None
            if closing:
                self._epoch += 1
        return pipeline

    def _stop_pipeline(self, pipeline: Any) -> None:
        if pipeline is None:
            return
        try:
            pipeline.set_state(self._gst.State.NULL)
        except Exception:  # noqa: BLE001
            log.exception("Failed to stop RGA crop pipeline")

    def close(self) -> None:
        """Stop the active pipeline; a later crop may create a fresh one.

        Safe to call while a crop is blocked in ``push-buffer``: dropping the
        pipeline to NULL unblocks it so shutdown cannot stall.
        """
        self._stop_pipeline(self._detach(closing=True))

    def _ensure_pipeline(
        self,
        src_w: int,
        src_h: int,
        crop_w: int,
        crop_h: int,
        region: Region,
    ) -> tuple[Any, Any, Any]:
        config = (src_w, src_h, crop_w, crop_h, region)
        with self._state_lock:
            if self._pipeline is not None and self._config == config:
                return self._gst, self._appsrc, self._appsink
            epoch = self._epoch

        self._stop_pipeline(self._detach())
        if self._gst is None:
            self._gst = self._load_gst()
        Gst = self._gst
        x, y, w, h = region
        pipeline_desc = (
            "appsrc name=src is-live=true block=true format=time "
            f"caps=video/x-raw,format=NV12,width={src_w},height={src_h},framerate=30/1 ! "
            f"rgaconvert crop-x={x} crop-y={y} crop-width={w} crop-height={h} ! "
            f"video/x-raw,format=NV12,width={crop_w},height={crop_h} ! "
            "appsink name=sink sync=false max-buffers=1 drop=true"
        )
        pipeline = Gst.parse_launch(pipeline_desc)
        appsrc = pipeline.get_by_name("src")
        appsink = pipeline.get_by_name("sink")
        if appsrc is None or appsink is None:
            pipeline.set_state(Gst.State.NULL)
            raise RuntimeError("failed to build RGA crop pipeline")

        try:
            state_result = pipeline.set_state(Gst.State.PLAYING)
            if state_result == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("failed to start RGA crop pipeline")
        except Exception:
            pipeline.set_state(Gst.State.NULL)
            raise

        with self._state_lock:
            superseded = self._epoch != epoch
            if not superseded:
                self._pipeline = pipeline
                self._appsrc = appsrc
                self._appsink = appsink
                self._config = config
        if superseded:
            # close() ran while this pipeline was being built; publishing it
            # now would leave a PLAYING pipeline that nothing tears down.
            self._stop_pipeline(pipeline)
            raise RuntimeError("RGA crop backend closed during pipeline setup")
        return Gst, appsrc, appsink

    def crop(
        self,
        frame_nv12: np.ndarray,
        src_w: int,
        src_h: int,
        crop_w: int,
        crop_h: int,
        *,
        dst: np.ndarray,
        region: Region,
    ) -> tuple[np.ndarray, int, int]:
        """Run one crop, retaining the pipeline for matching future crops."""
        packed = np.ascontiguousarray(validate_nv12(frame_nv12, src_w, src_h))
        with self._crop_lock:
            try:
                Gst, appsrc, appsink = self._ensure_pipeline(
                    src_w, src_h, crop_w, crop_h, region
                )
                buf = Gst.Buffer.new_allocate(None, packed.nbytes, None)
                copied = buf.fill(0, memoryview(packed).cast("B"))
                if copied != packed.nbytes:
                    raise RuntimeError(
                        f"copied {copied} of {packed.nbytes} bytes into RGA input"
                    )
                flow = appsrc.emit("push-buffer", buf)
                if flow != Gst.FlowReturn.OK:
                    raise RuntimeError(f"RGA input push failed: {flow}")
                sample = appsink.emit("try-pull-sample", int(2 * Gst.SECOND))
                if sample is None:
                    raise RuntimeError("RGA crop produced no output sample")
                out_buf = sample.get_buffer()
                success, map_info = out_buf.map(Gst.MapFlags.READ)
                if not success:
                    raise RuntimeError("failed to map RGA output buffer")
                try:
                    if map_info.size < dst.nbytes:
                        raise RuntimeError(
                            f"RGA output has {map_info.size} bytes, "
                            f"expected {dst.nbytes}"
                        )
                    out = np.frombuffer(
                        map_info.data, dtype=np.uint8, count=dst.size
                    )
                    np.copyto(dst, out.reshape(dst.shape))
                finally:
                    out_buf.unmap(map_info)
            except Exception:
                # A failed appsrc/pipeline is not safe to reuse. Reset it so the
                # next frame gets one clean reconstruction after CPU fallback.
                self._stop_pipeline(self._detach())
                raise
        return dst, region[0], region[1]


_rga_backend = _RgaCropBackend()
atexit.register(_rga_backend.close)


def rga_available() -> bool:
    """Return True when a Rockchip RGA GStreamer element is present."""
    global _rga_available
    with _rga_probe_lock:
        if _rga_available is not None:
            return _rga_available
        try:
            import gi

            gi.require_version("Gst", "1.0")
            from gi.repository import Gst

            Gst.init(None)
            factory = Gst.ElementFactory.find("rgaconvert")
            _rga_available = factory is not None
        except Exception:  # noqa: BLE001
            _rga_available = False
        return _rga_available


def crop_backend_name() -> str:
    return "rga" if rga_available() else "cpu"


def crop_center_bottom_nv12(
    frame_nv12: np.ndarray,
    src_w: int,
    src_h: int,
    crop_w: int,
    crop_h: int,
    *,
    dst: np.ndarray,
    region: Optional[Region] = None,
) -> tuple[np.ndarray, int, int]:
    """Crop *frame_nv12* to a packed NV12 *dst* buffer.

    Uses hardware RGA when ``rgaconvert`` is available; otherwise falls back to
    the CPU ``extract_nv12_crop`` path (Option C behaviour).
    """
    global _rga_logged
    region = region or center_bottom_nv12_region(src_w, src_h, crop_w, crop_h)
    if rga_available():
        try:
            return _rga_crop_center_bottom(
                frame_nv12, src_w, src_h, crop_w, crop_h, dst=dst, region=region
            )
        except Exception as exc:  # noqa: BLE001
            if not _rga_logged:
                log.warning(
                    "RGA crop failed (%s); using CPU extract_nv12_crop", exc
                )
                _rga_logged = True
    elif not _rga_logged:
        log.info("RGA unavailable; using CPU center-bottom NV12 crop")
        _rga_logged = True

    packed = validate_nv12(frame_nv12, src_w, src_h)
    extract_nv12_crop(packed, src_w, src_h, region, dst=dst)
    return dst, region[0], region[1]


def close_rga_crop_backend() -> None:
    """Release the reusable RGA pipeline, if one has been created."""
    _rga_backend.close()


def _rga_crop_center_bottom(
    frame_nv12: np.ndarray,
    src_w: int,
    src_h: int,
    crop_w: int,
    crop_h: int,
    *,
    dst: np.ndarray,
    region: Region,
) -> tuple[np.ndarray, int, int]:
    """Crop through the process-wide reusable GStreamer RGA backend."""
    return _rga_backend.crop(
        frame_nv12,
        src_w,
        src_h,
        crop_w,
        crop_h,
        dst=dst,
        region=region,
    )


__all__ = [
    "close_rga_crop_backend",
    "crop_backend_name",
    "crop_center_bottom_nv12",
    "rga_available",
]
