"""Hardware chase-recording encoder: camera DMA-BUF -> MPP H.264 -> Matroska.

Recording runs its own MPP instance rather than sharing the monitoring
stream's. The target design requires recordings to continue with no browser
connected, and a recording needs its own bitrate and keyframe interval instead
of the low-latency settings the live stream is tuned for.

Frames are leased straight from the camera frame ring as DMA-BUF descriptors,
so no NV12 frame is ever copied through the CPU for recording.

GStreamer muxes Matroska but does not write it: the finished bytes come back
through an appsink and :class:`~cat_follow.perception.recording_store.RecordingStore`
writes them, keeping segment naming, the disk quota, retention, and ``.part``
crash recovery in one place.

Every segment gets a fresh muxer, because a Matroska file that reuses an
earlier segment's headers is not independently playable.

Off the board (no GStreamer or no ``mpph264enc``) there is no real encoder.
:func:`create_recording_encoder` then returns the deterministic stub only when
``CAT_FOLLOW_RECORDING_ALLOW_STUB=1`` is set, and otherwise returns ``None`` so
the writer reports recording as unavailable instead of producing files that
look valid but contain synthetic bytes.
"""

from __future__ import annotations

import os
from typing import Optional, Protocol

from cat_follow.logger import get_logger
from cat_follow.perception.h264_encoder import MppH264Encoder
from cat_follow.perception.recording_writer import (
    AccessUnitEncoder,
    StubH264Encoder,
)

log = get_logger("perception.recording_encoder")

DEFAULT_RECORDING_FPS = 15
DEFAULT_RECORDING_BITRATE_KBPS = 4000


class FrameLeaseSource(Protocol):
    """The frame-ring subset the recording encoder consumes."""

    def acquire_latest_frame(self): ...


def allow_stub_recording() -> bool:
    """Whether an operator opted into synthetic recordings on this host."""
    raw = os.getenv("CAT_FOLLOW_RECORDING_ALLOW_STUB", "")
    return raw.strip().lower() in {"1", "true", "yes", "on"}


class MppRecordingEncoder:
    """Encode leased camera DMA-BUFs into Matroska segments."""

    def __init__(
        self,
        frame_source: FrameLeaseSource,
        *,
        width: int,
        height: int,
        fps: int = DEFAULT_RECORDING_FPS,
        bitrate_kbps: int = DEFAULT_RECORDING_BITRATE_KBPS,
        encoder_factory=None,
    ) -> None:
        self._frames = frame_source
        self._width = int(width)
        self._height = int(height)
        self._fps = int(fps)
        self._bitrate_kbps = int(bitrate_kbps)
        self._encoder_factory = encoder_factory or self._default_encoder
        self._encoder: Optional[MppH264Encoder] = None
        self._started = False
        self._last_frame_seq = -1

    @staticmethod
    def available() -> bool:
        return MppH264Encoder.available()

    def _default_encoder(self) -> MppH264Encoder:
        return MppH264Encoder(
            self._width,
            self._height,
            fps=self._fps,
            bitrate_kbps=self._bitrate_kbps,
            pixel_format="NV12",
            container="matroska",
        )

    def start(self) -> bool:
        """Mark the encoder usable; each segment builds its own pipeline."""
        self._started = True
        return True

    def stop(self) -> None:
        self._started = False
        self._teardown()

    def begin_segment(self) -> bool:
        """Build a fresh muxer so this segment is independently playable."""
        self._teardown()
        if not self._started:
            return False
        encoder = self._encoder_factory()
        if not encoder.start():
            return False
        self._encoder = encoder
        self._last_frame_seq = -1
        return True

    def end_segment(self) -> list[bytes]:
        """Flush the muxer and return the bytes that close this segment."""
        encoder = self._encoder
        if encoder is None:
            return []
        tail: list[bytes] = []
        try:
            tail.extend(encoder.finish())
        except Exception as exc:  # noqa: BLE001
            log.warning("Recording flush failed: %s", exc)
        finally:
            self._teardown()
        return tail

    def encode_tick(self, *, now_ms: int) -> list[bytes]:  # noqa: ARG002
        encoder = self._encoder
        if encoder is None:
            return []
        lease = self._frames.acquire_latest_frame()
        if lease is None:
            return encoder.poll()
        if not getattr(lease, "dmabuf", False):
            # Recording is a DMA-BUF consumer only; a NumPy-only frame would
            # mean the zero-copy camera path is not active.
            lease.release()
            return encoder.poll()
        if lease.frame_seq == self._last_frame_seq:
            # Latest-wins capture can outpace or lag this tick; re-encoding the
            # same capture would duplicate a frame in the file.
            lease.release()
            return encoder.poll()
        self._last_frame_seq = lease.frame_seq
        # Ownership of the lease transfers to the encoder, which holds the
        # camera buffer pinned until MPP has read it.
        return encoder.submit_dmabuf(lease)

    def _teardown(self) -> None:
        encoder = self._encoder
        self._encoder = None
        if encoder is None:
            return
        try:
            encoder.stop()
        except Exception as exc:  # noqa: BLE001
            log.warning("Recording encoder stop failed: %s", exc)


def create_recording_encoder(
    frame_source: Optional[FrameLeaseSource],
    *,
    width: int,
    height: int,
    fps: int = DEFAULT_RECORDING_FPS,
    bitrate_kbps: int = DEFAULT_RECORDING_BITRATE_KBPS,
) -> Optional[AccessUnitEncoder]:
    """Pick the recording encoder for this host, or ``None`` if there is none."""
    if frame_source is not None and MppRecordingEncoder.available():
        return MppRecordingEncoder(
            frame_source,
            width=width,
            height=height,
            fps=fps,
            bitrate_kbps=bitrate_kbps,
        )
    if allow_stub_recording():
        log.warning(
            "CAT_FOLLOW_RECORDING_ALLOW_STUB is set: recordings will contain "
            "synthetic access units, not real video."
        )
        return StubH264Encoder()
    log.error(
        "No hardware H.264 recording encoder is available (GStreamer or "
        "mpph264enc missing); chase recording will report unavailable."
    )
    return None


__all__ = [
    "DEFAULT_RECORDING_BITRATE_KBPS",
    "DEFAULT_RECORDING_FPS",
    "MppRecordingEncoder",
    "allow_stub_recording",
    "create_recording_encoder",
]
