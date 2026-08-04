"""
SharedState: thread-safe wrapper around MemoryPool.

One lock per logical resource.  Every get/set operates on the
pre-allocated buffers from pool.py — no new arrays are ever created
inside the get/set methods.
"""

import threading
import time
from enum import Enum
from typing import Callable, Dict, Optional, Tuple

import numpy as np

from cat_follow.memory.pool import MemoryPool, BBOX_LEN, ODOM_LEN
from cat_follow.memory.pool import FRAME_SHAPE


class FrameConsumer(str, Enum):
    """Ring readers ordered by product priority under slot pressure.

    Camera never acquires; it only writes via ``try_get_write_buffer``.
    Admission reserves reclaimable slots so Camera > Detector > Recording >
    Web UI holds when pins are scarce.
    """

    DETECTOR = "detector"
    RECORDING = "recording"
    STREAM = "stream"

    @property
    def refusable(self) -> bool:
        """Whether admission may deny this consumer to protect a higher tier."""
        return self is not FrameConsumer.DETECTOR


# The writer needs two slots it can always cycle through: the latest published
# frame, which it never reclaims, plus one reclaimable slot to capture into.
# Readers therefore never pin more than ``FRAME_RING_N - 2`` distinct slots, and
# the camera never drops a frame because of leases.
CAMERA_RESERVED_SLOTS = 2


class DmabufRequeueError(RuntimeError):
    """A superseded V4L2 buffer could not be handed back to the driver.

    Raised only after the ring operation that triggered the requeue has already
    committed, so callers must not treat it as "the frame was not published".
    """


class FrameLease:
    """Pinned, read-only-by-contract view of a published frame-ring slot.

    The camera cannot reuse ``slot_idx`` until :meth:`release` is called.
    Consumers should use this as a context manager so exceptions cannot leak
    slot references.

    When ``dmabuf_buffer_index`` is set, :meth:`release` also requeues the
    underlying V4L2 buffer through the registered callback -- unless the slot
    is still the latest published frame, whose buffer is only handed back once
    a newer capture supersedes it.

    ``consumer`` records the admission tier that pinned the slot, so releasing
    hands that tier's share of the ring budget back rather than a generic pin.
    """

    __slots__ = (
        "_owner",
        "consumer",
        "frame",
        "frame_seq",
        "capture_started_ns",
        "published_ns",
        "slot_generation",
        "slot_idx",
        "dmabuf_fd",
        "dmabuf_buffer_index",
        "dmabuf_stride",
        "dmabuf_size",
        "_released",
    )

    def __init__(
        self,
        owner: "SharedState",
        slot_idx: int,
        frame_seq: int,
        capture_started_ns: int,
        published_ns: int,
        slot_generation: int,
        frame: Optional[np.ndarray],
        *,
        consumer: FrameConsumer,
        dmabuf_fd: int = -1,
        dmabuf_buffer_index: int = -1,
        dmabuf_stride: int = 0,
        dmabuf_size: int = 0,
    ) -> None:
        self._owner = owner
        self.consumer = consumer
        self.slot_idx = slot_idx
        self.frame_seq = frame_seq
        self.capture_started_ns = capture_started_ns
        self.published_ns = published_ns
        self.slot_generation = slot_generation
        self.frame = frame
        self.dmabuf_fd = dmabuf_fd
        self.dmabuf_buffer_index = dmabuf_buffer_index
        self.dmabuf_stride = dmabuf_stride
        self.dmabuf_size = dmabuf_size
        self._released = False

    @property
    def dmabuf(self) -> bool:
        """True when this lease carries a borrowed camera DMA-BUF fd."""
        return self.dmabuf_fd >= 0

    def __enter__(self) -> "FrameLease":
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.release()

    @property
    def stale(self) -> bool:
        """Whether the slot generation changed while this lease was held."""
        return self._owner._frame_lease_stale(self.slot_idx, self.slot_generation)

    def release(self) -> None:
        """Release this reader's pin. Safe to call more than once."""
        if self._released:
            return
        self._owner._release_frame_slot(self.slot_idx, self.consumer)
        self._released = True


class SharedState:
    """Thread-safe accessor for every shared buffer.

    Parameters
    ----------
    pool : MemoryPool
        The pre-allocated buffer pool (from ``allocate_pool()``).
        SharedState does **not** allocate any new arrays.
    """

    def __init__(self, pool: MemoryPool) -> None:
        self._pool = pool

        # One lock per logical resource
        self._lock_frame = threading.Lock()
        self._frame_cv = threading.Condition(self._lock_frame)
        self._lock_tracking = threading.Lock()
        self._lock_bbox_tracker = self._lock_tracking
        self._lock_bbox_detector = threading.Lock()
        self._lock_detector_detections = threading.Lock()
        self._detector_cv = threading.Condition(self._lock_detector_detections)
        self._lock_tracked_targets = self._lock_tracking
        self._lock_odometry = threading.Lock()
        self._lock_cat_injection = threading.Lock()

        # Legacy detector-model selection slot. Detection is now RKNN-only and
        # env-driven (CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH); this value is kept
        # only for backward-compatible web UI reads and is not used to pick a
        # model at runtime.
        self._lock_detector_model = threading.Lock()
        self._detector_model = "rknn"

        # Ring ownership. A single camera writer marks one slot as WRITING;
        # zero-copy readers pin published slots with a refcount. The writer
        # never reuses the latest slot or a pinned slot.
        self._ring_n = self._pool.frame_ring.shape[0]
        self._write_idx = 0
        self._latest_idx = -1
        self._active_write_idx: Optional[int] = None
        self._slot_refcounts = [0] * self._ring_n
        # Pins held by refusable consumers (recording / stream), tracked apart
        # from the total so a detector pin never spends the refusable budget.
        self._slot_refusable_refcounts = [0] * self._ring_n
        # Odd means write in progress; even means stable/published.
        self._slot_generations = [0] * self._ring_n
        self._slot_frame_seqs = [0] * self._ring_n
        self._slot_capture_started_ns = [0] * self._ring_n
        self._slot_published_ns = [0] * self._ring_n
        self._frame_seq = 0

        self._bbox_detector_gen = -1
        # Full post-NMS detection set consumed by PredictiveTracker. Tuples are
        # immutable snapshots, so readers never share mutable detector data.
        self._detector_detections = ()
        self._detector_detections_gen = -1
        # Role -> immutable target snapshot:
        # (track_id, x, y, w, h, confidence, frames_since_update, valid,
        #  detector_backed). The final field is optional for compatibility.
        # PRIMARY_CAT remains mirrored to bbox_tracker for behavior consumers.
        self._tracked_targets = {}
        # Monotonic generation for the tracker bbox, bumped on every publish.
        # The vision adapter keys stability/freshness off *changes* in this
        # generation so a frozen tracker (dead thread) ages out instead of the
        # adapter counting its own poll rate as "stable" frames.
        self._bbox_tracker_gen = 0

        # Optional hardware-scaled lores gray frame (motion source). Allocated
        # lazily on first publish so single-stream setups pay nothing.
        self._lock_lores = threading.Lock()
        self._lores_gray = None

        # Edge-triggered request from an accepted START_CHASE command. The
        # detector consumes it and loads/warmups the lazily-unloaded RKNN model
        # before the next detection.
        self._detector_warmup_requested = threading.Event()
        # Runtime-switchable live-frame test injection. The camera thread owns
        # the compositor; web/headless controls only update this flag.
        self._cat_injection_enabled = threading.Event()
        self._cat_injection_bbox = None

        # H.264 consumes camera DMA-BUFs directly. Only live inject requires
        # materializing the full NV12 frame in the NumPy ring.
        self._lock_stream_clients = threading.Lock()
        self._stream_clients = 0

        # PerceptionLifecycleManager intent channel for camera/detector threads.
        self._lock_perception_intent = threading.Lock()
        self._capture_active = True
        self._detector_required = False
        self._detector_mission_override = False
        self._recording_required = False
        self._stream_forced_off = False
        self._detector_force_off = False

        # Optional zerocopy session + V4L2 requeue hook (camera owns session).
        self._zerocopy_session = None
        self._dmabuf_requeue_cb: Optional[Callable[[int], bool]] = None
        # Buffers whose QBUF failed. They are still owned by us, not the driver,
        # so they must stay claimed here until a retry (or session teardown)
        # hands them back; forgetting one leaks it from the V4L2 pool.
        self._pending_dmabuf_requeue: list[int] = []
        self._slot_dmabuf_fd = [-1] * self._ring_n
        self._slot_dmabuf_buffer_index = [-1] * self._ring_n
        self._slot_dmabuf_size = [0] * self._ring_n
        self._slot_dmabuf_stride = [0] * self._ring_n

        # Latest-wins denials per consumer. Cheap observability for status and
        # board soak; a detector denial means detection itself lost a capture.
        self._admission_denied = {consumer.value: 0 for consumer in FrameConsumer}

    def set_perception_intent(
        self,
        *,
        capture_active: bool,
        detector_required: bool,
        detector_mission_override: bool,
        stream_forced_off: bool,
        recording_required: bool = False,
        detector_force_off: bool = False,
    ) -> None:
        """Publish lifecycle intent for camera/detector owner threads."""

        with self._lock_perception_intent:
            self._capture_active = bool(capture_active)
            self._detector_required = bool(detector_required)
            self._detector_mission_override = bool(detector_mission_override)
            self._recording_required = bool(recording_required)
            self._stream_forced_off = bool(stream_forced_off)
            self._detector_force_off = bool(detector_force_off)

    def get_perception_intent(self) -> dict:
        """Return effective intent: detector demand already masked by force-off."""
        with self._lock_perception_intent:
            force_off = self._detector_force_off
            return {
                "capture_active": self._capture_active,
                "detector_required": self._detector_required and not force_off,
                "detector_mission_override": (
                    self._detector_mission_override and not force_off
                ),
                "recording_required": self._recording_required,
                "stream_forced_off": self._stream_forced_off,
                "detector_force_off": force_off,
            }

    def capture_active(self) -> bool:
        with self._lock_perception_intent:
            return self._capture_active

    def detector_required(self) -> bool:
        with self._lock_perception_intent:
            return self._detector_required and not self._detector_force_off

    def detector_mission_override(self) -> bool:
        with self._lock_perception_intent:
            return self._detector_mission_override and not self._detector_force_off

    def recording_required(self) -> bool:
        with self._lock_perception_intent:
            return self._recording_required

    def stream_forced_off(self) -> bool:
        with self._lock_perception_intent:
            return self._stream_forced_off

    def _take_dmabuf_requeue_index_locked(self, idx: int) -> int:
        """Extract a V4L2 buffer to requeue. Caller must hold ``_lock_frame``."""
        if (
            self._slot_dmabuf_buffer_index[idx] >= 0
            and self._slot_refcounts[idx] == 0
            and self._dmabuf_requeue_cb is not None
        ):
            requeue_index = self._slot_dmabuf_buffer_index[idx]
            self._slot_dmabuf_buffer_index[idx] = -1
            self._slot_dmabuf_fd[idx] = -1
            self._slot_dmabuf_size[idx] = 0
            self._slot_dmabuf_stride[idx] = 0
            return requeue_index
        return -1

    def _claim_failed_requeue(self, requeue_index: int) -> None:
        with self._lock_frame:
            if requeue_index not in self._pending_dmabuf_requeue:
                self._pending_dmabuf_requeue.append(requeue_index)

    def _finish_dmabuf_requeue(self, requeue_index: int) -> None:
        if requeue_index < 0:
            return
        with self._lock_frame:
            requeue_cb = self._dmabuf_requeue_cb
        if requeue_cb is None:
            # The session went away between taking the index and handing it
            # back, so nothing can reach the driver right now.
            self._claim_failed_requeue(requeue_index)
            return
        try:
            ok = requeue_cb(requeue_index)
        except BaseException:
            self._claim_failed_requeue(requeue_index)
            raise
        if ok is False:
            self._claim_failed_requeue(requeue_index)
            raise DmabufRequeueError(
                f"dmabuf requeue failed for V4L2 buffer index {requeue_index}"
            )

    def _finish_dmabuf_requeues(self, requeue_indices) -> None:
        """Attempt every requeue before raising, so one QBUF failure part-way
        through cannot strand the buffers queued behind it."""
        first_error: Optional[DmabufRequeueError] = None
        for requeue_index in requeue_indices:
            try:
                self._finish_dmabuf_requeue(requeue_index)
            except DmabufRequeueError as exc:
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error

    def _drain_pending_dmabuf_requeues(self) -> None:
        """Best-effort retry of buffers an earlier QBUF failure left claimed.

        Never raises: this runs opportunistically on the camera thread, whose
        caller already escalated the original failure.
        """
        with self._lock_frame:
            requeue_cb = self._dmabuf_requeue_cb
            pending = self._pending_dmabuf_requeue
            if requeue_cb is None or not pending:
                return
            self._pending_dmabuf_requeue = []
        still_failed = []
        for requeue_index in pending:
            try:
                if requeue_cb(requeue_index) is False:
                    still_failed.append(requeue_index)
            except Exception:  # noqa: BLE001
                still_failed.append(requeue_index)
        if still_failed:
            with self._lock_frame:
                self._pending_dmabuf_requeue = (
                    still_failed + self._pending_dmabuf_requeue
                )

    def pending_dmabuf_requeues(self) -> tuple:
        """V4L2 buffer indices still owned by us after a failed QBUF."""
        with self._lock_frame:
            return tuple(self._pending_dmabuf_requeue)

    # ── frame_latest ─────────────────────────────────────────────────

    def request_detector_warmup(self) -> None:
        """Request asynchronous RKNN load/warmup on the detector thread."""
        self._detector_warmup_requested.set()

    def consume_detector_warmup_request(self) -> bool:
        """Return and clear a pending detector warmup request."""
        if not self._detector_warmup_requested.is_set():
            return False
        self._detector_warmup_requested.clear()
        return True

    def set_cat_injection_enabled(self, enabled: bool) -> None:
        """Enable/disable live camera cat pixels (never synthetic detections)."""
        if enabled:
            self._cat_injection_enabled.set()
        else:
            self._cat_injection_enabled.clear()
            with self._lock_cat_injection:
                self._cat_injection_bbox = None

    def cat_injection_enabled(self) -> bool:
        return self._cat_injection_enabled.is_set()

    def set_stream_clients(self, count: int) -> None:
        with self._lock_stream_clients:
            self._stream_clients = max(0, int(count))

    def inc_stream_clients(self) -> int:
        with self._lock_stream_clients:
            self._stream_clients += 1
            return self._stream_clients

    def dec_stream_clients(self) -> int:
        with self._lock_stream_clients:
            self._stream_clients = max(0, self._stream_clients - 1)
            return self._stream_clients

    def get_stream_clients(self) -> int:
        with self._lock_stream_clients:
            return self._stream_clients

    def needs_numpy_frame_pack(self) -> bool:
        """True only when live injection requires CPU NV12 pixels."""
        return self.cat_injection_enabled()

    def attach_zerocopy_session(self, session, *, requeue_cb) -> None:
        """Register the native capture session owned by the camera thread."""
        with self._lock_frame:
            self._zerocopy_session = session
            self._dmabuf_requeue_cb = requeue_cb

    def zerocopy_session(self):
        with self._lock_frame:
            return self._zerocopy_session

    def publish_dmabuf_from_write(
        self,
        *,
        capture_started_ns: int,
        dmabuf_fd: int,
        buffer_index: int,
        image_size: int,
        stride: int = 0,
        frame: Optional[np.ndarray] = None,
    ) -> None:
        """Publish the active ring slot with optional CPU NV12 + dmabuf metadata."""
        published_ns = time.monotonic_ns()
        if capture_started_ns < 0 or capture_started_ns > published_ns:
            raise ValueError("capture_started_ns must be a past monotonic timestamp")
        if frame is not None:
            # The reserved slot belongs exclusively to this single writer until
            # it publishes, so the full-frame inject copy is done before taking
            # the lock rather than stalling every reader behind ~450 KiB.
            with self._lock_frame:
                write_idx = self._active_write_idx
            if write_idx is None:
                raise RuntimeError("no active frame write to publish")
            np.copyto(self._pool.frame_ring[write_idx], frame)
        requeue_indices = []
        with self._lock_frame:
            idx = self._active_write_idx
            if idx is None:
                raise RuntimeError("no active frame write to publish")
            if self._slot_generations[idx] % 2 != 1:
                raise RuntimeError("active frame slot is not marked WRITING")
            replaced_index = self._take_dmabuf_requeue_index_locked(idx)
            if replaced_index >= 0:
                requeue_indices.append(replaced_index)
            previous_latest = self._latest_idx
            self._slot_generations[idx] += 1
            self._frame_seq += 1
            self._slot_frame_seqs[idx] = self._frame_seq
            self._slot_capture_started_ns[idx] = int(capture_started_ns)
            self._slot_published_ns[idx] = published_ns
            self._slot_dmabuf_fd[idx] = int(dmabuf_fd)
            self._slot_dmabuf_buffer_index[idx] = int(buffer_index)
            self._slot_dmabuf_size[idx] = int(image_size)
            self._slot_dmabuf_stride[idx] = int(stride)
            self._latest_idx = idx
            self._write_idx = (idx + 1) % self._ring_n
            self._active_write_idx = None
            if previous_latest >= 0 and previous_latest != idx:
                superseded_index = self._take_dmabuf_requeue_index_locked(
                    previous_latest
                )
                if superseded_index >= 0:
                    requeue_indices.append(superseded_index)
            self._frame_cv.notify_all()
        self._finish_dmabuf_requeues(requeue_indices)

    def set_cat_injection_bbox(self, bbox) -> None:
        """Publish the injected sprite's tight ``xyxy`` pixel bounds."""
        snapshot = None if bbox is None else tuple(int(value) for value in bbox)
        with self._lock_cat_injection:
            self._cat_injection_bbox = snapshot

    def get_cat_injection_status(self) -> dict:
        """Return injection state and ground-truth bbox for diagnostics only."""
        with self._lock_cat_injection:
            bbox = self._cat_injection_bbox
        return {
            "enabled": self._cat_injection_enabled.is_set(),
            "bbox": list(bbox) if bbox is not None else None,
            "detection_fallback": False,
        }

    def set_frame_latest(self, src: np.ndarray) -> None:
        """Copy *src* into the next write slot and publish it as latest.

        This is a convenience method for tests or simple producers. The
        high-performance camera loop should use the ``get_write_buffer()``
        and ``publish_latest_from_write()`` pair to avoid an extra
        copy if the camera driver can write directly into the shared
        buffer.
        """
        # Copy into current write buffer, then publish under lock.
        write_buf = self.try_get_write_buffer()
        if write_buf is None:
            return
        np.copyto(write_buf, src)
        self.publish_latest_from_write()

    def get_frame_latest(self, dst: np.ndarray) -> None:
        """Copy the currently published latest frame into *dst*.

        If no frame has been published yet, *dst* is zeroed.
        """
        # Basic shape checks to catch incorrect caller usage early.
        if dst.shape != FRAME_SHAPE:
            raise ValueError(f"dst has wrong shape {dst.shape}, expected {FRAME_SHAPE}")
        if dst.dtype != self._pool.frame_ring.dtype:
            raise ValueError(f"dst has wrong dtype {dst.dtype}, expected {self._pool.frame_ring.dtype}")

        with self._lock_frame:
            if self._latest_idx < 0:
                dst.fill(0)
            else:
                np.copyto(dst, self._pool.frame_ring[self._latest_idx])

    # ── ring helpers (camera use) ─────────────────────────────────────

    def try_get_write_buffer(self) -> Optional[np.ndarray]:
        """Reserve and return a free ring slot, or ``None`` if all are busy.

        This is non-blocking by design: camera capture is latest-wins and must
        drop a frame rather than wait behind detector or stream work.
        """
        self._drain_pending_dmabuf_requeues()
        requeue_index = -1
        with self._lock_frame:
            if self._active_write_idx is not None:
                raise RuntimeError("frame write already active")

            for offset in range(self._ring_n):
                idx = (self._write_idx + offset) % self._ring_n
                if idx == self._latest_idx or self._slot_refcounts[idx] != 0:
                    continue
                requeue_index = self._take_dmabuf_requeue_index_locked(idx)
                self._active_write_idx = idx
                self._slot_generations[idx] += 1  # even -> odd (WRITING)
                buf = self._pool.frame_ring[idx]
                break
            else:
                buf = None
        if buf is None:
            return None
        try:
            self._finish_dmabuf_requeue(requeue_index)
        except BaseException:
            # The reservation is only useful if its previous dmabuf went back
            # to the driver. Give the slot up so a failed QBUF cannot wedge
            # every later capture behind "frame write already active".
            self.abort_frame_write()
            raise
        return buf

    def get_write_buffer(self) -> np.ndarray:
        """Return a writable view into the pool's current write slot.

        The caller (camera thread) may write the frame data into this
        buffer (in-place). After writing, call
        ``publish_latest_from_write()`` to make the frame visible to
        readers.
        """
        buf = self.try_get_write_buffer()
        if buf is None:
            raise RuntimeError("no free frame-ring write slot")
        if __debug__:
            assert buf.shape == FRAME_SHAPE, f"write buffer shape {buf.shape} != {FRAME_SHAPE}"
            assert buf.dtype == self._pool.frame_ring.dtype
        return buf

    def publish_latest_from_write(
        self, *, capture_started_ns: Optional[int] = None
    ) -> None:
        """Publish the active slot with capture timing and advance its sequence."""
        published_ns = time.monotonic_ns()
        if capture_started_ns is None:
            capture_started_ns = published_ns
        capture_started_ns = int(capture_started_ns)
        if capture_started_ns < 0 or capture_started_ns > published_ns:
            raise ValueError("capture_started_ns must be a past monotonic timestamp")
        requeue_index = -1
        with self._lock_frame:
            idx = self._active_write_idx
            if idx is None:
                raise RuntimeError("no active frame write to publish")
            if self._slot_generations[idx] % 2 != 1:
                raise RuntimeError("active frame slot is not marked WRITING")
            previous_latest = self._latest_idx
            self._slot_generations[idx] += 1  # odd -> even (stable)
            self._frame_seq += 1
            self._slot_frame_seqs[idx] = self._frame_seq
            self._slot_capture_started_ns[idx] = capture_started_ns
            self._slot_published_ns[idx] = published_ns
            self._latest_idx = idx
            self._write_idx = (idx + 1) % self._ring_n
            self._active_write_idx = None
            if previous_latest >= 0 and previous_latest != idx:
                requeue_index = self._take_dmabuf_requeue_index_locked(
                    previous_latest
                )
            self._frame_cv.notify_all()
        self._finish_dmabuf_requeue(requeue_index)

    def abort_frame_write(self) -> None:
        """Cancel an active write reservation without publishing its pixels."""
        with self._lock_frame:
            idx = self._active_write_idx
            if idx is None:
                return
            if self._slot_generations[idx] % 2 == 1:
                self._slot_generations[idx] += 1
            self._active_write_idx = None
            self._write_idx = (idx + 1) % self._ring_n

    def _pinned_slot_counts_locked(self) -> Tuple[int, int]:
        """Distinct pinned slots as ``(total, refusable)``.

        The active write slot is deliberately excluded: readers only ever pin
        the latest published slot, which the writer never reserves.
        """
        total = 0
        refusable = 0
        for idx in range(self._ring_n):
            if self._slot_refcounts[idx] > 0:
                total += 1
                if self._slot_refusable_refcounts[idx] > 0:
                    refusable += 1
        return total, refusable

    def _admit_acquire_locked(
        self, consumer: FrameConsumer, latest_idx: int
    ) -> bool:
        """Whether *consumer* may pin ``latest_idx`` without starving higher tiers.

        Priority is Camera > Detector > Recording > Web UI:

        - an already-pinned slot costs nothing, so same-latest multi-reader
          (detector + stream on the current capture) is always admitted;
        - a new distinct pin must leave ``CAMERA_RESERVED_SLOTS`` slots for the
          writer, which is what keeps the camera from dropping on leases;
        - while the detector is required it keeps one further distinct pin, so
          a slow RKNN tick can still pin the next capture;
        - while recording is required the stream gives up one beyond that, so
          recording wins the last refusable distinct pin. Reservations follow
          demand, so an idle recorder does not cost the live view its pin.
        """
        if self._slot_refcounts[latest_idx] > 0:
            return True

        pinned, refusable_pinned = self._pinned_slot_counts_locked()
        budget = self._ring_n - CAMERA_RESERVED_SLOTS
        if pinned + 1 > budget:
            return False
        if not consumer.refusable:
            return True

        # Intent flags are published under a different lock, and reading them
        # here without it keeps the frame lock free of nested acquisition. A
        # one-tick-stale snapshot only shifts which reader drops a frame.
        detector_required = self._detector_required and not self._detector_force_off
        recording_required = self._recording_required

        limit = budget
        if detector_required:
            limit -= 1
        if recording_required and consumer is FrameConsumer.STREAM:
            limit -= 1
        return refusable_pinned + 1 <= limit

    def acquire_latest_frame(
        self, *, consumer: FrameConsumer
    ) -> Optional[FrameLease]:
        """Pin and return the latest stable frame without copying pixels.

        Admission keeps the camera writer and the detector ahead of recording
        and the web UI: a refusable consumer is denied a *new* distinct slot
        that a higher tier still needs, while pinning an already-pinned slot is
        always allowed. Refusal returns ``None`` — a latest-wins drop at the
        reader, never a camera block.
        """
        with self._lock_frame:
            idx = self._latest_idx
            if idx < 0:
                return None
            slot_generation = self._slot_generations[idx]
            if slot_generation % 2 != 0:
                return None
            if not self._admit_acquire_locked(consumer, idx):
                self._admission_denied[consumer.value] += 1
                return None
            self._slot_refcounts[idx] += 1
            if consumer.refusable:
                self._slot_refusable_refcounts[idx] += 1
            frame_seq = self._slot_frame_seqs[idx]
            dmabuf_fd = self._slot_dmabuf_fd[idx]
            buffer_index = self._slot_dmabuf_buffer_index[idx]
            image_size = self._slot_dmabuf_size[idx]
            stride = self._slot_dmabuf_stride[idx]
            frame_view = None
            if dmabuf_fd < 0:
                frame_view = self._pool.frame_ring[idx]
            elif self.needs_numpy_frame_pack():
                frame_view = self._pool.frame_ring[idx]
            return FrameLease(
                self,
                idx,
                frame_seq,
                self._slot_capture_started_ns[idx],
                self._slot_published_ns[idx],
                slot_generation,
                frame_view,
                consumer=consumer,
                dmabuf_fd=dmabuf_fd,
                dmabuf_buffer_index=buffer_index,
                dmabuf_stride=stride,
                dmabuf_size=image_size,
            )

    def admission_denied_counts(self) -> Dict[str, int]:
        """Acquire refusals per consumer, keyed by ``FrameConsumer`` value.

        A non-zero ``detector`` count means detection lost a capture to slot
        pressure, which is more serious than a recording or stream drop.
        """
        with self._lock_frame:
            return dict(self._admission_denied)

    def latest_frame_generation(self) -> int:
        """Return the latest published capture sequence, or zero before startup."""
        with self._lock_frame:
            return self._frame_seq

    def wait_for_new_frame(
        self,
        last_generation: int,
        stop_event: threading.Event,
        *,
        timeout_s: Optional[float] = None,
    ) -> int:
        """Wait for a capture generation newer than *last_generation*.

        The bounded wait slices keep shutdown responsive even though
        ``threading.Event`` and ``threading.Condition`` cannot be waited on
        atomically.
        """
        deadline = (
            None if timeout_s is None else time.monotonic() + max(0.0, timeout_s)
        )
        with self._frame_cv:
            while self._frame_seq <= last_generation and not stop_event.is_set():
                wait_s = 0.05
                if deadline is not None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    wait_s = min(wait_s, remaining)
                self._frame_cv.wait(wait_s)
            return self._frame_seq

    def _release_frame_slot(self, slot_idx: int, consumer: FrameConsumer) -> None:
        requeue_index = -1
        requeue_cb = None
        with self._lock_frame:
            if not 0 <= slot_idx < self._ring_n:
                raise ValueError(f"invalid frame-ring slot {slot_idx}")
            # Both counters are validated before either is touched so a bad
            # release cannot leave admission accounting inconsistent.
            if self._slot_refcounts[slot_idx] <= 0:
                raise RuntimeError(f"frame-ring slot {slot_idx} is not acquired")
            if consumer.refusable and self._slot_refusable_refcounts[slot_idx] <= 0:
                raise RuntimeError(
                    f"frame-ring slot {slot_idx} has no {consumer.value} pin"
                )
            self._slot_refcounts[slot_idx] -= 1
            if consumer.refusable:
                self._slot_refusable_refcounts[slot_idx] -= 1
            if (
                self._slot_refcounts[slot_idx] == 0
                and self._slot_dmabuf_buffer_index[slot_idx] >= 0
                and slot_idx != self._latest_idx
            ):
                requeue_index = self._slot_dmabuf_buffer_index[slot_idx]
                self._slot_dmabuf_buffer_index[slot_idx] = -1
                self._slot_dmabuf_fd[slot_idx] = -1
                self._slot_dmabuf_size[slot_idx] = 0
                self._slot_dmabuf_stride[slot_idx] = 0
        self._finish_dmabuf_requeue(requeue_index)

    def _frame_lease_stale(self, slot_idx: int, slot_generation: int) -> bool:
        with self._lock_frame:
            return self._slot_generations[slot_idx] != slot_generation

    # ── bbox_tracker ─────────────────────────────────────────────────

    def set_bbox_tracker(
        self,
        x: float,
        y: float,
        w: float,
        h: float,
        valid: float,
        *,
        refresh_generation: bool = True,
    ) -> None:
        """Write tracker bbox, optionally marking it as a fresh observation."""
        with self._lock_bbox_tracker:
            self._write_bbox_tracker(x, y, w, h, valid, refresh_generation)

    def _write_bbox_tracker(
        self,
        x: float,
        y: float,
        w: float,
        h: float,
        valid: float,
        refresh_generation: bool,
    ) -> None:
        buf = self._pool.bbox_tracker
        buf[0] = x
        buf[1] = y
        buf[2] = w
        buf[3] = h
        buf[4] = valid
        if refresh_generation:
            self._bbox_tracker_gen += 1

    def get_bbox_tracker(self) -> Tuple[float, float, float, float, float]:
        """Return a snapshot ``(x, y, w, h, valid)`` under lock.

        Returns a plain tuple (cheap, immutable) so the caller holds a
        consistent copy that won't change once the lock is released.
        """
        with self._lock_bbox_tracker:
            buf = self._pool.bbox_tracker
            return (float(buf[0]), float(buf[1]), float(buf[2]),
                    float(buf[3]), float(buf[4]))

    def get_bbox_tracker_with_gen(
        self,
    ) -> Tuple[float, float, float, float, float, int]:
        """Return ``(x, y, w, h, valid, gen)`` under lock.

        ``gen`` increments on every publish, so a consumer can tell a genuinely
        new tracker observation from a repeated poll of an unchanged buffer.
        """
        with self._lock_bbox_tracker:
            buf = self._pool.bbox_tracker
            return (float(buf[0]), float(buf[1]), float(buf[2]),
                    float(buf[3]), float(buf[4]), int(self._bbox_tracker_gen))

    # ── bbox_detector ────────────────────────────────────────────────

    def set_bbox_detector(
        self,
        x: float,
        y: float,
        w: float,
        h: float,
        valid: float,
        frame_gen: int = -1,
    ) -> None:
        """Write detector bbox into the pre-allocated array under lock.

        ``frame_gen`` is the capture sequence from the detector's frame lease;
        it lets the tracker match the bbox to the exact inferred pixels.
        """
        with self._lock_bbox_detector:
            buf = self._pool.bbox_detector
            buf[0] = x
            buf[1] = y
            buf[2] = w
            buf[3] = h
            buf[4] = valid
            self._bbox_detector_gen = int(frame_gen)

    def get_bbox_detector(self) -> Tuple[float, float, float, float, float]:
        """Return a snapshot ``(x, y, w, h, valid)`` under lock."""
        with self._lock_bbox_detector:
            buf = self._pool.bbox_detector
            return (float(buf[0]), float(buf[1]), float(buf[2]),
                    float(buf[3]), float(buf[4]))

    def get_bbox_detector_with_gen(
        self,
    ) -> Tuple[float, float, float, float, float, int]:
        """Return ``(x, y, w, h, valid, frame_gen)`` under lock."""
        with self._lock_bbox_detector:
            buf = self._pool.bbox_detector
            return (float(buf[0]), float(buf[1]), float(buf[2]),
                    float(buf[3]), float(buf[4]), int(self._bbox_detector_gen))

    def set_detector_detections(self, detections, frame_gen: int) -> None:
        """Publish all ``(x1,y1,x2,y2,confidence,class_id)`` detections."""
        snapshot = tuple(
            (
                float(det[0]),
                float(det[1]),
                float(det[2]),
                float(det[3]),
                float(det[4]),
                int(det[5]),
            )
            for det in detections
        )
        with self._lock_detector_detections:
            self._detector_detections = snapshot
            self._detector_detections_gen = int(frame_gen)
            self._detector_cv.notify_all()

    def get_detector_detections_with_gen(self):
        """Return the immutable full detection snapshot and its frame generation."""
        with self._lock_detector_detections:
            return self._detector_detections, self._detector_detections_gen

    def wait_for_detector_update(
        self,
        last_generation: int,
        stop_event: threading.Event,
        *,
        timeout_s: float,
    ) -> int:
        """Wait for detector output newer than *last_generation* or a deadline."""
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._detector_cv:
            while (
                self._detector_detections_gen <= last_generation
                and not stop_event.is_set()
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._detector_cv.wait(min(0.05, remaining))
            return self._detector_detections_gen

    def set_tracked_targets(self, targets) -> None:
        """Publish immutable PRIMARY_CAT/SECONDARY_CAT tracking snapshots."""
        snapshot = self._tracked_targets_snapshot(targets)
        with self._lock_tracked_targets:
            self._tracked_targets = snapshot

    @staticmethod
    def _tracked_targets_snapshot(targets) -> dict:
        snapshot = {}
        for role, target in targets.items():
            values = (
                int(target[0]),
                float(target[1]),
                float(target[2]),
                float(target[3]),
                float(target[4]),
                float(target[5]),
                int(target[6]),
                float(target[7]),
            )
            if len(target) > 8:
                values += (float(target[8]),)
            snapshot[str(role)] = values
        return snapshot

    def publish_tracking_snapshot(
        self,
        targets,
        bbox: Tuple[float, float, float, float, float],
        *,
        detector_backed: bool,
    ) -> None:
        """Atomically publish role targets and the legacy primary bbox.

        Prediction-only/coasting publications update display coordinates but
        do not advance the observation generation consumed by VisionAdapter.
        """
        snapshot = self._tracked_targets_snapshot(targets)
        with self._lock_tracking:
            self._tracked_targets = snapshot
            self._write_bbox_tracker(*bbox, detector_backed)

    def get_tracking_snapshot(self):
        """Return role targets and generation-aware primary bbox atomically."""
        with self._lock_tracking:
            buf = self._pool.bbox_tracker
            bbox = (
                float(buf[0]),
                float(buf[1]),
                float(buf[2]),
                float(buf[3]),
                float(buf[4]),
                int(self._bbox_tracker_gen),
            )
            return dict(self._tracked_targets), bbox

    def get_tracked_targets(self) -> dict:
        """Return a detached role-to-target snapshot."""
        with self._lock_tracked_targets:
            return dict(self._tracked_targets)

    # ── odometry ─────────────────────────────────────────────────────

    def set_odometry(self, x: float, y: float, heading_deg: float) -> None:
        """Write odometry into the pre-allocated array under lock."""
        with self._lock_odometry:
            buf = self._pool.odometry
            buf[0] = x
            buf[1] = y
            buf[2] = heading_deg

    def get_odometry(self) -> Tuple[float, float, float]:
        """Return a snapshot ``(x, y, heading_deg)`` under lock."""
        with self._lock_odometry:
            buf = self._pool.odometry
            return (float(buf[0]), float(buf[1]), float(buf[2]))

    # ── lores gray frame (optional hardware-scaled motion source) ─────

    def crop_buffer_for_slot(self, slot_idx: int) -> np.ndarray:
        """Return the paired NV12 crop buffer for capture slot *slot_idx*."""
        ring_n = self._pool.crop_ring.shape[0]
        return self._pool.crop_ring[int(slot_idx) % ring_n]

    def set_lores_gray(self, gray: np.ndarray) -> None:
        """Publish a single-channel lores gray frame for motion detection.

        The backing buffer is allocated on first use (and reallocated if the
        lores geometry changes), then reused in-place for every later frame.
        """
        with self._lock_lores:
            if self._lores_gray is None or self._lores_gray.shape != gray.shape:
                self._lores_gray = np.empty(gray.shape, dtype=np.uint8)
            np.copyto(self._lores_gray, gray)

    def get_lores_gray(
        self, dst: "np.ndarray | None" = None
    ) -> "np.ndarray | None":
        """Copy lores gray into reusable *dst*, allocating only when needed."""
        with self._lock_lores:
            if self._lores_gray is None:
                return None
            if (
                dst is None
                or dst.shape != self._lores_gray.shape
                or dst.dtype != self._lores_gray.dtype
            ):
                dst = np.empty_like(self._lores_gray)
            np.copyto(dst, self._lores_gray)
            return dst

    def has_lores_gray(self) -> bool:
        """Return True if a lores gray frame has been published."""
        with self._lock_lores:
            return self._lores_gray is not None

    # ── detector model selection ──────────────────────────────────────
    def set_detector_model(self, model_key: str) -> None:
        """Set the active detector model key (currently ``rknn``)."""
        with self._lock_detector_model:
            self._detector_model = str(model_key)

    def get_detector_model(self) -> str:
        """Return the currently-selected detector model key."""
        with self._lock_detector_model:
            return str(self._detector_model)
