"""
Unit tests for cat_follow.memory.shared_state — thread-safe SharedState.

Covers:
  - Single-thread get/set round-trips for every resource.
  - Concurrent writer + reader on bbox_tracker to verify no torn reads.
  - Frame copy-out and zero-copy lease ownership.

Run:
    python -m pytest tests/test_shared_state.py -v
or:
    python tests/test_shared_state.py
"""

import sys
import os
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest

from cat_follow.memory.pool import allocate_pool, FRAME_RING_N, FRAME_SHAPE, BBOX_LEN
from cat_follow.memory.shared_state import (
    CAMERA_RESERVED_SLOTS,
    DmabufRequeueError,
    FrameConsumer,
    SharedState,
)


# ── helpers ──────────────────────────────────────────────────────────────

def _make_shared() -> SharedState:
    return SharedState(allocate_pool())


# ── single-thread tests ─────────────────────────────────────────────────

def test_needs_numpy_frame_pack():
    shared = _make_shared()
    assert shared.needs_numpy_frame_pack() is False
    shared.inc_stream_clients()
    # H.264 imports the DMA-BUF directly; viewers no longer require a full
    # frame copy into the NumPy ring.
    assert shared.needs_numpy_frame_pack() is False
    shared.dec_stream_clients()
    shared.set_cat_injection_enabled(True)
    assert shared.needs_numpy_frame_pack() is True


def test_dmabuf_lease_requeue_callback():
    shared = _make_shared()
    requeued: list[int] = []

    def _requeue(index: int) -> None:
        requeued.append(index)

    shared.attach_zerocopy_session(object(), requeue_cb=_requeue)
    write_buf = shared.try_get_write_buffer()
    assert write_buf is not None
    shared.publish_dmabuf_from_write(
        capture_started_ns=time.monotonic_ns(),
        dmabuf_fd=42,
        buffer_index=3,
        image_size=460800,
        stride=640,
        frame=None,
    )
    lease = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert lease is not None
    assert lease.dmabuf_fd == 42
    assert lease.dmabuf_buffer_index == 3
    assert lease.dmabuf_stride == 640
    assert lease.frame is None
    lease.release()
    # The latest slot remains owned by SharedState even with no active reader.
    # It is requeued as soon as a newer frame supersedes it.
    assert requeued == []
    write_buf = shared.try_get_write_buffer()
    assert write_buf is not None
    write_buf.fill(1)
    shared.publish_latest_from_write()
    assert requeued == [3]


def test_pinned_superseded_dmabuf_requeues_on_final_release():
    shared = _make_shared()
    requeued: list[int] = []
    shared.attach_zerocopy_session(
        object(), requeue_cb=lambda index: requeued.append(index)
    )

    assert shared.try_get_write_buffer() is not None
    shared.publish_dmabuf_from_write(
        capture_started_ns=time.monotonic_ns(),
        dmabuf_fd=42,
        buffer_index=3,
        image_size=460800,
        stride=640,
    )
    lease = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert lease is not None

    assert shared.try_get_write_buffer() is not None
    shared.publish_dmabuf_from_write(
        capture_started_ns=time.monotonic_ns(),
        dmabuf_fd=43,
        buffer_index=4,
        image_size=460800,
        stride=640,
    )
    assert requeued == []

    lease.release()
    assert requeued == [3]


def test_failed_requeue_releases_the_reserved_write_slot():
    """A QBUF failure must not wedge the ring in "frame write already active"."""
    shared = _make_shared()
    # No callback yet, so superseded slots keep their dmabuf index and a later
    # reservation is the thing that has to hand them back to the driver.
    shared.attach_zerocopy_session(object(), requeue_cb=None)
    for index in range(FRAME_RING_N):
        assert shared.try_get_write_buffer() is not None
        shared.publish_dmabuf_from_write(
            capture_started_ns=time.monotonic_ns(),
            dmabuf_fd=40 + index,
            buffer_index=index,
            image_size=460800,
            stride=640,
        )

    shared.attach_zerocopy_session(object(), requeue_cb=lambda _index: False)
    try:
        shared.try_get_write_buffer()
    except DmabufRequeueError:
        pass
    else:
        raise AssertionError("expected DmabufRequeueError from failed requeue")

    # The reservation must have been given back, not left active.
    shared.attach_zerocopy_session(object(), requeue_cb=lambda _index: True)
    assert shared.try_get_write_buffer() is not None


def _fill_ring_with_retained_dmabufs(shared, first_index=10):
    """Publish one dmabuf frame per slot while no requeue callback exists.

    With no callback the ring cannot hand buffers back, so every slot keeps its
    V4L2 index and a later publish has both a replaced and a superseded buffer
    to return in one go.
    """
    shared.attach_zerocopy_session(object(), requeue_cb=None)
    for offset in range(FRAME_RING_N):
        assert shared.try_get_write_buffer() is not None
        shared.publish_dmabuf_from_write(
            capture_started_ns=time.monotonic_ns(),
            dmabuf_fd=40 + offset,
            buffer_index=first_index + offset,
            image_size=460800,
            stride=640,
        )


def test_publish_returns_every_buffer_when_one_requeue_fails():
    """A QBUF failure must not strand the buffers queued behind it."""
    shared = _make_shared()
    _fill_ring_with_retained_dmabufs(shared)

    # Reserve the oldest slot while the callback is still absent, so it keeps
    # its own buffer and publishing supersedes a second one.
    assert shared.try_get_write_buffer() is not None

    attempted: list[int] = []

    def _requeue(index: int) -> bool:
        attempted.append(index)
        return index != 10

    shared.attach_zerocopy_session(object(), requeue_cb=_requeue)
    with pytest.raises(DmabufRequeueError):
        shared.publish_dmabuf_from_write(
            capture_started_ns=time.monotonic_ns(),
            dmabuf_fd=44,
            buffer_index=20,
            image_size=460800,
            stride=640,
        )

    # Both buffers were attempted, and only the failed one is still claimed.
    assert attempted == [10, 13]
    assert shared.pending_dmabuf_requeues() == (10,)


def test_claimed_buffer_is_retried_on_the_next_capture():
    shared = _make_shared()
    _fill_ring_with_retained_dmabufs(shared)
    assert shared.try_get_write_buffer() is not None

    shared.attach_zerocopy_session(
        object(), requeue_cb=lambda index: index != 10
    )
    with pytest.raises(DmabufRequeueError):
        shared.publish_dmabuf_from_write(
            capture_started_ns=time.monotonic_ns(),
            dmabuf_fd=44,
            buffer_index=20,
            image_size=460800,
            stride=640,
        )
    assert shared.pending_dmabuf_requeues() == (10,)

    retried: list[int] = []
    shared.attach_zerocopy_session(
        object(), requeue_cb=lambda index: retried.append(index) or True
    )
    assert shared.try_get_write_buffer() is not None

    assert 10 in retried
    assert shared.pending_dmabuf_requeues() == ()


def test_failed_reservation_requeue_keeps_the_buffer_claimed():
    """The ring, not the driver, still owns a buffer whose QBUF failed."""
    shared = _make_shared()
    _fill_ring_with_retained_dmabufs(shared)

    shared.attach_zerocopy_session(object(), requeue_cb=lambda _index: False)
    with pytest.raises(DmabufRequeueError):
        shared.try_get_write_buffer()

    assert shared.pending_dmabuf_requeues() == (10,)


def test_perception_intent_masks_detector_demand_when_forced_off():
    shared = _make_shared()
    shared.set_perception_intent(
        capture_active=True,
        detector_required=True,
        detector_mission_override=True,
        stream_forced_off=False,
    )
    assert shared.get_perception_intent()["detector_required"] is True

    shared.set_perception_intent(
        capture_active=True,
        detector_required=True,
        detector_mission_override=True,
        stream_forced_off=False,
        detector_force_off=True,
    )
    intent = shared.get_perception_intent()
    assert intent["detector_force_off"] is True
    assert intent["detector_required"] is False
    assert intent["detector_mission_override"] is False


def test_bbox_tracker_set_get():
    shared = _make_shared()
    shared.set_bbox_tracker(10.0, 20.0, 30.0, 40.0, 1.0)
    result = shared.get_bbox_tracker()
    assert result == (10.0, 20.0, 30.0, 40.0, 1.0)


def test_bbox_tracker_default_zero():
    shared = _make_shared()
    result = shared.get_bbox_tracker()
    assert result == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_detector_frame_generation_and_bbox_tagging():
    shared = _make_shared()

    # A detector lease carries the exact capture sequence for bbox tagging.
    src = np.full(FRAME_SHAPE, 7, dtype=np.uint8)
    shared.set_frame_latest(src)
    lease1 = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert lease1 is not None
    gen1 = lease1.frame_seq
    assert gen1 == 1
    assert np.array_equal(lease1.frame, src)

    shared.set_bbox_detector(1.0, 2.0, 3.0, 4.0, 1.0, frame_gen=gen1)
    x, y, w, h, valid, gen = shared.get_bbox_detector_with_gen()
    assert (x, y, w, h, valid, gen) == (1.0, 2.0, 3.0, 4.0, 1.0, gen1)
    lease1.release()

    shared.set_frame_latest(np.full(FRAME_SHAPE, 8, dtype=np.uint8))
    lease2 = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert lease2 is not None
    gen2 = lease2.frame_seq
    assert gen2 == 2
    assert shared.get_bbox_detector_with_gen()[5] != gen2
    lease2.release()


def test_bbox_detector_set_get():
    shared = _make_shared()
    shared.set_bbox_detector(100.0, 200.0, 50.0, 60.0, 1.0)
    result = shared.get_bbox_detector()
    assert result == (100.0, 200.0, 50.0, 60.0, 1.0)


def test_bbox_detector_default_zero():
    shared = _make_shared()
    result = shared.get_bbox_detector()
    assert result == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_tracked_targets_are_role_keyed_and_detached():
    shared = _make_shared()
    source = {
        "PRIMARY_CAT": (7, 10.0, 20.0, 30.0, 40.0, 0.9, 0, 1.0),
        "SECONDARY_CAT": (8, 50.0, 60.0, 20.0, 25.0, 0.8, 1, 1.0),
    }
    shared.set_tracked_targets(source)
    result = shared.get_tracked_targets()
    assert result == source
    result.clear()
    assert set(shared.get_tracked_targets()) == {"PRIMARY_CAT", "SECONDARY_CAT"}


def test_tracking_snapshot_coasting_does_not_refresh_generation():
    shared = _make_shared()
    detected = {"PRIMARY_CAT": (7, 10, 20, 30, 40, 0.8, 0, 1, 1)}
    shared.publish_tracking_snapshot(
        detected, (10, 20, 30, 40, 0.8), detector_backed=True
    )
    _, first_bbox = shared.get_tracking_snapshot()

    coasted = {"PRIMARY_CAT": (7, 12, 20, 30, 40, 0.8, 0, 1, 0)}
    shared.publish_tracking_snapshot(
        coasted, (12, 20, 30, 40, 0.8), detector_backed=False
    )
    targets, second_bbox = shared.get_tracking_snapshot()

    assert targets == coasted
    assert second_bbox[:4] == (12.0, 20.0, 30.0, 40.0)
    assert abs(second_bbox[4] - 0.8) < 1e-6
    assert second_bbox[5] == first_bbox[5]


def test_lores_gray_reuses_caller_buffer():
    shared = _make_shared()
    shared.set_lores_gray(np.full((24, 32), 7, dtype=np.uint8))
    first = shared.get_lores_gray()
    second = shared.get_lores_gray(first)
    assert second is first
    assert np.all(second == 7)


def test_odometry_set_get():
    shared = _make_shared()
    shared.set_odometry(1.5, 2.5, 90.0)
    result = shared.get_odometry()
    assert result == (1.5, 2.5, 90.0)


def test_odometry_default_zero():
    shared = _make_shared()
    result = shared.get_odometry()
    assert result == (0.0, 0.0, 0.0)


def test_frame_latest_set_get():
    shared = _make_shared()
    src = np.full(FRAME_SHAPE, 42, dtype=np.uint8)
    shared.set_frame_latest(src)

    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_latest(dst)
    assert np.all(dst == 42)


def test_frame_latest_does_not_alias_src():
    """set_frame_latest must copy, not reference, the source array."""
    shared = _make_shared()
    src = np.full(FRAME_SHAPE, 99, dtype=np.uint8)
    shared.set_frame_latest(src)

    src[:] = 0  # mutate source after set

    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_latest(dst)
    assert np.all(dst == 99), "SharedState must hold a copy, not a reference"


def test_wait_for_new_frame_wakes_on_publish():
    shared = _make_shared()
    stop_event = threading.Event()
    result = []

    waiter = threading.Thread(
        target=lambda: result.append(
            shared.wait_for_new_frame(0, stop_event, timeout_s=1.0)
        )
    )
    waiter.start()
    shared.set_frame_latest(np.zeros(FRAME_SHAPE, dtype=np.uint8))
    waiter.join(timeout=1.0)

    assert not waiter.is_alive()
    assert result == [1]


def test_wait_for_detector_update_wakes_on_publish():
    shared = _make_shared()
    stop_event = threading.Event()
    result = []

    waiter = threading.Thread(
        target=lambda: result.append(
            shared.wait_for_detector_update(-1, stop_event, timeout_s=1.0)
        )
    )
    waiter.start()
    shared.set_detector_detections([], frame_gen=7)
    waiter.join(timeout=1.0)

    assert not waiter.is_alive()
    assert result == [7]


def test_event_waits_observe_stop_with_bounded_latency():
    shared = _make_shared()
    stop_event = threading.Event()
    result = []

    waiter = threading.Thread(
        target=lambda: result.append(
            shared.wait_for_new_frame(0, stop_event, timeout_s=1.0)
        )
    )
    waiter.start()
    stop_event.set()
    waiter.join(timeout=0.2)

    assert not waiter.is_alive()
    assert result == [0]


def test_frame_lease_pins_slot_until_release():
    shared = _make_shared()
    shared.set_frame_latest(np.full(FRAME_SHAPE, 11, dtype=np.uint8))
    lease = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)

    assert lease is not None
    assert lease.frame_seq == 1
    assert np.shares_memory(lease.frame, shared._pool.frame_ring)
    assert not lease.stale

    # Publish enough frames to wrap the write cursor. The pinned slot must
    # remain unchanged while the camera rotates through other free slots.
    for value in (22, 33, 44, 55):
        write_buf = shared.try_get_write_buffer()
        assert write_buf is not None
        write_buf.fill(value)
        shared.publish_latest_from_write()

    assert np.all(lease.frame == 11)
    assert not lease.stale
    lease.release()

    # Once released, the writer may reclaim the old slot.
    for value in range(66, 66 + FRAME_RING_N):
        write_buf = shared.try_get_write_buffer()
        assert write_buf is not None
        write_buf.fill(value)
        shared.publish_latest_from_write()
        if lease.stale:
            break
    assert lease.stale


def test_camera_keeps_a_write_slot_under_maximum_reader_pressure():
    """Readers can never take the writer's last reclaimable slot.

    Admission caps distinct reader pins at ``FRAME_RING_N -
    CAMERA_RESERVED_SLOTS``, so capture keeps succeeding however many acquires
    the readers attempt; the excess acquires are refused instead of the camera
    dropping frames.
    """
    shared = _make_shared()
    leases = []
    for value in range(1, FRAME_RING_N + 3):
        write_buf = shared.try_get_write_buffer()
        assert write_buf is not None, f"camera starved with {len(leases)} pins"
        write_buf.fill(value)
        shared.publish_latest_from_write()
        lease = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
        if lease is not None:
            leases.append(lease)

    assert len(leases) == FRAME_RING_N - CAMERA_RESERVED_SLOTS
    assert shared.admission_denied_counts()["detector"] > 0
    # Pinned slots were never recycled underneath their readers.
    assert np.all(leases[0].frame == 1)

    # Releasing a pin hands its share of the budget back.
    leases.pop().release()
    extra = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert extra is not None
    extra.release()
    for lease in leases:
        lease.release()


def test_admission_reserves_detector_slot_and_prefers_recording_over_stream():
    shared = _make_shared()
    shared.set_perception_intent(
        capture_active=True,
        detector_required=True,
        detector_mission_override=True,
        recording_required=True,
        stream_forced_off=False,
    )
    # Pin an older slot as the detector would during a slow RKNN tick.
    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    detector = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert detector is not None

    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()

    # With recording requested, stream may not open a second distinct pin.
    assert (
        shared.acquire_latest_frame(consumer=FrameConsumer.STREAM) is None
    )
    assert shared.admission_denied_counts() == {
        "detector": 0,
        "recording": 0,
        "stream": 1,
    }

    # Recording may take the last refusable distinct pin; camera still writes.
    recording = shared.acquire_latest_frame(consumer=FrameConsumer.RECORDING)
    assert recording is not None
    assert shared.try_get_write_buffer() is not None
    shared.abort_frame_write()

    detector.release()
    recording.release()


def test_admission_same_latest_allows_multi_reader_without_extra_slot():
    shared = _make_shared()
    shared.set_perception_intent(
        capture_active=True,
        detector_required=True,
        detector_mission_override=True,
        stream_forced_off=False,
    )
    shared.set_frame_latest(np.zeros(FRAME_SHAPE, dtype=np.uint8))
    detector = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    stream = shared.acquire_latest_frame(consumer=FrameConsumer.STREAM)
    assert detector is not None
    assert stream is not None
    assert detector.slot_idx == stream.slot_idx
    detector.release()
    stream.release()


def test_admission_admits_same_latest_stream_beside_a_distinct_recording_pin():
    """Mission pressure only refuses a *new distinct* pin.

    Detector on an older slot plus recording on the latest spends the whole
    distinct-pin budget, yet the stream can still co-read that same latest slot
    for free instead of losing the frame.
    """
    shared = _make_shared()
    shared.set_perception_intent(
        capture_active=True,
        detector_required=True,
        detector_mission_override=True,
        stream_forced_off=False,
    )
    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    detector = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert detector is not None

    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    recording = shared.acquire_latest_frame(consumer=FrameConsumer.RECORDING)
    assert recording is not None

    stream = shared.acquire_latest_frame(consumer=FrameConsumer.STREAM)
    assert stream is not None
    assert stream.slot_idx == recording.slot_idx
    assert shared.admission_denied_counts()["stream"] == 0

    # The camera still owns a reclaimable slot with all three readers pinned.
    assert shared.try_get_write_buffer() is not None
    shared.abort_frame_write()

    for lease in (detector, recording, stream):
        lease.release()


def test_admission_keeps_recording_ahead_of_the_stream():
    """The web UI cannot take a pin that recording still needs.

    Asking first is not enough: while recording is requested, the last
    refusable pin belongs to it.
    """
    shared = _make_shared()
    shared.set_perception_intent(
        capture_active=True,
        detector_required=True,
        detector_mission_override=False,
        recording_required=True,
        stream_forced_off=False,
    )
    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()

    assert shared.acquire_latest_frame(consumer=FrameConsumer.STREAM) is None
    assert shared.admission_denied_counts()["stream"] == 1
    recording = shared.acquire_latest_frame(consumer=FrameConsumer.RECORDING)
    assert recording is not None
    recording.release()


def test_admission_gives_the_stream_a_slot_when_recording_is_idle():
    """Reservations follow demand, so an idle recorder costs the stream nothing.

    Under SEARCH/CHASE with the detector holding an older slot, the live view
    still gets its own pin as long as nothing is recording.
    """
    shared = _make_shared()
    shared.set_perception_intent(
        capture_active=True,
        detector_required=True,
        detector_mission_override=True,
        recording_required=False,
        stream_forced_off=False,
    )
    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    detector = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert detector is not None

    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    stream = shared.acquire_latest_frame(consumer=FrameConsumer.STREAM)
    assert stream is not None
    assert stream.slot_idx != detector.slot_idx
    assert shared.admission_denied_counts()["stream"] == 0

    detector.release()
    stream.release()


def test_admission_off_mission_still_reserves_a_detector_slot():
    """Detector headroom does not depend on SEARCH/CHASE.

    Recording and the stream are both refusable, so off-mission they would
    otherwise fill the ring and deny the detector its next capture.
    """
    shared = _make_shared()
    shared.set_perception_intent(
        capture_active=True,
        detector_required=True,
        detector_mission_override=False,
        stream_forced_off=False,
    )
    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    recording = shared.acquire_latest_frame(consumer=FrameConsumer.RECORDING)
    assert recording is not None

    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    assert shared.acquire_latest_frame(consumer=FrameConsumer.STREAM) is None
    assert shared.admission_denied_counts()["stream"] == 1

    detector = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert detector is not None
    assert detector.slot_idx != recording.slot_idx
    assert shared.admission_denied_counts()["detector"] == 0

    detector.release()
    recording.release()


def test_admission_counts_detector_denial_when_budget_is_spent():
    """With the detector off, refusable readers may use the whole budget.

    The detector then loses its first capture, and that refusal is counted
    instead of disappearing silently.
    """
    shared = _make_shared()
    shared.set_perception_intent(
        capture_active=True,
        detector_required=False,
        detector_mission_override=False,
        stream_forced_off=False,
    )
    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    recording = shared.acquire_latest_frame(consumer=FrameConsumer.RECORDING)
    assert recording is not None

    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    stream = shared.acquire_latest_frame(consumer=FrameConsumer.STREAM)
    assert stream is not None
    assert stream.slot_idx != recording.slot_idx

    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    assert shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR) is None
    assert shared.admission_denied_counts()["detector"] == 1

    # Whoever was refused, the camera keeps writing.
    assert shared.try_get_write_buffer() is not None
    shared.abort_frame_write()

    recording.release()
    stream.release()


def test_releasing_a_refusable_pin_returns_its_budget():
    shared = _make_shared()
    shared.set_perception_intent(
        capture_active=True,
        detector_required=True,
        detector_mission_override=False,
        stream_forced_off=False,
    )
    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    recording = shared.acquire_latest_frame(consumer=FrameConsumer.RECORDING)
    assert recording is not None

    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    assert shared.acquire_latest_frame(consumer=FrameConsumer.STREAM) is None

    recording.release()
    stream = shared.acquire_latest_frame(consumer=FrameConsumer.STREAM)
    assert stream is not None
    stream.release()


def test_admission_off_mission_stream_may_pin_beside_detector():
    shared = _make_shared()
    shared.set_perception_intent(
        capture_active=True,
        detector_required=False,
        detector_mission_override=False,
        stream_forced_off=False,
    )
    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    detector = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert detector is not None

    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    stream = shared.acquire_latest_frame(consumer=FrameConsumer.STREAM)
    assert stream is not None
    assert stream.slot_idx != detector.slot_idx

    detector.release()
    stream.release()


def test_admission_detector_still_acquires_when_stream_denied():
    shared = _make_shared()
    shared.set_perception_intent(
        capture_active=True,
        detector_required=True,
        detector_mission_override=True,
        stream_forced_off=False,
    )
    leases = []
    for value in range(1, 3):
        write_buf = shared.try_get_write_buffer()
        assert write_buf is not None
        write_buf.fill(value)
        shared.publish_latest_from_write()
        lease = shared.acquire_latest_frame(consumer=FrameConsumer.RECORDING)
        if lease is not None:
            leases.append(lease)

    assert shared.try_get_write_buffer() is not None
    shared.publish_latest_from_write()
    assert shared.acquire_latest_frame(consumer=FrameConsumer.STREAM) is None
    detector = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert detector is not None
    detector.release()
    for lease in leases:
        lease.release()


def test_slow_frame_reader_never_observes_torn_pixels():
    shared = _make_shared()
    shared.set_frame_latest(np.full(FRAME_SHAPE, 9, dtype=np.uint8))
    lease = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)
    assert lease is not None
    errors = []

    def _writer():
        for value in range(20, 80):
            write_buf = shared.try_get_write_buffer()
            if write_buf is None:
                continue
            write_buf.fill(value)
            shared.publish_latest_from_write()

    writer = threading.Thread(target=_writer)
    writer.start()
    for _ in range(20):
        if not np.all(lease.frame == 9):
            errors.append("pinned frame changed")
            break
        time.sleep(0.001)
    writer.join(timeout=1.0)
    lease.release()

    assert not writer.is_alive()
    assert not errors


def test_bbox_tracker_overwrite():
    shared = _make_shared()
    shared.set_bbox_tracker(1.0, 2.0, 3.0, 4.0, 1.0)
    shared.set_bbox_tracker(5.0, 6.0, 7.0, 8.0, 0.0)
    result = shared.get_bbox_tracker()
    assert result == (5.0, 6.0, 7.0, 8.0, 0.0)


def test_odometry_overwrite():
    shared = _make_shared()
    shared.set_odometry(1.0, 2.0, 3.0)
    shared.set_odometry(10.0, 20.0, 30.0)
    assert shared.get_odometry() == (10.0, 20.0, 30.0)


# ── concurrent tests ────────────────────────────────────────────────────

def test_concurrent_bbox_tracker_no_torn_reads():
    """One writer, one reader on bbox_tracker for many iterations.

    The writer writes 5-tuples where all five values equal the iteration
    index (e.g. (7,7,7,7,7) at iteration 7).  The reader asserts that
    every read is such a "uniform" 5-tuple — i.e. all five values are the
    same, proving no partial/torn write was observed.
    """
    shared = _make_shared()
    iterations = 5_000
    errors: list = []
    stop = threading.Event()

    def writer():
        for i in range(iterations):
            v = float(i)
            shared.set_bbox_tracker(v, v, v, v, v)
        stop.set()

    def reader():
        while not stop.is_set():
            tup = shared.get_bbox_tracker()
            # All five values must be the same (from one write iteration)
            if len(set(tup)) != 1:
                errors.append(tup)
                break  # one failure is enough

    t_w = threading.Thread(target=writer, name="writer")
    t_r = threading.Thread(target=reader, name="reader")
    t_r.start()
    t_w.start()
    t_w.join()
    t_r.join()

    assert len(errors) == 0, f"Torn read detected: {errors[0]}"


def test_concurrent_odometry_no_torn_reads():
    """Same pattern for odometry (3 values)."""
    shared = _make_shared()
    iterations = 5_000
    errors: list = []
    stop = threading.Event()

    def writer():
        for i in range(iterations):
            v = float(i)
            shared.set_odometry(v, v, v)
        stop.set()

    def reader():
        while not stop.is_set():
            tup = shared.get_odometry()
            if len(set(tup)) != 1:
                errors.append(tup)
                break

    t_w = threading.Thread(target=writer, name="odom-writer")
    t_r = threading.Thread(target=reader, name="odom-reader")
    t_r.start()
    t_w.start()
    t_w.join()
    t_r.join()

    assert len(errors) == 0, f"Torn read detected: {errors[0]}"


# ── run as script ────────────────────────────────────────────────────────

if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"  PASS  {fn.__name__}")
    print(f"\nAll {len(tests)} tests passed.")
