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
from cat_follow.memory.pool import allocate_pool, FRAME_RING_N, FRAME_SHAPE, BBOX_LEN
from cat_follow.memory.shared_state import SharedState


# ── helpers ──────────────────────────────────────────────────────────────

def _make_shared() -> SharedState:
    return SharedState(allocate_pool())


# ── single-thread tests ─────────────────────────────────────────────────

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
    lease1 = shared.acquire_latest_frame()
    assert lease1 is not None
    gen1 = lease1.frame_seq
    assert gen1 == 1
    assert np.array_equal(lease1.frame, src)

    shared.set_bbox_detector(1.0, 2.0, 3.0, 4.0, 1.0, frame_gen=gen1)
    x, y, w, h, valid, gen = shared.get_bbox_detector_with_gen()
    assert (x, y, w, h, valid, gen) == (1.0, 2.0, 3.0, 4.0, 1.0, gen1)
    lease1.release()

    shared.set_frame_latest(np.full(FRAME_SHAPE, 8, dtype=np.uint8))
    lease2 = shared.acquire_latest_frame()
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


def test_frame_lease_pins_slot_until_release():
    shared = _make_shared()
    shared.set_frame_latest(np.full(FRAME_SHAPE, 11, dtype=np.uint8))
    lease = shared.acquire_latest_frame()

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


def test_frame_ring_drops_write_when_all_slots_are_pinned():
    shared = _make_shared()
    leases = []
    for value in range(1, FRAME_RING_N + 1):
        write_buf = shared.try_get_write_buffer()
        assert write_buf is not None
        write_buf.fill(value)
        shared.publish_latest_from_write()
        lease = shared.acquire_latest_frame()
        assert lease is not None
        leases.append(lease)

    assert shared.try_get_write_buffer() is None

    leases[0].release()
    assert shared.try_get_write_buffer() is not None
    shared.abort_frame_write()
    for lease in leases[1:]:
        lease.release()


def test_slow_frame_reader_never_observes_torn_pixels():
    shared = _make_shared()
    shared.set_frame_latest(np.full(FRAME_SHAPE, 9, dtype=np.uint8))
    lease = shared.acquire_latest_frame()
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
