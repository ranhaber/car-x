import numpy as np
import time

from cat_follow.memory.pool import FRAME_SHAPE, allocate_pool
from cat_follow.memory.shared_state import SharedState


def test_frame_ring_publish_and_detector_copy():
    pool = allocate_pool()
    shared = SharedState(pool)

    # Destination buffer to receive latest frame
    dst = np.empty(FRAME_SHAPE, dtype=np.uint8)

    # Publish first frame (all 11)
    write0 = shared.get_write_buffer()
    write0[:] = 11
    shared.publish_latest_from_write()
    shared.get_frame_latest(dst)
    assert np.all(dst == 11)

    # Publish second frame (all 22)
    write1 = shared.get_write_buffer()
    write1[:] = 22
    shared.publish_latest_from_write()
    shared.get_frame_latest(dst)
    assert np.all(dst == 22)

    # Ensure the two ring slots are not identical
    assert not np.array_equal(pool.frame_ring[0], pool.frame_ring[1])

    lease = shared.acquire_latest_frame()
    assert lease is not None
    with lease:
        assert np.shares_memory(lease.frame, pool.frame_ring)
        assert np.all(lease.frame == 22)


def test_frame_lease_carries_capture_and_publish_timestamps():
    shared = SharedState(allocate_pool())
    capture_started_ns = time.monotonic_ns() - 1_000_000

    write = shared.get_write_buffer()
    write.fill(0)
    shared.publish_latest_from_write(capture_started_ns=capture_started_ns)

    lease = shared.acquire_latest_frame()
    assert lease is not None
    with lease:
        assert lease.capture_started_ns == capture_started_ns
        assert lease.published_ns >= capture_started_ns

