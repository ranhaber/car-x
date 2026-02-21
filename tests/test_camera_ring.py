import numpy as np
import time

from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState


def test_frame_ring_publish_and_detector_copy():
    pool = allocate_pool()
    shared = SharedState(pool)

    # Destination buffer to receive latest frame
    dst = np.empty(pool.frame_for_detector.shape, dtype=np.uint8)

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

    # copy latest to detector frame and confirm
    shared.copy_latest_to_detector_frame()
    assert np.array_equal(pool.frame_for_detector, pool.frame_ring[shared._latest_idx])

