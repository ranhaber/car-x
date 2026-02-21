import threading
import time

from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState
from cat_follow.threads.tracker import run_tracker_loop


def test_tracker_initializes_from_detector_and_publishes_bbox():
    pool = allocate_pool()
    shared = SharedState(pool)

    stop_event = threading.Event()
    th = threading.Thread(target=run_tracker_loop, args=(shared, stop_event), daemon=True)
    th.start()

    # Publish a detector bbox; tracker should pick it up and publish to bbox_tracker
    shared.set_bbox_detector(120.0, 130.0, 50.0, 60.0, 1.0)

    # Wait briefly for the tracker thread to observe and act
    time.sleep(0.3)

    tbbox = shared.get_bbox_tracker()
    assert tbbox[4] == 1.0
    # Coordinates should match detector (fallback path) or be close
    assert abs(tbbox[0] - 120.0) < 1e-6
    assert abs(tbbox[1] - 130.0) < 1e-6

    stop_event.set()
    th.join(timeout=1)
