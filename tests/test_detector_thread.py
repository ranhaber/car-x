import threading
import time

from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState
from cat_follow.threads.detector import run_detector_loop


def test_detector_stub_publishes_bbox():
    pool = allocate_pool()
    shared = SharedState(pool)

    stop_event = threading.Event()
    th = threading.Thread(target=run_detector_loop, args=(shared, stop_event), kwargs={"model_path": None, "target_fps": 10.0})
    th.daemon = True
    th.start()

    found = False
    deadline = time.time() + 2.0
    while time.time() < deadline:
        bx = shared.get_bbox_detector()
        if bx[4] > 0:
            found = True
            assert bx[2] > 0 and bx[3] > 0
            break
        time.sleep(0.05)

    stop_event.set()
    th.join(timeout=1.0)
    assert found, "Detector stub did not publish a bbox within timeout"
