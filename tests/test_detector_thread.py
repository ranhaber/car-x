import threading
import time

from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState
from cat_follow.perception_config import PerceptionConfig
import cat_follow.threads.detector as detector_module
from cat_follow.threads.detector import run_detector_loop


def test_detector_stub_publishes_bbox(monkeypatch):
    # The deterministic stub is opt-in; production hard-fails without an NPU.
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_ALLOW_STUB", "1")
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


def test_detector_preserves_primary_confidence(monkeypatch):
    stop_event = threading.Event()

    class _Backend:
        loaded = True
        consecutive_failures = 0
        last_error = None

        def infer_all(self, frame_bgr, score_threshold):
            assert score_threshold == 0.5
            stop_event.set()
            return [(10.0, 20.0, 40.0, 60.0, 0.67, 17)]

        def unload(self):
            pass

    monkeypatch.setattr(
        detector_module, "build_validated_backend", lambda config, path: _Backend()
    )
    shared = SharedState(allocate_pool())
    run_detector_loop(
        shared,
        stop_event,
        target_fps=100.0,
        config=PerceptionConfig(motion_gating=False),
    )

    assert abs(shared.get_bbox_detector()[4] - 0.67) < 1e-6
    detections, _ = shared.get_detector_detections_with_gen()
    assert detections[0][4] == 0.67
