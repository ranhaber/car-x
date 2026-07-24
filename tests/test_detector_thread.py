import threading
import time

import numpy as np

from cat_follow.memory.pool import FRAME_H, FRAME_SHAPE, allocate_pool
from cat_follow.memory.shared_state import SharedState
from cat_follow.perception_config import PerceptionConfig
import cat_follow.threads.detector as detector_module
from cat_follow.threads.detector import run_detector_loop


def test_center_bottom_model_crop_copies_matching_y_and_uv_planes():
    frame = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    frame[:FRAME_H] = np.arange(FRAME_SHAPE[1], dtype=np.uint8)
    frame[FRAME_H:] = 173
    crop, offset_x, offset_y = detector_module._center_bottom_crop(
        frame, (320, 320)
    )

    assert crop.shape == (480, 320)
    assert (offset_x, offset_y) == (160, 160)
    assert not np.shares_memory(crop, frame)
    assert np.array_equal(crop[0], frame[160, 160:480])
    assert np.all(crop[320:] == 173)


def test_detector_acquires_frame_once_per_tick(monkeypatch):
    stop_event = threading.Event()
    acquire_calls = {"count": 0}
    original_acquire = SharedState.acquire_latest_frame

    def _counting_acquire(self):
        acquire_calls["count"] += 1
        return original_acquire(self)

    monkeypatch.setattr(SharedState, "acquire_latest_frame", _counting_acquire)

    class _Backend:
        loaded = True
        consecutive_failures = 0
        last_error = None

        def infer_all(self, frame_bgr, score_threshold):
            stop_event.set()
            return []

        def unload(self):
            pass

    monkeypatch.setattr(
        detector_module, "build_validated_backend", lambda config, path: _Backend()
    )
    shared = SharedState(allocate_pool())
    shared.set_frame_latest(np.zeros(FRAME_SHAPE, dtype=np.uint8))
    run_detector_loop(
        shared,
        stop_event,
        target_fps=100.0,
        config=PerceptionConfig(motion_gating=False),
    )

    assert acquire_calls["count"] == 1


def test_detector_prefers_direct_nv12_backend_input(monkeypatch):
    stop_event = threading.Event()

    class _Backend:
        loaded = True
        consecutive_failures = 0
        last_error = None
        last_perf = {"pre": 1.0, "invoke": 2.0, "post": 0.5}

        def infer_all_nv12(self, frame_nv12, score_threshold):
            assert frame_nv12.shape == (480, 320)
            assert frame_nv12.dtype == np.uint8
            assert score_threshold == 0.5
            stop_event.set()
            return []

        def infer_all(self, _frame_bgr, _score_threshold):
            raise AssertionError("BGR compatibility path should not be used")

        def unload(self):
            pass

    monkeypatch.setattr(
        detector_module, "build_validated_backend", lambda config, path: _Backend()
    )
    shared = SharedState(allocate_pool())
    shared.set_frame_latest(np.zeros(FRAME_SHAPE, dtype=np.uint8))

    run_detector_loop(
        shared,
        stop_event,
        target_fps=100.0,
        config=PerceptionConfig(motion_gating=False),
    )


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
            assert frame_bgr.shape == (320, 320, 3)
            stop_event.set()
            return [(10.0, 20.0, 40.0, 60.0, 0.67, 17)]

        def unload(self):
            pass

    monkeypatch.setattr(
        detector_module, "build_validated_backend", lambda config, path: _Backend()
    )
    shared = SharedState(allocate_pool())
    shared.set_frame_latest(np.zeros(FRAME_SHAPE, dtype=np.uint8))
    run_detector_loop(
        shared,
        stop_event,
        target_fps=100.0,
        config=PerceptionConfig(motion_gating=False),
    )

    assert abs(shared.get_bbox_detector()[4] - 0.67) < 1e-6
    assert shared.get_bbox_detector()[:4] == (170.0, 180.0, 30.0, 40.0)
    detections, _ = shared.get_detector_detections_with_gen()
    assert detections[0][:4] == (170.0, 180.0, 200.0, 220.0)
    assert detections[0][4] == 0.67
