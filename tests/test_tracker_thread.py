import threading
import time

from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState
from cat_follow.perception.vision_adapter import VisionAdapter
from cat_follow.runtime.shared_state import SharedState as ContractSharedState
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


def test_tracker_thread_publishes_only_primary_cat():
    shared = SharedState(allocate_pool())
    stop_event = threading.Event()
    thread = threading.Thread(
        target=run_tracker_loop,
        args=(shared, stop_event),
        daemon=True,
    )
    thread.start()

    # The higher-confidence cat becomes PRIMARY_CAT even though it is second.
    shared.set_detector_detections(
        [
            (100.0, 100.0, 150.0, 160.0, 0.7, 17),
            (300.0, 200.0, 380.0, 290.0, 0.95, 17),
        ],
        frame_gen=1,
    )
    deadline = time.time() + 1.0
    while time.time() < deadline and shared.get_bbox_tracker()[4] == 0:
        time.sleep(0.01)

    assert shared.get_bbox_tracker() == (300.0, 200.0, 80.0, 90.0, 0.95)
    targets = shared.get_tracked_targets()
    assert set(targets) == {"PRIMARY_CAT", "SECONDARY_CAT"}
    assert targets["PRIMARY_CAT"][1:5] == (300.0, 200.0, 80.0, 90.0)
    assert targets["SECONDARY_CAT"][1:5] == (100.0, 100.0, 50.0, 60.0)
    stop_event.set()
    thread.join(timeout=1)


def test_tracker_coasts_primary_between_detector_generations():
    shared = SharedState(allocate_pool())
    stop_event = threading.Event()
    thread = threading.Thread(
        target=run_tracker_loop,
        args=(shared, stop_event),
        kwargs={"target_fps": 30.0},
        daemon=True,
    )
    thread.start()

    shared.set_detector_detections(
        [(100.0, 100.0, 150.0, 160.0, 0.9, 17)],
        frame_gen=1,
    )
    deadline = time.time() + 1.0
    while time.time() < deadline and shared.get_bbox_tracker()[4] == 0:
        time.sleep(0.01)
    assert shared.get_bbox_tracker()[4] == 0.9

    shared.set_detector_detections(
        [(120.0, 100.0, 170.0, 160.0, 0.9, 17)],
        frame_gen=2,
    )
    deadline = time.time() + 1.0
    while time.time() < deadline and shared.get_bbox_tracker()[0] < 120.0:
        time.sleep(0.01)

    before = shared.get_bbox_tracker()
    time.sleep(0.15)
    after = shared.get_bbox_tracker()
    assert after[4] == 0.9
    assert after[0] > before[0]

    stop_event.set()
    thread.join(timeout=1)


def test_coasting_tracker_does_not_refresh_or_accumulate_vision_stability():
    shared = SharedState(allocate_pool())
    contract = ContractSharedState()
    adapter = VisionAdapter(
        shared,
        contract,
        image_width=640,
        image_height=480,
        stability_frames=2,
    )
    stop_event = threading.Event()
    thread = threading.Thread(
        target=run_tracker_loop,
        args=(shared, stop_event),
        kwargs={"target_fps": 120.0},
        daemon=True,
    )
    thread.start()
    try:
        initial_generation = shared.get_bbox_tracker_with_gen()[5]
        shared.set_detector_detections(
            [(100.0, 100.0, 150.0, 160.0, 0.73, 17)],
            frame_gen=1,
        )
        deadline = time.time() + 1.0
        while time.time() < deadline:
            _, bbox = shared.get_tracking_snapshot()
            if bbox[4] > 0 and bbox[5] > initial_generation:
                break
            time.sleep(0.005)

        first = adapter.update()
        first_generation = shared.get_bbox_tracker_with_gen()[5]
        time.sleep(0.08)  # tracker remains alive and publishes coasting boxes
        coasted = adapter.update()
        targets, bbox = shared.get_tracking_snapshot()

        assert targets["PRIMARY_CAT"][8] == 0.0
        assert targets["PRIMARY_CAT"][6] >= 1
        assert bbox[5] == first_generation
        assert coasted.received_ms == first.received_ms
        assert coasted.cat_visible_stable is False
        assert abs(coasted.confidence - 0.73) < 1e-6
    finally:
        stop_event.set()
        thread.join(timeout=1)
