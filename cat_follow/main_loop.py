"""
Main loop: pre-allocate memory, start threads, start Web UI, poll commands,
run state machine and motion.

Run from car-x root:
    python -m cat_follow.main_loop

Then open http://localhost:5000 in your browser.
"""

import time
import sys
import os
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.state_machine import StateMachine, State, Event
from cat_follow.commands import poll_commands
from cat_follow.calibration import Calibration
from cat_follow.motion import driver as motion_driver
from cat_follow.motion import center_cat
from cat_follow import odometry

# Memory and shared state (Steps 1-2)
from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState

# Thread stubs (Step 3)
from cat_follow.threads.camera import run_camera_loop
from cat_follow.threads.tracker import run_tracker_loop
from cat_follow.threads.detector import run_detector_loop

# Web UI (Step 4c)
from cat_follow.web_ui.app import create_app, set_tracker_fps


def main():
    # ------------------------------------------------------------------
    # 1. Load config / calibration
    # ------------------------------------------------------------------
    calib = Calibration()
    sm = StateMachine()
    odometry.reset(0, 0, 0)

    # ------------------------------------------------------------------
    # 2-3. Pre-allocate pool and create SharedState
    # ------------------------------------------------------------------
    pool = allocate_pool()
    shared = SharedState(pool)

    # ------------------------------------------------------------------
    # 4. (TFLite interpreter would be created here — skipped in stub)
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # 5. Start worker threads
    # ------------------------------------------------------------------
    stop_event = threading.Event()

    camera_thread = threading.Thread(
        target=run_camera_loop, args=(shared, stop_event),
        name="CatFollow-Camera", daemon=True,
    )
    tracker_thread = threading.Thread(
        target=run_tracker_loop, args=(shared, stop_event),
        name="CatFollow-Tracker", daemon=True,
    )
    detector_thread = threading.Thread(
        target=run_detector_loop, args=(shared, stop_event),
        name="CatFollow-Detector", daemon=True,
    )

    camera_thread.start()
    tracker_thread.start()
    detector_thread.start()

    print("[main] Camera, Tracker, Detector threads started.")

    # ------------------------------------------------------------------
    # 6. Start Web UI (Flask) in a background thread
    # ------------------------------------------------------------------
    app = create_app(shared=shared, state_machine=sm)

    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False),
        name="CatFollow-Flask", daemon=True,
    )
    flask_thread.start()
    print("[main] Web UI started on http://0.0.0.0:5000")

    # ------------------------------------------------------------------
    # 7. Main loop
    # ------------------------------------------------------------------
    image_width, image_height = 640, 480
    tick_sec = 1.0 / 30.0
    lost_count = 0
    lost_threshold = 15
    frame_count = 0
    detect_every_k = 10
    tracker_fps_counter = 0
    tracker_fps_timer = time.monotonic()

    def on_cat_location(x: float, y: float):
        sm.dispatch(Event.CAT_LOCATION_RECEIVED, (x, y))
        print(f"[CMD] cat_location ({x}, {y}) -> state={sm.state.value}")

    def on_stop():
        sm.dispatch(Event.STOP_COMMAND)
        motion_driver.stop()
        print(f"[CMD] stop -> state={sm.state.value}")

    print(f"[main] Main loop running at ~30 Hz. State: {sm.state.value}")

    try:
        while True:
            t0 = time.monotonic()

            # Poll commands (thread-safe via lock)
            poll_commands(on_cat_location=on_cat_location, on_stop=on_stop)

            # Copy frame to detector every K frames
            frame_count += 1
            if frame_count % detect_every_k == 0:
                shared.copy_latest_to_detector_frame()

            # Read bbox from shared state (from tracker thread)
            bbox = shared.get_bbox_tracker()
            bbox_valid = bbox[4] > 0
            bbox_xywh = (bbox[0], bbox[1], bbox[2], bbox[3]) if bbox_valid else None

            # State machine logic
            state = sm.state

            if state == State.IDLE:
                motion_driver.stop()

            elif state == State.GOTO_TARGET:
                # Stub: immediately pretend we're at target
                sm.dispatch(Event.AT_TARGET)

            elif state in (State.SEARCH, State.LOST_SEARCH):
                if bbox_valid:
                    sm.dispatch(Event.CAT_FOUND, bbox_xywh)
                    lost_count = 0

            elif state in (State.APPROACH, State.TRACK):
                if bbox_valid:
                    lost_count = 0
                    center_cat.center_cat_control(
                        bbox_xywh, image_width, image_height, calib,
                        target_distance_cm=calib.get_target_distance_cm(),
                    )
                    if state == State.APPROACH:
                        # Stub: pretend at 15 cm
                        sm.dispatch(Event.DISTANCE_AT_15CM)
                else:
                    lost_count += 1
                    if lost_count >= lost_threshold:
                        sm.dispatch(Event.CAT_LOST)
                        motion_driver.stop()

            # Update odometry into shared state
            pos = odometry.get_position()
            heading = odometry.get_heading_deg()
            shared.set_odometry(pos[0], pos[1], heading)

            # Tracker FPS reporting
            tracker_fps_counter += 1
            now = time.monotonic()
            if now - tracker_fps_timer >= 1.0:
                fps = tracker_fps_counter / (now - tracker_fps_timer)
                set_tracker_fps(fps)
                tracker_fps_counter = 0
                tracker_fps_timer = now

            elapsed = time.monotonic() - t0
            time.sleep(max(0, tick_sec - elapsed))

    except KeyboardInterrupt:
        print("\n[main] Shutting down...")
        stop_event.set()
        sm.dispatch(Event.STOP_COMMAND)
        motion_driver.stop()

        camera_thread.join(timeout=2)
        tracker_thread.join(timeout=2)
        detector_thread.join(timeout=2)
        print("[main] Bye.")


if __name__ == "__main__":
    main()
