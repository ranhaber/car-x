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
import logging

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.logger import get_logger, LOG_FILE
from cat_follow.state_machine import StateMachine, State, Event
from cat_follow.commands import poll_commands
from cat_follow.calibration import Calibration, CALIBRATION_IMAGE_SIZE
from cat_follow.motion import driver as motion_driver
from cat_follow.motion import center_cat
from cat_follow.motion.goto_xy import compute_goto
from cat_follow.motion.search import compute_search_tick, compute_full_circle_tick
from cat_follow import location
from cat_follow import range_sensor

# Memory and shared state
from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState

# Worker threads
from cat_follow.threads.camera import run_camera_loop
from cat_follow.threads.tracker import run_tracker_loop
from cat_follow.threads.detector import run_detector_loop

# Web UI
from cat_follow.web_ui.app import create_app, set_tracker_fps

log = get_logger("main_loop")

# Suppress noisy Flask/werkzeug access logs (they still go to the file)
logging.getLogger("werkzeug").setLevel(logging.WARNING)


def main():
    # ------------------------------------------------------------------
    # 1. Load config / calibration
    # ------------------------------------------------------------------
    calib = Calibration()
    sm = StateMachine()
    location.reset(0, 0, 0)
    # Optional: on Pi with hardware, uncomment to use motors + ultrasonic:
    # from picarx import Picarx
    # px = Picarx()
    # motion_driver.set_car(px)
    # range_sensor.set_car(px)
    log.info("Calibration loaded. State machine ready.")

    # ------------------------------------------------------------------
    # 2-3. Pre-allocate pool and create SharedState
    # ------------------------------------------------------------------
    pool = allocate_pool()
    shared = SharedState(pool)
    log.info("Memory pool allocated. SharedState created.")

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
    log.info("Camera, Tracker, Detector threads started.")

    # ------------------------------------------------------------------
    # 6. Start Web UI (Flask) in a background thread
    # ------------------------------------------------------------------
    app = create_app(shared=shared, state_machine=sm)

    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False),
        name="CatFollow-Flask", daemon=True,
    )
    flask_thread.start()
    log.info("Web UI started on http://0.0.0.0:5000")

    # ------------------------------------------------------------------
    # 7. Main loop
    # ------------------------------------------------------------------
    image_width, image_height = CALIBRATION_IMAGE_SIZE
    tick_sec = 1.0 / 30.0
    lost_count = 0
    lost_threshold = 15
    frame_count = 0
    detect_every_k = 10
    tracker_fps_counter = 0
    tracker_fps_timer = time.monotonic()
    prev_state = sm.state
    search_start_time = 0.0  # set when entering GOTO_TARGET, SEARCH, or LOST_SEARCH
    search_prev_heading = None  # for full-circle accumulated turn
    search_accumulated_deg = 0.0
    obstacle_arc_start_time = 0.0  # when ultrasonic < 15 cm we arc until clear

    def on_cat_location(x: float, y: float):
        sm.dispatch(Event.CAT_LOCATION_RECEIVED, (x, y))
        log.info("CMD cat_location (%.2f, %.2f) -> state=%s", x, y, sm.state.value)

    def on_stop():
        sm.dispatch(Event.STOP_COMMAND)
        motion_driver.stop()
        log.info("CMD stop -> state=%s", sm.state.value)

    log.info("Main loop running at ~30 Hz. State: %s. Log file: %s", sm.state.value, LOG_FILE)

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

            # Read ultrasonic in all phases except IDLE (for display and obstacle avoid)
            ultrasonic_cm = range_sensor.get_distance_cm() if state != State.IDLE else None
            target_cm = calib.get_target_distance_cm()
            obstacle_close = (
                state != State.IDLE
                and ultrasonic_cm is not None
                and ultrasonic_cm < target_cm
            )

            if obstacle_close:
                # Stop and arc around: something is closer than 15 cm
                if obstacle_arc_start_time <= 0:
                    obstacle_arc_start_time = time.monotonic()
                cycle_sec = time.monotonic() - obstacle_arc_start_time
                steer, speed = compute_search_tick(cycle_sec, calib)
                motion_driver.set_steer(steer)
                motion_driver.forward(speed)
                location.update(tick_sec, speed, steer, calib.get_cm_per_sec(speed))
            else:
                obstacle_arc_start_time = 0.0
                if state == State.IDLE:
                    motion_driver.stop()

                elif state == State.GOTO_TARGET:
                    # Search arc the whole way until we reach target; can find cat on the way
                    target = sm.target_xy
                    if target is not None:
                        if search_start_time <= 0:
                            search_start_time = time.monotonic()
                        pos = location.get_position()
                        heading = location.get_heading_deg()
                        tx_cm = target[0] * 100.0
                        ty_cm = target[1] * 100.0
                        _, _, arrived = compute_goto(
                            pos[0], pos[1], heading, tx_cm, ty_cm, calib,
                        )
                        if arrived:
                            motion_driver.stop()
                            sm.dispatch(Event.AT_TARGET)
                            log.info("At target (%.1f, %.1f) cm", tx_cm, ty_cm)
                        elif bbox_valid:
                            sm.dispatch(Event.CAT_FOUND, bbox_xywh)
                            lost_count = 0
                        else:
                            cycle_sec = time.monotonic() - search_start_time
                            steer, speed = compute_search_tick(cycle_sec, calib)
                            motion_driver.set_steer(steer)
                            motion_driver.forward(speed)
                            location.update(tick_sec, speed, steer, calib.get_cm_per_sec(speed))
                    else:
                        sm.dispatch(Event.AT_TARGET)

                elif state in (State.SEARCH, State.LOST_SEARCH):
                    # Full circle: steer left until we've turned 360°; then stop (no cat found)
                    if bbox_valid:
                        sm.dispatch(Event.CAT_FOUND, bbox_xywh)
                        lost_count = 0
                    else:
                        heading = location.get_heading_deg()
                        if search_prev_heading is None:
                            search_prev_heading = heading
                            search_accumulated_deg = 0.0
                        # Unwrap delta so we accumulate actual rotation
                        delta = heading - search_prev_heading
                        if delta > 180:
                            delta -= 360
                        elif delta < -180:
                            delta += 360
                        search_accumulated_deg += delta
                        search_prev_heading = heading
                        if search_accumulated_deg >= 360.0:
                            motion_driver.stop()
                            sm.dispatch(Event.SEARCH_CYCLE_DONE)
                            log.info("Search circle complete, no cat found; stopping.")
                        else:
                            steer, speed = compute_full_circle_tick(calib)
                            motion_driver.set_steer(steer)
                            motion_driver.forward(speed)
                            location.update(tick_sec, speed, steer, calib.get_cm_per_sec(speed))

                elif state in (State.APPROACH, State.TRACK):
                    if bbox_valid:
                        lost_count = 0
                        center_cat.center_cat_control(
                            bbox_xywh, image_width, image_height, calib,
                            target_distance_cm=calib.get_target_distance_cm(),
                        )
                        # Only transition to TRACK when ultrasonic distance <= target (no bbox fallback)
                        if state == State.APPROACH:
                            if ultrasonic_cm is not None and ultrasonic_cm <= target_cm + 5.0:
                                sm.dispatch(Event.DISTANCE_AT_15CM)
                    else:
                        lost_count += 1
                        if lost_count >= lost_threshold:
                            sm.dispatch(Event.CAT_LOST)
                            motion_driver.stop()

            # Log state changes; reset search timing when entering search states
            new_state = sm.state
            if new_state != prev_state:
                log.info("State: %s -> %s", prev_state.value, new_state.value)
                if new_state == State.GOTO_TARGET:
                    search_start_time = time.monotonic()
                if new_state in (State.SEARCH, State.LOST_SEARCH):
                    search_start_time = time.monotonic()
                    search_prev_heading = None
                    search_accumulated_deg = 0.0
                prev_state = new_state

            # Update location into shared state (for Web UI status)
            pos = location.get_position()
            heading = location.get_heading_deg()
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
        log.info("Shutting down...")
        stop_event.set()
        sm.dispatch(Event.STOP_COMMAND)
        motion_driver.stop()

        camera_thread.join(timeout=2)
        tracker_thread.join(timeout=2)
        detector_thread.join(timeout=2)
        log.info("Bye.")


if __name__ == "__main__":
    main()
