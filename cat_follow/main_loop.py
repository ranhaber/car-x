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
from cat_follow.threads.detector import (
    run_detector_loop,
    DetectorHandshake,
    DetectorFatalHook,
    DETECTOR_READY_TIMEOUT_S,
)

# Web UI
from cat_follow.web_ui.app import create_app, set_tracker_fps

log = get_logger("main_loop")

# Main loop tunables
LOST_THRESHOLD = 15          # frames without bbox before CAT_LOST
DETECT_EVERY_K = 10          # run detector every K frames
APPROACH_TRACK_MARGIN_CM = 5.0  # transition APPROACH -> TRACK when distance <= target_cm + this

# Suppress noisy Flask/werkzeug access logs (they still go to the file)
logging.getLogger("werkzeug").setLevel(logging.WARNING)


def main():
    # ------------------------------------------------------------------
    # 1. Load config / calibration
    # ------------------------------------------------------------------
    calib = Calibration()
    sm = StateMachine()
    location.reset(0, 0, 0)
    from picarx import Picarx
    px = Picarx()
    motion_driver.set_car(px)
    range_sensor.set_car(px)
    log.info("Calibration loaded. State machine ready.")

    # ------------------------------------------------------------------
    # 2-3. Pre-allocate pool and create SharedState
    # ------------------------------------------------------------------
    pool = allocate_pool()
    shared = SharedState(pool)
    log.info("Memory pool allocated. SharedState created.")

    # ------------------------------------------------------------------
    # 4. Detection backend is loaded lazily by the detector thread (RKNN NPU);
    #    startup validation happens via preflight_perception() below.
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # 5. Start worker threads
    # ------------------------------------------------------------------
    stop_event = threading.Event()
    detector_handshake = DetectorHandshake()

    # A perception fatal error must stop the vehicle: latch a stop and cut the
    # motors immediately.  The main loop below also observes ``stop_event``.
    detector_fatal_hook = DetectorFatalHook()

    def _on_detector_fatal(message: str) -> None:
        log.error("Perception fatal (%s); stopping motors.", message)
        try:
            motion_driver.stop()
        except Exception:  # noqa: BLE001
            pass
        stop_event.set()

    detector_fatal_hook.set_handler(_on_detector_fatal)

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
        kwargs={"handshake": detector_handshake, "on_fatal": detector_fatal_hook},
        name="CatFollow-Detector", daemon=True,
    )

    camera_thread.start()
    tracker_thread.start()
    detector_thread.start()

    # Startup handshake: block until the detector worker validates its backend
    # (or reports stub mode). A validation failure raises here and aborts
    # startup instead of leaving the app running blind.
    try:
        npu_ready = detector_handshake.wait_ready(timeout=DETECTOR_READY_TIMEOUT_S)
    except Exception:
        stop_event.set()
        raise
    log.info(
        "Camera, Tracker, Detector threads started (detector=%s).",
        "NPU" if npu_ready else "stub",
    )

    # ------------------------------------------------------------------
    # 6. Start Web UI (Flask) in a background thread
    # ------------------------------------------------------------------
    app = create_app(shared=shared, state_machine=sm, calibration=calib, picarx=px)

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
    frame_count = 0
    tracker_fps_counter = 0
    tracker_fps_timer = time.monotonic()
    prev_state = sm.state
    search_start_time = 0.0  # set when entering GOTO_TARGET, SEARCH, or LOST_SEARCH
    search_prev_heading = None  # for full-circle accumulated turn
    search_accumulated_deg = 0.0
    obstacle_arc_start_time = 0.0  # when ultrasonic < target_cm we arc until clear
    # Fail-closed sensor-fault handling: a genuinely faulty/stuck ultrasonic
    # keeps returning None and used to look "clear".  Tolerate transient None
    # (HC-SR04 out-of-range in open space) but stop after a sustained streak.
    ultrasonic_none_streak = 0
    ULTRASONIC_FAULT_TICKS = 15  # ~0.5 s at 30 Hz

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

            # A worker thread (e.g. the detector) escalating a fatal error sets
            # stop_event; observe it here and fail safe rather than driving on.
            if stop_event.is_set():
                log.error("stop_event set (worker escalation); stopping motors.")
                motion_driver.stop()
                sm.dispatch(Event.STOP_COMMAND)
                break

            # Poll commands (thread-safe via lock)
            poll_commands(on_cat_location=on_cat_location, on_stop=on_stop)

            # NOTE: The detector thread now owns the detector-frame snapshot
            # (``snapshot_detector_frame``) atomically with its generation
            # counter, so the main loop no longer copies here (a second writer
            # would desync the frame/bbox generation used by the tracker).
            frame_count += 1

            # Read bbox from shared state (from tracker thread)
            bbox = shared.get_bbox_tracker()
            bbox_valid = bbox[4] > 0
            bbox_xywh = (bbox[0], bbox[1], bbox[2], bbox[3]) if bbox_valid else None

            # State machine logic
            state = sm.state

            # Read ultrasonic in all phases except IDLE (for display and obstacle avoid)
            ultrasonic_cm = range_sensor.get_distance_cm() if state != State.IDLE else None
            target_cm = calib.get_target_distance_cm()

            # Track consecutive None reads while driving to detect a stuck/faulty
            # sensor (fail-closed) instead of treating None as "clear".
            if state != State.IDLE and ultrasonic_cm is None:
                ultrasonic_none_streak += 1
            else:
                ultrasonic_none_streak = 0
            ultrasonic_fault = ultrasonic_none_streak >= ULTRASONIC_FAULT_TICKS

            if ultrasonic_fault:
                # Sensor fault: refuse to drive blind.  Stop and hold until the
                # sensor recovers (a valid read resets the streak).
                if ultrasonic_none_streak == ULTRASONIC_FAULT_TICKS:
                    log.error(
                        "Ultrasonic returned None for %d ticks; failing closed (stop).",
                        ultrasonic_none_streak,
                    )
                motion_driver.stop()
                obstacle_arc_start_time = 0.0
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick_sec - elapsed))
                continue

            obstacle_close = (
                state != State.IDLE
                and ultrasonic_cm is not None
                and ultrasonic_cm < target_cm
            )

            if obstacle_close:
                # Stop and arc around: something closer than target_cm
                if obstacle_arc_start_time <= 0:
                    log.info("Obstacle detected! Distance: %.1f cm", ultrasonic_cm)
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
                        steer, speed, arrived = compute_goto(
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
                            if ultrasonic_cm is not None and ultrasonic_cm <= target_cm + APPROACH_TRACK_MARGIN_CM:
                                sm.dispatch(Event.DISTANCE_AT_15CM)
                    else:
                        lost_count += 1
                        if lost_count >= LOST_THRESHOLD:
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
