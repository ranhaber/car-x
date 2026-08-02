# Code Review: `cat_follow`

> **Historical snapshot.** This review primarily evaluates the legacy
> `main_loop.py`/web prototype and the original contract runtime. It is not a
> specification of the approved future FSM. Use
> `docs/Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`
> for target behavior and its “Current implementation conflicts” section for
> migration status.

**Reviewed:** `C:\Users\rahaber\my_projects\car-x\cat_follow`  
**Scope:** Structure, correctness, thread safety, maintainability, consistency.

---

## Summary

The module is a well-structured cat-follow feature for PiCar-X with clear separation of concerns, thread-safe shared state, and a Blueprint-based Web UI. All previously identified issues have been addressed.

---

## Architecture and thread safety

- **State machine:** `StateMachine` uses a lock for `state`, `target_xy_cm`, `last_bbox`, and `dispatch()`; safe for main loop (writer) and Flask (reader).
- **Commands:** `poll_commands` uses a lock and runs callbacks outside the lock.
- **Range sensor:** `_last_distance_cm` and `_last_read_time` are protected by a lock; `get_distance_cm()` and `get_last_distance_cm()` are thread-safe. Production contract runtime injects `EdgeTimedUltrasonic.latest_distance_cm` via `set_reader()`; legacy `main_loop.py` still uses `set_car(Picarx)` polling.
- **Calibration:** All reads and writes (`get_*`, `set_all_calibration_data`, `get_all_calibration_data`, `reload`, `save`) are under a single lock.
- **Web UI FPS:** Tracker FPS and stream FPS use locks for cross-thread access.
- **Streaming:** The legacy MJPEG route (`routes_streaming.py`) has been removed; there is no MJPEG/software video fallback. H.264 streaming is the only monitoring path (see `routes_h264.py`).
- **Memory:** Pre-allocated pool and `SharedState` with per-resource locks; no per-frame allocations in the hot path.
- **Detector:** Single RKNN `run_detector_loop`; the deterministic stub is explicitly enabled for development only.
- **Tracker:** `PredictiveTracker` maintains sticky primary/secondary identities; only PRIMARY_CAT feeds behavior.

---

## Configuration and constants

- **Main loop:** `LOST_THRESHOLD`, `DETECT_EVERY_K`, `APPROACH_TRACK_MARGIN_CM` are named constants at the top of `main_loop.py`; obstacle and approach logic use `target_cm` from calibration.
- **Goto:** `motion/goto_xy.py` defines `GOTO_ARRIVAL_CM` for the arrival threshold; runtime goto uses this module; `calibration/goto_xy.py` is documented for calibration runs only.

---

## Documentation and layout

- **README:** Notes that runtime goto uses `motion/goto_xy.py` and calibration uses `calibration/goto_xy.py`; Web UI points to `templates/main.html` and `web_ui/static/`.
- **Odometry:** Comment states that state is single-threaded (main loop only).
- **Calibration routines:** Docstring explains mock Picarx is used when `picarx` is not installed (e.g. PC); useful for standalone calibration scripts.
- **web_ui/static:** Directory present (e.g. `.gitkeep`) so Flask static folder exists.

---

## What’s working well

- **Architecture:** State machine, commands, motion, calibration, memory pool, camera/tracker/detector threads; detection runs independently of the web UI (per project rules).
- **Web UI:** Blueprint-based (pages, streaming, control, status, stream_config, detector, calibration); context object passed into route inits.
- **Logging:** Centralized logger, file + console handlers, named loggers per module.
- **Calibration loader:** Uses project logger in `_save_json`.
- **center_cat:** Specific exceptions for range_sensor access.
- **Location:** Pluggable providers (odometry default); bicycle model and heading normalization are correct.

---

*End of code review.*
