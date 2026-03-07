# Code Review: `cat_follow`

**Reviewed:** `C:\Users\rahaber\my_projects\car-x\cat_follow`  
**Scope:** Structure, correctness, thread safety, maintainability, and consistency.

---

## Summary

The module is a well-structured cat-follow feature for PiCar-X with a clear state machine, shared memory pool, and separate camera/tracker/detector threads. A few **critical** and **medium** issues should be addressed; the rest are minor or optional.

| Severity | Count | Notes |
|----------|--------|--------|
| Critical | 1 | Detector: TFLite implementation never used (overwritten by stub) |
| Medium   | 4 | Duplicate code, thread safety, missing type, logging |
| Minor    | 5 | Docstrings, constants, dead code, style |

---

## Critical

### 1. **`threads/detector.py`: TFLite implementation is never used**

The file defines `run_detector_loop` twice. The first (lines 97–216) is the full TFLite-capable implementation; the second (lines 237–281) is the old stub. In Python the second definition overwrites the first, so **the TFLite detector is never used** and the process always runs the stub (fixed bbox every second).

**Fix:** Remove the orphaned docstring (lines 216–223) and the entire stub block (lines 225–281). Keep only the TFLite implementation.

---

## Medium

### 2. **Duplicate `limits` modules**

- `cat_follow/limits.py` and `cat_follow/motion/limits.py` are identical.
- Only `motion/*` imports limits (from `motion.limits`). The top-level `limits.py` is unused.

**Recommendation:** Delete `cat_follow/limits.py` to avoid drift and confusion, or add a comment that motion uses `motion/limits` and the top-level file is legacy/deprecated.

### 3. **`StateMachine` is not thread-safe**

- The main loop calls `sm.dispatch()` and reads `sm.state`; the Flask handler reads `sm.state` in `/api/status`.
- Enum reference reads are often safe in CPython, but the state can change between read and use, and the pattern is fragile.

**Recommendation:** Add a short lock (e.g. `threading.Lock`) around `dispatch()` and around reads of `state`/`target_xy`/`last_bbox` if they are used from more than one thread, or document “single writer (main loop), readers (Flask) accept possibly stale state.”

### 4. **`range_sensor`: shared globals without a lock**

`_last_distance_cm` and `_last_read_time` are updated in `get_distance_cm()` and read in `get_last_distance_cm()` (e.g. from the Flask thread). For a single float/None this is often “safe” in CPython but not guaranteed.

**Recommendation:** Use a small lock (or document that `get_last_distance_cm` is best-effort and may be stale).

### 5. **`Calibration.reload()` / `set_all_calibration_data()` not thread-safe**

The main loop calls `get_cm_per_sec()`, `get_max_steer_angle_deg()`, `get_target_distance_cm()`, etc., while the Web UI can call `save()` / `set_all_calibration_data()`. That can lead to inconsistent or partially updated state.

**Recommendation:** Protect calibration reads/writes with a lock, or ensure calibration is only changed when the car is IDLE and the UI is serialized with the main loop.

### 6. **`calibration/loader.py`: `_save_json` uses `print` on error**

```python
except Exception as e:
    print(f"Error saving calibration file {name}: {e}")
```

**Recommendation:** Use the project logger (e.g. `cat_follow.logger.get_logger("calibration")`) instead of `print` for consistency and log aggregation.

---

## Minor

### 7. **`motion/goto_xy.py` vs `calibration/goto_xy.py`**

Two different `compute_goto` implementations:

- **`motion/goto_xy.py`** – used by main loop; uses `limits.clamp_steer` and `limits.clamp_speed`; arrival at 10 cm; base_speed 30.
- **`calibration/goto_xy.py`** – not used by main loop; no limits; arrival 10 cm; base_speed 40.

**Recommendation:** Clarify the role of `calibration/goto_xy.py` (e.g. for calibration-only scripts or future use). If it’s dead code, remove or document.

### 8. **Magic numbers in main loop**

Examples: `lost_threshold = 15`, `detect_every_k = 10`, `target_cm + 5.0` for APPROACH→TRACK, `10.0` cm arrival in `goto_xy`, obstacle distance “15 cm” in comments vs `calib.get_target_distance_cm()`.

**Recommendation:** Move tunable values to a small config (or named constants) and use `target_distance_cm` (or one named constant) instead of hardcoding “15 cm” in comments.

### 9. **`center_cat.py`: broad `except Exception`**

```python
try:
    from cat_follow import range_sensor
    dist_cm = range_sensor.get_distance_cm()
except Exception:
    dist_cm = None
```

**Recommendation:** Catch a specific exception (e.g. `ImportError`) or re-raise after logging so real bugs are visible.

### 10. **`motion/calibration_routines.py`: mock only when `picarx` is missing**

The mock `Picarx` is defined only inside `except ImportError`. So:

- If `picarx` is installed, `Picarx` is the real class.
- If not, `Picarx` is the mock, but `main_loop` will already fail on `from picarx import Picarx`.

So the mock is only useful for scripts that import `calibration_routines` without running `main_loop`. Consider documenting that, or providing a small test/script that uses the mock explicitly.

### 11. **Web UI: two `main.html` and `static`**

- `web_ui/main.html` references `/static/css/style.css` and `/static/js/main.js`.
- The app serves `templates/main.html` (inline styles, CDN Alpine.js) and sets `static_folder` to `web_ui/static`. If `web_ui/static` is missing, any route or template that uses `/static/` will 404.

**Recommendation:** Either add `web_ui/static` and put assets there, or remove unused `web_ui/main.html` and document that the live UI is `templates/main.html`.

---

## What’s working well

- **Architecture:** Clear separation: state machine, commands, motion, calibration, memory pool, threads. Detection works independently of the web UI (per project rules).
- **Memory:** Pre-allocated pool and `SharedState` with per-resource locks avoid per-frame allocations and give predictable behavior.
- **Commands:** `poll_commands` with lock and callbacks outside the lock is correct and avoids holding the lock during slow work.
- **Odometry:** Bicycle model and heading normalization are implemented correctly.
- **State machine:** Table-driven transitions and explicit events are easy to follow and extend.
- **Calibration:** Single place for speed/steer/bbox-distance and JSON load/save is clear.
- **Logging:** Centralized logger and file + console handlers are consistent.

---

## Suggested order of fixes

1. **Immediate:** Remove duplicate stub and docstring in `threads/detector.py` so the TFLite detector is used.
2. **Short term:** Add thread safety for `StateMachine` and/or document single-writer usage; add lock or documentation for `range_sensor` and `Calibration` if they are used from multiple threads.
3. **Cleanup:** Remove or document `cat_follow/limits.py`; replace `print` in `_save_json` with logger; narrow exception in `center_cat.py`; clarify or remove `calibration/goto_xy.py` and duplicate `main.html`/static layout.

---

*End of code review.*
