# Implementation Steps and Testing Plan

This document lists **coding steps** to implement the [software architecture](DESIGN_SOFTWARE_ARCHITECTURE_MEMORY_AND_PARALLELISM.md) (pre-allocated memory, shared state, parallel threads) and **how to test each step**.

---

## Overview of Steps

| Step | What you build | How you test it |
|------|----------------|-----------------|
| 1 | Memory pool (pre-allocate frames + bbox structs) | Unit test: alloc once, check shapes/sizes, no alloc in get/set |
| 2 | SharedState (wraps pool + locks, get/set API) | Unit test: concurrent get/set from two threads, no corruption |
| 3 | Thread stubs (camera, tracker, detector) that only read/write shared state | Integration test: start threads, main reads back written values |
| 4 | Main loop using SharedState instead of direct detector/motion | Run main loop with mock commands; assert state transitions and shared reads |
| 5 | Camera thread with real or mock capture → frame_latest | Test: run camera thread, main reads frame_latest non-zero or pattern |
| 6 | Tracker thread (frame_latest → bbox_tracker) | Test: feed known frame + bbox, run tracker, read bbox_tracker |
| 7 | Detector thread (frame_for_detector → bbox_detector) | Test: feed test image, run detector, read bbox_detector (or stub detector) |
| 8 | Full pipeline: camera + tracker + detector + main loop | Manual/smoke: run app, inject command, observe state and bbox flow |

---

## Step 1: Memory pool (pre-allocate all buffers)

**Coding:**

- Add `cat_follow/memory/` package.
- In `cat_follow/memory/pool.py`:
  - Define constants: `FRAME_H`, `FRAME_W`, `FRAME_C = 3`, `FRAME_SHAPE`, `FRAME_NBYTES`, and `FRAME_RING_N`.
  - Implement `allocate_pool()` that:
    - Allocates a small rotating `frame_ring` with shape `(FRAME_RING_N, H, W, C)` plus `frame_for_detector`.
    - Allocates `bbox_tracker` and `bbox_detector` as arrays of 5 floats (x, y, w, h, valid).
    - Allocates `odometry_xyh` as 3 floats (x, y, heading).
    - Returns a simple namespace or dataclass holding these (no locks yet).
  - Ensure no allocation happens inside any “get current frame” or “set bbox” logic; camera writes into a pre-allocated ring slot and calls `publish_latest_from_write()` to publish the newest index.

**Testing:**

- **Unit test** in `tests/test_memory_pool.py`:
  1. Call `allocate_pool()` once.
  2. Assert `frame_latest.shape == (480, 640, 3)` and `frame_latest.dtype == np.uint8`; same for `frame_for_detector`.
  3. Assert `frame_latest.nbytes == FRAME_NBYTES` (640*480*3).
  4. Assert `len(bbox_tracker) == 5`, `len(bbox_detector) == 5`, `len(odometry_xyh) == 3`.
  5. Write into `frame_latest` and `bbox_tracker`, then read back and assert values; confirm same buffer (e.g. id or base pointer) after multiple “set” operations to ensure no realloc.

**How to run:**  
`python -m pytest tests/test_memory_pool.py -v`  
or  
`python tests/test_memory_pool.py`

---

## Step 2: SharedState (locks + get/set API)

**Coding:**

- In `cat_follow/memory/shared_state.py`:
  - Define `SharedState` that takes the pool (from `allocate_pool()`).
  - Add locks: e.g. `lock_frame`, `lock_bbox_tracker`, `lock_bbox_detector` (and optionally one for odometry if you want).
  - Implement:
    - `get_frame_latest()` / `set_frame_latest(src)` (copy into pool’s `frame_latest` under `lock_frame`), or swap index if using double buffer.
    - `get_frame_for_detector()` / `copy_latest_to_detector_frame()` (under lock(s)).
    - `get_bbox_tracker()` / `set_bbox_tracker(x, y, w, h, valid)` under `lock_bbox_tracker`.
    - `get_bbox_detector()` / `set_bbox_detector(x, y, w, h, valid)` under `lock_bbox_detector`.
    - `get_odometry_xyh()` / `set_odometry_xyh(x, y, h)`.
  - All get/set must use only the pre-allocated pool buffers; no new arrays allocated in get/set.

**Testing:**

- **Unit test** in `tests/test_shared_state.py`:
  1. `pool = allocate_pool()`, `shared = SharedState(pool)`.
  2. Single-thread: `set_bbox_tracker(1, 2, 3, 4, 1)` then `assert get_bbox_tracker() == (1, 2, 3, 4, 1)`.
  3. **Concurrent test:** Start two threads; one repeatedly writes `bbox_tracker` with distinct values (e.g. iteration index in x), the other repeatedly reads and checks that the read value is one of the written 5-tuples (no partial/corrupt 5-tuple). Run for a short burst (e.g. 1000 iterations each). This checks that the lock prevents torn reads/writes.

**How to run:**  
`python -m pytest tests/test_shared_state.py -v`

---

## Step 3: Thread stubs (camera, tracker, detector)

**Coding:**

- Add `cat_follow/threads/` package.
- **Camera stub** in `cat_follow/threads/camera.py`:
  - Function `run_camera_loop(shared: SharedState, stop_event: threading.Event)`.
  - Loop: while not stop_event: capture into a pre-allocated write buffer (or produce a known pattern in mock mode), write into the pool's current ring slot (via `shared.get_write_buffer()`), then call `shared.publish_latest_from_write()` and sleep(1/30).
- **Tracker stub** in `cat_follow/threads/tracker.py`:
  - `run_tracker_loop(shared, stop_event)`: each iteration read `frame_latest` (via `get_frame_latest()`), run an OpenCV single-object tracker (KCF/CSRT/MOSSE), and write `bbox_tracker`. Tracker re-initialization uses detector outputs with IoU checks, temporal confirmation, smoothing, and a cooldown to avoid thrash.
- **Detector stub** in `cat_follow/threads/detector.py`:
  - `run_detector_loop(shared, stop_event)`: every K iterations (e.g. K=10), read `frame_for_detector`, run detection (stub or TFLite) and write `bbox_detector`. Sleep(1/30) so it runs at same rate but only “detects” every K.

**Testing:**

- **Integration test** in `tests/test_thread_stubs.py`:
  1. Alloc pool, create SharedState, create `stop_event`.
  2. Start camera thread (stub), tracker thread (stub), detector thread (stub).
  3. In main: repeatedly call `shared.copy_latest_to_detector_frame()` every K iterations (or have stub detector read `frame_latest` if you prefer), then sleep a bit.
  4. After a short run (e.g. 1–2 seconds), stop threads.
  5. Assert that `get_frame_latest()` (or the buffer) was written (e.g. not all zeros, or matches the pattern the camera stub writes). For ring behavior test `tests/test_camera_ring.py` verifies rotating write slots and `copy_latest_to_detector_frame()`.
  6. Assert that `get_bbox_tracker()` and `get_bbox_detector()` were written (e.g. valid=1 and reasonable numbers). This verifies that threads can read/write shared state without crashing and that main sees their outputs.

**How to run:**  
`python -m pytest tests/test_thread_stubs.py -v`

---

## Step 4: Main loop uses SharedState

**Coding:**

- In `cat_follow/main_loop.py`:
  - At startup: call `allocate_pool()`, create `SharedState(pool)`, then start camera, tracker, detector threads (pass `shared` and a `stop_event`).
  - In the main loop: **do not** call `get_cat_bbox()` directly. Instead:
    - Every tick, copy `frame_latest` → `frame_for_detector` every K frames (so detector stub has something to read).
    - Read `shared.get_bbox_tracker()` (and optionally `get_bbox_detector()` for re-init).
    - Use the tracker bbox as “current bbox” for state machine and for `center_cat_control`.
    - Keep existing state machine and command polling; only the source of “bbox” changes from `get_cat_bbox(None, ...)` to `shared.get_bbox_tracker()`.
  - On exit: set `stop_event`, join all threads.

**Testing:**

- **Integration test** in `tests/test_main_loop_shared_state.py` (or extend existing main_loop test):
  1. Use stub threads (Step 3) and run main loop in a thread for a few seconds.
  2. From test thread, call `set_cat_location(10, 10)` (or inject command however you do in tests).
  3. Assert that state machine eventually reaches GOTO_TARGET then (after AT_TARGET) SEARCH, and that APPROACH/TRACK receive bbox from shared state (e.g. stub tracker sets bbox, main reads it and runs center_cat_control).
  4. Optional: assert that no exception is raised and that `get_bbox_tracker()` is read repeatedly in the main loop (e.g. add a small counter in a test subclass or via a mock).
- **Manual test:** Run `python -m cat_follow.main_loop`; send a cat_location command (e.g. via file or in-code); confirm state transitions and that the loop keeps running with bbox from shared state.

**How to run:**  
`python -m pytest tests/test_main_loop_shared_state.py -v`  
and  
`python -m cat_follow.main_loop` (keyboard interrupt to stop).

---

## Step 5: Camera thread with real or mock capture

**Coding:**

- Replace the camera stub with either:
  - **Mock:** Fill `frame_latest` with a deterministic pattern (e.g. gradient or constant color) so vision tests are reproducible, or
  - **Real:** Use picamera2/vilib to capture into the **pre-allocated** buffer (or copy one frame into `frame_latest` each time). Ensure no per-frame allocation; reuse the same buffer.
- Keep the same `run_camera_loop(shared, stop_event)` interface.

**Testing:**

- **Mock:** In test, start camera thread with mock capture; after N frames, read `frame_latest` and assert pattern (e.g. center pixel value, or sum of array).
- **Real (on Pi):** Run the app and confirm MJPEG or logging shows non-zero frames; no need for an automated test if no Pi in CI.

**How to run:**  
`python -m pytest tests/test_camera_thread.py -v` (mock); on device, run main loop and observe.

---

## Step 6: Tracker thread (real OpenCV tracker)

**Coding:**

- In `cat_follow/threads/tracker.py`: create OpenCV tracker (e.g. KCF) once at start. Each iteration: read `frame_latest`, call `tracker.update(frame)`; write result to `shared.set_bbox_tracker(...)`. If “re_init” is set (from detector), call `tracker.init(frame, bbox)` and clear the flag. Use only pre-allocated buffers; the frame reference from shared state is already pre-allocated.

**Testing:**

- **Unit/integration:** Provide a synthetic frame (e.g. black with a white rectangle) and an initial bbox; run tracker for a few frames while moving the rectangle in the frame (or use a pre-recorded small video). Assert that `get_bbox_tracker()` stays near the moving rectangle (e.g. IoU or center distance within threshold).
- Alternatively: run tracker on one static frame and assert bbox unchanged (no crash).

**How to run:**  
`python -m pytest tests/test_tracker_thread.py -v`

---

## Step 7: Detector thread (TFLite or stub)

**Coding:**

- In `cat_follow/threads/detector.py`: every K frames, read `frame_for_detector`, run TFLite (or stub: return fixed bbox). Write result to `shared.set_bbox_detector(...)`. Use a single TFLite interpreter and pre-allocated input buffer (view over resized frame) created at startup.

**Testing:**

- **Stub detector:** Run detector loop; feed a test image (e.g. all zeros); stub returns (320, 240, 80, 80, 1). Assert `get_bbox_detector()` equals that after a short wait.
- **Real TFLite (if available):** Run on an image containing a cat (or COCO cat class); assert bbox_detector has valid=1 and plausible coordinates.

**How to run:**  
`python -m pytest tests/test_detector_thread.py -v`

---

## Step 8: Full pipeline (camera + tracker + detector + main)

**Coding:**

- Ensure main loop:
  - Starts all threads with shared state.
  - Copies `frame_latest` → `frame_for_detector` every K frames.
  - Reads `bbox_tracker` (and `bbox_detector` when re-init needed).
  - Runs state machine and motion as already done in Step 4.
- Add optional logging (e.g. state, bbox every second) for observability.

**Testing:**

- **Smoke / manual:** Run `python -m cat_follow.main_loop` on the Pi (or with mock camera). Send cat_location; confirm IDLE → GOTO_TARGET → SEARCH → APPROACH (when cat “found”) and that bbox values in logs are non-zero when tracker/detector are running. Send stop; confirm IDLE. No crash over 1–2 minutes.
- **Automated (optional):** Integration test that runs the full loop with mock camera and stub detector/tracker for a fixed duration and asserts at least one state transition and that main loop never raises.

**How to run:**  
Manual: `python -m cat_follow.main_loop`.  
Optional: `python -m pytest tests/test_full_pipeline.py -v`

---

## Test Summary Table

| Step | Test file | What is verified |
|------|-----------|------------------|
| 1 | `test_memory_pool.py` | Single alloc; correct shapes/sizes; same buffers on repeated use |
| 2 | `test_shared_state.py` | Get/set correctness; concurrent read/write without corruption |
| 3 | `test_thread_stubs.py` | Stub threads write to shared state; main reads their output |
| 4 | `test_main_loop_shared_state.py` | Main loop uses shared bbox; state machine and motion still behave |
| 5 | `test_camera_ring.py` | Camera ring publish and detector-copy behavior |
| 6 | `test_tracker_thread.py` | Tracker produces bbox from frame (synthetic or static) |
| 7 | `test_detector_thread.py` | Detector (stub or TFLite) writes bbox_detector |
| 8 | Manual / `test_full_pipeline.py` | Full app runs; state transitions; no crash |

---

## Order and Dependencies

- Steps 1 and 2 can be done in parallel after the package layout exists; 2 depends on 1.
- Step 3 depends on 2 (threads need SharedState).
- Step 4 depends on 3 (main loop starts those threads).
- Steps 5–7 can be done in any order after 3; they replace stubs with real or mock implementations.
- Step 8 is final integration after 5–7 are in place.

Running tests after each step keeps regressions visible and ensures the architecture (pre-alloc, shared state, thread roles) is validated incrementally.
