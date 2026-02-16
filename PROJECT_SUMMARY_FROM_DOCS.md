# Project Summary (from all *.md docs)

Single reference drawn from all project `.md` files: **goals**, **hardware and capabilities**, **software architecture**, **high-level design**, and **small testable steps**.

---

## 1. Project goals

- **Cat-follow behavior:** PiCar-X starts at home **(0,0)**; receives a **cat location (x,y) in meters** from outside (e.g. Web UI, MQTT, HTTP, file); drives toward that point; **searches** for the cat (e.g. arc); **approaches** to ~**15 cm** while keeping the cat in the **center of the frame**; **tracks** the cat at ~15 cm if it moves; on **cat lost** re-searches; **stops only on explicit stop command**.
- **Camera fixed:** Camera points straight ahead. **Car** steers and drives to center the cat in the frame (no pan-tilt following).
- **Performance:** Tracking at **30 FPS** (camera + tracker); detection every **K** frames (e.g. 5-10) for re-init; reproducible motion via **calibration** (speed-time-distance, steering limits, bbox-distance).
- **Modular, testable:** State machine pure logic; motion/vision/odometry/commands separable; each step testable in isolation (stubs, mocks, unit/integration tests).

- **Web UI:** A browser-based interface to monitor and control the car:
  - **Main tab:** Live MJPEG video stream (10 FPS, user-selectable resolution); shows a **rectangle around the detected/tracked cat** and **state text overlay**; continues until user navigates away.
  - **Main tab - Send target:** Two text fields X (meters), Y (meters) + "Send" button; starts the cat-follow sequence.
  - **Main tab - Stop:** Button; car goes to IDLE and stops.
  - **Main tab - Status bar:** Current state, odometry, tracker FPS, stream FPS, app version, CPU %, RAM %, CPU temp (polled every 1 s).
  - **Main tab - Resolution selector:** Dropdown: 640x480 / 320x240 / 160x120 (default = tracker resolution).
  - **Calibration tab:** View and edit speed-time-distance, steering limits, bbox-distance; save reloads values immediately.

---

## 2. Hardware used and capabilities

### 2.1 Platform

- **Raspberry Pi 4 Model B, 4 GB RAM** - main compute; runs Python 3, picar-x, vilib, OpenCV, TFLite.
- **SunFounder PiCar-X** - wheeled car with front steering, two drive motors, Robot HAT, camera, ultrasonic, grayscale.

### 2.2 Robot HAT (on Pi GPIO)

- **Connection:** 40-pin GPIO header (I2C: GPIO2/3, SPI: 8-11, 6; UART: 14/15; various GPIOs for motors, servos, ultrasonic, etc.).
- **Uses:** MCU (0x14, 0x15 on I2C), PWM/servos (P0-P11 via MCU), motors (D4, D5, P12, P13), camera pan/tilt (P0, P1), steering (P2), ultrasonic (D2, D3), grayscale (A0-A2), speaker, buttons, LED.
- **Free for add-ons:** **I2C** (HAT QWIIC/STEMMA port), **UART** (GPIO14/15), **SPI** (CE1). Used for optional **IMU** (e.g. BNO055, BNO085, MPU6050, ICM-20948) for heading/odometry.

### 2.3 Camera

- **Type:** OV5647 (CSI); 5 MP; typical use **640x480** or 720p for processing.
- **Frame rate:** 60 FPS at 640x480 or 30 FPS at 720p (vilib/picamera2).
- **H.264:** Pi 4 has **hardware H.264 encoding** via VideoCore VI GPU. Reserved for future use (recording, low-bandwidth streaming). Current stream uses **MJPEG** (simpler, lower latency, sufficient for LAN at 10 FPS).
- **Role:** Fixed forward; frames feed detector (COCO "cat") and OpenCV tracker; no pan-tilt for following.

### 2.4 Motion

- **API:** `forward(speed)`, `backward(speed)`, `stop()`, `set_dir_servo_angle(angle)` (e.g. +/-30 deg).
- **No encoders:** Position is **estimated** (time + commanded speed + steering; optional IMU fusion).

### 2.5 Position / odometry

- **Out of the box:** No wheel encoders, no IMU. **Option A:** Time + commanded speed + steering -> dead reckoning (x, y, theta). **Option B:** Add wheel encoders. **Option C:** Add I2C IMU (e.g. BNO055/BNO085) on HAT I2C for better heading. **Option E:** "Set origin here" at last stop so (0,0) is defined in software.

### 2.6 Vision / AI (existing stack)

- **vilib:** Picamera2, TFLite object detection (COCO; **class 16 = cat**), image classification (labels include cat breeds), face/hands/pose, Flask MJPEG.
- **Detection FPS on Pi 4:** ~3-10 (vilib default); ~5-15 with EfficientDet/small input; 20+ with Coral Edge TPU.
- **Tracking:** OpenCV KCF/CSRT/MOSSE: **20-30+ FPS** on Pi 4; used in a hybrid with detector every K frames.

### 2.7 Web UI (same host)

- **Host:** Same Raspberry Pi 4; Flask (`threaded=True`) serves the UI and streams; no extra hardware.
- **Camera:** One camera feed is used both for **processing** (detector/tracker) and for the **live stream**; the stream is a copy of `frame_latest` with the current **cat bbox** drawn as a rectangle.
- **Network:** Browser connects to Pi over LAN (Wi-Fi or Ethernet); typical port e.g. 5000 (Flask).

---

## 3. Software architecture

### 3.1 Principles

1. **Pre-allocated memory at startup** - All large buffers (frame buffers, bbox/odometry structs, command queue) allocated once before any thread starts; **no per-frame allocation** in hot paths.
2. **Shared memory between threads** - Camera, tracker, detector, and main thread share pre-allocated buffers via a **SharedState** (or pool) with **one lock per resource**; short critical sections (copy/swap only).
3. **Explicit parallel design** - Clear owner (writer) and readers for each buffer; optional detector in a **separate process** with `SharedMemory` if CPU/GIL limits FPS.
4. **Web UI** - Served from the same process (Flask thread); consumes **read-only** copy of current frame + bbox for the live stream (rectangle overlay); HTTP endpoints inject **cat_location (x,y) in meters** and **stop** into the command queue; calibration tab reads/writes calibration via API; no large alloc in stream path (reuse encode buffer or frame copy).

### 3.2 Pre-allocated buffers (at startup)

| Buffer | Type | Size | Writer | Readers |
|--------|------|------|--------|---------|
| `frame_latest` | ndarray (480,640,3) uint8 | ~0.9 MB | Camera thread | Tracker, Main (copy), Web UI stream |
| `frame_for_detector` | ndarray (480,640,3) uint8 | ~0.9 MB | Main (copy) | Detector |
| `bbox_tracker` | 4 float + 1 int | 20 B | Tracker | Main, Web UI stream |
| `bbox_detector` | 4 float + 1 int | 20 B | Detector | Main, Tracker (re-init) |
| `odometry_xyh` | 3 float | 12 B | Main | Main, Web UI status |
| Command queue | Fixed capacity (e.g. 8) | ~200 B | Command I/O (Web UI, file) | Main |

Total ~1.8 MB + small structs; TFLite interpreter and input buffer created once.

### 3.3 Thread/process roles

| Unit | Role | Inputs (shared) | Outputs (shared) | Rate |
|------|------|------------------|------------------|------|
| **Camera** | Capture into pre-alloc buffer | - | `frame_latest` | 30 FPS |
| **Tracker** | OpenCV tracker update | `frame_latest` | `bbox_tracker` | 30 FPS |
| **Detector** | TFLite cat detection | `frame_for_detector` | `bbox_detector` | Every K frames |
| **Main** | Commands, state machine, motion | bboxes, odometry, commands | Motion, copy frame->detector | ~30 Hz |
| **Web UI (Flask)** | HTTP requests + MJPEG stream | `frame_latest`, `bbox_tracker` (read) | MJPEG with bbox overlay; command queue (write) | 10 FPS stream; on-request commands |

### 3.4 Sync and locking

- One lock per logical resource (frame, bbox_tracker, bbox_detector); **double-buffer** for `frame_latest` optional (lock-free read).
- Main copies `frame_latest` -> `frame_for_detector` every K frames; detector runs when ready; tracker re-inits from `bbox_detector` when set.
- **commands.py** uses `threading.Lock` (Option B) to protect globals. Flask thread writes; main loop reads. No torn reads.

### 3.5 Startup order

1. Load config/calibration.
2. Allocate pool (frames, bbox arrays, odometry, command queue).
3. Create SharedState (pool + locks).
4. Create TFLite interpreter (and input buffer).
5. Start camera, tracker, detector threads (or detector process with shared memory).
6. Start **Web UI** server (Flask, `threaded=True`): routes for main tab (stream + buttons), calibration tab; stream generator reads frame + bbox, draws rectangle, yields MJPEG.
7. Run main loop (poll commands, read shared state, state machine, motion).

### 3.6 Web UI integration

- **Stream:** Flask generator reads `frame_latest` (and `bbox_tracker`) under lock, draws rectangle and state text on a local copy, resizes to user-selected resolution (640x480 / 320x240 / 160x120), encodes to JPEG, yields as MJPEG at **10 FPS**; no per-frame alloc (reuse one encode buffer). Built-in stub for H.264 later.
- **Commands:** "Send target (x,y)" -> POST /api/target `{"x": float, "y": float}` (meters) -> `set_cat_location(x, y)` under lock. "Stop" -> POST /api/stop -> `set_stop_command()` under lock. Event-driven (like ISR): Flask handler fires immediately on request.
- **Status:** GET /api/status every 1 s -> JSON: `{ state, odometry, tracker_fps, stream_fps, app_version, cpu_percent, ram_percent, cpu_temp }`.
- **Calibration tab:** GET /api/calibration -> JSON; POST /api/calibration -> validate + write files + reload. `Calibration.to_dict()` and `Calibration.update_from_dict(data)` added when we build this tab.

---

## 4. High-level design

### 4.1 Components

- **State machine** - IDLE -> GOTO_TARGET -> SEARCH -> APPROACH -> TRACK <-> LOST_SEARCH; transitions on events; `stop_command` -> IDLE from any state.
- **Odometry** - (x, y, theta) from (0,0); time + speed + steering (and optional IMU).
- **Cat target (x,y)** - From Web UI (meters), file, or MQTT; optional from camera later.
- **Cat detector** - TFLite/COCO class 16 (cat); every K frames; returns bbox or None.
- **Cat tracker** - OpenCV (e.g. KCF); every frame; 30 FPS; re-init from detector.
- **Distance to cat** - From bbox size (calibration) or ultrasonic; "15 cm" threshold.
- **Motion** - go_to_xy, center_cat (steer + drive to center bbox, camera straight), search_arc, stop; uses calibration limits.
- **Commands** - set_cat_location(x,y) in meters, stop; protected by lock; polled by main (sources: Web UI, file, MQTT, etc.).
- **Web UI** - See section 4.6 for finalized design.

### 4.2 Data flow

- **Commands:** Web UI (or file/MQTT) -> cat_location(x,y) meters / stop -> command queue (lock) -> main loop poll -> state machine -> motion.
- **Vision:** Camera -> frame_latest -> Tracker -> bbox_tracker; every K frames frame_for_detector -> Detector -> bbox_detector (re-init tracker). Main reads bbox_tracker (and bbox_detector when needed).
- **Control:** bbox center vs image center -> steering; bbox size/distance -> forward/back; calibration clamps steer/speed.
- **Web UI stream:** Flask reads frame_latest + bbox_tracker under lock -> draws rectangle + state text -> resizes to selected resolution -> encodes JPEG -> yields MJPEG at 10 FPS -> browser `<img>` tag.
- **Web UI status:** Browser polls GET /api/status every 1 s -> JSON with state, odometry, tracker FPS, stream FPS, app version, CPU %, RAM %, CPU temp.

### 4.3 States and events (summary)

| State | Motion | Exits |
|-------|--------|--------|
| IDLE | Stop | cat_location_received -> GOTO_TARGET |
| GOTO_TARGET | Drive toward (x,y) | at_target/timeout -> SEARCH; stop -> IDLE |
| SEARCH | Arc/search pattern | cat_found -> APPROACH; stop -> IDLE |
| APPROACH | Center cat, approach to 15 cm | distance<=15cm -> TRACK; cat_lost -> LOST_SEARCH; stop -> IDLE |
| TRACK | Center cat, hold ~15 cm | cat_lost -> LOST_SEARCH; stop -> IDLE |
| LOST_SEARCH | Arc/search | cat_found -> APPROACH; timeout -> SEARCH; stop -> IDLE |

### 4.4 Calibration (modular)

- **Speed-time-distance** - speed -> cm/s for goto and odometry.
- **Steering limits** - max steer angle, min turn radius (safe U-turn).
- **Bbox-distance** - bbox size -> distance (cm) for 15 cm logic.
- Stored in **calibration/** JSONs; single loader used by motion and odometry.

### 4.5 File layout (target)

```
cat_follow/
  __init__.py         # __version__ defined here
  memory/             # pool.py (pre-alloc), shared_state.py (locks + get/set)
  threads/            # camera.py, tracker.py, detector.py
  state_machine.py
  commands.py         # with threading.Lock (Option B)
  motion/             # driver, goto_xy, center_cat, search, limits
  vision/             # detector, tracker, distance
  odometry.py
  calibration/        # loader + JSONs
  main_loop.py
  web_ui/             # Flask app, templates, static
    app.py            # Flask routes, stream generator, API endpoints
    templates/        # Jinja2: main.html, calibration.html
    static/           # CSS, JS (Alpine.js via CDN)
tests/                # test_*.py per module and integration
```

### 4.6 Web UI - finalized design decisions

#### Technology and hosting

- **Frontend:** Alpine.js (via CDN) + plain HTML/CSS. No build step. Flask serves via Jinja2 templates.
- **Backend:** Flask (`threaded=True`) on the same Pi 4. Commands handled async on Flask's request handler thread (event-driven, like ISR).
- **Stream protocol:** **MJPEG** (multipart JPEG). Built-in stub/placeholder for H.264 later. Each frame is an independent JPEG; browser displays via `<img>` tag.
- **Stream FPS:** **10 FPS** (configurable). Lighter than tracker FPS (30).
- **Stream resolution:** User selects from **3 options**: 640x480 (default, same as tracker), 320x240 (half), 160x120 (quarter). Default = tracker resolution.

#### Main tab

| Element | Details |
|---------|---------|
| **Live stream** | MJPEG at 10 FPS; shows rectangle around tracked cat (from bbox_tracker); state name as text overlay; stream continues until user navigates away. |
| **Send target** | Two text fields: X (meters), Y (meters) + "Send" button. POST /api/target `{"x": 1.5, "y": 0.8}`. |
| **Stop** | Button. POST /api/stop. Car goes to IDLE. |
| **Status bar** | Updated every 1 s via GET /api/status: current state, odometry (x,y,heading), **tracker FPS**, **stream FPS**, **app version**, **CPU %**, **RAM %**, **CPU temp**. |
| **Resolution selector** | Dropdown: 640x480 / 320x240 / 160x120. Changes stream resolution on the fly. |

#### Calibration tab

- Form to view/edit speed-time-distance, steering limits, bbox-distance.
- GET /api/calibration to read; POST /api/calibration to write (with validation).
- Save triggers reload so motion/vision use new values immediately.
- `Calibration.to_dict()` and `Calibration.update_from_dict(data)` added when we build this tab.

#### API endpoints (summary)

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Main tab HTML |
| `/calibration` | GET | Calibration tab HTML |
| `/stream` | GET | MJPEG stream (multipart) |
| `/api/target` | POST | Send cat_location(x, y) in meters |
| `/api/stop` | POST | Send stop command |
| `/api/status` | GET | JSON: state, odometry, tracker FPS, stream FPS, app version, CPU %, RAM %, CPU temp |
| `/api/calibration` | GET | JSON: current calibration values |
| `/api/calibration` | POST | JSON: update calibration values |
| `/api/stream/resolution` | POST | Change stream resolution (640x480 / 320x240 / 160x120) |

#### Thread safety

- `commands.py` uses `threading.Lock` (Option B) to protect globals. Flask thread writes; main loop reads. No torn reads.

#### Version

- `__version__` defined in `cat_follow/__init__.py` so the Web UI can read it at runtime and display it in the status bar.

---

## 5. Design steps broken into small, testable steps

### 5.1 Phase 0 - Setup and state machine (already done)

| Step | What to do | How to test |
|------|------------|-------------|
| 0.1 | `cat_follow/`, states and events (no transitions) | Import; print state. |
| 0.2 | Transition table; `dispatch(event, payload)` | Unit test: event sequence -> assert state. |
| 0.3 | Stub motion: stop, go_to_xy, search_arc, approach/track_bbox | Call each; assert last command or log. |
| 0.4 | Stub commands: get_cat_location, get_stop_command (e.g. file) | Set (x,y) in file; assert state machine receives event. |

### 5.2 Phase 1 - Memory and shared state (Steps 1-3 done, Step 4 next)

| Step | What to do | How to test | Status |
|------|------------|-------------|--------|
| 1.1 | **Memory pool:** `allocate_pool()` | Unit test: shapes, sizes, no realloc. | DONE |
| 1.2 | **SharedState:** wrap pool with locks; get/set | Unit test: get/set; concurrent no corruption. | DONE |
| 1.3 | **Thread stubs:** camera/tracker/detector loops | Integration: start threads; assert shared state written. | DONE |
| 1.4 | **Main loop** uses SharedState | Integration: inject cat_location; assert transitions. | NEXT |

### 5.3 Phase 2 - Idle and odometry

| Step | What to do | How to test |
|------|------------|-------------|
| 2.1 | Odometry: init (0,0), update from speed + dir_angle | Mock motor; assert (x,y) change. |
| 2.2 | IDLE: only stop(); no drive | Start in IDLE; assert motion is stop only. |
| 2.3 | Real picar-x in IDLE | On Pi: car still; state IDLE. |

### 5.4 Phase 3 - GOTO target

| Step | What to do | How to test |
|------|------------|-------------|
| 3.1 | Command interface: on (x,y) push cat_location_received | Simulate (x,y); assert state -> GOTO_TARGET. |
| 3.2 | motion/goto_xy: bearing to (x,y), steer, drive until at target | Unit test with mocked odometry. On Pi: drive and stop near target. |
| 3.3 | GOTO_TARGET: call go_to_xy each tick; on at_target -> SEARCH | On Pi: send (x,y); car drives; state -> SEARCH when close. |

### 5.5 Phase 4 - Search

| Step | What to do | How to test |
|------|------------|-------------|
| 4.1 | Camera + detector: 640x480, every 5th frame COCO, filter cat | On Pi: point at cat; assert bbox when visible. |
| 4.2 | motion/search: search_arc() using steering limits | On Pi: car does arc. |
| 4.3 | SEARCH: search_arc + get_cat_bbox; on first bbox -> APPROACH | On Pi: no cat -> arc; cat in view -> APPROACH. |

### 5.6 Phase 5 - Approach to 15 cm

| Step | What to do | How to test |
|------|------------|-------------|
| 5.1 | Calibration: bbox area/width vs distance; estimate_distance_cm | Offline: saved images, known distance; assert estimate. |
| 5.2 | Motion: center_cat + approach (30 FPS bbox from tracker) | On Pi: cat in frame; approach; stop when ~15 cm. |
| 5.3 | APPROACH: distance<=15 cm -> TRACK; bbox None for N frames -> LOST_SEARCH | On Pi: approach -> TRACK or LOST_SEARCH. |

### 5.7 Phase 6 - Track at 30 FPS

| Step | What to do | How to test |
|------|------------|-------------|
| 6.1 | 30 FPS loop: tracker.update(), every K detector + re-init; measure FPS | Run; assert ~30 FPS. |
| 6.2 | track_bbox: same as approach (center + ~15 cm) | Moving cat; car follows. |
| 6.3 | TRACK: N frames no bbox -> cat_lost -> LOST_SEARCH | Move cat; then hide -> LOST_SEARCH. |

### 5.8 Phase 7 - Lost and stop

| Step | What to do | How to test |
|------|------------|-------------|
| 7.1 | LOST_SEARCH: search_arc + detector; cat_found -> APPROACH | Hide cat then show; car re-acquires. |
| 7.2 | stop_command from any state -> IDLE + stop | From each state send stop; assert IDLE. |

### 5.9 Phase 8 - Real camera/tracker/detector + full pipeline

| Step | What to do | How to test |
|------|------------|-------------|
| 8.1 | Camera thread: real capture into frame_latest; no per-frame alloc | Mock: assert pattern. On Pi: non-zero frames. |
| 8.2 | Tracker thread: OpenCV KCF; re-init from bbox_detector | Synthetic frame + bbox; assert bbox follows. |
| 8.3 | Detector thread: every K, TFLite; pre-alloc input buffer | Stub: assert bbox_detector. TFLite: cat image -> plausible bbox. |
| 8.4 | Full pipeline: all threads + main loop; optional logging | Smoke: run main_loop; state transitions; no crash. |

### 5.10 Phase 9 - Web UI

| Step | What to do | How to test |
|------|------------|-------------|
| 9.1 | **Flask app + main tab HTML** (Alpine.js): stream `<img>`, target fields (X/Y meters), Send + Stop buttons, status bar, resolution dropdown. | Browser: open main tab; see layout. |
| 9.2 | **MJPEG stream** at 10 FPS: read frame + bbox under lock; draw rectangle + state text; resize to selected resolution; encode JPEG; yield. Stub for H.264. | Browser: see live stream; with stub bbox see rectangle. |
| 9.3 | **POST /api/target:** receive {x,y} meters; call set_cat_location under lock. | POST with curl or test; assert command queue receives. |
| 9.4 | **POST /api/stop:** call set_stop_command under lock. | POST; assert state -> IDLE. |
| 9.5 | **GET /api/status:** return state, odometry, tracker FPS, stream FPS, app version, CPU %, RAM %, CPU temp. | GET; assert JSON with all fields. |
| 9.6 | **POST /api/stream/resolution:** change stream resolution (640x480 / 320x240 / 160x120). | Change; assert stream uses new size. |
| 9.7 | **Calibration tab HTML** + GET/POST /api/calibration. | Open tab; edit; save; assert values changed. |
| 9.8 | **Stream overlay:** Rectangle follows cat until Stop; state text on frame. | Run APPROACH/TRACK; stream shows rectangle; press Stop; behavior stops. |

### 5.11 Test files (implementation plan)

| Step | Test file | Verifies |
|------|-----------|----------|
| 1.1 | test_memory_pool.py | Single alloc; shapes/sizes; same buffers on reuse |
| 1.2 | test_shared_state.py | Get/set; concurrent read/write no corruption |
| 1.3 | test_thread_stubs.py | Stub threads write shared state; main reads |
| 1.4 | test_main_loop_shared_state.py | Main uses shared bbox; state machine + motion |
| 8.1 | test_camera_thread.py | Camera fills frame_latest (mock pattern) |
| 8.2 | test_tracker_thread.py | Tracker bbox from frame (synthetic/static) |
| 8.3 | test_detector_thread.py | Detector writes bbox_detector (stub/TFLite) |
| 8.4 | test_full_pipeline.py or manual | Full run; state flow; no crash |
| 9.1 | test_web_ui_stream.py | Stream endpoint returns frames; overlay when bbox valid |
| 9.3-9.4 | test_web_ui_commands.py | POST target/stop triggers commands |
| 9.5 | test_web_ui_status.py | GET /api/status returns all fields |
| 9.7 | test_web_ui_calibration.py | GET/POST calibration returns and updates values |

---

## 6. Document index

| Document | Content |
|---------|---------|
| DESIGN_CAT_FOLLOW_STATE_MACHINE.md | High-level design, state machine, events, baby-step phases 0-8 |
| DESIGN_CAT_FOLLOW_CLARIFICATIONS_AND_FILE_PLAN.md | Camera straight, car centers cat, calibration, file plan, module roles |
| DESIGN_SOFTWARE_ARCHITECTURE_MEMORY_AND_PARALLELISM.md | Pre-alloc, shared memory, thread/process plan, sync, file layout |
| IMPLEMENTATION_STEPS_AND_TESTING.md | Coding steps 1-8 for memory/threads/main loop; test per step |
| CAT_DETECTION_RECOGNITION_TRACKING_RPI4.md | Detection/recognition/tracking options on Pi 4; vilib COCO, KCF hybrid |
| POSITION_TRACKING_OPTIONS.md | (0,0)->(x,y): time+speed+steer, encoders, IMU, visual odometry |
| ROBOT_HAT_PINS_AND_IMU.md | HAT pins, free I2C/UART/SPI, IMU choices (BNO055, BNO085, etc.) |
| CODEBASE_SUMMARY.md | car-x repo layout; picar-x, vilib, robot-hat |
| cat_follow/README.md | cat_follow layout, run stub, tests, next steps |

---

*This summary is the single reference for project goals, hardware, architecture, high-level design, Web UI, and testable implementation steps.*
