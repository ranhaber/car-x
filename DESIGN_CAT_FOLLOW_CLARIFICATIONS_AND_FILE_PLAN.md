# Cat-Follow Design: Clarifications, Calibration, and File Plan

This document refines the cat-follow design with your clarifications, adds **calibration** (speed–time–distance, max U-turn radius), and defines a **modular file plan**.

---

## 1. Clarifications (Camera Straight, Car Centers Cat)

### 1.1 Camera Stays Straight

- The **camera is fixed** (no pan-tilt to follow the cat). It points **straight ahead** relative to the car.
- All “center the cat in the frame” behavior is done by **moving the car** (steering + forward/back), not by moving the camera.

### 1.2 After Finding the Cat: Car Moves Forward and Centers Cat in Frame

- Once the cat is found (APPROACH), the car:
  - **Maneuvers** (steers left/right and drives forward/back) so that the **cat stays in the middle of the frame**.
  - Keeps **moving forward** (when safe) toward the cat until ~15 cm.
- The **camera stays straight**; the **car** turns to keep the cat centered.

### 1.3 If the Cat Moves: Car Moves With It, Cat in Middle

- In TRACK (and APPROACH), if the cat moves:
  - The **car moves with it** (steer + drive) so that the cat stays in the **middle of the frame**.
  - Control loop: **bbox center in image** → **car steering and speed** so that bbox center → image center.

### 1.4 Always Keep Cat in Middle of Frame

- In both **APPROACH** and **TRACK**, the car **always** tries to keep the cat in the **middle of the frame**:
  - **Lateral:** bbox center X vs image center X → steering (left/right).
  - **Distance:** bbox size or separate estimate → forward/back to maintain ~15 cm in TRACK, or to approach in APPROACH.

So the motion controller input is always **(bbox center in image, bbox size or distance)** and the output is **(steer angle, forward/back speed)** with **camera fixed**.

---

## 2. Control Loop (Car Centers Cat, Camera Straight)

```
  Frame → Detector/Tracker → bbox (x, y, w, h)
                                    │
  Image center (cx_img, cy_img)      │   bbox center (cx_cat, cy_cat)
  Frame size (W, H)                  │   bbox area or width
                                    ▼
  ┌─────────────────────────────────────────────────────────────┐
  │  Centering controller                                        │
  │  - Error X = cx_cat - cx_img  →  steering (left/right)       │
  │  - Error Y or bbox size       →  forward/back (approach 15cm)  │
  └─────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
  picar-x: set_dir_servo_angle(steer), forward(speed) or backward(speed)
```

- **No camera pan-tilt** in this loop; only **car** motion.
- Calibration (below) maps **speed vs time → distance** and limits **steering** (max U-turn radius).

---

## 3. Calibration (Modular)

The system needs **reproducible motion** and **safe limits**. All of this is isolated in a **calibration** module and data so the rest of the code stays modular.

### 3.1 Speed–Time–Distance

- **Goal:** Know “if I drive forward at speed S for time T, I travel distance D.”
- **Measure:** Run the car at fixed speed (e.g. 30, 50, 70) for 1 s on a flat surface; measure D (tape or odometry). Repeat a few times.
- **Storage:** Table or curve: `(speed, time) → distance` (or per-speed cm/s).
- **Use:** GOTO_TARGET and APPROACH/TRACK use this to estimate “how long to drive” for a given distance, and to convert odometry time-steps into distance.

**Suggested format (e.g. JSON or Python dict):**

```python
# calibration/speed_time_distance.json (concept)
{
  "speed_to_cm_per_sec": {
    "30": 12.0,
    "50": 22.0,
    "70": 32.0
  },
  "notes": "Measured on flat floor; speed = picar-x 0-100"
}
```

### 3.2 Max U-Turn Radius (Min Turning Radius)

- **Goal:** Limit steering so the car never turns sharper than a safe “max U-turn” (min turning radius). Avoids flipping or unrealistic turns.
- **Measure:** Set steering to max left/right; drive; measure radius (or diameter) of the circle. That is “min turn radius” or “max curvature.”
- **Storage:** e.g. `max_steer_angle_deg` (e.g. ±25° instead of ±30°) or `min_turn_radius_cm`.
- **Use:** When converting “center cat in frame” to steering, **clamp** steering to this limit.

**Suggested format:**

```python
# calibration/steering_limits.json (concept)
{
  "max_steer_angle_deg": 25,
  "min_turn_radius_cm": 40,
  "notes": "Do not exceed to avoid tip-over"
}
```

### 3.3 Other Calibrations (Optional but Useful)

| Calibration | Purpose |
|-------------|---------|
| **Distance vs bbox size** | Map bbox area (or width) to distance to cat (cm). Used for “15 cm” and TRACK distance. |
| **Odometry scale** | Scale factor from (time × speed) to cm (e.g. per motor). |
| **Dead zone (centering)** | Ignore small bbox-center errors so the car doesn’t jitter. |

All of these can live under a single **calibration** module and be loaded from files so the rest of the system stays modular and testable.

---

## 4. Modular Architecture (High Level)

- **State machine** – Only states and transitions; no hardware.
- **Motion** – Uses picar-x + calibration; implements “go to (x,y)”, “center bbox (steer + drive)”, “search arc”, “stop”.
- **Vision** – Detector + tracker; returns bbox; no knowledge of states or motion.
- **Odometry** – Position (x,y) and heading; no knowledge of vision or states.
- **Calibration** – Load and expose speed–time–distance, steering limits, bbox–distance, etc.; used by motion and odometry.
- **Commands** – External input (cat location, stop); pushes events; no knowledge of motion or vision.
- **Main loop** – Ticks state machine, reads vision/odometry/commands, calls motion; thin glue only.

---

## 5. File Plan (Modular)

```
car-x/
├── cat_follow/                    # Cat-follow feature (modular)
│   ├── __init__.py
│   │
│   ├── state_machine.py           # States, events, transition table only
│   ├── commands.py                # External input: cat_location (x,y), stop
│   │
│   ├── motion/                    # Motion (car) – uses picar-x + calibration
│   │   ├── __init__.py
│   │   ├── driver.py              # Thin wrapper: stop(), forward(), set_steer(), etc.
│   │   ├── goto_xy.py             # Go to target (x,y) using odometry + calibration
│   │   ├── center_cat.py          # Steer + drive to keep bbox center at image center
│   │   ├── search.py              # Search pattern (e.g. arc) – uses steering limits
│   │   └── limits.py              # Apply calibration limits (steer clamp, speed)
│   │
│   ├── vision/                    # Vision – detector + tracker only
│   │   ├── __init__.py
│   │   ├── detector.py            # get_cat_bbox() from vilib/TFLite COCO
│   │   ├── tracker.py             # 30 FPS: tracker.update() + detector every K frames
│   │   └── distance.py            # estimate_distance_cm(bbox) from calibration
│   │
│   ├── odometry.py                # (x,y), heading from time + speed + steering
│   ├── calibration/               # All calibration data and loaders
│   │   ├── __init__.py
│   │   ├── loader.py              # Load JSON/config; expose dict or object
│   │   ├── speed_time_distance.json
│   │   ├── steering_limits.json
│   │   └── bbox_distance.json     # Optional: bbox size → distance
│   │
│   └── main_loop.py               # Tick: commands → state_machine → motion; 30 FPS where needed
│
├── tests/                         # Tests (per module)
│   ├── test_state_machine.py
│   ├── test_odometry.py
│   ├── test_calibration.py
│   ├── test_motion_center_cat.py  # Mock bbox → assert steer/speed
│   └── ...
│
├── picar-x/                       # Existing
├── vilib/                         # Existing
├── robot-hat/                     # Existing
└── ...
```

---

## 6. Module Responsibilities (Summary)

| Module | Responsibility | Depends on |
|--------|----------------|------------|
| **state_machine** | States (IDLE, GOTO_TARGET, SEARCH, APPROACH, TRACK, LOST_SEARCH); events; transition table; `dispatch(event, payload)` returns new state. | Nothing (pure). |
| **commands** | Read cat_location (x,y) and stop_command (file/socket/MQTT); emit events. | Nothing. |
| **motion.driver** | Wrap picar-x: stop(), forward(speed), backward(speed), set_steer(angle). | picar-x, calibration (limits). |
| **motion.goto_xy** | Compute bearing and distance to (x,y); call driver + odometry; stop when at target. | odometry, calibration (speed–time–distance), motion.driver. |
| **motion.center_cat** | Given bbox + image size, compute steer and forward/back to put cat center in image center; respect 15 cm in TRACK. | calibration (steering limits, speed, bbox–distance), motion.driver. |
| **motion.search** | Run search pattern (arc) using steering limits; no vision. | motion.driver, calibration. |
| **motion.limits** | Clamp steer to max U-turn, speed to max, etc. | calibration. |
| **vision.detector** | Run TFLite/COCO; return best “cat” bbox or None. | vilib or picar-x camera. |
| **vision.tracker** | 30 FPS: frame in → tracker.update(); every K frames run detector and re-init; return bbox or None. | vision.detector. |
| **vision.distance** | estimate_distance_cm(bbox) using calibration. | calibration (bbox_distance). |
| **odometry** | Update (x,y), heading from time, speed, steer; expose get_position(), get_heading(). | calibration (optional scale). |
| **calibration** | Load JSONs; expose speed→cm/s, max_steer, bbox→distance, dead zones. | Files only. |
| **main_loop** | Loop: read commands → state_machine.dispatch(); in APPROACH/TRACK get bbox from vision.tracker, call motion.center_cat; in GOTO_TARGET call motion.goto_xy; in SEARCH/LOST_SEARCH call motion.search; on stop call motion.driver.stop(). | All of the above. |

---

## 7. State Machine (Unchanged, Motion Semantics Updated)

- **APPROACH:** Car moves **forward** and **steers** so the cat stays in the **middle of the frame**; camera straight. Approach until distance ≤ 15 cm → TRACK.
- **TRACK:** Car **steers and drives** to keep the cat in the **middle of the frame** and ~15 cm away; camera straight. If cat moves, car follows.
- **SEARCH / LOST_SEARCH:** Car performs **arc** (or other pattern) within **max U-turn** limits; camera straight.

No pan-tilt in any state; all “center the cat” is done by **car motion** only.

---

## 8. Calibration in the File Plan

- **calibration/** holds:
  - **speed_time_distance.json** – speed → cm/s (or time → distance) for motion and odometry.
  - **steering_limits.json** – max steer angle, min turn radius (max U-turn radius).
  - **bbox_distance.json** (optional) – bbox size → distance (cm) for 15 cm logic.
- **calibration/loader.py** loads these and exposes a single interface (e.g. `get_speed_cm_per_sec(speed)`, `get_max_steer_angle()`, `get_distance_cm(bbox)`).
- **motion** and **odometry** depend only on this interface, so you can swap or tune calibration without touching motion logic.

---

## 9. Summary

- **Camera stays straight;** the **car** steers and drives to keep the cat in the **middle of the frame** (APPROACH and TRACK).
- **Modular layout:** state_machine, commands, motion (driver, goto_xy, center_cat, search, limits), vision (detector, tracker, distance), odometry, calibration (loader + JSONs), main_loop.
- **Calibration:** speed–time–distance, max U-turn (steering limits), and optional bbox–distance; all in **calibration/** with a single loader used by motion and odometry.
- **File plan** above is the target layout; each module can be tested in isolation (mocks for picar-x, camera, and calibration).

If you want, next step can be a minimal `state_machine.py` + `calibration/loader.py` + one motion stub (e.g. `center_cat`) so you can run a first test loop on the Pi.
