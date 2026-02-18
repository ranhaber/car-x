# Design: Ultrasonic as source of truth for distance

Use the PiCar-X ultrasonic sensor as the only source for "how far is the object in front" (e.g. the cat). No bbox-based distance fallback.

---

## 1. What the project already has

### Hardware and pins
- **PiCar-X** uses **robot_hat** with ultrasonic on **D2 (trigger)** and **D3 (echo)**.
- **picar-x/picarx/picarx.py**: `Picarx(ultrasonic_pins=['D2','D3'])` creates `self.ultrasonic = Ultrasonic(Pin(trig), Pin(echo))`.
- **Picarx.get_distance()** → returns `self.ultrasonic.read()` (distance in **cm**).
- **robot_hat/modules.py**: `Ultrasonic.read(times=10)` returns cm (or **-1** on timeout/error).

### Current use in cat_follow
- **motion/driver.py** has `set_car(car)` to inject a Picarx instance; used for motors and steering. **Distance is not exposed** in cat_follow yet.
- **Distance today**: Only from **bbox area** via `calibration.get_distance_cm_from_bbox_area(area)` in `center_cat` and main_loop (APPROACH→TRACK).

---

## 2. Design: add range_sensor, use ultrasonic when available

### 2.1 Abstraction: range_sensor
- **cat_follow/range_sensor.py**
  - **set_car(car)** – same as driver: when running on Pi with Picarx, main_loop calls `set_car(px)` so range_sensor can call `car.get_distance()`.
  - **Read interval:** Hardware is read at most every **60 ms** (MIN_READ_INTERVAL_SEC). Within that interval we return the last cached value. This avoids pinging the HC-SR04 too fast (many modules need ~60 ms between readings).
  - **get_distance_cm() → float | None**
    - If car is set: call `car.get_distance()` when the interval has elapsed; otherwise return cached value. If result is valid (e.g. `1 ≤ d ≤ 500`), return `d`; else return `None`.
    - If car is not set (stub): return `None`.
  - **get_last_distance_cm()** – returns last valid distance (for Web UI display) without triggering a read.
  - No bbox-based fallback: distance is ultrasonic only.

### 2.2 Where to use ultrasonic
- **Read interval:** Ultrasonic is read in the main loop in **all phases except IDLE** (so GOTO_TARGET, SEARCH, LOST_SEARCH, APPROACH, TRACK). In IDLE we do not read.
- **Obstacle avoid:** When state ≠ IDLE and `ultrasonic_cm < target_cm` (15 cm), the car **stops normal motion and runs the search arc** (alternate left/right steer at low speed) until distance ≥ 15 cm again. This “arc around” avoids driving into something close in front.
- **center_cat_control** (motion/center_cat.py)
  - Use only `range_sensor.get_distance_cm()` for forward/back/stop. If `None`, steer only and stop (no forward/back).
- **main_loop (APPROACH → TRACK)**
  - Dispatch `DISTANCE_AT_15CM` only when ultrasonic is not `None` and `<= target_cm + 5`. No bbox-based fallback.

### 2.3 Wiring at startup
- In **main_loop** (or wherever Picarx is created): after `set_car(px)` (if any), call **range_sensor.set_car(px)** so the same car instance is used for motors and ultrasonic. If no car (stub), both driver and range_sensor stay no-op / return None.

---

## 3. Implementation steps

| Step | What | Where |
|------|------|------|
| 1 | Add **range_sensor.py** with `set_car`, `get_distance_cm()`; valid range e.g. 1–500 cm. | cat_follow/range_sensor.py |
| 2 | In **center_cat**: if `range_sensor.get_distance_cm()` is not None, use it as dist_cm; else use bbox-area calibration. | cat_follow/motion/center_cat.py |
| 3 | In **main_loop**: in APPROACH, if ultrasonic distance is not None and ≤ target_cm+5, dispatch DISTANCE_AT_15CM; else keep bbox-area condition. | cat_follow/main_loop.py |
| 4 | In **main_loop** (or entrypoint): call `range_sensor.set_car(px)` when `set_car(px)` is called. | cat_follow/main_loop.py |

No change to calibration JSON or to Picarx/robot_hat; ultrasonic is read at runtime. Optional: expose last ultrasonic reading in `/api/status` for debugging.

---

## 4. Notes
- **What ultrasonic measures:** Distance to the **nearest object in front** of the sensor (cone). If the cat is centered and close, that is usually the cat; if the cat is off to the side, the sensor might see the floor or something else. So ultrasonic is “distance ahead,” not “distance to cat” unless the cat is in front. Still, for APPROACH/TRACK when we’re centering the cat, it’s a good primary source.
- **Invalid reads:** robot_hat returns -1 on timeout; we treat that as “no valid distance” and no forward/back; APPROACH won't transition to TRACK.
- **Stub / no HW:** When `set_car` is never called (e.g. development), `get_distance_cm()` always returns `None` and center_cat only steers and stops; APPROACH never transitions to TRACK via distance.
