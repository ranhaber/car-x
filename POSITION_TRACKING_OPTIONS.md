# PiCar-X Position Tracking: (0,0) → (x,y)

This document summarizes ways to let the car know its location relative to a “last stop” origin **(0,0)** and suggest implementation options for the [PiCar-X](https://docs.sunfounder.com/projects/picar-x-v20/en/latest/).

---

## 1. Goal

- Define an origin **(0, 0)** at “last stop” (e.g. when the program starts or when the user resets).
- Maintain an estimated position **(x, y)** as the car moves.
- Use only onboard capabilities where possible; suggest hardware additions where they help.

---

## 2. What the PiCar-X Already Has (from docs and code)

From the [SunFounder PiCar-X docs](https://docs.sunfounder.com/projects/picar-x-v20/en/latest/):

- **Robot HAT** — MCU, PWM, ADC, motor driver, I2S/speaker. No wheel encoders or IMU in the standard kit.
- **Camera** — 2-axis camera module (pan/tilt).
- **Ultrasonic** — Distance to obstacles (not global position).
- **Line tracking** — 3-channel grayscale (line/cliff, not position).

From the **car-x** codebase:

- **Movement API:** `forward(speed)`, `backward(speed)`, `stop()`, `set_dir_servo_angle(angle)`.
- **Steering:** Front steering servo; `dir_current_angle` is kept in `Picarx` (e.g. ±30°).
- **No feedback:** Motors are PWM-only; there are **no wheel encoders** and **no IMU** in the repo. So we have *commanded* speed and steering, not measured distance or heading.

So: position must be **estimated** (e.g. dead reckoning) or obtained from **extra hardware** or **vision**.

---

## 3. Methods to Get Position (from web and literature)

### Dead reckoning (no GPS)

- Estimate position by integrating motion from the start point using **velocity** and **heading** over time.
- **With wheel encoders:** measure distance and rotation from wheel ticks → accurate odometry (e.g. ~0.2% error with calibration).
- **Without encoders:** use **commanded** speed and steering (and optionally IMU) and integrate; errors grow due to slip, uneven speed, and model inaccuracies.
- **IMU (gyro + accelerometer):** improves heading and can help velocity; typically ~2% error; fusion with odometry (e.g. Kalman filter) gives better results than either alone.
- **Limitations:** Slippage, uneven floors, and calibration errors cause drift. Without encoders, drift is larger; periodic correction (e.g. visual or known landmarks) helps for longer runs.

References (concepts): [TM129 Robotics – Dead reckoning](https://innovationoutside.github.io/tm129-robotics2020/04.%20Not%20quite%20intelligent%20robots/04.2%20Robot%20navigation%20using%20dead%20reckoning.html), differential steering / trajectory models (e.g. [Rossum’s differential steering](https://rossum.sourceforge.net/papers/DiffSteer/)).

---

## 4. Implementation Options for PiCar-X

### Option A — Time + commanded speed + steering (no new hardware)

**Idea:** Treat the car as moving at a constant commanded speed and steering angle over short time steps; integrate to get (x, y) and heading θ.

**Available in code:**

- `px.dir_current_angle` — steering angle (e.g. -30 to +30°).
- `px.forward(speed)` / `px.backward(speed)` — commanded speed (e.g. 0–100).
- You know *when* you start/stop and *what* you commanded.

**Approach:**

1. **Origin:** Set (x, y) = (0, 0) and θ = 0 at “last stop”.
2. **Time step:** Every Δt (e.g. 0.05–0.1 s), read:
   - Commanded speed (and sign: forward/backward).
   - Current steering angle `dir_current_angle`.
3. **Model:** Convert speed to an approximate linear velocity *v* (e.g. map 0–100 to m/s with a calibrated constant). Use steering angle to get turn rate (e.g. simple Ackermann-like or “bicycle” model), or approximate:  
   - `distance = v * Δt`  
   - `Δθ = f(steering_angle, v, Δt)`  
   then:  
   - `x += distance * cos(θ)`  
   - `y += distance * sin(θ)`  
   - `θ += Δθ`
4. **Reset:** When the car “stops” (e.g. `stop()` called), you can either keep (x, y) as “last stop” or call “set origin here” to reset (0,0) to current estimate.

**Pros:** No hardware changes; works with current picar-x API.  
**Cons:** Drift (slippage, speed not exact, steering not exact); good for short runs and relative “where am I from last stop”.

---

### Option B — Add wheel encoders (best accuracy)

- Attach encoders to the rear (driven) wheels and measure rotation → distance per wheel → distance and heading change.
- Use differential-drive (or Ackermann-adapted) odometry to update (x, y, θ).
- PiCar-X does **not** ship with encoders; you’d need to add them (e.g. magnetic or optical) and read via GPIO or Robot HAT.
- Gives the most accurate and reliable (x,y) from (0,0) for indoor use without external beacons.

---

### Option C — Add an IMU (gyro + accelerometer)

- Use a gyro for heading rate; optionally fuse with accelerometer (and optionally with Option A) in a small filter (e.g. complementary or Kalman).
- Improves θ and can reduce drift when combined with time+speed (Option A).
- Requires I2C/SPI IMU (e.g. MPU6050, BNO055) and wiring to the Pi or Robot HAT.

---

### Option D — Visual odometry (camera)

- Use the existing camera and vilib/OpenCV to estimate motion between frames (feature flow or visual odometry).
- No extra hardware, but CPU-heavy and sensitive to lighting/texture; best combined with other methods.
- More complex to implement and tune.

---

### Option E — Software “origin at last stop”

- Regardless of how (x, y) is computed, implement a clear **“set origin here”** action:
  - On start: (x, y) = (0, 0), θ = 0.
  - On user command or “I’ve stopped”: set `(x_origin, y_origin) = (x, y)` and from then on report position as `(x - x_origin, y - y_origin)` so the car “knows” its location relative to last stop.
- This satisfies “from the last stop (0,0) → (x,y)” in the software sense once you have any position estimate.

---

## 5. Suggested path for your project

1. **Implement Option A** in the existing codebase:
   - Add a small **odometry** helper that:
     - Starts at (0, 0) and θ = 0.
     - Is updated at a fixed rate (e.g. 10–20 Hz) with `dir_current_angle` and last commanded speed/direction.
     - Exposes `get_position() -> (x, y)` and `get_heading() -> θ`, and `set_origin_here()` to reset (0,0) to current estimate.
   - Calibrate: measure real distance vs. time at a few speeds to set the speed→velocity mapping.
2. **Add Option E** so “last stop” is always (0,0) in your app logic.
3. **Later:** If you need better accuracy, add encoders (Option B) or an IMU (Option C) and fuse with the same (x, y, θ) state.

---

## 6. References

- [SunFounder PiCar-X Kit (English)](https://docs.sunfounder.com/projects/picar-x-v20/en/latest/)
- [PiCar-X Hardware](https://docs.sunfounder.com/projects/picar-x-v20/en/latest/hardware/cpn_hardware.html) — Robot HAT, Camera, Ultrasonic
- [TM129 Robotics: Dead reckoning](https://innovationoutside.github.io/tm129-robotics2020/04.%20Not%20quite%20intelligent%20robots/04.2%20Robot%20navigation%20using%20dead%20reckoning.html)
- Differential steering / trajectory: [Rossum – DiffSteer](https://rossum.sourceforge.net/papers/DiffSteer/)

---

*Summary: The car can “know” its location from the last stop (0,0) as (x,y) by **time + commanded speed + steering** with no new hardware; for better accuracy, add wheel encoders or an IMU and fuse with the same (x,y) state. The docs do not describe built-in encoders or IMU; this proposal fits the current PiCar-X hardware and your car-x codebase.*
