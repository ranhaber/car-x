# Design: Ultrasonic as source of truth for distance

> **Legacy/prototype companion document.** The production hardware notes in
> this file remain useful, but the historical single-source distance behavior
> is not the canonical target safety contract. The target requires both lidar
> and ultrasonic, with a configurable 15 cm `BRAKE_REVERSE` trigger; see
> `cat_follow/docs/Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`.

> **ROCK 4D production:** The contract runtime (`runtime.app --with-prototype-perception`)
> uses **`cat_follow/perception/edge_ultrasonic.py`** with libgpiod v1 both-edge
> events, not the legacy `robot_hat.Ultrasonic` polling path described below.
> See `cat_follow/docs/Software_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md`
> §4.7 and `cat_follow/README.md` (Ultrasonic subsection).

Use the PiCar-X ultrasonic sensor as the source for "how far is the object in front" (e.g. the cat). No bbox-based distance fallback.

---

## 1. Current production stack (ROCK 4D)

```text
EdgeTimedUltrasonic (CatFollow-UltrasonicIRQ)
  TRIG gpiochip2:16, ECHO gpiochip1:21
  libgpiod v1 LINE_REQ_EV_BOTH_EDGES + kernel event timestamps
  CPU affinity core 3; best-effort SCHED_FIFO priority 70
  └─ latest_distance_cm()  [nonblocking cache]
  └─ range_sensor.set_reader(...)
  └─ range_sensor.get_distance_cm()  [60 ms throttle]
  └─ RangeAdapter (CatFollow-RangeAdapter, ~20 Hz)
  └─ SharedState.range → DecisionEngine

Picarx(enable_ultrasonic=False)  # motors/servos only; no D2/D3 claim
```

Env vars: `CAT_FOLLOW_ULTRASONIC_*` in `scripts/car-x.env`. Production uses
`CAT_FOLLOW_ULTRASONIC_REQUIRE_REALTIME=0` when the kernel denies `SCHED_FIFO`.

**Legacy path:** `main_loop.py` still uses `range_sensor.set_car(Picarx)` and
`Picarx.get_distance()` → `robot_hat.Ultrasonic.read()` (GPIO polling). This path
has not been migrated.

Tests: `tests/test_edge_ultrasonic.py`, `tests/test_range_sensor.py`.

---

## 2. Original design (historical — pre edge worker)

### Hardware and pins
- **PiCar-X** uses **robot_hat** with ultrasonic on **D2 (trigger)** and **D3 (echo)**.
- **picar-x/picarx/picarx.py**: `Picarx(ultrasonic_pins=['D2','D3'])` creates `self.ultrasonic = Ultrasonic(Pin(trig), Pin(echo))` when `enable_ultrasonic=True` (default).
- **Picarx.get_distance()** → returns `self.ultrasonic.read()` (distance in **cm**), or `-1` when ultrasonic disabled.
- **robot_hat/modules.py**: `Ultrasonic.read(times=10)` returns cm (or **-1** on timeout/error) via GPIO busy-wait on ECHO.

### cat_follow abstraction: range_sensor
- **cat_follow/range_sensor.py**
  - **set_car(car)** — legacy: call `car.get_distance()` when interval elapsed.
  - **set_reader(callable)** — production: nonblocking provider (e.g. `EdgeTimedUltrasonic.latest_distance_cm`).
  - **Read interval:** at most every **60 ms** (`MIN_READ_INTERVAL_SEC`); cached value within interval.
  - **get_distance_cm() → float | None** — valid range 1–500 cm; invalid/timeout → `None`.
  - **get_last_distance_cm()** — display-only cache; no hardware read.
  - No bbox-based fallback: distance is ultrasonic only.

### Wiring at startup
- **Contract runtime:** `EdgeTimedUltrasonic.from_env()` + `range_sensor.set_reader()`; `Picarx(enable_ultrasonic=False)`.
- **Legacy main_loop:** `range_sensor.set_car(px)` when Picarx is created.

---

## 3. Notes
- **What ultrasonic measures:** Distance to the **nearest object in front** of the sensor (cone). Not "distance to cat" unless the cat is centered in front.
- **Invalid reads:** Timeouts and out-of-range values become `None`; `DecisionEngine` fails closed when no fresh valid obstacle sensor is available.
- **Stub / no HW:** When neither reader nor car is set, `get_distance_cm()` returns `None`.
