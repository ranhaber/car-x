# Validation Matrix
**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)  
**Version:** 1.0  
**Status:** Baseline Acceptance Matrix

## 1. Purpose
This document defines the minimum test matrix, pass/fail criteria, and evidence required to validate PRD v1.1 and the System Architecture baseline.

## 2. Test Conditions
- Environment: daylight only, synthetic grass, configured perimeter active.
- Hardware: PiCar-X, Radxa ROCK 4D, Robot HAT, ultrasonic (HC-SR04),
  onboard camera, overhead camera link, and Slamtec RPLIDAR C1 when available.
  _(TMF8829 dToF is on hold; C1 validation is pending.)_
- Build: release-equivalent runtime configuration.
- Safety posture: conservative.

## 3. Evidence Required Per Test
- Timestamped run log.
- State transition log.
- Key telemetry: packet age, cat detection status, veto events, motor command outputs.
- Final verdict: pass/fail with reason code.

## 4. Acceptance Criteria Summary
- Chase continues until overhead sends `stop_chase`.
- On `stop_chase`, system stops chase/tracking behavior and transitions to `IDLE`.
- On `return_home`, system transitions to `RETURN_HOME` and then `HOME` after completion.
- Control precedence is never violated: `failsafe > obstacle veto > pursuit logic`.
- Timeout actions occur within configured thresholds.
- No collision events in validation suite.

## 5. Validation Matrix

| Test ID | Scenario | Setup | Expected Behavior | Pass Criteria |
|---|---|---|---|---|
| VM-01 | Boot to standby | System powered, no chase command | Enters `HOME` then ready state without motor motion | No unexpected motion; no failsafe |
| VM-02 | Start chase path | Valid overhead stream and start command | `HOME -> CHASE_A` or `IDLE -> CHASE_A` | Correct transition and stable control loop |
| VM-03 | Stage A behavior | Cat tracked by overhead, not yet in local FOV | Uses overhead for target intent and local planner for steering | Steering comes from local planner; no direct overhead motor command |
| VM-04 | Stage A to B handover | Cat appears in local FOV | `CHASE_A -> TRACK_B` | Transition occurs within 250 ms after stable detection |
| VM-05 | Local visual centering | Cat visible off-center in frame | Vehicle steers to reduce horizontal error | Error magnitude decreases over time; no unstable oscillation |
| VM-06 | Camera-loss fallback | In `TRACK_B`, suppress local cat detection for >350 ms | Returns to `CHASE_A` | Transition occurs within timeout + 1 control cycle |
| VM-07 | Final approach and stop | `TRACK_B` with decreasing dToF distance | Enters `BRAKE`, stops at local proximity target | Stop command issued at target threshold; no forward creep after stop |
| VM-08 | Obstacle veto during chase | Place obstacle ahead while in `CHASE_A` | Obstacle logic overrides pursuit | Veto asserted; command reflects avoidance or stop |
| VM-09 | Obstacle veto during local track | Place obstacle ahead while in `TRACK_B` | Obstacle logic still overrides tracking | No collision; veto precedence preserved |
| VM-10 | Overhead stale warning | Delay overhead packets to >300 ms and <700 ms | Speed reduced to safe crawl | Speed reduction logged before failsafe |
| VM-11 | Overhead stale failsafe | Delay overhead packets to >700 ms | Enter `FAILSAFE` hard-stop | Hard-stop executed and reason logged |
| VM-12 | Dead-end recovery | Create blocked route with no progress >2 s | Recovery behavior triggered | Recovery mode entered and logged |
| VM-13 | Recovery escalation | Keep blocked condition >5 s | Escalates to `FAILSAFE` | Failsafe entered with dead-end reason |
| VM-14 | Boundary enforcement | Drive toward perimeter boundary | Boundary breach triggers safety action | Stops before/at boundary with failsafe reason |
| VM-15 | stop_chase command | Send `stop_chase` during active chase | Stops chase/tracking and transitions to `IDLE` | Transition occurs within 200 ms of command receipt |
| VM-16 | Return home command and completion | Send `return_home` with valid home coordinates | Transitions to `RETURN_HOME`, navigates home, then enters `HOME` | Reaches home tolerance zone and stops safely |
| VM-17 | go_to command and completion | Send `go_to` with valid target coordinates | Transitions to `GOTO`, navigates to target, then enters `IDLE` | Reaches target tolerance zone and stops safely |
| VM-18 | Obstacle too close | Place obstacle <10 cm from car | Enters `FAILSAFE` with `obstacle_too_close` reason | Hard-stop executed; no collision |
| VM-19 | Precedence audit | Inject simultaneous pursuit and hazard commands | Arbitration order maintained | No cycle violates precedence rule |
| VM-20 | Endurance run | 20-minute mixed scenario run | Stable operation without unsafe event | No collision, no unhandled exception, deterministic recovery |
| VM-21 | Lidar scan health | Launch `sllidar_c1.launch.py`; observe `/scan` | `/scan` publishes at ~10 Hz with valid ranges in the C1 FoV | `ros2 topic hz /scan` steady; no dropouts >1 s over 2 min; Control UI lidar field fresh when `--web-ui --ros-nav` |
| VM-22 | Lidar obstacle veto | Place obstacle <10 cm in the front sector during `CHASE_A`/`GOTO` | Lidar `RangeState` (LIDAR_C1) drives `FAILSAFE` alongside ultrasonic | `obstacle_too_close` + `lidar_obstacle` constraints logged; hard-stop; no collision; UI constraint chips show veto |
| VM-23 | Lidar-assisted navigation | `go_to` target with Nav2 running and fresh `NavigationState` | Steering follows `path_correction`; speed capped by `speed_limit` | Reaches target tolerance; `navigation` constraint logged; precedence preserved; UI shows path_correction/speed_limit |
| VM-24 | Perception headless efficiency | Stop MJPEG/H.264 stream / disconnect browser | Detection + tracking continue; model unloads after idle; CPU drops | Detection events continue with no viewer (`stream_clients=0`); idle CPU reduced vs. streaming |
| VM-25 | Sensor/nav freshness expiry | Freeze range/lidar/nav producers while retaining their last values | `DecisionEngine` ages each input from monotonic `received_ms`; stale values lose authority | No motor output based on stale data; navigation stops with an explicit constraint |
| VM-26 | Planner silence with live odometry | Stop `/cmd_vel` while `/odom` continues | Cached planner speed/steering terms clear after 500 ms | `speed_limit=0`, `path_correction=0`; no stale planner command revives |
| VM-27 | Failsafe latch | Inject detector fatal, control exception, critical overrun, or three consecutive overruns | Synchronous e-stop and `FAILSAFE`; healthy later ticks cannot re-drive | Motors remain inhibited until accepted operator `clear_failsafe` |
| VM-28 | Control-channel authentication | Configure web and UDP secrets; send missing/wrong/correct tokens | Motion routes/UDP commands reject invalid tokens and accept valid tokens; web stop/e-stop remain open | HTTP 401 or UDP drop for invalid token; no side effect; valid command succeeds |
| VM-29 | Monitoring and telemetry resilience | Fail telemetry sink transiently; age map/pose/scan; provide odom pose after map TF failure | Failed CRITICAL event is retried; stale status is reported; odom pose is not drawn over map grid | CRITICAL record persists after sink recovery; `pose_on_map=false` for odom pose |
| VM-ULTRA-1 | Edge ultrasonic worker health | Start `runtime.app --with-prototype-perception --picarx`; observe journal and `/api/status` range | `CatFollow-UltrasonicIRQ` starts; TRIG/ECHO on `gpiochip2:16` / `gpiochip1:21`; range updates without busy-wait CPU spike | Journal: `Ultrasonic edge worker ready`; no ~1-core `RangeAdapter`/polling saturation; range field fresh in status |
| VM-ULTRA-2 | Ultrasonic obstacle failsafe | Place obstacle `<10 cm` during active chase/goto | `RangeAdapter` → `DecisionEngine` FAILSAFE at configured threshold | Same pass criteria as VM-18 via new stack; hard-stop; constraint logged |
| VM-ULTRA-3 | RT scheduling degradation | `CAT_FOLLOW_ULTRASONIC_REQUIRE_REALTIME=0`; start service | Worker pins to core 3; ranging continues without `SCHED_FIFO` | Service active; affinity confirmed; valid range reads; journal may note RT skip |
| VM-ULTRA-4 | GPIO exclusivity | Start contract runtime; inspect GPIO consumers | Only `cat-follow-ultrasonic` owns D2/D3; Picarx has `enable_ultrasonic=False` | No libgpiod EBUSY; single consumer on TRIG/ECHO lines |
| VM-ULTRA-5 | Legacy path gap (documented) | Run `main_loop.py` on hardware | Still uses `range_sensor.set_car(Picarx)` polling path | Known unmigrated path; not used by production systemd unit |
| VM-ODOM-1 | LiDAR-odometry stationary sanity | Launch mapping/nav with `CAT_FOLLOW_ODOM_SOURCE=lidar`; leave the car still | Exactly one `/odom` publisher (RF2O); steady rate; near-zero drift; `odom->base_link` in TF | `ros2 topic info /odom` shows 1 publisher (`rf2o_laser_odometry`); `ros2 topic hz /odom` ~10 Hz; pose moves only a few cm over ~25 s; `tf2_echo odom base_link` updates |
| VM-ODOM-2 | LiDAR-odometry under motion | Move the whole chassis (wheels unpowered / no encoder) | `/odom` and `odom->base_link` track real translation/rotation | Pose follows physical motion; returns near start on a loop; UI pose arrow moves |
| VM-ODOM-3 | Mapping with LiDAR odometry | Teleop a full survey with `mapping.launch.py` (lidar source) | slam_toolbox builds a closed, consistent yard map | Map saved; no gross smearing; loop closes; usable for localization |
| VM-ODOM-4 | Navigation with LiDAR odometry | `go_to` with `rock4d_nav.launch.py` on the saved map | Localizes and reaches goals; precedence preserved | Reaches target tolerance; `navigation` constraint logged; lidar/ultrasonic veto still wins |
| VM-ODOM-5 | Feature-poor / dynamic stress | Run in open space, near repetitive fences, or with moving objects | Degrades gracefully; no false pose authority driving unsafe motion | Drift/failure is bounded; stale `/odom` fails closed; no collision |
| VM-ODOM-6 | Bicycle-odometry source disabled | Start with `CAT_FOLLOW_ODOM_SOURCE=bicycle` (or `--odom-source bicycle`) | Request is rejected (would publish frozen `/odom`); tolerant resolver warns and falls back to `lidar`; no bicycle `OdomPublisher` starts; direct activation raises a clear error | stderr/journal warns `bicycle ... disabled ... falling back to ... 'lidar'`; `start_bicycle_odom=False`; `OdomPublisher()` / `spin_in_thread(start_bicycle_odom=True)` raise `RuntimeError`; only RF2O owns `/odom` |
| VM-ODOM-7 | Localization requires a saved map | Start `rock4d_nav.launch.py` / `ros-nav.service` with `CAT_FOLLOW_MAP_FILE` unset or pointing at a missing map | Launch aborts with a clear error before slam_toolbox starts; mapping (`mapping.launch.py`) remains map-free | Clear message about unset `CAT_FOLLOW_MAP_FILE` or missing `<basename>.posegraph`/`.data`; localization does not start; a valid map launches normally |
| VM-ODOM-8 | Non-finite ROS input rejected | Publish `/cmd_vel_smoothed` or `/odom` with NaN/inf fields | Values fail closed: NaN is never clamped into full authority; drive terms drop and the sample is dropped; rate-limited warning logged | `speed_limit`/`path_correction` do not jump to 1.0 on NaN; navigation ages out to safe stop; journal shows rate-limited `dropping non-finite` warnings |
| VM-ODOM-9 | Empty front lidar sector fail-closed | Present a scan with no usable forward beams (all beyond `range_max` or below `range_min`) | Lidar `RangeState` reports no distance (confidence 0), never a synthesized clear range; a meaningful, rate-limited diagnostic is logged | `distance_cm=None`; DecisionEngine treats it as unavailable; journal shows rate-limited `front lidar sector unusable ... in_sector/usable/total` |

### 5.1 ROCK 4D platform bring-up evidence (2026-07-19)

| Test | Result | Evidence |
|------|--------|----------|
| I2C8 / Robot HAT MCU | Pass | `0x14` detected; non-root ADC transaction completed |
| Direct GPIO | Pass | Motor direction, MCU reset, ultrasonic trigger/echo verified; production uses libgpiod edge worker on `gpiochip2:16` / `gpiochip1:21` |
| MCU PWM | Pass | P0/P1/P2 servos and P12/P13 motors exercised |
| Motor backend | Pass | Elevated-wheel forward, reverse, stop, and emergency stop completed |
| Runtime service | Pass | Real `PiCarXBackend` service started and stopped cleanly |
| Power stability | Pass (bench) | Dual-rail test completed without ROCK reset |
| MIPI camera | Pass | Radxa Camera 4K / IMX415 detected at I2C5 `0x1a`; RKISP captured 30 frames at 30 FPS and produced a visible image |
| Perception optimization | Pass (host) | Motion-gated detector, lazy/idle-unload backend, adaptive OpenCV threads + affinity, hardware-lores motion path, event-driven HC-SR04, and safety-review regressions landed; 376 tests green |
| Edge ultrasonic (ROCK 4D) | Pass (bench, 2026-07-23) | `CatFollow-UltrasonicIRQ` active on core 3; busy-wait CPU spike eliminated; ranging via cached edge timestamps | Journal shows edge worker ready; VM-ULTRA-1..4 criteria met on deployed board |
| ROS 2 / Nav2 / C1 integration | Hardware bring-up pass; validation pending | `cat_follow_bringup` package implemented; C1 health OK and `/scan` steady at ~10 Hz on ROCK 4D; full VM-21 duration/UI checks and VM-22..VM-23 remain |
| Contract web UI monitoring | Pass (host) | `--web-ui` on `runtime.app`; `/api/status` exposes SharedSnapshot + perception diagnostics; Control page shows constraints / lidar / nav / phase; stream_clients reported for VM-24; `/api/map` canvas for occupancy + pose (needs `--ros-nav` + `/map`) |
| RPLidar C1 | Pass (bring-up) | C1 detected on `/dev/rplidar` -> `ttyUSB0`; firmware 1.02, hardware rev 18, health OK; Standard mode `/scan` measured at ~10 Hz on 2026-07-22 |
| LiDAR odometry (RF2O) | Stationary sanity pass; motion pending | 2026-07-22: `CAT_FOLLOW_ODOM_SOURCE=lidar` mapping launch shows single `/odom` publisher (`rf2o_laser_odometry`) at ~10 Hz, `odom->base_link` in TF, ~1-3 cm drift over 25 s stationary, slam_toolbox lifecycle `active`. VM-ODOM-2..5 pending drivable chassis; VM-ODOM-6 fallback ownership passed stationary |

## 6. Quantitative Thresholds
- Stage transition reaction target: <= 250 ms unless timeout-governed.
- `stop_chase` reaction target: <= 200 ms.
- Overhead stale thresholds:
  - warning at `>300 ms`
  - failsafe at `>700 ms`
- Camera-loss fallback threshold: `>350 ms`.
- Dead-end trigger: `>2.0 s`; escalation at `>5.0 s`.

## 7. Failure Classification
- **Critical fail:** collision, missing failsafe, precedence violation.
- **Major fail:** incorrect state transition, timeout action missing/late.
- **Minor fail:** non-critical oscillation, delayed but safe response.

Any critical fail blocks release.

## 8. Test Execution Order (Recommended)
1. Functional flow tests: VM-01 through VM-07.
2. Safety/timeout tests: VM-08 through VM-14.
3. Command/return tests: VM-15 through VM-17.
4. Robustness tests: VM-18 through VM-20.
5. ROS/headless integration tests: VM-21 through VM-24.
6. Safety-hardening regression tests: VM-25 through VM-29.
7. Ultrasonic edge-path tests: VM-ULTRA-1 through VM-ULTRA-5.

## 9. Exit Criteria
Validation suite is considered complete when:
- All critical tests pass.
- Zero critical fails.
- Any major/minor failures have approved disposition.
- Results are archived with logs and configuration snapshot.
