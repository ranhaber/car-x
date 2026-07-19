# Detailed Software Architecture
**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)  
**Based on:** PRD v1.1 and HLD v1.1  
**Version:** 1.0  
**Status:** Draft for Implementation Planning

## 1. Purpose
This document defines the target software architecture for the autonomous yard navigator. It uses the PRD/HLD as the source of truth and describes how the current `cat_follow` codebase should evolve toward that design.

The architecture keeps existing useful patterns from `cat_follow` and `cat_ball_tracker`, but it does not treat the current prototype structure as final.

## 2. Source-of-Truth Decisions
- Target design follows PRD/HLD, not current code-as-is.
- `DecisionEngine` runs in a dedicated control thread.
- Target FSM states replace the current prototype state names.
- Range sensing supports ultrasonic and Lidar C1 backends. Both are used in the current build: ultrasonic for near-field obstacle detection and Lidar C1 for final approach ranging. The TMF8829 dToF backend is on hold and not used in the current build.
- `CommsManager` is the production overhead ingress path.
- Existing web/API command injection remains useful for dev/test, not production chase control.
- All coordinates and distances use centimeters (`cm`) for coherence with local range sensing and final stop behavior.
- Adopt selected runtime patterns from `cat_ball_tracker`:
  - preallocated buffers
  - bounded async logging
  - optional CPU affinity behind config flags

## 3. Target Package Layout
```text
cat_follow/
  app/
    runtime.py
    lifecycle.py
  comms/
    overhead_client.py
    schemas.py
  control/
    decision_engine.py
    fsm.py
    arbitration.py
    commands.py
  navigation/
    interface.py
    local_planner.py
  perception/
    vision_tracker.py
    range_safety.py
    range_backends/
      ultrasonic.py
      lidar_c1.py
      tmf8829.py  # on hold
  motion/
    motor_interface.py
    driver.py
  memory/
    shared_state.py
    pool.py
  telemetry/
    async_logger.py
    events.py
  calibration/
    loader.py
  web_ui/
    ...
```

Existing modules may be migrated incrementally into this structure. The package names above define target ownership boundaries.

## 4. Runtime Model

### 4.1 Process Model
The car runs as a single Python process on Raspberry Pi 4B.

The process owns:
- worker threads for sensors/comms
- dedicated control thread for `DecisionEngine`
- optional web UI thread for monitoring/configuration
- async telemetry thread

The web UI must not be required for detection, tracking, or control.

### 4.2 Thread Model
| Thread | Target Name | Responsibility | Target Rate |
|---|---|---|---|
| Main | `CatFollow-Main` | startup, shutdown, signal handling | event-driven |
| Control | `CatFollow-Control` | `DecisionEngine`, FSM validation, arbitration, motor command | 50 Hz target, 20 Hz degraded minimum |
| Overhead | `CatFollow-Comms` | receive overhead packets and commands | ~10 Hz input |
| Vision | `CatFollow-Vision` | local cat detection/tracking | camera-dependent |
| Range | `CatFollow-Range` | Lidar C1 / ultrasonic polling and obstacle state | 20-30 Hz target |
| Navigation | `CatFollow-Nav` | local obstacle-aware constraints | TBD |
| Telemetry | `CatFollow-Log` | bounded async event logging | event-driven |
| Web UI | `CatFollow-Flask` | monitoring/config/dev injection only | request-driven |

## 5. Shared State Model
All worker threads publish into a synchronized `SharedState` snapshot. The control thread reads one coherent snapshot per tick.

Target state groups:
- `overhead`: cat/car global position, command, packet age
- `home`: return-home target initialized by overhead/home setup
- `vision`: cat visible/stable, offset, confidence
- `range`: distance, obstacle severity, critical veto
- `navigation`: steering constraints, no-progress/dead-end flags
- `system`: thermal, heartbeat, fault flags
- `fsm`: current state and last transition

The current `memory/shared_state.py` and `memory/pool.py` are good foundations. Continue using preallocated buffers for frames and avoid per-frame allocation in hot paths.

All coordinate fields (`x`, `y`) and distance fields use centimeters (`cm`) unless explicitly documented otherwise.

### 5.1 Coordinate Convention
All global positions use the `yard` coordinate frame.

V1 convention:
- Origin is the fixed overhead-calibrated yard origin.
- Recommended physical origin is the bottom-left yard corner from the overhead/operator view.
- Units are centimeters (`cm`).
- `+X` points right in the yard.
- `+Y` points forward in the yard.
- Heading uses radians.
- `heading = 0` points along `+X`.
- Positive heading rotates counter-clockwise toward `+Y`.
- Heading is normalized to `[-pi, pi)`.

Overhead `car.heading` is optional and non-authoritative. `Navigation` owns authoritative car-local heading (`theta`).

### 5.2 Authoritative State Ownership
Each state field has exactly one authoritative owner. Other modules may observe or cache values, but they must not overwrite ownership.

| State | Authoritative Owner | Notes |
|---|---|---|
| Car global position | `CommsManager` from overhead observations | Used for global chase and return-home context |
| Car heading / `theta` | `Navigation` / local pose estimator | Overhead may provide observations, but local navigation owns authoritative heading |
| Home position | `CommsManager` from overhead/home initialization | Return-home target; must be set before autonomous return |
| Global cat position | `CommsManager` from overhead observations | Strategic guidance only |
| Local cat tracking | `VisionTracker` | Owns visual lock, local offset, and local target identity |
| Range / obstacle severity | `RangeSafety` | Owns dToF/ultrasonic obstacle state and critical veto |
| Navigation constraints | `Navigation` | Owns local path/map constraints, not final commands |
| Current FSM state | `FSM` | Validated mode holder |
| Final motion command | `DecisionEngine` | Only source of final speed/steering/brake request |
| Hardware actuation | `MotorInterface` | Only module that writes to drivetrain hardware |

Overhead data is not authoritative for car-local heading, local steering, or obstacle decisions. It provides global positional observations only.

### 5.3 Freshness Guarantees
Every shared-state group must carry:
- `timestamp_ms`: producer timestamp when the data was generated or sampled. This is useful for cross-device log correlation and should be synchronized with NTP/Chrony where possible.
- `received_ms`: PiCar-X local monotonic timestamp when the data entered the car process. This is authoritative for freshness, timeout, and failsafe decisions.
- `fresh: bool`: computed by max-age policy.
- `authority`: string/enum naming the producer.
- `confidence`: producer-specific confidence when available.

Max usable ages:

| Data Group | Max Age | Expired Behavior |
|---|---:|---|
| `vision` | 200 ms | Local visual lock is invalid |
| `range` | 100 ms | Range cannot authorize final approach; obstacle state becomes conservative |
| `navigation` | 200 ms | Navigation constraints are stale; reduce speed or stop depending on state |
| `overhead` warning | 300 ms | Reduce speed to safe crawl |
| `overhead` expired | 700 ms | Enter `FAILSAFE` |
| `system/thermal` | 1000 ms | Treat thermal state as unknown and conservative |

Freshness is computed from PiCar-X local monotonic time:

```text
age_ms = now_monotonic_ms - received_ms
```

Freshness must not be computed from producer `timestamp_ms`, because synchronized wall-clock time can drift, jump, or be temporarily unavailable.

The control thread must not mix a fresh field with an expired field as if both describe the same moment. If required data for a state is expired, `DecisionEngine` must degrade, fallback, or stop according to the state-specific policy.

### 5.4 Control Timing Contract
The `CatFollow-Control` thread targets `50 Hz` with a `20 ms` tick budget.

Per-tick budget:
| Operation | Budget |
|---|---:|
| Snapshot acquire | `<1 ms` |
| Freshness evaluation | `<1 ms` |
| `DecisionEngine` | `<5 ms` |
| FSM validation | `<1 ms` |
| Motor command write | `<2 ms` |
| Telemetry enqueue | `<1 ms` |
| Total control tick | `<20 ms` |

Overrun behavior:
- Single tick over `20 ms`: emit `control_tick_overrun` telemetry.
- `3` consecutive overruns: apply conservative speed limiting.
- Any tick over `100 ms`: command safe stop and emit critical telemetry.
- If the loop cannot maintain at least `20 Hz`, reduce speed or stop.

Telemetry file I/O must never run inside the control tick; it belongs to the async telemetry thread.

## 6. Core Components

### 6.1 `CommsManager`
Production path for overhead camera data.

Responsibilities:
- receive overhead packets
- validate schema and timestamps
- update shared overhead state
- detect stale stream
- receive reliable commands including `start_chase`, `stop_chase`, `return_home`, and `go_to`

The current web/API command queue should become a dev/test injection path. It may reuse the internal command channel but must not be the production overhead interface.

### 6.2 `VisionTracker`
Wraps local camera cat tracking.

Responsibilities:
- run local detection/tracking independent of the web UI
- publish `cat_visible`, `cat_visible_stable`, `x_offset_norm`, and confidence
- provide camera-loss timing

Implementation should preserve the existing `camera`, `tracker`, and `detector` separation where useful. If AI inference becomes expensive, use a latest-wins frame handoff rather than a backlog.

### 6.3 `RangeSafety`
Hardware abstraction for local range and obstacle safety.

Supported backends:
- `UltrasonicRangeBackend`: PiCar-X ultrasonic range path for near-field obstacle detection
- `LidarC1RangeBackend`: Slamtec RPLIDAR C1 backend used for final approach ranging
- `TMF8829RangeBackend`: on hold; retained as a placeholder but not used in the current build

Responsibilities:
- normalize backend output into one range/obstacle model
- report `obstacle_detected`, `obstacle_critical`, severity, and confidence
- provide final approach range only when visual lock confirms target identity

### 6.4 `Navigation`
Dedicated pluggable local navigation component.

Responsibilities:
- consume target direction from `DecisionEngine`
- account for obstacles/dead ends
- output steering constraints or avoidance recommendations
- report `no_progress` and `dead_end`
- own local path/map state when the selected implementation supports mapping
- own authoritative car-local heading / `theta`

Exact SLAM/local planner implementation remains TBD.

`Navigation` does not decide final motion. It outputs constraints and recommendations only; `DecisionEngine` resolves navigation-vs-tracking conflicts and owns final speed/steering/brake authority.

### 6.5 `DecisionEngine`
Central motion decision maker.

Responsibilities:
- read synchronized shared state each control tick
- apply timeout and thermal policy
- apply safety precedence
- request FSM transitions
- compute final speed/steering/brake request
- emit decision reason codes

No perception, navigation, or web module may directly command motion.

### 6.6 `FSM`
Validated state holder.

Target states:
- `HOME`
- `IDLE`
- `CHASE_A`
- `TRACK_B`
- `BRAKE`
- `GOTO`
- `RETURN_HOME`
- `FAILSAFE`

The current prototype states should be replaced, not preserved as production states.

If the FSM rejects a requested transition, it must log the rejected transition and reason. The control tick must then hold the current safe state or command a safe stop; a rejected transition must never result in raw or stale motor output.

### 6.7 `MotorInterface`
Single hardware output boundary.

Responsibilities:
- execute validated speed/steering
- controlled stop
- emergency stop
- isolate PiCar-X driver details from control logic

## 7. Control Tick Flow
At each `CatFollow-Control` tick:
1. Read one shared-state snapshot.
2. Evaluate failsafe conditions.
3. Evaluate obstacle veto.
4. Evaluate command state (`start_chase`, `stop_chase`, `return_home`, `go_to`).
5. Compute stage-specific pursuit decision.
6. Request FSM transition.
7. Validate transition.
8. Send final command to `MotorInterface`.
9. Log decision event.

Authoritative priority:
1. Failsafe
2. Obstacle veto
3. DecisionEngine pursuit decision
4. MotorInterface execution

## 8. Control Fusion Policy
`DecisionEngine` owns all final fusion and arbitration. No lower-level module may blend commands independently.

Control fusion follows this order:
1. If any unrecoverable failsafe is active, command `emergency_stop`.
2. Else if obstacle severity is critical, apply obstacle veto (`stop_or_escape`) and ignore pursuit steering.
3. Else if in `TRACK_B`, use camera tracking as the primary pursuit term and navigation as a non-critical constraint.
4. Else if in `CHASE_A`, use overhead global target direction as pursuit intent and navigation as the steering constraint.
5. Else if in `BRAKE`, use fresh visual lock plus range state to decide stop/brake behavior.

Non-critical steering influences may be blended:

```text
steering_request =
    w_tracking * camera_follow
  + w_navigation * navigation_constraint
```

Stage-specific rules:
- In `CHASE_A`, `w_tracking = 0` because local camera has not acquired the cat.
- In `TRACK_B`, camera tracking is primary, but navigation may constrain steering to avoid obstacles.
- Critical obstacle avoidance is never weighted or blended; it is a veto.
- Overhead is never a direct steering source.

Speed is bounded by all active limits:

```text
speed_request = min(
    pursuit_speed,
    alignment_speed_limit,
    obstacle_distance_limit,
    navigation_speed_limit,
    thermal_speed_limit
)
```

The selected weights and gains are tuning parameters, but the authority order is not configurable in V1.

## 9. State Transition Rules
| From | To | Trigger |
|---|---|---|
| `HOME` | `CHASE_A` | `start_chase` accepted |
| `IDLE` | `CHASE_A` | `start_chase` accepted |
| `CHASE_A` | `TRACK_B` | `cat_visible_stable >= 3 frames` |
| `TRACK_B` | `CHASE_A` | local cat lost for >350 ms |
| `TRACK_B` | `BRAKE` | fresh visual lock active and valid range final-approach condition |
| `BRAKE` | `TRACK_B` | cat moves away before stop completes and visual lock remains fresh |
| `BRAKE` | `CHASE_A` | visual lock expires during braking and overhead remains valid |
| `BRAKE` | `FAILSAFE` | visual lock expires during braking and safe fallback is unavailable |
| `HOME` | `GOTO` | `go_to` accepted |
| `IDLE` | `GOTO` | `go_to` accepted |
| `GOTO` | `IDLE` | go-to target reached |
| any chase state | `IDLE` | `stop_chase` accepted |
| any non-failsafe | `RETURN_HOME` | `return_home` accepted with valid home coordinates |
| `RETURN_HOME` | `HOME` | return-home completed successfully |
| any | `FAILSAFE` | obstacle distance <10 cm or critical obstacle condition |
| any | `FAILSAFE` | unrecoverable safety condition |

Distance alone must not trigger `TRACK_B`. Camera confirmation is required.

### 9.1 Final Approach Target Identity
Local range sensing (Lidar C1 / ultrasonic) cannot classify what it sees. It may detect a cat, flower line, chair, wall, or any nearby obstacle.

Therefore, `BRAKE` requires:
- fresh visual lock from `VisionTracker`
- valid range from `RangeSafety`
- target direction consistency between visual offset and range zone when available

If visual lock expires during `BRAKE`, range data must no longer be treated as cat distance. The system must stop, fall back to `CHASE_A`, or enter `FAILSAFE` depending on overhead freshness and obstacle conditions.

## 10. Safety, Timeout, and Hysteresis Policy
- Overhead stale warning: `>300 ms` -> safe crawl.
- Overhead stale failsafe: `>700 ms` -> `FAILSAFE`.
- Camera-loss fallback: `>350 ms` in `TRACK_B` -> `CHASE_A`.
- No-progress/dead-end: `>2.0 s` -> recovery.
- Recovery escalation: `>5.0 s` -> `FAILSAFE`.
- Thermal warning: `75C`.
- Thermal speed limit: `80C`.
- Thermal return-home/failsafe: `85C`.

### 10.1 Hysteresis Rules
Hysteresis is required anywhere a boolean condition can rapidly toggle across a threshold.

| Condition | Enter Rule | Exit Rule |
|---|---|---|
| Local visual lock | `cat_visible` for >= 3 consecutive frames | no valid detection for >350 ms |
| Critical obstacle veto | obstacle severity >= critical threshold for 2 consecutive range samples | severity below clear threshold for 3 consecutive samples |
| Overhead stale warning | packet age >300 ms | fresh packet received with valid timestamp |
| Overhead failsafe | packet age >700 ms | manual/system recovery only; no automatic resume |
| No-progress recovery | no progress for >2.0 s | progress resumes for >=1.0 s |
| Recovery failsafe | blocked/no-progress for >5.0 s | manual/system recovery only |
| Thermal speed limit | temperature >=80C | temperature <=75C for >=10 s |
| Thermal return-home/failsafe | temperature >=85C | manual/system recovery only |

Obstacle hysteresis must use separate assert and clear thresholds to avoid repeated veto/clear oscillation near the boundary.

## 11. CPU and Performance Design

### 11.1 Preallocated Buffers
Use preallocated frame and inference buffers for camera frames and detector inputs. Avoid unbounded allocation in frame-rate paths.

Reuse ideas from:
- current `cat_follow/memory/pool.py`
- current `cat_follow/memory/shared_state.py`
- `cat_ball_tracker` preallocated ring/double-buffer pattern

### 11.2 Optional CPU Affinity
CPU affinity may be enabled by configuration on Linux.

Recommended target:
- control/comms/range threads: one reserved core when enabled
- AI/vision work: remaining cores

Affinity must be optional and must safely no-op on Windows or unsupported platforms.

### 11.3 Bounded Async Logging
Telemetry logging uses a bounded queue. If the queue is full, low-priority telemetry may be dropped, but safety/failsafe events should be preserved whenever possible.

This prevents logging backpressure from slowing the control loop.

## 12. Telemetry Events
Telemetry should be JSONL and include at minimum:
- timestamp
- event type
- current state
- requested state
- decision reason
- overhead packet age
- local cat visibility
- obstacle veto/severity
- thermal state
- speed/steering/brake output
- failsafe reason when applicable
- freshness status for each shared-state group
- authority source for state used in each decision

Systemd journal should still receive operational logs for service debugging.

## 13. Current Code Migration Notes

### 13.1 Main Loop
Current `main_loop.py` runs the control behavior in the main thread at approximately 30 Hz. Target architecture moves this into `CatFollow-Control`.

Migration:
- extract control branches into `DecisionEngine`
- keep startup/lifecycle in main
- keep Flask optional and non-authoritative

### 13.2 Current FSM
Current states should be replaced with PRD/HLD states. The current `IDLE` concept remains valid but its semantics should be tightened to mean stationary, safe, not chasing, and not necessarily at home.

Migration:
- preserve/refine `IDLE`; add distinct `HOME` for successful return-home completion
- replace target-driven approach path with `CHASE_A` and `GOTO`
- replace visual tracking path with `TRACK_B`
- add explicit `BRAKE`, `RETURN_HOME`, and `FAILSAFE`

### 13.3 Range Sensor
Current `range_sensor.py` should become the ultrasonic backend under `RangeSafety`.

Migration:
- preserve throttled/cached read behavior
- add Lidar C1 backend (TMF8829 backend is on hold)
- normalize all outputs to the same range/obstacle model

### 13.4 Web Control Routes
Current web routes that inject target/stop commands should remain as dev/test controls.

Migration:
- production chase commands come from `CommsManager`
- web injection routes call the same internal command/event path but are marked non-production

### 13.5 Vision Threads
Current camera/tracker/detector threads are useful and should be evolved, not discarded.

Migration:
- make `VisionTracker` the public facade
- keep lower-level camera/detector workers behind that interface
- keep detection independent of web UI

## 14. Open Issues
- Exact `Navigation` / SLAM implementation remains TBD.
- ACK/status packet schema remains deferred to detailed interface specification.
- Lidar C1 driver/API integration details remain to be selected. TMF8829 driver work is on hold.

## 15. Implementation Order
1. Implement target `SharedState` groups, ownership metadata, timestamps, and freshness helpers.
2. Add bounded async telemetry skeleton early so later modules can emit decision/fault events.
3. Introduce `DecisionEngine` shell without changing motor behavior.
4. Add target FSM states and transition validation.
5. Move control loop into dedicated `CatFollow-Control` thread.
6. Add `CommsManager` and make web injection dev/test only.
7. Wire `MotorInterface` as the single actuation boundary.
8. Refactor local camera/tracker/detector behind `VisionTracker`.
9. Wrap current range sensor as `UltrasonicRangeBackend`.
10. Add `RangeSafety` interface and Lidar C1 backend (TMF8829 backend on hold).
11. Add navigation interface placeholder.
12. Add optional CPU affinity config after runtime behavior is stable.
