# Detailed Software Architecture

**Project:** Autonomous Yard Navigator and Cat Tracker  
**Platform:** Radxa ROCK 4D with Radxa 4K IMX415 camera  
**Document version:** 2.0  
**Protocol version:** V1  
**Status:** Approved target design — **not implemented yet**  
**Canonical source:** `Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`  
**Date:** 2026-07-25

## 1. Purpose

This document defines the target detailed software architecture for the
autonomous yard navigator. It describes package ownership, runtime threading,
shared-state contracts, module APIs, control-loop behavior, perception
lifecycle, protocol transactions, and migration notes from the current
`cat_follow` codebase.

This document describes the **approved target design**, not current executable
behavior. Section 18 summarizes current-vs-target gaps, including conflicts with
`cat_follow/control/types.py` and the separate legacy `main_loop.py` path.

## 2. Source-of-truth decisions

- Target behavior follows
  `Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`,
  not current code-as-is.
- Production platform is ROCK 4D + IMX415 + RKNN-only inference.
- Local odometry is RF2O lidar odometry only; bicycle/wheel odometry is
  unsupported.
- Both lidar and forward ultrasonic are required for autonomous motion.
- `DecisionEngine` runs in dedicated `CatFollow-Control` and is the sole
  drivetrain authority.
- `NavigationManager` owns Nav2 goal lifecycle.
- `PerceptionLifecycleManager` owns camera/detector/recording/stream consumer
  lifecycle.
- `CommsManager` is the production overhead ingress path; web/API injection is
  dev/test only.
- Detection, tracking, and mission-required recording MUST work headlessly
  without a browser or monitoring-stream client.
- Yard positions and perimeter geometry use calibrated centimeters; Nav2 motion
  policy and telemetry use physical meters per second.
- Commands except emergency stop are transactional and ACKed only after
  control-loop application.
- Monitoring video is hardware H.264 only; there is no MJPEG/software fallback.

Retained useful patterns from the current codebase:

- preallocated frame buffers and refcounted frame-ring ownership;
- bounded async telemetry;
- optional CPU affinity behind config flags;
- separation of camera, detector, tracker, and range adapter threads.

Explicitly superseded prior decisions:

- Raspberry Pi 4 / TFLite production stack;
- `CHASE_A`, `TRACK_B`, `BRAKE` FSM names and final-stop semantics;
- additive camera/Nav2 steering fusion;
- 700 ms overhead-stale blanket failsafe during chase;
- 10 cm close-obstacle immediate failsafe;
- bicycle odometry fallback;
- PhaseMachine-owned mission inference in `SEARCH`/`CHASE`;
- always-on camera capture;
- predictive geofence path veto.

## 3. Target package layout

```text
cat_follow/
  runtime/
    app.py
    control_loop.py
    lifecycle.py
  comms/
    comms_manager.py
    messages.py
    overhead_client.py
  control/
    decision_engine.py
    fsm.py
    types.py
    commands.py
  navigation/
    navigation_manager.py
    ros_bridge.py
    interface.py
  perception/
    perception_lifecycle_manager.py
    vision_adapter.py
    phase.py
    h264_encoder.py
    recording_store.py
    range_adapter.py
    edge_ultrasonic.py
  threads/
    camera.py
    detector.py
    tracker.py
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
    app.py
    routes_h264.py
    routes_status.py
    ...
```

`NavigationManager` and `PerceptionLifecycleManager` are new target ownership
boundaries. Existing modules migrate incrementally into the structure above.

## 4. Runtime model

### 4.1 Process model

The car runs as a single Python process on ROCK 4D.

The process owns:

- worker threads for sensors, camera, detector, comms, and navigation bridge;
- dedicated real-time control thread for FSM and `DecisionEngine`;
- optional web UI thread for monitoring/configuration/dev injection;
- bounded async telemetry thread.

The web UI must not be required for detection, tracking, recording, or control.

### 4.2 Thread model

| Thread | Target name | Responsibility | Target rate |
|---|---|---|---|
| Main | `CatFollow-Main` | startup, shutdown, signal handling | event-driven |
| Control | `CatFollow-Control` | FSM, command transactions, arbitration, motor output | 50 Hz target, 20 Hz degraded minimum |
| Overhead | `CatFollow-Comms` | observations, commands, mission events, ACKs | input-driven |
| Camera | `CatFollow-Camera` | IMX415 NV12 capture, frame ring, hardware pause/resume | camera-dependent |
| Detector | `CatFollow-Detector` | RKNN YOLO inference | mission/config cadence |
| Tracker | `CatFollow-Tracker` | local track publication and association evidence | detector-driven |
| Range (hardware) | `CatFollow-UltrasonicIRQ` | libgpiod HC-SR04 edge events | ~17 Hz ping |
| Range (contract) | `CatFollow-RangeAdapter` | poll/cache -> `SharedState.range` | ~20 Hz |
| Lidar (ROS) | via `ros_bridge` | `/scan`, RF2O odom, direct safety inputs | sensor-dependent |
| Navigation bridge | `CatFollow-NavBridge` | ROS/Nav2 interface for `NavigationManager` | action/feedback driven |
| Telemetry | `CatFollow-Log` | bounded async JSONL logging | event-driven |
| Web UI | `CatFollow-Flask` | monitoring, H.264 clients, dev injection | request-driven |

### 4.3 Startup sequence

Before accepting autonomous motion:

1. load and verify durable home, map, geofence, and calibration versions;
2. initialize RF2O lidar odometry and local Nav2/SLAM localization;
3. obtain fresh overhead yard car pose;
4. transform through calibrated yard-to-map relationship;
5. seed or validate ROS localization within startup tolerances;
6. make local Nav2/SLAM localization authoritative;
7. validate `NavigationManager`, control loop, motor feedback, lidar, and
   ultrasonic;
8. publish readiness/degradation reasons.

Subsequent overhead car poses MAY be used for telemetry and bounded validation
but MUST NOT continuously overwrite authoritative local localization or hide
RF2O drift/failure.

## 5. Shared state model

All worker threads publish into synchronized `SharedState`. The control thread
reads one coherent snapshot per tick.

Target state groups:

- `overhead`: observation sequence, car pose, cats with `target_id`,
  `selected_target_id`, freshness, confidence
- `mission`: command ID, mission ID, active objective, active `target_id`,
  frozen home record/version, handoff/post-roll deadlines
- `vision`: associated local track ID, association evidence, offset, confidence
- `range`: lidar and ultrasonic distance, validity, fault, direct safety flags
- `navigation`: goal intent ID, action correlation, safe steering envelope,
  speed caps, path viability, completion status
- `perception_lifecycle`: detector/recording/stream reference counts and camera
  hardware state
- `system`: thermal profile, watchdog, fault flags, motion inhibition reasons
- `fsm`: current state, state-entry time, latched failsafe causes

Continue using preallocated buffers in `memory/pool.py` and
`memory/shared_state.py`. Avoid per-frame allocation in hot paths.

### 5.1 Coordinate convention

Global yard positions use the calibrated yard frame:

- units: centimeters (`cm`) for yard/perimeter geometry;
- heading: radians;
- yard coordinates are transformed explicitly into ROS navigation frames;
- overhead yard coordinates are not treated as ROS map coordinates without
  calibration.

Nav2 policies, caps, requests, and telemetry use physical `m/s`. Conversion to
PiCar-X normalized motor commands uses calibrated `speed_time_distance`
measurements.

### 5.2 Authoritative ownership

| State | Authoritative owner | Notes |
|---|---|---|
| Yard car/cat observations | Overhead system via `CommsManager` | Includes `target_id` and `selected_target_id` |
| Primary cat exit declaration | Overhead mission event only | Never inferred locally or from car geofence |
| Authoritative local pose after startup | Local Nav2/SLAM | Overhead seeds startup only |
| Durable home record | Persisted home store + FSM mission context | Versioned, checksummed, calibration-associated |
| Nav2 goal lifecycle | `NavigationManager` | Includes moving-goal refresh and correlation |
| Local track and association evidence | Detector/tracker pipeline | Bound to active `target_id` in `CHASE` |
| Direct lidar/ultrasonic safety | Range adapters + `DecisionEngine` | Required even when costmaps integrate sensors |
| Perception consumers and camera HW state | `PerceptionLifecycleManager` | Reference-count driven |
| Current FSM state | `FSM` | Validated mode holder |
| Final motion command | `DecisionEngine` | Sole drivetrain authority |
| Hardware actuation | `MotorInterface` | Only module writing drivetrain hardware |

Overhead data is strategic guidance and target selection, not a direct steering
source.

### 5.3 Freshness guarantees

Every shared-state group MUST carry:

- `timestamp_ms`: producer timestamp for cross-device correlation;
- `received_ms`: local monotonic receive time for freshness decisions;
- `fresh`: producer hint only;
- `authority`: producer name;
- `confidence`: when available.

Freshness for control decisions MUST be recomputed in `DecisionEngine` from
local monotonic time:

```text
age_ms = now_monotonic_ms - received_ms
```

Deployment configuration MUST define explicit max-age limits for lidar,
ultrasonic, localization, overhead, local track, navigation output, and motor
feedback. No component may silently substitute a different timeout.

Target dual-sensor rule: in normal autonomous driving, loss of either required
sensor commands zero motion, retains objective for up to 2 seconds, then enters
`FAILSAFE` if not recovered. During `BRAKE_REVERSE`, either required sensor
loss enters `FAILSAFE` immediately.

### 5.4 Control timing contract

`CatFollow-Control` targets 50 Hz with a 20 ms tick budget.

Per-tick budget:

| Operation | Budget |
|---|---:|
| Snapshot acquire | `<1 ms` |
| Freshness evaluation | `<1 ms` |
| Queued command/event application | `<2 ms` |
| `DecisionEngine` | `<5 ms` |
| FSM validation | `<1 ms` |
| Motor command write | `<2 ms` |
| Telemetry enqueue | `<1 ms` |
| Total control tick | `<20 ms` |

Overrun behavior:

- single tick over 20 ms: emit `control_tick_overrun`;
- 3 consecutive overruns: emergency stop and latch `FAILSAFE`;
- any tick over 100 ms: emergency stop, latch `FAILSAFE`, critical telemetry;
- any unhandled tick exception: emergency stop and latch `FAILSAFE`.

Telemetry file I/O MUST NOT run inside the control tick.

## 6. Core components

### 6.1 `CommsManager`

Production ingress for protocol V1.

Responsibilities:

- receive and validate overhead observations;
- maintain latest observation sequence, selected `target_id`, and staleness;
- receive reliable commands and mission events;
- deduplicate by command/event ID;
- queue accepted work for the control loop;
- emit ACKs only after control-loop application or rejection is committed;
- durably retain deduplication results within the protocol retention window.

Command handling:

- `SET_HOME`
- `START_CHASE(target_id)`
- `STOP_CHASE(target_id optional but validated)`
- `GO_TO(request_yolo, request_recording)`
- `RETURN_HOME`
- `CLEAR_FAILSAFE`
- synchronous `EMERGENCY_STOP`

Web/API injection reuses the internal command path but is non-production.

Authentication/token policy for dev/test adapters remains configurable; accepted
`EMERGENCY_STOP` invokes motor stop synchronously before ACK.

### 6.2 `NavigationManager`

Dedicated Nav2 goal lifecycle owner.

Responsibilities:

- own `NavigateToPose` action clients;
- derive goal intents from FSM state and mission context;
- transform yard targets into navigation-frame goals;
- refresh moving cat goals at most 2 Hz with at least 25 cm displacement;
- cancel/preempt immediately on safety or command preemption;
- correlate action results to goal intents and ignore late/wrong results;
- publish path viability and safe steering envelope;
- classify expected replacements as neutral;
- report exhausted failures and qualified completion.

Completion for `GOTO` and `RETURN_HOME` requires:

1. correlated Nav2 `SUCCEEDED`;
2. fresh local pose within 20 cm XY and 0.3 rad yaw;
3. continuous 1 s dwell.

Nav2 BackUp MUST remain disabled.

Ultrasonic MUST be published as validated `sensor_msgs/Range` and integrated
through a validated local-costmap `RangeSensorLayer`. Lidar remains integrated
independently. Direct ultrasonic safety MUST remain active even if the costmap
layer is disabled for diagnosis.

### 6.3 `PerceptionLifecycleManager`

Owns named consumers and reference counts:

- `detector`
- `recording`
- `stream`

Responsibilities:

- increment/decrement consumer references from mission policy and actual stream
  clients;
- translate references into camera hardware activation and detector cadence;
- force all consumers off in `HOME` and `FAILSAFE`;
- manage ready-inactive camera state with verified `STREAMOFF`/`STREAMON` or
  equivalent;
- preserve independent recording and monitoring encoder instances;
- expose requested vs active consumer state in telemetry.

Mission policy overrides legacy PhaseMachine gating in `SEARCH`, `CHASE`, and
`GOTO` with `request_yolo=true`. PhaseMachine MAY optimize cadence elsewhere
but MUST NOT suppress a mission-required consumer.

Lifecycle summary:

| FSM state | Detector | Recording | Stream | Camera |
|---|---|---|---|---|
| `HOME` | Forced off | Forced off | Forced off | Ready-inactive/closed |
| `IDLE` | Off | Post-roll/handoff only | Actual clients | Active only with consumer |
| `GETTING_CLOSE` | Off unless diagnostic | Chase request | Actual clients | Active if needed |
| `SEARCH` | Required on | Chase request | Actual clients | Active |
| `CHASE` | Required on | Chase request | Actual clients | Active |
| `BRAKE_REVERSE` | Inherit saved objective | Inherit saved request | Actual clients | Reference-based |
| `GOTO` | Exactly `request_yolo` | Exactly `request_recording` | Actual clients | Active if any consumer |
| `RETURN_HOME` | Off | Retain mission/post-roll | Actual clients | Active if needed |
| `FAILSAFE` | Forced off | Forced off | Forced off | Ready-inactive/closed |

Recording:

- separate hardware H.264 encoder;
- segmented crash-tolerant Matroska;
- storage quota and minimum free-space reserve;
- delete oldest finalized segments first;
- never delete active segment during quota cleanup;
- stop and report low space without stopping the mission;
- auto-resume while still requested after recovery;
- 10 s post-roll after `STOP_CHASE` or primary-left handoff.

Monitoring stream:

- hardware H.264 only;
- runs only while at least one actual client is connected and FSM does not
  force it off;
- no MJPEG/software fallback;
- recording demand MUST NOT be represented as a fake stream client.

### 6.4 Vision pipeline

Current `threads/camera.py`, `threads/detector.py`, and `threads/tracker.py`
evolve behind the lifecycle manager and shared-state contracts.

Responsibilities:

- IMX415 NV12 capture via frame ring with refcounted leases;
- RKNN-only inference;
- publish local detections/tracks and association evidence;
- remain operational with zero web clients or stream consumers when mission
  policy requires detector consumer;
- on fatal onboard camera/RKNN failure, degrade per state rules without
  silently associating a different target.

Production frame-path patterns from the current codebase remain valid:

- four-slot packed-NV12 frame ring;
- refcounted `FrameLease` readers;
- detector copies only its 320x320 NV12 crop into preallocated RKNN input.

See `Frame_Ring_Ownership_Audit.md` for ownership contract details.

### 6.5 `RangeSafety` and direct sensor inputs

Supported production backends:

- forward ultrasonic via `edge_ultrasonic.py` -> `range_adapter.py`
- lidar C1 via `ros_bridge.py`

Responsibilities:

- normalize both sources into shared state with validity/fault semantics;
- expose direct close-obstacle inputs to `DecisionEngine`;
- publish ultrasonic to Nav2 costmaps through validated `RangeSensorLayer`;
- never allow costmap disablement to disable direct ultrasonic safety.

Both sensors are required for autonomous motion.

### 6.6 `DecisionEngine`

Central motion decision maker.

Responsibilities:

- apply control precedence from the canonical target redesign;
- apply queued commands and mission events transactionally;
- enforce dual-sensor hold/recovery/failsafe rules;
- compute non-additive CHASE steering clamp and speed-cap fusion;
- manage `BRAKE_REVERSE` phase output and saved objective restoration;
- request FSM transitions;
- emit applied-command and veto-reason telemetry.

No perception, navigation, comms, or web module may directly command motion.

### 6.7 `FSM`

Validated state holder for canonical states:

- `HOME`
- `IDLE`
- `GETTING_CLOSE`
- `SEARCH`
- `CHASE`
- `BRAKE_REVERSE`
- `GOTO`
- `RETURN_HOME`
- `FAILSAFE`

Required mission context includes at least:

- current state and state-entry monotonic time;
- command ID and mission ID;
- active objective type;
- active `target_id` or null;
- frozen home record and home version;
- overhead observation sequence and timestamp;
- local track ID and association evidence;
- `NavigationManager` goal intent ID and action correlation;
- sensor-health hold start/reason;
- overhead-invalid retention start and last valid moving-target goal;
- SEARCH timeout stage and observation-waypoint status;
- handoff wait deadline and recording post-roll deadline;
- `BRAKE_REVERSE` saved objective, phase, attempt count, clearance-reset
  timer;
- thermal profile;
- requested and active perception consumers;
- latched failsafe causes.

If the FSM rejects a transition, it logs the rejection and the control tick
holds the current safe state or commands zero motion.

### 6.8 `MotorInterface`

Single hardware output boundary:

- execute validated normalized drive/steering;
- controlled stop;
- emergency stop;
- isolate PiCar-X driver details from control logic.

## 7. Control tick flow

At each `CatFollow-Control` tick:

1. Read one shared-state snapshot.
2. Apply synchronous emergency-stop handling if asserted.
3. Evaluate actual geofence breach, motor/control fatal state, watchdog, and
   an already-latched `FAILSAFE`.
4. Evaluate active `BRAKE_REVERSE` health, exhaustion, preemption, and phase
   output.
5. Evaluate stale/faulted required lidar or ultrasonic hold/escalation.
6. Apply the critical thermal return policy.
7. Drain, validate, and atomically apply queued commands/mission events.
8. Evaluate localization/navigation/target/perception degradation.
9. Apply Nav2 constraints and compute final mission drive/steering or
   zero-motion veto.
10. Request and validate any resulting FSM transition.
11. Update `NavigationManager` and `PerceptionLifecycleManager` intents.
12. Send the final command to `MotorInterface`.
13. Commit transaction result state and enqueue ACK/telemetry.

Authoritative precedence:

1. synchronous emergency stop;
2. actual car geofence breach;
3. motor/control fatal or watchdog;
4. latched `FAILSAFE`;
5. required-health loss during active `BRAKE_REVERSE`;
6. exhausted `BRAKE_REVERSE` attempts;
7. command/event preemption of `BRAKE_REVERSE`;
8. active `BRAKE_REVERSE` phase output;
9. stale/faulted required lidar or ultrasonic hold/escalation;
10. critical thermal return policy;
11. command and mission-event transitions;
12. localization/navigation/target/perception degradation;
13. Nav2 obstacle/path/no-progress constraints;
14. mission-state motion policy;
15. recording and monitoring lifecycle.

## 8. Control fusion policy

`DecisionEngine` owns all final fusion.

In `CHASE`, look/drive mode selection (`Look_Drive_Path_Design.md`) decides
whether chassis follow `path_correction` (`LOOK_AT` / `PATH_FOLLOW`) or a
vision request clamped into the Nav2 envelope (`BODY_STEER`, pan at forward
only). Modes are mutually exclusive; camera and Nav2 steering MUST NOT be
added or weighted-summed:

```text
# LOOK_AT / PATH_FOLLOW:
applied_steering = clamp(path_correction, safe_min, safe_max)

# BODY_STEER only:
applied_steering = clamp(
    camera_steering_request,
    nav2_safe_steering_min,
    nav2_safe_steering_max
)

# HOLD:
applied_steering = 0; applied_speed = 0; brake

applied_speed_mps = min(
    pursuit_speed_request_mps,
    nav2_speed_cap_mps,
    alignment_speed_cap_mps,
    obstacle_speed_cap_mps,
    thermal_speed_cap_mps
)
```

Pan commands ship atomically with chassis output via `MotorInterface.apply_look`.
Rules:

- camera and Nav2 steering MUST NOT be added or weighted;
- path viability and every safety veto remain mandatory;
- overhead never directly drives steering;
- close obstacle or tracked cat within trigger distance enters `BRAKE_REVERSE`,
  not a separate arrival/brake state;
- `BRAKE_REVERSE` reverse is direct normalized/time-bounded control, not Nav2
  BackUp.

Telemetry MUST record requested physical speed, all applied caps, requested
steering, Nav2 safe envelope, look/drive mode / pan / pixel error / look reason,
final normalized/applied commands, calibration version, command source, and
every zero-motion veto reason.

## 9. State transition rules

Normative transition tables live in the canonical target redesign. Summary:

### 9.1 Command/event matrix highlights

- `SET_HOME`: accepted only in `HOME`/`IDLE` with durable persistence success.
- `START_CHASE(target_id)`: accepted only from `HOME`/`IDLE` after full
  validation; enters `GETTING_CLOSE`.
- `STOP_CHASE`: from chase states enters `IDLE` with recording post-roll; from
  chase `BRAKE_REVERSE` only when saved objective is chase.
- `GO_TO`: accepted from `HOME`/`IDLE`; independent `request_yolo` and
  `request_recording`.
- `RETURN_HOME`: accepted from chase states and `GOTO`; from `BRAKE_REVERSE`
  replaces saved objective after stop.
- matching `PRIMARY_CAT_LEFT_PERIMETER(target_id)`: from chase states enters
  handoff `IDLE`.
- `CLEAR_FAILSAFE`: accepted only with cause-specific clearance; enters clean
  `IDLE` without restarting mission consumers automatically.

### 9.2 Autonomous chase transitions

- `GETTING_CLOSE -> SEARCH` at valid overhead distance `<=200 cm`.
- `SEARCH -> CHASE` after three consecutive unambiguous associated observations.
- `SEARCH` timeout stage 1: one safe observation waypoint, remain in `SEARCH`.
- `SEARCH` timeout stage 2: `RETURN_HOME` if safe, else `FAILSAFE`.
- `CHASE -> SEARCH` or `GETTING_CLOSE` directly on associated-track loss based
  on overhead distance; two-hop `CHASE -> GETTING_CLOSE -> SEARCH` is
  forbidden.
- any normal driving state with close trigger from either required sensor ->
  `BRAKE_REVERSE`.
- `RETURN_HOME` success -> `HOME` with forced perception consumers off.

### 9.3 Target handoff

Matching primary-left event in chase states or chase `BRAKE_REVERSE`:

1. immediate zero motion;
2. cancel Nav2/reverse;
3. enter `IDLE`;
4. start configurable 10 s handoff wait and recording post-roll.

During handoff:

- valid `START_CHASE(new_target_id)` -> `GETTING_CLOSE`;
- explicit `RETURN_HOME` -> `RETURN_HOME`;
- timeout -> `RETURN_HOME` if safe, else `FAILSAFE`.

Local camera visibility never overrides a valid matching event. The car does
not promote a local secondary track.

### 9.4 `BRAKE_REVERSE`

Trigger: fresh valid lidar or ultrasonic reading strictly below
`CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM` (default `<15 cm`).

Phases:

1. `STOP`
2. `CENTER`
3. `SETTLE` for 100 ms
4. `REVERSE` for 0.5 s at normalized `-0.30`
5. `STOP`
6. `RECHECK`

Maximum three reverse attempts. Clearance reset requires both sensors `>20 cm`
continuously for 2 s. Loss of either required sensor during reverse enters
`FAILSAFE` immediately.

## 10. Safety, degradation, and thermal policy

Unconditional failsafe from any state:

- emergency stop;
- confirmed car geofence crossing;
- motor/control fatal error;
- control-loop watchdog expiration.

Onboard camera/RKNN fatal failure alone does not enter `FAILSAFE`:

- `SEARCH`/`CHASE` -> `GETTING_CLOSE` overhead-only degraded;
- `GETTING_CLOSE` continues overhead/Nav2;
- `GOTO`/`RETURN_HOME` continue when navigation and safety remain valid;
- `HOME`/`IDLE` remain stopped and degraded;
- `BRAKE_REVERSE` completes only if required non-vision safety remains valid.

Recording, encoder, storage, or monitoring-stream failure causes no FSM
transition and no motion veto; it produces degraded telemetry and resumes when
healthy while still requested.

Critical thermal behavior:

- any active objective other than `RETURN_HOME` -> `RETURN_HOME` if safe, else
  `FAILSAFE`;
- `RETURN_HOME` continues under critical-return profile until unsafe;
- `HOME`/`IDLE` remain stopped and degraded;
- thermal during `BRAKE_REVERSE` stops motion and re-evaluates objective toward
  `RETURN_HOME` only after clearance/health checks pass.

## 11. CPU and performance design

### 11.1 Preallocated buffers

Use preallocated frame and inference buffers for camera and RKNN paths. Avoid
unbounded allocation in frame-rate paths.

Reuse:

- `memory/pool.py`
- `memory/shared_state.py`
- refcounted NV12 frame ring with owned-buffer discipline

### 11.2 Optional CPU affinity

CPU affinity MAY be enabled on Linux.

Recommended ROCK 4D target when enabled:

- camera threads: cores `0,1`
- detector/tracker: core `2`
- ultrasonic edge worker: core `3`
- control/comms/range adapter: shared/unpinned
- ROS/Nav2: A72 cores `4-7`

Affinity MUST safely no-op on unsupported platforms.

### 11.3 Bounded async logging

Telemetry uses a bounded queue. Safety/failsafe events are preserved ahead of
lower-severity records. Logging backpressure MUST NOT slow the control loop.

### 11.4 Monitoring map freshness

The web map is non-authoritative and MUST NOT present stale data as live.
Monitoring video uses H.264 hardware encoding only; lack of encoder capacity or
clients means no monitoring video, not a software fallback stream.

## 12. Telemetry events

Telemetry MUST be JSONL and include at minimum:

- monotonic timestamp and applied control sequence;
- event type and decision reason;
- current/requested FSM state;
- command/event ID, ACK result, and rejection reason;
- active `target_id`, observation sequence, and handoff status;
- local track association evidence;
- lidar/ultrasonic freshness, hold timers, and veto reasons;
- Nav2 goal intent/correlation, safe envelope, speed caps, completion dwell;
- perception consumer requested vs active counts and camera hardware state;
- recording quota/reserve status and post-roll deadlines;
- thermal profile and motion inhibition causes;
- final normalized drive/steering command.

Systemd journal remains useful for operational service debugging.

## 13. Target configuration defaults

Environment names MUST use the `CAT_FOLLOW_*` namespace. Canonical defaults:

| Configuration | Default | Meaning |
|---|---:|---|
| `CAT_FOLLOW_SEARCH_ENTRY_DISTANCE_CM` | `200` | Enter SEARCH at or below this valid overhead distance |
| `CAT_FOLLOW_SEARCH_SPEED_CAP_MPS` | `0.10` | SEARCH and retained-goal speed cap |
| `CAT_FOLLOW_SEARCH_LOCK_OBSERVATIONS` | `3` | Consecutive unambiguous associated observations |
| `CAT_FOLLOW_SEARCH_INTERVAL_SEC` | `10` | Each SEARCH timeout interval |
| `CAT_FOLLOW_OVERHEAD_INVALID_MAX_SEC` | `10` | Last valid goal retention in GETTING_CLOSE/SEARCH |
| `CAT_FOLLOW_SENSOR_RECOVERY_SEC` | `2` | Required-sensor zero-motion recovery interval |
| `CAT_FOLLOW_HANDOFF_WAIT_SEC` | `10` | IDLE wait after primary target exit |
| `CAT_FOLLOW_RECORDING_POSTROLL_SEC` | `10` | Recording post-roll after chase stop/exit |
| `CAT_FOLLOW_NAV_MOVING_GOAL_MAX_HZ` | `2` | Maximum moving-target goal submission rate |
| `CAT_FOLLOW_NAV_MOVING_GOAL_MIN_DISPLACEMENT_CM` | `25` | Minimum displacement for normal refresh |
| `CAT_FOLLOW_NAV_COMPLETION_XY_CM` | `20` | Local-pose XY completion tolerance |
| `CAT_FOLLOW_NAV_COMPLETION_YAW_RAD` | `0.3` | Local-pose yaw completion tolerance |
| `CAT_FOLLOW_NAV_COMPLETION_DWELL_SEC` | `1` | Continuous in-tolerance dwell |
| `CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM` | `15` | Trigger on fresh valid reading strictly below this |
| `CAT_FOLLOW_BRAKE_REVERSE_SETTLE_MS` | `100` | Centered stop before reverse |
| `CAT_FOLLOW_BRAKE_REVERSE_DURATION_SEC` | `0.5` | Bounded reverse duration |
| `CAT_FOLLOW_BRAKE_REVERSE_NORMALIZED` | `-0.30` | Direct normalized reverse command |
| `CAT_FOLLOW_BRAKE_REVERSE_MAX_ATTEMPTS` | `3` | Maximum reverse phases before failsafe |
| `CAT_FOLLOW_BRAKE_REVERSE_RESET_CM` | `20` | Both sources must exceed this to reset attempts |
| `CAT_FOLLOW_BRAKE_REVERSE_RESET_SEC` | `2` | Continuous dual-source clearance duration |
| `CAT_FOLLOW_NAV_ULTRASONIC_COSTMAP` | `1` | Enable validated RangeSensorLayer integration |
| `CAT_FOLLOW_NAV2_BACKUP_ENABLED` | `0` | Nav2 BackUp disabled |

Deployment-calibrated values MUST be validated at startup for overhead
confidence gates, association bearing gate, recording quota/reserve, and sensor
freshness limits.

## 14. Current code migration notes

### 14.1 Legacy `main_loop.py`

`main_loop.py` is a separate legacy prototype path and MUST NOT be treated as
production architecture:

- control runs in the main thread at ~30 Hz;
- ultrasonic uses legacy `range_sensor.set_car(Picarx)` polling;
- it does not implement canonical FSM states, `NavigationManager`,
  `PerceptionLifecycleManager`, transactional protocol, RF2O-only odometry,
  or dual-sensor target safety.

Target production runtime is `runtime/app.py` with dedicated
`CatFollow-Control`.

### 14.2 Current FSM and control types

`cat_follow/control/types.py` and `cat_follow/control/fsm.py` still encode the
superseded `CHASE_A`, `TRACK_B`, and `BRAKE` model. Migration MUST replace
them with canonical states, mission context, and complete transition matrices
from the target redesign.

### 14.3 Current runtime wiring

Useful current pieces to preserve/evolve:

- `runtime/app.py` lifecycle shell;
- `threads/camera.py`, `threads/detector.py`, `threads/tracker.py`;
- `memory/shared_state.py`, `memory/pool.py`;
- `perception/edge_ultrasonic.py`, `perception/range_adapter.py`;
- `navigation/ros_bridge.py` as the ROS interface foundation;
- `web_ui/routes_h264.py` as the monitoring entry point once client-only H.264
  behavior is complete.

Missing target pieces:

- `NavigationManager`
- `PerceptionLifecycleManager`
- durable/versioned home store and transactional `SET_HOME`
- reliable `mission_event` handling
- stable `target_id` protocol fields
- hardware segmented recording store
- non-additive CHASE clamp in `DecisionEngine`
- formal `BRAKE_REVERSE` state machine

### 14.4 Web control routes

Current web routes that inject commands remain dev/test only. Production chase
control comes from `CommsManager`.

## 15. Implementation order

1. Replace control types/FSM with canonical states, mission context, and
   transition validation.
2. Add transactional command/event queueing and ACK-after-apply semantics in
   comms + control loop.
3. Introduce durable/versioned home store and mission home freeze.
4. Implement `NavigationManager` on top of `ros_bridge.py`.
5. Implement `PerceptionLifecycleManager` and camera STREAMOFF/STREAMON policy.
6. Wire dual-required-sensor hold/recovery/failsafe in `DecisionEngine`.
7. Implement non-additive CHASE clamp and Nav2 safe envelope consumption.
8. Implement formal `BRAKE_REVERSE` phases and saved-objective restoration.
9. Add stable `target_id` protocol fields and reliable `mission_event` handling.
10. Add hardware segmented recording, quota/reserve, post-roll, and H.264-only
    monitoring behavior.
11. Expand telemetry to reconstruct transitions, holds, goal correlation, and
    applied commands.
12. Retire legacy `main_loop.py` production use after target runtime acceptance.

## 16. Validation requirements

Target tests MUST cover:

- every command/event matrix cell and rejection/idempotency path;
- transactional ACK after control-loop application and duplicate replay;
- primary-left handoff, post-roll, new target, explicit return, and timeout;
- dual-sensor hold/recovery/failsafe in all driving states and reverse immediate
  fail;
- `<15 cm` `BRAKE_REVERSE` phases, attempts, preemption, and clearance reset;
- non-additive steering clamp, speed caps, and path viability;
- lifecycle-table rows and headless SEARCH/CHASE with no web clients;
- recording quota/resume/post-roll and absence of MJPEG/software fallback;
- durable/versioned `SET_HOME`, persistence failure, and mission home freeze;
- RF2O-only startup localization and overhead seed/validation behavior.

Detailed cases remain in the canonical target redesign and validation matrix.

## 17. Related documents

- `Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`
- `PRD_Autonomous_Yard_Navigator_Cat_Tracker.md`
- `High_Level_Design_Autonomous_Yard_Navigator_Cat_Tracker.md`
- `Interface_and_Data_Contract_Specification_Autonomous_Yard_Navigator_Cat_Tracker.md`
- `Software_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md`
- `Validation_Matrix_Autonomous_Yard_Navigator_Cat_Tracker.md`
- `Frame_Ring_Ownership_Audit.md`

## 18. Current vs target migration summary

The current repository does **not** implement this target. The highest-impact
conflicts are:

| Area | Current behavior | Target behavior |
|---|---|---|
| FSM | `CHASE_A`, `TRACK_B`, `BRAKE` in `control/types.py` / `fsm.py` | `GETTING_CLOSE`, `SEARCH`, `CHASE`, `BRAKE_REVERSE`, handoff semantics |
| Close obstacle | ~10 cm failsafe | `<15 cm` formal `BRAKE_REVERSE` with saved objective |
| Overhead loss in chase | ~700 ms failsafe | state-specific retained goal, local-track continuation, or return |
| Navigation ownership | advisory/incomplete bridge | full `NavigationManager` goal lifecycle |
| CHASE fusion | weighted/additive steering model in docs/code intent | non-additive clamp inside Nav2 safe envelope |
| Odometry | bicycle fallback described in prior docs | RF2O only |
| Inference/platform | prior Pi/TFLite assumptions | ROCK 4D + IMX415 + RKNN only |
| Protocol | no stable `target_id` / mission_event | V1 commands/events with transactional ACK |
| Perception lifecycle | effectively always-on camera | reference-counted consumers + hardware pause |
| Recording/stream | incomplete | hardware H.264 segmented recording + client-only monitoring |
| Legacy runtime | `main_loop.py` polling path | `runtime/app.py` dedicated control thread |

These are migration gaps, not permission to partially reinterpret the target.
