# System Architecture Document

**Project:** Autonomous Yard Navigator and Cat Tracker  
**Platform:** Radxa ROCK 4D with Radxa 4K IMX415 camera  
**Document version:** 2.0  
**Protocol version:** V1  
**Status:** Approved target design — **not implemented yet**  
**Canonical source:** `Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`  
**Date:** 2026-07-25

## 1. Purpose

This document defines the target system architecture for the autonomous yard
navigator and cat tracker. It describes runtime topology, authority boundaries,
mission FSM, safety posture, protocol contracts, and perception lifecycle for
the production ROCK 4D platform.

This document describes the **approved target design**, not current executable
behavior. Section 15 lists known migration gaps, including conflicts with
`cat_follow/control/types.py` and the legacy `main_loop.py` path.

For behavior covered here, the canonical target redesign supersedes conflicting
requirements in prior PRD, HLD, and v1.0 architecture documents.

## 2. Architecture principles

1. **DecisionEngine is the sole drivetrain authority.** No camera, detector,
   communications component, Nav2 adapter, or web route writes motor or steering
   commands directly.
2. **Detection and tracking are headless.** The processing loop runs without a
   browser, MJPEG/H.264 client, or web UI connection.
3. **Global sensing is strategic; local sensing and Nav2 own tactical motion.**
   Overhead yard observations guide mission intent. Local Nav2/SLAM,
   `NavigationManager`, and direct safety inputs own obstacle-aware motion.
4. **Dual-sensor safety is mandatory.** Lidar and forward ultrasonic are both
   required for autonomous motion and remain direct `DecisionEngine` inputs even
   when integrated with Nav2 costmaps.
5. **Emergency stop and hard safety vetoes preempt mission behavior.**
6. **Commands are transactional.** All mission commands except emergency stop
   are deduplicated, validated, applied atomically at a control-loop boundary,
   and ACKed only after committed application or rejection.
7. **Coordinate transforms are explicit.** Yard coordinates are not treated as
   ROS navigation-frame coordinates without calibration.
8. **Safety telemetry is reconstructable.** Every motion inhibition records
   source, freshness, and reason.

The following prior behavior is explicitly **not** part of the target:

- Raspberry Pi 4 / TFLite production stack;
- `CHASE_A`, `TRACK_B`, and final-stop `BRAKE` state semantics;
- bicycle/wheel odometry support or fallback;
- a fixed 10 cm final stop at the tracked cat;
- immediate 700 ms overhead-stale failsafe during chase;
- additive or weighted camera/Nav2 steering;
- always-on camera capture and PhaseMachine-owned mission inference;
- software/MJPEG monitoring fallback;
- predictive geofence path veto;
- lidar-only or ultrasonic-only autonomous motion.

## 3. Production platform

| Layer | Target choice |
|---|---|
| Compute | Radxa ROCK 4D |
| Onboard camera | Radxa 4K IMX415 |
| Onboard inference | RKNN only |
| Local navigation | ROS 2 / Nav2 |
| Local odometry | RF2O lidar odometry only |
| Forward safety sensing | Lidar C1 + forward ultrasonic |
| Strategic sensing | Overhead yard camera/service |
| Monitoring video | Hardware H.264 only, client-driven |
| Recording | Separate hardware H.264 segmented Matroska |

Bicycle/wheel odometry is unsupported. Loss of RF2O/localization is handled by
health and failsafe rules, not by a stale-data odometry fallback.

## 4. Runtime topology

### 4.1 Process model

- Single Python application process on ROCK 4D.
- Dedicated real-time control thread for FSM, arbitration, and motor output.
- Worker threads for camera, detector, comms, range, navigation bridge, and
  telemetry.
- Optional web UI thread for monitoring, configuration, and dev/test injection
  only.

The web UI is never required for detection, tracking, recording policy, or
control.

### 4.2 Thread and component map

| Component | Target name / owner | Responsibility |
|---|---|---|
| Main | `CatFollow-Main` | Boot, config load, lifecycle, signal handling |
| Control | `CatFollow-Control` | FSM, `DecisionEngine`, command transactions, motor output |
| Overhead comms | `CatFollow-Comms` | Overhead observations, commands, mission events, ACKs |
| Camera | `CatFollow-Camera` | IMX415 NV12 capture, frame ring, hardware pause/resume |
| Detector | `CatFollow-Detector` | RKNN YOLO inference and local track publication |
| Range (ultrasonic) | `CatFollow-UltrasonicIRQ` + `CatFollow-RangeAdapter` | Forward ultrasonic sampling and `SharedState.range` |
| Range (lidar) | via `ros_bridge` | `/scan` and direct safety publication |
| Navigation bridge | `CatFollow-NavBridge` | ROS/Nav2 interface consumed by `NavigationManager` |
| Telemetry | `CatFollow-Log` | Bounded async JSONL logging |
| Web UI | `CatFollow-Flask` | Monitoring, H.264 client serving, dev/test injection |

### 4.3 Authority boundaries

| Concern | Owner |
|---|---|
| Yard-level car/cat observations and selected `target_id` | Overhead system |
| `PRIMARY_CAT_LEFT_PERIMETER(target_id)` declaration | Overhead system only |
| Startup overhead car pose seed/validation | Overhead + startup sequence |
| Authoritative local motion localization after startup acceptance | Local Nav2/SLAM |
| Nav2 `NavigateToPose` clients and goal lifecycle | `NavigationManager` |
| Camera/detector/recording/stream consumer lifecycle | `PerceptionLifecycleManager` |
| Final drivetrain command | `DecisionEngine` |
| Hardware actuation | `MotorInterface` |
| Durable versioned home | Persisted home store + FSM mission context |

`NavigationManager` and `PerceptionLifecycleManager` are target ownership
boundaries. They do not exist as specified in the current repository.

Nav2 BackUp recovery MUST remain disabled. `<15 cm` close-obstacle recovery is
the separate bounded `BRAKE_REVERSE` state owned by `DecisionEngine`.

## 5. Core modules

### 5.1 `DecisionEngine`

Central motion decision maker. Each control tick:

1. reads one synchronized shared-state snapshot;
2. applies control precedence (Section 7);
3. validates/applies queued commands and mission events transactionally;
4. requests FSM transitions;
5. computes final normalized drive/steering or zero-motion veto;
6. emits decision reason codes and applied-command telemetry.

No other module may command motion.

### 5.2 `FSM`

Validated mission state holder for the canonical states:

| State | Purpose |
|---|---|
| `HOME` | Stopped at durable home within completion tolerance |
| `IDLE` | Stopped, ready for command, or waiting during target handoff |
| `GETTING_CLOSE` | Follow selected overhead `target_id` with Nav2; onboard detection not required |
| `SEARCH` | Move slowly near overhead target while acquiring verified local lock |
| `CHASE` | Pursue associated local track inside Nav2 safety/path constraints |
| `BRAKE_REVERSE` | Formal bounded close-obstacle recovery with saved objective |
| `GOTO` | Explicit Nav2 destination with independent YOLO/recording requests |
| `RETURN_HOME` | Navigate to frozen mission home |
| `FAILSAFE` | Latched zero-motion safety state requiring cause-specific clearance |

There is no "arrived at cat" or mission-success state. Close proximity to a
tracked cat uses the same `BRAKE_REVERSE` policy as any close obstacle.

State groups:

- Chase states: `GETTING_CLOSE`, `SEARCH`, `CHASE`
- Normal autonomous driving: chase states, `GOTO`, `RETURN_HOME`
- Stationary: `HOME`, `IDLE`, `FAILSAFE`

### 5.3 `NavigationManager`

Owns all Nav2 goal lifecycle:

- `NavigateToPose` action clients;
- state-derived goal intents;
- yard-to-navigation-frame transforms;
- moving-goal refresh (`2 Hz` max, `25 cm` min displacement by default);
- cancel/preemption and safety-immediate cancel;
- action result and goal-intent correlation;
- path viability and safe steering envelope publication;
- retries, exhausted-failure reporting, and completion qualification.

Moving cat goals are refreshed intentionally; expected replacement MUST NOT
count as navigation failure.

### 5.4 `PerceptionLifecycleManager`

Owns named consumers and reference counts:

- `detector`
- `recording`
- `stream`

Consumers are independent. Detector and recording never depend on stream
clients. Stream reference count is the actual connected-client count.

Mission policy overrides legacy PhaseMachine gating in `SEARCH`, `CHASE`, and
`GOTO` with `request_yolo=true`.

Camera hardware uses ready-inactive (`STREAMOFF` or equivalent) when no
consumer needs frames. `HOME` and `FAILSAFE` force all consumers off.

### 5.5 `CommsManager`

Production ingress for:

- overhead observations with stable per-cat `target_id` and
  `selected_target_id`;
- reliable commands (`SET_HOME`, `START_CHASE`, `STOP_CHASE`, `GO_TO`,
  `RETURN_HOME`, `CLEAR_FAILSAFE`);
- reliable ACKed `mission_event` envelopes, including
  `PRIMARY_CAT_LEFT_PERIMETER(target_id)`.

Web/API injection remains dev/test only and reuses the internal command path.

### 5.6 `RangeSafety` and direct sensor inputs

Both lidar and ultrasonic are required for autonomous motion:

- normalized into shared state with freshness and fault semantics;
- published to Nav2 (`sensor_msgs/Range` + validated `RangeSensorLayer`);
- consumed directly by `DecisionEngine` regardless of costmap integration.

Disabling the ultrasonic costmap layer for diagnosis MUST NOT disable direct
ultrasonic safety.

### 5.7 `MotorInterface`

Single hardware output boundary:

- `set_motion(speed, steering)`
- controlled stop
- emergency stop

## 6. Data and protocol contracts

### 6.1 Overhead observation (V1)

Every cat MUST have a stable `target_id`. The selected target is named by
`selected_target_id`, never by array position.

```json
{
  "protocol_version": 1,
  "type": "overhead_observation",
  "observation_seq": 1842,
  "observed_at_ms": 1785012345678,
  "perimeter_id": "yard-v3",
  "calibration_version": 7,
  "car": { "x_cm": 412.4, "y_cm": 226.8, "yaw_rad": 1.12, "confidence": 0.96 },
  "cats": [
    {
      "target_id": "cat-17",
      "x_cm": 531.0,
      "y_cm": 301.2,
      "confidence": 0.93,
      "inside_perimeter": true
    }
  ],
  "selected_target_id": "cat-17"
}
```

### 6.2 Commands and mission events

Every target-scoped command MUST carry `target_id`. `START_CHASE` requires it.
`STOP_CHASE` SHOULD carry the expected active `target_id` and MUST reject as
`WRONG_TARGET` when mismatched.

Mission exit is declared only by overhead:

```json
{
  "protocol_version": 1,
  "type": "mission_event",
  "event_id": "evt-31bd",
  "mission_id": "mission-204",
  "issued_at_ms": 1785012399000,
  "name": "PRIMARY_CAT_LEFT_PERIMETER",
  "target_id": "cat-17",
  "perimeter_id": "yard-v3",
  "observation_seq": 1901
}
```

Commands and mission events except emergency stop:

1. are deduplicated by ID;
2. are queued for the control loop;
3. are validated and applied atomically at a control-loop boundary;
4. are ACKed only after committed application or rejection;
5. report resulting state and reason.

Emergency stop remains synchronous and is not delayed for transactional
mission processing.

### 6.3 Durable home

`SET_HOME` is accepted only in `HOME` or `IDLE` when localization, calibration,
geofence, stopped motion, and durable persistence all succeed.

The home record MUST be versioned, checksummed, calibration/map-associated, and
durably committed before ACK. Active missions freeze the home version at
acceptance; updating home during an active mission is forbidden.

## 7. Control precedence

Every control tick applies the following precedence, highest first:

1. synchronous emergency stop;
2. actual car geofence breach;
3. motor/control fatal error or control-loop watchdog;
4. already-latched `FAILSAFE`;
5. required-health loss during active `BRAKE_REVERSE`;
6. exhausted `BRAKE_REVERSE` attempts;
7. command/event preemption of `BRAKE_REVERSE`;
8. active `BRAKE_REVERSE` phase output;
9. stale/faulted required lidar or ultrasonic hold/escalation;
10. critical thermal return policy;
11. command and mission-event transitions;
12. localization, navigation, target, and perception degradation rules;
13. Nav2 obstacle/path/no-progress constraints;
14. mission-state motion policy;
15. recording and monitoring lifecycle.

Lower-precedence inputs MUST NOT reauthorize motion vetoed at a higher level.
Recording and streaming never authorize motion.

## 8. Mission flow

### 8.1 Chase lifecycle

1. `START_CHASE(target_id)` accepted from `HOME` or `IDLE` after validation.
2. Mission freezes durable home version and enters `GETTING_CLOSE`.
3. `NavigationManager` tracks overhead `target_id` as a moving Nav2 goal.
4. At `<=200 cm` valid overhead distance, enter `SEARCH`.
5. Three consecutive unambiguous associated local observations bind the local
   track to `target_id` and enter `CHASE`.
6. In `CHASE`, camera pursuit is clamped inside the Nav2 safe steering envelope
   (Section 9).
7. Local track loss with valid overhead transitions directly to `SEARCH`
   (`<=200 cm`) or `GETTING_CLOSE` (`>200 cm`).
8. Matching `PRIMARY_CAT_LEFT_PERIMETER(target_id)` enters handoff `IDLE` with
   configurable post-roll and 10-second wait for a new target or return.

Onboard perception or recording MAY be unavailable at chase acceptance if
overhead-only pursuit remains safe; the mission starts degraded and reports the
missing facilities.

### 8.2 Overhead degradation

In `GETTING_CLOSE` or `SEARCH`, stale or fresh-but-invalid overhead data:

1. retains the last valid Nav2 goal for at most 10 seconds;
2. caps motion at SEARCH speed (`0.10 m/s` default);
3. continues only while localization, geofence, `NavigationManager`, lidar, and
   ultrasonic remain valid;
4. on same-target recovery, refreshes the goal;
5. on different `target_id`, stops and enters `IDLE`;
6. on timeout, enters `RETURN_HOME` if safe, otherwise `FAILSAFE`.

In `CHASE`, overhead loss has no fixed timeout while the associated local
track, localization, geofence, `NavigationManager`, lidar, and ultrasonic all
remain valid.

### 8.3 Dual-sensor health

In any normal autonomous driving state, if either lidar or ultrasonic becomes
stale, invalid, or faulted:

1. command zero motion immediately;
2. retain current FSM state and objective;
3. start a configurable 2-second recovery timer;
4. resume only if both sources recover within 2 seconds;
5. enter `FAILSAFE` if the timer expires.

During `BRAKE_REVERSE`, loss of either required source enters `FAILSAFE`
immediately.

`HOME` and `IDLE` remain stopped and report degraded health without automatic
failsafe escalation from sensor staleness alone. Later motion commands are
rejected until both sensors are healthy.

### 8.4 `BRAKE_REVERSE`

Triggered in normal driving states by a fresh valid lidar or ultrasonic reading
strictly below `CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM` (default `<15 cm`).

Phases: `STOP` → `CENTER` → `100 ms SETTLE` → `0.5 s REVERSE at -0.30`
normalized → `STOP` → `RECHECK`.

Maximum three reverse attempts. Attempt count resets only after both sensors
report `>20 cm` continuously for 2 seconds. `CLEAR_FAILSAFE` does not bypass
clearance rules.

There is no rear-facing sensor; production release requires explicit hardware
validation of the bounded reverse maneuver.

### 8.5 Perception, recording, and H.264 lifecycle

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

Recording uses a separate hardware H.264 encoder, segmented crash-tolerant
Matroska files, storage quota, minimum free-space reserve, and automatic resume
while still requested. Recording/stream failure causes no FSM transition.

Monitoring video uses hardware H.264 only while at least one client is
connected and the FSM does not force it off. There is no MJPEG or
software-encoding fallback.

## 9. CHASE fusion policy

Camera pursuit produces a steering request. Nav2 produces a safe steering
envelope, speed limit, and path-viability result. Fusion MUST be:

```text
applied_steering = clamp(
    camera_steering_request,
    nav2_safe_steering_min,
    nav2_safe_steering_max
)

applied_speed_mps = min(
    pursuit_speed_request_mps,
    nav2_speed_cap_mps,
    alignment_speed_cap_mps,
    obstacle_speed_cap_mps,
    thermal_speed_cap_mps
)
```

Camera and Nav2 steering MUST NOT be added or combined by weighted sum.
`DecisionEngine` performs the clamp and remains sole drivetrain authority.

## 10. Geofence and navigation completion

- Car geofence crossing by the localized lidar/`base_link` center enters
  `FAILSAFE` immediately.
- There is no predictive path-veto transition based solely on a planned path
  approaching the polygon.
- Cat exit is declared only by overhead mission event, never inferred from the
  car geofence.

`GOTO` and `RETURN_HOME` complete only when:

1. correlated Nav2 result is `SUCCEEDED`;
2. fresh authoritative local pose is within `20 cm` XY and `0.3 rad` yaw;
3. both tolerances hold continuously for `1 second`.

After configured Nav2 retries are exhausted:

- `GOTO -> IDLE`
- chase states -> `RETURN_HOME` if safe, otherwise `FAILSAFE`
- `RETURN_HOME -> FAILSAFE`

## 11. Startup sequence

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
but MUST NOT continuously overwrite authoritative local localization.

## 12. Observability

Log at minimum:

- FSM transitions with monotonic timestamps;
- command/event IDs, ACK results, and applied control sequence;
- overhead observation sequence, selected `target_id`, and staleness;
- local track association evidence and lock resets;
- dual-sensor health, hold timers, and veto reasons;
- Nav2 goal intent/correlation IDs, safe envelope, and completion dwell;
- requested/applied physical speed caps and normalized drive command;
- perception consumer reference counts and camera hardware state;
- recording requested vs actual, quota, and low-space events;
- failsafe causes and clearance checks.

Telemetry MUST be machine-parseable (JSONL recommended) and sufficient to
reconstruct every transition, hold, retry, and applied command.

## 13. Validation scope

Target validation MUST cover:

- every command/event matrix cell, including rejection and idempotency;
- transactional ACK after control-loop application;
- wrong target, stale sequence, duplicate event, and matching primary-left
  handoff behavior;
- dual-sensor hold/recovery/failsafe in every driving state and immediate
  reverse failure;
- non-additive steering clamp and speed-cap fusion;
- `<15 cm` `BRAKE_REVERSE` phases, attempts, preemption, and clearance reset;
- lifecycle-table rows, headless SEARCH/CHASE operation, and H.264-only
  monitoring behavior;
- durable/versioned `SET_HOME`, mission home freeze, and RF2O-only startup
  localization acceptance.

Detailed acceptance cases live in the canonical target redesign and the
validation matrix.

## 14. Implementation staging

Expected target implementation areas include:

- `cat_follow/control/types.py`
- `cat_follow/control/fsm.py`
- `cat_follow/control/decision_engine.py`
- `cat_follow/comms/messages.py`
- `cat_follow/comms/comms_manager.py`
- `cat_follow/runtime/app.py`
- `cat_follow/runtime/control_loop.py`
- new `NavigationManager` and `PerceptionLifecycleManager` components
- `cat_follow/navigation/ros_bridge.py`
- `cat_follow/threads/camera.py`
- `cat_follow/threads/detector.py`
- `cat_follow/perception/h264_encoder.py`
- `cat_follow/web_ui/routes_h264.py`
- `ros_ws/cat_follow_bringup/config/nav2_params.yaml`

No partial deployment may advertise this document as current behavior until the
complete FSM, protocol, safety, navigation, lifecycle, and validation
requirements are implemented and accepted.

## 15. Current vs target migration

The current repository does **not** implement this target. Known conflicts
include:

1. **`cat_follow/control/types.py` and `cat_follow/control/fsm.py`** still use
   the old `CHASE_A`, `TRACK_B`, and `BRAKE` model and lack the canonical
   states and complete transitions (`GETTING_CLOSE`, `SEARCH`, `CHASE`,
   `BRAKE_REVERSE`, handoff semantics, and mission-context fields).
2. **Close-obstacle behavior** currently enters `FAILSAFE` around 10 cm rather
   than executing formal `<15 cm` `BRAKE_REVERSE`.
3. **Chase overhead expiry** currently uses an approximately 700 ms failsafe
   rather than state-specific retained-goal, local-track, and return behavior.
4. **`NavigationManager`** does not exist; the ROS bridge lacks complete moving
   goal output, refresh, cancel, correlation, safe envelope, and completion
   behavior.
5. **Chase/navigation authority** is advisory/incomplete and does not implement
   the non-additive camera-request clamp inside a Nav2 safe steering envelope.
6. **Camera lifecycle** is effectively always active rather than managed by
   named consumers, reference counts, and STREAMOFF/STREAMON readiness.
7. **Detector activation** is primarily PhaseMachine/motion-gated rather than
   mission-policy-required in `SEARCH`/`CHASE` and requested `GOTO`.
8. **Hardware H.264 recording**, storage quota, reserve, crash recovery,
   post-roll, and degraded retry are not implemented.
9. **Protocol** lacks stable `target_id` on overhead cats/selection and chase
   commands, plus the reliable ACKed `mission_event` envelope.
10. **Durable versioned home**, active-mission home freezing, and transactional
    `SET_HOME` acceptance are incomplete or absent.
11. **Dual-required-sensor** 2-second hold, stationary degradation, and
    reverse-immediate-fail rules are not implemented.
12. **Ultrasonic costmap integration** through validated `RangeSensorLayer` is
    incomplete.

### Legacy `main_loop.py`

The legacy prototype path in `main_loop.py` is separate from the target
`runtime.app` architecture:

- it runs control behavior in the main thread at approximately 30 Hz;
- it still uses legacy ultrasonic polling via `range_sensor.set_car(Picarx)`;
- it does not implement the canonical FSM, `NavigationManager`,
  `PerceptionLifecycleManager`, transactional protocol, or dual-sensor target
  safety model.

Migration MUST treat `main_loop.py` as legacy/dev-only and MUST NOT use it as
the reference for production behavior. Target production runtime is
`runtime.app` with dedicated `CatFollow-Control`, RF2O-only odometry, RKNN-only
inference, and the module set listed above.

These are migration gaps, not permission to partially reinterpret the target.
Refer to the canonical target redesign for normative state tables, command
matrices, configuration defaults, and full validation requirements.
