# High-Level Design (HLD)
**Project:** Autonomous Yard Navigator and Cat Tracker  
**Target platform:** Radxa ROCK 4D with Radxa 4K IMX415 camera on a PiCar-X chassis  
**Based on:** `PRD_Autonomous_Yard_Navigator_Cat_Tracker.md`  
**Normative source:** `Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`  
**Version:** 2.0  
**Status:** Approved target design — not implemented yet  
**Date:** 2026-07-25

## 1. System overview
This is the high-level design for a future multi-layer robotics runtime. It
combines overhead strategic tracking, RKNN-only onboard perception, ROS 2/Nav2
navigation, RF2O lidar odometry, lidar and ultrasonic safety, durable mission
home, and centralized drivetrain arbitration.

This document specifies the target architecture and does **not** claim the
current executable implements it. The canonical target document controls any
conflict.

## 2. Core Design Principle
Only one module decides final motion: `DecisionEngine`.

All other modules either:
- observe the environment
- report state
- provide constraints
- execute a validated command

The FSM stores and validates the active mode, but it does not independently choose motion.

## 3. High-level pipeline
```text
Overhead service -> CommsManager -----------------------> SharedState
                         | target_id / reliable events        |
IMX415 -> PerceptionLifecycleManager -> RKNN detector --------|
Lidar C1 -> RF2O / lidar safety -------------------------------|
Ultrasonic -> RangeSensorLayer / direct safety ----------------|
ROS 2/Nav2 <-> NavigationManager ------------------------------|
Durable home store --------------------------------------------|
                                                               v
                                                  DecisionEngine + FSM
                                                               |
                                                               v
                                                        MotorInterface

Recording H.264 <-> PerceptionLifecycleManager <-> client-only H.264 stream
```

The web UI and monitoring stream are optional. Detection, tracking, recording,
navigation, and control MUST operate headlessly.

## 4. Authority hierarchy
`DecisionEngine` is the sole drivetrain decision authority. Sensor, camera,
communications, Nav2, recording, and web modules provide facts, intent, or
constraints but never write drivetrain commands.

Control precedence is: synchronous emergency stop; actual car-geofence breach;
motor/control fatal or watchdog; latched `FAILSAFE`; reverse safety/exhaustion
and preemption; active `BRAKE_REVERSE`; required-sensor health; critical
thermal return; mission events and commands; navigation/perception degradation;
Nav2 path constraints; state motion policy; then recording/stream lifecycle.
Lower-priority input cannot reauthorize vetoed motion.

## 5. Hardware Layer
- **Compute:** Radxa ROCK 4D
- **Chassis:** SunFounder PiCar-X
- **Global tracking:** overhead camera system
- **Local vision:** Radxa 4K IMX415
- **Inference:** RKNN only
- **Odometry:** RF2O lidar odometry only; bicycle/wheel odometry is unsupported
- **Local range/safety:** Slamtec RPLIDAR C1 plus forward ultrasonic; both are required for autonomous motion
- **Actuation:** PiCar-X drive motors and steering servo
- **Power:** 2x 18650 high-discharge cells

## 6. Coordinates and timing
- Yard positions and perimeter geometry use calibrated centimeters.
- Nav2 speeds and navigation telemetry use physical meters per second.
- Direct bounded reverse uses a normalized motor command and duration.
- Yard frame:
  - `+X` = right in yard
  - `+Y` = forward in yard
- Overhead host and PiCar-X should use NTP/Chrony for cross-device log correlation.
- Safety/control freshness and durations use local monotonic time.
- Yard-to-map transforms are calibrated and versioned; yard coordinates MUST
  NOT be treated as ROS map coordinates directly.

## 7. Runtime Modules

### 7.1 `CommsManager`
Receives overhead observations, commands, and reliable mission events.

Responsibilities:
- validate protocol, freshness, confidence, calibration, sequence, and identity
- preserve stable per-cat `target_id` and `selected_target_id`
- deduplicate commands/events and queue them for atomic control-loop application
- ACK only the committed applied or rejected result
- retain mission-event deduplication across restart for the protocol window

Outputs:
- overhead car and cat observations with sequence and receive time
- selected `target_id`, command/event records, and communication health

Only the overhead system may issue the reliable
`PRIMARY_CAT_LEFT_PERIMETER(target_id)` event. Local perception cannot infer or
override it. Overhead pose may seed startup localization; accepted local
Nav2/SLAM localization is authoritative afterward.

### 7.2 `VisionTracker`
Consumes IMX415 frames and RKNN detections. There is no alternate inference
backend.

Responsibilities:
- report fresh local tracks, bearing, confidence, and last-seen time
- associate a local track with the overhead-selected `target_id` using
  calibrated bearing and uncertainty
- require three consecutive unambiguous associated observations for lock
- preserve identity and reject ambiguous or competing associations

Outputs:
- local track ID, associated `target_id`, bearing/steering request, confidence,
  freshness, and ambiguity status

### 7.3 `RangeSafety`
Reads lidar and ultrasonic inputs and produces independent health, range, and
safety facts for each source.

Responsibilities:
- validate freshness and fault state independently
- feed both sensors directly to `DecisionEngine`
- publish validated ultrasonic `sensor_msgs/Range` into a Nav2
  `RangeSensorLayer` while retaining direct safety authority
- trigger `BRAKE_REVERSE` when either fresh valid reading is strictly below
  the configurable default of 15 cm

Range does not identify the cat. Any close object, including the tracked cat,
uses the same `BRAKE_REVERSE` policy.

### 7.4 `NavigationManager`
Owns all Nav2 mission integration:

- `NavigateToPose` clients and state-derived goal intents;
- calibrated yard-to-map transforms;
- moving-target goal refresh, cancellation, preemption, and correlation;
- retries, expected replacement classification, and exhausted failure;
- path viability, safe steering envelope, and speed cap;
- completion qualification using correlated success plus fresh local pose.

Moving target goals default to at most 2 Hz and at least 25 cm displacement.
Safety cancellation bypasses rate limiting. Nav2 BackUp remains disabled.
RF2O is the only supported odometry source.

### 7.5 `PerceptionLifecycleManager`
Owns the IMX415 device and independent reference-counted consumers:

- `detector`;
- `recording`;
- `stream`.

It implements verified ready-inactive `STREAMOFF`/`STREAMON` behavior without
busy-looping. Mission-required detection or recording never depends on stream
clients. `HOME` and `FAILSAFE` force all consumers off.

### 7.6 `DecisionEngine`
The central decision maker.

Responsibilities:
- select requested state transitions
- perform sensor arbitration
- enforce safety precedence
- compute final speed/steering/brake request
- select look/drive mode (pan look-at vs body vision steer inside Nav2 envelope;
  see `Look_Drive_Path_Design.md`)
- manage formal `BRAKE_REVERSE` output and saved-objective restoration
- emit decision reasons for telemetry

Inputs:
- overhead state from `CommsManager`
- local visual state from `VisionTracker`
- range/obstacle state from `RangeSafety`
- navigation constraints from `NavigationManager`
- current FSM state
- thermal/system health state

Output:
```json
{
  "requested_state": "HOME | IDLE | GETTING_CLOSE | SEARCH | CHASE | BRAKE_REVERSE | GOTO | RETURN_HOME | FAILSAFE",
  "steering": 0.0,
  "speed_mps": 0.0,
  "normalized_drive": 0.0,
  "reason": "string",
  "look": {
    "look_drive_mode": "PATH_FOLLOW | LOOK_AT | PAN_RESET | BODY_STEER | HOLD",
    "pan_deg": 0.0,
    "pan_forward_deg": 0.0,
    "look_reason": "string",
    "pixel_error_px": 0.0,
    "camera_request": 0.0
  }
}
```

### 7.7 `FSM`
The FSM is the validated mode holder and execution wrapper.

Responsibilities:
- store current state
- validate requested transitions from `DecisionEngine`
- reject invalid transitions
- expose current mode to other modules

The FSM cannot change state without a `DecisionEngine` request, except for hard failsafe paths required by process-level safety handling.

States:
| State | Role |
|---|---|
| `HOME` | Stopped at the durable home pose |
| `IDLE` | Stationary and ready, including target handoff |
| `GETTING_CLOSE` | Nav2 pursuit of the selected overhead target |
| `SEARCH` | Slow local-lock acquisition near that target |
| `CHASE` | Associated local-track pursuit within Nav2 constraints |
| `BRAKE_REVERSE` | Bounded close-obstacle recovery with saved objective |
| `GOTO` | Nav2 operator destination |
| `RETURN_HOME` | Navigation to the frozen mission home |
| `FAILSAFE` | Latched zero-motion safety state |

There is no `CHASE_A`, `TRACK_B`, final `BRAKE`, or arrived-at-cat state.

### 7.8 `MotorInterface`
The only module that writes to drivetrain hardware.

Responsibilities:
- execute validated speed/steering commands
- perform controlled stop
- perform emergency stop
- prevent direct motor access from perception or navigation modules

### 7.9 Durable home store
Persists a checksummed, versioned home record associated with map and
calibration versions. `SET_HOME` commits before ACK and is accepted only while
stopped in `HOME` or `IDLE`. Mission acceptance freezes the home version;
active missions cannot change it.

### 7.10 `TelemetryLogger`
Records state, sensor, decision, and safety events.

Logs should go to systemd journal for service visibility and to structured JSONL telemetry for replay/tuning.

## 8. Chase behavior

### 8.1 `GETTING_CLOSE`
`NavigationManager` follows the selected overhead `target_id` as a filtered
moving Nav2 goal. RKNN detection is not required. Valid target distance at or
below the configurable default of 200 cm enters `SEARCH`.

### 8.2 `SEARCH`
RKNN detection runs continuously at mission cadence. Three consecutive
unambiguous observations passing the overhead/local association gate bind the
local track to `target_id` and enter `CHASE`. The first configurable 10-second
timeout permits at most one safe observation waypoint; the second returns home
when safe or enters `FAILSAFE`.

### 8.3 `CHASE`
The associated local track produces pursuit steering. Nav2 supplies path
viability, a safe steering envelope, and speed caps. Local-track loss with
valid overhead transitions directly to `SEARCH` at `<= 200 cm`, otherwise
directly to `GETTING_CLOSE`.

## 9. Control Fusion
The `DecisionEngine` clamps rather than adds steering influences:
```text
applied_steering = clamp(
    camera_steering_request,
    nav2_safe_steering_min,
    nav2_safe_steering_max
)
```

Additive or weighted camera/Nav2 steering is forbidden. Path viability and all
safety vetoes remain mandatory.

Speed is bounded by:
```text
speed = min(
    pursuit_speed,
    alignment_speed_limit,
    obstacle_distance_limit,
    thermal_speed_limit
)
```

## 10. `BRAKE_REVERSE`
A fresh valid lidar or ultrasonic reading strictly below
`CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM` (default 15 cm) enters
`BRAKE_REVERSE` from any normal driving state. Entry saves the interrupted
objective, navigation intent, phase, attempt count, and perception policy.

Each attempt is `STOP -> CENTER -> SETTLE 100 ms -> REVERSE 0.5 s at
normalized -0.30 -> STOP -> RECHECK`. Steering remains centered during settle
and reverse. Both required sensors, health, commands/events, geofence,
localization, and the saved objective are rechecked before later motion.

There are at most three attempts. A blocked third recheck enters `FAILSAFE`.
The count resets only when both sensors stay fresh, valid, and strictly above
20 cm for two seconds. Loss of either sensor during any reverse phase enters
`FAILSAFE` immediately.

## 11. Timeout Policy
- In `GETTING_CLOSE`/`SEARCH`, retain the last validated overhead goal for at
  most 10 seconds at the default 0.10 m/s SEARCH cap while all permissions
  remain valid; then return home when safe or fail.
- In `CHASE`, overhead loss has no fixed timeout while the associated local
  track and every navigation/safety permission remain valid.
- Required lidar or ultrasonic loss stops motion and preserves the objective;
  recovery within two seconds resumes, otherwise enter `FAILSAFE`.
- The configured local-track stale default is 350 ms; loss follows the direct
  state transitions in Section 8 rather than an old staged fallback.
- Nav2 retries and objective-specific exhaustion replace generic inherited
  no-progress timers.

## 12. Thermal policy
Thermal thresholds and speed/compute profiles are deployment-configured and
validated. A critical condition replaces any active objective other than
`RETURN_HOME` with `RETURN_HOME` when safe return is possible, otherwise
`FAILSAFE`. `RETURN_HOME` may continue under the critical-return profile only
while safe; `HOME` and `IDLE` remain stopped and degraded.

## 13. Shared State Model
Modules publish into a synchronized snapshot consumed by the `DecisionEngine`.

The exact timestamp, freshness, authority, and confidence fields for each group are defined in the Detailed Software Architecture and the future Interface & Data Contract Specification. The HLD schema below is illustrative only.

The synchronized snapshot includes mission identity and correlation, not just
unscoped cat coordinates. The exact schema is defined by the canonical target
and interface specification.

```json
{
  "car": {
    "x": 0.0,
    "y": 0.0,
    "heading": 0.0,
    "confidence": 1.0
  },
  "home": {
    "version": 7,
    "durable": true,
    "frozen_for_mission": true
  },
  "cat_global": {
    "target_id": "cat-17",
    "x": 0.0,
    "y": 0.0,
    "confidence": 1.0
  },
  "cat_local": {
    "track_id": "local-3",
    "associated_target_id": "cat-17",
    "bearing_rad": 0.0,
    "confidence": 1.0
  },
  "required_sensors": {
    "lidar_fresh_valid": true,
    "ultrasonic_fresh_valid": true
  },
  "system": {
    "navigation_goal_intent_id": "nav-42",
    "thermal_c": 0.0,
    "monotonic_time_ms": 0
  }
}
```

## 14. Protocol and target handoff
Protocol V1 commands and mission events are identified, deduplicated, queued,
atomically applied by the control loop, and ACKed after the result commits.
`START_CHASE` requires `target_id`; overhead observations carry stable cat IDs
and `selected_target_id`.

Only a fresh matching ACKed
`PRIMARY_CAT_LEFT_PERIMETER(target_id)` event may declare target exit. It stops
motion, cancels Nav2/reverse, enters `IDLE`, starts the 10-second handoff and
recording post-roll, and waits for a new overhead-selected target or explicit
return. Handoff timeout returns home when safe or enters `FAILSAFE`.

## 15. Perception, recording, and stream policy
The lifecycle policy is:

| State | Detector | Recording | Monitoring stream |
|---|---|---|---|
| `HOME` | Forced off | Forced off | Forced off |
| `IDLE` | Off | Post-roll/handoff only | Actual clients |
| `GETTING_CLOSE` | Off | Chase request | Actual clients |
| `SEARCH`, `CHASE` | Required on | Chase request | Actual clients |
| `BRAKE_REVERSE` | Inherit saved request | Inherit saved request | Actual clients |
| `GOTO` | Exactly `request_yolo` | Exactly `request_recording` | Actual clients |
| `RETURN_HOME` | Off | Retain active request | Actual clients |
| `FAILSAFE` | Forced off | Forced off | Forced off |

Recording uses a separate hardware H.264 encoder and segmented,
crash-tolerant Matroska files with quota, minimum free-space reserve, and
oldest-finalized-segment cleanup. It operates without a stream client and
automatically retries while requested after health or space recovery.

Monitoring uses hardware H.264 only and runs only for actual clients. There is
no MJPEG or software-encoding fallback. Recording/stream failures only degrade
telemetry; they do not change FSM state or motion authority.

## 16. Observability
The system logs:
- state transitions
- sensor snapshots
- overhead packet freshness
- active `target_id`, local association, and overhead exit events
- lidar and ultrasonic health, holds, and reverse phases
- Nav2 goal intent/action correlation and completion checks
- home persistence and frozen version
- detector/recording/stream requested and actual consumers
- thermal events
- decision reasons
- motor command outputs
- failsafe triggers

Logs should be written to:
- systemd journal for service-level operations
- structured JSONL telemetry for replay and tuning

## 17. Current implementation status
This architecture is a migration target. As of 2026-07-25, the repository
still uses substantial legacy behavior: `CHASE_A`/`TRACK_B`/`BRAKE`, an
approximately 10 cm close-obstacle failsafe, approximately 700 ms
overhead-stale failsafe, incomplete Nav2 goal authority without the required
safe-envelope clamp, always-active camera operation, and incomplete target ID,
mission event, durable home, dual-sensor health, recording, and lifecycle
support.

`NavigationManager` and `PerceptionLifecycleManager` are required target
boundaries, not claims about existing classes. The system MUST NOT be
represented as implementing this HLD until canonical migration and validation
requirements pass.
