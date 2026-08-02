# Product Requirements Document (PRD)
**Project:** Autonomous Yard Navigator and Cat Tracker  
**Target platform:** Radxa ROCK 4D with Radxa 4K IMX415 camera on a PiCar-X chassis  
**Version:** 2.0  
**Status:** Approved target requirements — not implemented yet  
**Date:** 2026-07-25  
**Normative source:** `Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`

## 1. Purpose and scope
The product is a headless autonomous yard vehicle that follows a specifically
selected cat, navigates to operator destinations, and returns to a durable home
pose. An overhead service supplies strategic yard observations and target
identity; onboard perception and ROS 2/Nav2 provide local pursuit and safe
navigation.

This PRD describes the required target product. It does **not** describe the
current executable as implemented. If this document and the canonical target
document differ, the canonical target document controls.

## 2. Production platform
The supported production configuration is:

- Radxa ROCK 4D compute;
- Radxa 4K IMX415 onboard camera;
- RKNN inference only, with no TFLite or other inference fallback;
- ROS 2/Nav2 local navigation;
- RF2O lidar odometry only;
- Slamtec RPLIDAR C1 plus forward ultrasonic sensing;
- an overhead yard camera/service for strategic car and cat observations;
- PiCar-X drive motors and steering.

Bicycle/wheel odometry is unsupported and MUST NOT be used as a fallback.
Autonomous motion requires both fresh, valid lidar and fresh, valid ultrasonic
data.

## 3. Product requirements

### 3.1 Authority and identity
- `DecisionEngine` MUST be the sole drivetrain decision authority. Perception,
  communications, Nav2, and web components MUST NOT write motor or steering
  commands directly.
- Every chase MUST be scoped to a stable, nonempty `target_id`. Overhead
  observations, chase commands, local-track association, events, ACKs, and
  telemetry MUST preserve that identity.
- Only the overhead system may declare
  `PRIMARY_CAT_LEFT_PERIMETER(target_id)`. Onboard visibility, local tracking,
  and the car geofence MUST NOT infer or override that event.
- A matching primary-cat-exit event MUST stop motion, cancel navigation or
  reverse, enter `IDLE`, and begin the configurable 10-second handoff wait.
  Overhead selects any replacement target. Timeout returns home when safe or
  enters `FAILSAFE`.
- Emergency stop and hard safety vetoes MUST preempt every mission behavior.

### 3.2 Target mission FSM
The target FSM consists of:

| State | Required purpose |
|---|---|
| `HOME` | Stopped at the durable home pose within completion tolerance. |
| `IDLE` | Stopped and ready, including target-handoff wait. |
| `GETTING_CLOSE` | Follow the selected overhead target with Nav2. |
| `SEARCH` | Search near that target while acquiring a verified local lock. |
| `CHASE` | Pursue the associated local track within Nav2 safety constraints. |
| `BRAKE_REVERSE` | Execute bounded close-obstacle recovery and preserve the interrupted objective. |
| `GOTO` | Execute an operator destination through Nav2. |
| `RETURN_HOME` | Navigate to the frozen mission home. |
| `FAILSAFE` | Latched zero-motion state requiring cause-specific operator clearance. |

There is no `CHASE_A`, `TRACK_B`, final `BRAKE`, or “arrived at cat” state.
At a default overhead distance of `<= 200 cm`, `GETTING_CLOSE` enters
`SEARCH`. Three consecutive unambiguous, identity-associated local
observations enter `CHASE`.

### 3.3 Navigation and pursuit
- `NavigationManager` MUST own Nav2 `NavigateToPose` clients, frame transforms,
  goal generation and refresh, cancellation/preemption, action correlation,
  retries, path viability, safe steering envelopes, and completion
  qualification.
- Startup MAY use a fresh overhead car pose to seed or validate ROS
  localization. After acceptance, local Nav2/SLAM localization using RF2O is
  authoritative.
- Camera pursuit supplies a steering request. `DecisionEngine` MUST clamp that
  request to the Nav2 safe steering envelope. Camera and Nav2 steering MUST
  NOT be added or combined by weighted sum.
- `GOTO` and `RETURN_HOME` complete only after correlated Nav2 success plus a
  fresh local pose within `20 cm` XY and `0.3 rad` yaw continuously for one
  second.
- Nav2 BackUp recovery MUST remain disabled.

### 3.4 Overhead and local-perception degradation
- In `GETTING_CLOSE` and `SEARCH`, invalid overhead data MAY retain the last
  validated goal for at most 10 seconds at the default `0.10 m/s` SEARCH cap,
  while all local navigation and safety permissions remain valid. This replaces
  the old 700 ms blanket failsafe.
- In `CHASE`, overhead loss has no fixed timeout while the associated local
  track, localization/geofence, navigation, lidar, and ultrasonic remain valid.
- Loss of the associated local track transitions directly to `SEARCH` when the
  valid overhead distance is `<= 200 cm`, or directly to `GETTING_CLOSE` when
  it is greater. If overhead is also unavailable, return home when safe or
  enter `FAILSAFE`.
- Fatal onboard camera or RKNN failure degrades `SEARCH` or `CHASE` to
  overhead-only `GETTING_CLOSE`; it MUST NOT silently select another target.

### 3.5 Safety and `BRAKE_REVERSE`
Both lidar and ultrasonic are independent, mandatory direct safety inputs to
`DecisionEngine`, even when integrated into Nav2.

During normal autonomous driving, loss of either sensor MUST stop motion while
preserving the objective. Motion may resume only if both recover within the
configurable default of two seconds; otherwise the system enters `FAILSAFE`.
Loss of either sensor during `BRAKE_REVERSE` enters `FAILSAFE` immediately.

A fresh valid reading from either required sensor strictly below the
configurable `CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM` default of `15 cm` MUST
enter `BRAKE_REVERSE`, including when the nearby object is the tracked cat.
Each attempt performs:

1. stop and confirm zero drive;
2. center steering;
3. settle stopped for 100 ms;
4. reverse straight for 0.5 seconds at normalized `-0.30`;
5. stop;
6. recheck both sensors, health, commands, events, localization, geofence, and
   the saved objective.

At most three reverse attempts are allowed. Exhaustion enters `FAILSAFE`.
Attempt count resets only after both sensors remain fresh, valid, and strictly
above `20 cm` for two continuous seconds.

### 3.6 Persistent home
- `SET_HOME` MUST be accepted only while stopped in `HOME` or `IDLE`, with
  valid localization, map, transforms, calibration, and geofence containment.
- Home MUST be versioned, checksummed, associated with map/calibration, and
  durably committed before ACK.
- Mission acceptance MUST freeze the durable home version. Home MUST NOT be
  changed during an active mission.
- `RETURN_HOME` uses the frozen home; it does not require home coordinates in
  each command.

### 3.7 Perception, recording, and monitoring
`PerceptionLifecycleManager` MUST own independent, reference-counted
`detector`, `recording`, and `stream` consumers and the IMX415 hardware
lifecycle. Detection and tracking MUST work headlessly without a browser or
monitoring client.

- `SEARCH` and `CHASE` require RKNN detection. `GETTING_CLOSE` does not.
- `GOTO` independently requests YOLO and recording; neither is implicit.
- Chase recording follows mission policy through search, chase, reverse,
  return, and the configured 10-second post-roll.
- Recording MUST use hardware H.264 in segmented, crash-tolerant Matroska
  files with quota, free-space reserve, and oldest-finalized-segment cleanup.
- Monitoring MUST use hardware H.264 only and run only for actual connected
  clients. There is no MJPEG or software-encoding fallback.
- Recording and monitoring are independent. Their failure degrades telemetry
  but MUST NOT change FSM state or authorize/veto motion.
- `HOME` and `FAILSAFE` force all consumers off and leave the camera
  ready-inactive or closed.

## 4. Reliable protocol requirements
Protocol V1 remains the target envelope version until overhead implementation
and interoperability validation are complete.

- Non-emergency commands and mission events MUST be identified, deduplicated,
  queued, atomically applied at a control-loop boundary, and ACKed only after
  application or rejection.
- Overhead observations MUST contain stable cat `target_id` values and a
  `selected_target_id`; array position MUST NOT define identity.
- `START_CHASE(target_id)` requires a valid durable home, selected-target
  observation, localization/calibration, healthy `NavigationManager`, and both
  required sensors.
- The ACKed `PRIMARY_CAT_LEFT_PERIMETER(target_id)` event MUST carry an event
  ID, mission ID, perimeter ID, and observation sequence and reject wrong,
  duplicate, regressive, or stale events without changing state.

## 5. Configuration and observability
Target configuration MUST use validated `CAT_FOLLOW_*` settings. Canonical
defaults, including the configurable 15 cm reverse trigger, are defined by the
normative target document.

Telemetry MUST make command/event correlation, `target_id`, home version,
sensor source/freshness, safety holds, FSM transitions, navigation goals,
perception consumers, recording/stream status, requested motion, applied caps,
final commands, and every zero-motion reason reconstructable.

## 6. Acceptance criteria
Release requires automated, fault-injection, long-duration, and production-car
validation of:

- every FSM command/event transition and safety preemption;
- target identity and overhead-only cat-exit authority;
- RF2O-only localization and all `NavigationManager` goal lifecycle rules;
- dual-required-sensor holds, escalation, and every `BRAKE_REVERSE` phase;
- durable home persistence and mission freezing;
- headless RKNN detection and complete perception lifecycle behavior;
- hardware H.264 recording retention/recovery and client-only monitoring;
- operation across production surface, payload, battery, thermal, and steering
  calibration ranges.

## 7. Current implementation status
These are target requirements, not implementation claims. As of 2026-07-25,
the repository still contains substantial legacy behavior, including the old
`CHASE_A`/`TRACK_B`/`BRAKE` FSM, approximately 10 cm close-obstacle failsafe,
approximately 700 ms overhead-stale failsafe, incomplete Nav2 goal authority,
always-active camera behavior, missing stable `target_id`/mission-event
contracts, and missing target `NavigationManager`,
`PerceptionLifecycleManager`, recording, durable-home, and sensor-health
semantics.

The target MUST NOT be advertised as implemented until the canonical
specification's migration and validation requirements are complete.
