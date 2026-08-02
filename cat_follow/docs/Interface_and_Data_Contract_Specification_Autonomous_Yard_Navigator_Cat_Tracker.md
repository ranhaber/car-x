# Interface and Data Contract Specification

**Project:** Autonomous Yard Navigator and Cat Tracker  
**Target hardware:** Radxa ROCK 4D with Radxa 4K IMX415 camera  
**Document version:** 2.0  
**Protocol version:** V1  
**Status:** Approved target contract — **not implemented yet**  
**Date:** 2026-07-26  
**Canonical behavior authority:** `Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`

## 1. Purpose and authority

This document defines the concrete Protocol V1 wire schemas, shared-state contracts,
command and mission-event semantics, `NavigationManager` goal-intent contracts,
perception lifecycle status, geofence and sensor-health fields, telemetry, and
synchronization rules for the autonomous yard navigator.

It is the approved **target** interface contract. Nothing in this document claims
that the current executable implements it. Section 18 lists known implementation
gaps.

For the behavior covered here, this document supersedes conflicting requirements in
older drafts of this file and in other repository documents. When this document
conflicts with `Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`,
the target redesign document wins.

The following prior interface semantics are explicitly superseded and MUST NOT appear
as normative requirements in new implementations:

- `tracking` message type and single-cat `cat` object without stable `target_id`;
- `CHASE_A`, `TRACK_B`, and final-stop `BRAKE` FSM states;
- fixed `10 cm` obstacle `FAILSAFE` as the close-obstacle policy;
- immediate `700 ms` overhead-expired failsafe during chase;
- bicycle/wheel odometry fallback;
- additive or weighted camera/Nav2 steering;
- software/MJPEG monitoring fallback;
- predictive geofence path veto;
- ACK that reports a destination state before the control loop has applied it.

## 2. Normative language and units

`MUST`, `MUST NOT`, `SHOULD`, and `MAY` are normative.

### 2.1 Unit rules

| Domain | Unit | Notes |
|---|---|---|
| Overhead yard positions and perimeter geometry | centimeters (`cm`) | Calibrated yard frame |
| Operator command geometry (`SET_HOME`, `GO_TO`) | centimeters (`cm`) | Yard frame; keys are `x_cm` / `y_cm` |
| Overhead headings | radians | Non-authoritative for motion after startup |
| Nav2 speed policy, caps, requests, navigation telemetry | physical meters per second (`m/s`) | Authoritative for navigation motion policy |
| Direct reverse maneuver | normalized motor command + bounded duration | Sole specified direct normalized/time-bounded maneuver |
| Applied drivetrain output to hardware | normalized `-1.0..1.0` | `MotorInterface` converts using calibrated mapping |
| Durations and freshness | monotonic time | Safety freshness MUST NOT depend on synchronized wall clock |
| ROS navigation-frame poses | meters and radians | Nav2 message requirement; converted once from `cm` at the navigation boundary |

### 2.2 Freshness definition

“Fresh and valid” means within the configured age limit, structurally valid,
finite, in range, and not faulted.

For every incoming message or sensor sample, the runtime records:

- producer timestamp when available (`*_at_ms`, `timestamp_ms`, or ROS header stamp);
- `received_ms`: PiCar-X local monotonic receive timestamp.

Freshness is computed locally:

```text
age_ms = now_monotonic_ms - received_ms
```

Rules:

- Use producer timestamps for cross-device log correlation only.
- Use `received_ms` for safety freshness, timeout, hold, and failsafe decisions.
- If NTP/Chrony is unavailable or clocks drift, control behavior MUST remain safe
  because freshness uses monotonic receive time.

## 3. Protocol V1 model

### 3.1 Message classes

Protocol V1 uses separate message types:

| Type | Purpose | ACK required | Retry |
|---|---|---|---|
| `overhead_observation` | Yard-level car and cat observations | No | No |
| `command` | Operator/overhead mission commands | Yes | Yes |
| `mission_event` | Reliable overhead mission events | Yes | Yes |
| `ack` | Application result for command or mission event | No | No |

There is no `tracking` message type in the target contract.

### 3.2 Versioning

Every protocol message includes:

```json
{
  "protocol_version": 1
}
```

Protocol remains **V1** because no upstream overhead implementation has been
deployed. This redesign redefines V1 before first deployment. Backward compatibility
with the repository's incomplete draft schema is not a requirement.

### 3.3 Transport reliability

#### 3.3.1 Lossy observations

`overhead_observation` messages are fast, lossy, latest-wins updates.

Rules:

- No ACK and no retry.
- Duplicate or regressive `observation_seq` values are ignored for control authority.
- Missing packets are tolerated until freshness and invalid-retention rules apply.
- The receiver stores the latest accepted observation per sender.

Recommended nominal rate: `10 Hz`.

#### 3.3.2 Reliable commands and mission events

`command` and `mission_event` messages are reliable transactions.

Rules:

- Sender retries until an ACK is received.
- Retry reuses the same `command_id` or `event_id`.
- Duplicate IDs return the stored committed result without re-executing side effects.
- Deduplication results MUST be retained durably enough to prevent replay after a
  process restart within the defined protocol retention window.

Recommended retry defaults:

| Name | Default |
|---|---:|
| `CAT_FOLLOW_COMMAND_ACK_TIMEOUT_MS` | `200` |
| `CAT_FOLLOW_COMMAND_MAX_RETRIES` | `5` |
| `CAT_FOLLOW_COMMAND_ID_CACHE_SIZE` | `100` |

When UDP transport has `CAT_FOLLOW_COMMS_TOKEN` configured, every command and
mission-event datagram MUST include a matching top-level JSON `token`. Missing or
invalid tokens are dropped before parsing and receive no ACK. Observation datagrams
do not require the command token.

### 3.3.3 Transactional application

All commands and mission events except emergency stop:

1. are received and deduplicated by `command_id` or `event_id`;
2. are queued for the control loop;
3. are validated and applied atomically at a control-loop boundary;
4. are ACKed only after application or rejection is committed;
5. report the resulting FSM state and machine-readable reason.

An ACK MUST NOT claim a destination state before the control loop has applied it.
Emergency stop remains synchronous and is not delayed for transactional mission
processing.

Every ACK MUST include `applied_control_seq`, the monotonic control-loop sequence
number at which the result became committed.

Duplicate command or event retries MUST return the stored result and the original
`applied_control_seq`.

## 4. Overhead observation schema

### 4.1 Schema

```json
{
  "protocol_version": 1,
  "type": "overhead_observation",
  "observation_seq": 1842,
  "observed_at_ms": 1785012345678,
  "perimeter_id": "yard-v3",
  "calibration_version": 7,
  "car": {
    "x_cm": 412.4,
    "y_cm": 226.8,
    "yaw_rad": 1.12,
    "confidence": 0.96
  },
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

### 4.2 Field rules

| Field | Required | Rules |
|---|---|---|
| `observation_seq` | Yes | Monotonic per overhead sender; used for dedup, invalidation, and event correlation |
| `observed_at_ms` | Yes | Producer observation time for log correlation |
| `perimeter_id` | Yes | Stable identifier for the cat perimeter definition |
| `calibration_version` | Yes | Must match an accepted runtime calibration version |
| `car.x_cm`, `car.y_cm` | Yes | Calibrated yard position in centimeters |
| `car.yaw_rad` | Yes | Non-authoritative after startup localization acceptance |
| `car.confidence` | Yes | Must be `>= CAT_FOLLOW_OVERHEAD_MIN_CONFIDENCE` to be valid |
| `cats[]` | Yes | MAY be empty; each cat MUST have stable `target_id` |
| `cats[].target_id` | Yes | Stable identity; never inferred from array position |
| `cats[].inside_perimeter` | Yes | Cat-perimeter membership; separate from car geofence |
| `selected_target_id` | Yes when a chase target is selected | Names the overhead-selected target; null only when no target is selected |

Rules:

- Every cat MUST have a stable `target_id`.
- The selected target is named by `selected_target_id`, never by array position.
- Overhead owns yard-level observations, selected `target_id`, and declaration that
  the selected cat left its perimeter.
- Cat exit is declared only by the reliable `PRIMARY_CAT_LEFT_PERIMETER` mission
  event and MUST NOT be inferred from the car geofence.
- If `car.confidence` is below threshold, the car observation is invalid.
- If the active chase `target_id` is absent from `cats[]`, or its confidence is below
  threshold, that target observation is invalid.
- A fresh-but-invalid observation uses the same policy as stale overhead data.

### 4.3 Receiver rules

- Ignore duplicate or regressive `observation_seq` values for authority updates.
- Do not request retry for missing observation packets.
- Retain the latest structurally valid packet and its receive metadata.
- Compute yard distances and perimeter checks only from valid observations.
- During `GETTING_CLOSE` or `SEARCH`, invalid or stale overhead triggers
  last-valid-goal retention for at most `CAT_FOLLOW_OVERHEAD_INVALID_MAX_SEC`.
- During `CHASE`, overhead loss has no fixed timeout while the associated local
  track and all chase permissions remain valid.

## 5. Command schema

### 5.1 Envelope

```json
{
  "protocol_version": 1,
  "type": "command",
  "command_id": "cmd-9f24",
  "issued_at_ms": 1785012346000,
  "name": "START_CHASE",
  "args": {}
}
```

Field rules:

- `command_id`: stable transaction ID; retries reuse it.
- `issued_at_ms`: sender timestamp for log correlation.
- `name`: uppercase command name from Section 5.2.
- `args`: command-specific parameters; use `{}` when none are required.

Target-scoped commands MUST carry `target_id` in `args`. Commands with no target
semantics omit it.

### 5.2 Command names

| Name | Target scoped | Purpose |
|---|---|---|
| `SET_HOME` | No | Persist a durable versioned home record |
| `START_CHASE` | Yes | Begin overhead-guided pursuit of `target_id` |
| `STOP_CHASE` | SHOULD carry expected `target_id` | Stop active chase and enter handoff/post-roll `IDLE` |
| `GO_TO` | No | Navigate to an explicit Nav2 destination |
| `RETURN_HOME` | No | Navigate to frozen mission home |
| `CLEAR_FAILSAFE` | No | Leave latched failsafe after cause-specific clearance |
| `EMERGENCY_STOP` | No | Synchronous zero-motion failsafe; not transactional |

`STOP_CHASE` MUST be rejected as `WRONG_TARGET` when `args.target_id` names a
different active chase target than the FSM context.

### 5.3 `SET_HOME`

```json
{
  "protocol_version": 1,
  "type": "command",
  "command_id": "cmd-home-01",
  "issued_at_ms": 1785012346000,
  "name": "SET_HOME",
  "args": {}
}
```

With no `home` argument the current localized pose becomes home. An explicit
pose uses the yard frame in centimeters:

```json
"args": {
  "home": {"x_cm": 100.0, "y_cm": 200.0, "yaw_rad": 0.0, "frame_id": "yard"}
}
```

Acceptance rules:

- Accepted only in `HOME` or `IDLE`.
- Requires valid ROS localization, map, transforms, yard/ROS calibration, and car
  inside the car geofence.
- Requires the car to be stopped.
- Requires durable persistence to succeed before ACK.
- Rejected while a mission is active if it would mutate the frozen home version.

The persisted home record MUST include at least:

- `home_version`
- `checksum`
- `calibration_version`
- `map_id`
- `frame_id`
- `x`, `y` (yard centimeters) and `x_m`, `y_m`, `yaw_rad` (Nav2 metric pose)
- `persisted_at_ms`

### 5.4 `START_CHASE`

```json
{
  "protocol_version": 1,
  "type": "command",
  "command_id": "cmd-9f24",
  "issued_at_ms": 1785012346000,
  "name": "START_CHASE",
  "args": {
    "target_id": "cat-17"
  }
}
```

Acceptance rules:

- Accepted only from `HOME` or `IDLE`.
- Requires valid durable home.
- Requires nonempty `target_id`.
- Requires fresh valid overhead car observation and fresh valid overhead cat
  observation for the same `target_id`.
- Requires valid localization/map/calibration and car inside geofence.
- Requires healthy `NavigationManager`.
- Requires fresh valid lidar and ultrasonic.
- Requires chaseable target inside configured cat perimeter.
- On acceptance: freeze home version, request chase recording, enter `GETTING_CLOSE`.
- Onboard perception or recording MAY be unavailable at acceptance if
  overhead-only pursuit remains safe; mission starts degraded and reports unavailable
  facilities.

A new `START_CHASE(new_target_id)` received during handoff `IDLE` MUST identify the
intended new target and enter `GETTING_CLOSE`.

### 5.5 `STOP_CHASE`

```json
{
  "protocol_version": 1,
  "type": "command",
  "command_id": "cmd-stop-01",
  "issued_at_ms": 1785012350000,
  "name": "STOP_CHASE",
  "args": {
    "target_id": "cat-17"
  }
}
```

Acceptance rules:

- From `GETTING_CLOSE`, `SEARCH`, or `CHASE`: stop immediately, cancel Nav2, enter
  `IDLE`, retain recording for `CAT_FOLLOW_RECORDING_POSTROLL_SEC`.
- From `BRAKE_REVERSE`: accepted only when saved objective is chase; stop reverse,
  cancel Nav2, enter `IDLE`, start same post-roll.
- From `HOME` or `IDLE`: idempotent success with zero motion; cancel pending handoff;
  existing post-roll ends at its deadline.
- From `GOTO`, `RETURN_HOME`, or `FAILSAFE`: reject with `INVALID_STATE`.

### 5.6 `GO_TO`

```json
{
  "protocol_version": 1,
  "type": "command",
  "command_id": "cmd-a103",
  "issued_at_ms": 1785012350000,
  "name": "GO_TO",
  "args": {
    "target": {
      "x_cm": 210.0,
      "y_cm": -80.0,
      "yaw_rad": 0.0,
      "frame_id": "yard"
    },
    "request_yolo": false,
    "request_recording": true
  }
}
```

The destination is centimeters in the yard frame. `NavigationManager` performs
the single conversion to Nav2 meters. `x` / `y` without the `_cm` suffix are
accepted from older senders and carry the same centimeter units.

Acceptance rules:

- Accepted only from `HOME` or `IDLE`; any other state is rejected with
  `invalid_state` rather than acknowledged as accepted.
- Requires a finite destination, localization, geofence, `NavigationManager`,
  lidar, and ultrasonic validation.
- `request_yolo` and `request_recording` are independent booleans; neither is
  implicitly enabled by `GO_TO`.
- Monitoring streaming remains client-driven and is not implied by this command.

### 5.7 `RETURN_HOME`

```json
{
  "protocol_version": 1,
  "type": "command",
  "command_id": "cmd-ret-01",
  "issued_at_ms": 1785012355000,
  "name": "RETURN_HOME",
  "args": {}
}
```

Acceptance rules:

- In `HOME` within completion tolerance: idempotent success.
- From `IDLE`, any chase state, or `GOTO`: immediate stop/cancel then `RETURN_HOME`
  after safety validation.
- In `RETURN_HOME`: idempotent success; retain correlated goal.
- In `BRAKE_REVERSE`: stop reverse, cancel saved objective, set `RETURN_HOME`, and
  re-evaluate clearance before motion.
- In `FAILSAFE`: reject.
- If safe return cannot be established: enter `FAILSAFE`; do not remain motion-capable.

Home coordinates are read from the frozen durable home record; the command does not
carry inline home geometry.

### 5.8 `CLEAR_FAILSAFE`

```json
{
  "protocol_version": 1,
  "type": "command",
  "command_id": "cmd-clr-01",
  "issued_at_ms": 1785012360000,
  "name": "CLEAR_FAILSAFE",
  "args": {
    "operator_confirmed": true
  }
}
```

Acceptance requires explicit operator confirmation, cause-specific clearance for
every latched cause, stopped motor feedback, healthy control loop and watchdog,
fresh valid lidar and ultrasonic, and valid motion-inhibition output.

Acceptance enters clean `IDLE`, cancels/discards Nav2 goals, handoff context, and
interrupted objectives. It does not restart a mission, detector, recording, or stream.

The reverse attempt count does not reset merely because `CLEAR_FAILSAFE` was accepted.

### 5.9 `EMERGENCY_STOP`

```json
{
  "protocol_version": 1,
  "type": "command",
  "command_id": "cmd-estop-01",
  "issued_at_ms": 1785012365000,
  "name": "EMERGENCY_STOP",
  "args": {}
}
```

Rules:

- Processed synchronously outside the transactional queue.
- Commands zero motion immediately, cancels Nav2 and reverse output, enters `FAILSAFE`,
  and latches cause.
- ACK MAY be emitted after the synchronous stop is committed but MUST NOT wait for
  the next full control-loop transaction boundary.

### 5.10 Rejection reasons

Rejected command ACKs MUST use `applied: false`, retain the actual state, and include
a specific `reason`.

Recommended rejection reasons:

- `WRONG_TARGET`
- `STALE_OBSERVATION`
- `DUPLICATE_SUPERSEDED`
- `INVALID_STATE`
- `HOME_INVALID`
- `HOME_PERSIST_FAILED`
- `TARGET_INVALID`
- `CALIBRATION_MISMATCH`
- `GEOFENCE_INVALID`
- `LOCALIZATION_INVALID`
- `SAFETY_HEALTH_INVALID`
- `NAVIGATION_UNAVAILABLE`
- `OPERATOR_CONFIRMATION_REQUIRED`
- `FAILSAFE_ACTIVE`
- `INVALID_COMMAND`
- `INVALID_PARAMS`

## 6. Reliable mission-event schema

### 6.1 Envelope

Only the overhead system may declare:

```text
PRIMARY_CAT_LEFT_PERIMETER(target_id)
```

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

Mandatory fields: `event_id`, `target_id`, `perimeter_id`, `observation_seq`.

### 6.2 Matching rules

A matching event is one whose:

- `target_id` equals the active chase target;
- `event_id` is new for the current mission;
- `observation_seq` is not regressive for the current mission context.

Wrong-target, duplicate, regressive-sequence, and stale events are logged and ACKed
as rejected; they do not change state.

Local camera visibility never overrides a valid matching event.

### 6.3 Applied behavior

A matching event in `GETTING_CLOSE`, `SEARCH`, `CHASE`, or a `BRAKE_REVERSE` whose
saved objective is chase:

1. commands immediate zero motion;
2. cancels Nav2/reverse;
3. enters `IDLE`;
4. starts `CAT_FOLLOW_HANDOFF_WAIT_SEC` handoff wait and recording post-roll.

During handoff:

- valid `START_CHASE(new_target_id)` enters `GETTING_CLOSE`;
- the exited target cannot be restarted from the stale event/observation;
- explicit `RETURN_HOME` enters `RETURN_HOME`;
- timeout enters `RETURN_HOME` if safe return is possible, otherwise `FAILSAFE`.

The car does not promote a local secondary track. Overhead selects and names the
next target.

## 7. ACK schema

### 7.1 Schema

```json
{
  "protocol_version": 1,
  "type": "ack",
  "message_id": "evt-31bd",
  "message_type": "mission_event",
  "applied": true,
  "resulting_state": "IDLE",
  "reason": "PRIMARY_TARGET_EXIT_HANDOFF",
  "applied_control_seq": 88214
}
```

Field rules:

| Field | Rules |
|---|---|
| `message_id` | `command_id` or `event_id` being acknowledged |
| `message_type` | `command` or `mission_event` |
| `applied` | `true` only after committed application or committed rejection |
| `resulting_state` | Actual FSM state after commit |
| `reason` | Machine-readable applied or rejection reason |
| `applied_control_seq` | Monotonic control-loop sequence at commit time |

Rejected ACKs MUST use `applied: false`, retain the actual state, and include a
specific reason such as `WRONG_TARGET`, `STALE_OBSERVATION`,
`DUPLICATE_SUPERSEDED`, `INVALID_STATE`, `HOME_INVALID`, or
`SAFETY_HEALTH_INVALID`.

Accepted command example:

```json
{
  "protocol_version": 1,
  "type": "ack",
  "message_id": "cmd-9f24",
  "message_type": "command",
  "applied": true,
  "resulting_state": "GETTING_CLOSE",
  "reason": "START_CHASE_ACCEPTED",
  "applied_control_seq": 88210
}
```

Duplicate retry example:

```json
{
  "protocol_version": 1,
  "type": "ack",
  "message_id": "cmd-9f24",
  "message_type": "command",
  "applied": true,
  "resulting_state": "GETTING_CLOSE",
  "reason": "START_CHASE_ACCEPTED",
  "applied_control_seq": 88210
}
```

The duplicate retry returns the original committed result and the original
`applied_control_seq`; it does not re-execute side effects.

## 8. Canonical FSM contract summary

Detailed transition behavior is defined in the target redesign document. This
section defines the interface-visible state/event vocabulary and matrix summary.

### 8.1 States

| State | Interface meaning |
|---|---|
| `HOME` | Stopped at durable home within completion tolerance |
| `IDLE` | Stopped, ready for command, or handoff wait |
| `GETTING_CLOSE` | Overhead/Nav2 pursuit; onboard detector not required |
| `SEARCH` | Slow acquisition near target while verifying local lock |
| `CHASE` | Local-track pursuit inside Nav2 safety/path constraints |
| `BRAKE_REVERSE` | Bounded close-obstacle recovery with saved objective |
| `GOTO` | Explicit Nav2 destination with optional YOLO/recording |
| `RETURN_HOME` | Navigate to frozen mission home |
| `FAILSAFE` | Latched zero-motion safety state |

There is no “arrived at cat” or mission-success state. Close proximity to a tracked
cat uses the same `BRAKE_REVERSE` policy as any close obstacle.

Superseded states that MUST NOT appear in new interfaces or telemetry:

- `CHASE_A`
- `TRACK_B`
- `BRAKE`

### 8.2 State groups

- Chase states: `GETTING_CLOSE`, `SEARCH`, `CHASE`
- Normal autonomous driving states: `GETTING_CLOSE`, `SEARCH`, `CHASE`, `GOTO`,
  `RETURN_HOME`
- Stationary states: `HOME`, `IDLE`, `FAILSAFE`

### 8.3 Required mission context exposed to protocol consumers

The runtime MUST expose enough mission context in shared state and telemetry to
reconstruct:

- current state and state-entry monotonic time;
- command ID and mission ID;
- active objective type;
- active `target_id`, or null;
- frozen home record and `home_version`;
- active overhead `observation_seq` and timestamp;
- local track ID and association evidence;
- `NavigationManager` goal intent ID and action correlation;
- sensor-health hold start and reason;
- overhead-invalid retention start and last valid moving-target goal;
- SEARCH timeout stage and observation-waypoint status;
- handoff wait deadline and recording post-roll deadline;
- `BRAKE_REVERSE` saved objective, phase, attempt count, and clearance-reset timer;
- requested and active perception consumers;
- latched failsafe causes.

An active mission freezes the durable home version at mission acceptance. Updating
home while a mission is active is forbidden.

### 8.4 Global safety events

From every state, each of the following enters `FAILSAFE` immediately:

- emergency stop;
- confirmed crossing of configured car geofence;
- motor/control fatal error;
- control-loop watchdog expiration.

In any normal autonomous driving state, if either required lidar or ultrasonic source
becomes stale, invalid, or faulted:

1. command zero motion immediately;
2. retain current FSM state and objective;
3. start `CAT_FOLLOW_SENSOR_RECOVERY_SEC` recovery timer;
4. resume only if both sources become fresh and valid within the interval and all
   other permissions remain valid;
5. enter `FAILSAFE` if the timer expires.

During `BRAKE_REVERSE`, loss of either required source enters `FAILSAFE` immediately.

`HOME` and `IDLE` remain stopped and report degraded health without automatic failsafe
escalation from sensor staleness alone. Later motion commands MUST be rejected until
both sensors are healthy.

### 8.5 Command and mission-event matrix summary

“Reject” means no state or objective mutation and an ACK with a reason. “Same” means
successful idempotent application unless noted.

| Current state | `SET_HOME` | `START_CHASE` | `STOP_CHASE` | `GO_TO` | `RETURN_HOME` | Matching primary-left event | `CLEAR_FAILSAFE` |
|---|---|---|---|---|---|---|---|
| `HOME` | Same after durable update | `GETTING_CLOSE` | Same | `GOTO` | Same if within tolerance | Reject/no active target | Reject |
| `IDLE` | Same after durable update | `GETTING_CLOSE` | Same; cancel handoff | `GOTO` | `RETURN_HOME` | Reject/no active target | Reject |
| `GETTING_CLOSE` | Reject | Reject | `IDLE` | Reject | `RETURN_HOME` | `IDLE` handoff | Reject |
| `SEARCH` | Reject | Reject | `IDLE` | Reject | `RETURN_HOME` | `IDLE` handoff | Reject |
| `CHASE` | Reject | Reject | `IDLE` | Reject | `RETURN_HOME` | `IDLE` handoff | Reject |
| `BRAKE_REVERSE` | Reject | Reject | `IDLE` only if saved chase; else reject | Reject | Stop, then `RETURN_HOME` | `IDLE` only if matching saved chase; else reject | Reject |
| `GOTO` | Reject | Reject | Reject | Reject | `RETURN_HOME` | Reject/no active target | Reject |
| `RETURN_HOME` | Reject | Reject | Reject | Reject | Same | Reject/no active target | Reject |
| `FAILSAFE` | Reject | Reject | Reject | Reject | Reject | Reject | `IDLE` if all clearance checks pass |

Every accepted motion command remains subject to Section 5 validation rules.

### 8.6 Autonomous transition summary

| From | Condition | To |
|---|---|---|
| `GETTING_CLOSE` | Valid target distance `<= CAT_FOLLOW_SEARCH_ENTRY_DISTANCE_CM` | `SEARCH` |
| `SEARCH` | Three consecutive unambiguous associated observations | `CHASE` |
| `SEARCH` | First SEARCH interval expires without lock | `SEARCH` with one observation waypoint |
| `SEARCH` | Second SEARCH interval expires without lock | `RETURN_HOME` or `FAILSAFE` |
| `CHASE` | Local track lost and fresh valid overhead distance `<= 200 cm` | `SEARCH` |
| `CHASE` | Local track lost and fresh valid overhead distance `> 200 cm` | `GETTING_CLOSE` |
| `CHASE` | Local track lost while overhead unavailable | `RETURN_HOME` or `FAILSAFE` |
| `GETTING_CLOSE` or `SEARCH` | Overhead recovers with different `target_id` | `IDLE` |
| `GETTING_CLOSE` or `SEARCH` | Overhead invalid-retention timer expires | `RETURN_HOME` or `FAILSAFE` |
| Any normal driving state | Close trigger from either required sensor | `BRAKE_REVERSE` |
| `BRAKE_REVERSE` | RECHECK clear and saved objective remains valid | Saved state |
| `BRAKE_REVERSE` | RECHECK blocked and attempts remain | `BRAKE_REVERSE` |
| `BRAKE_REVERSE` | Blocked with attempts exhausted | `FAILSAFE` |
| `GOTO` | Correlated completion accepted | `IDLE` |
| `RETURN_HOME` | Correlated completion accepted | `HOME` |
| `IDLE` handoff | Handoff timer expires | `RETURN_HOME` or `FAILSAFE` |

A two-hop `CHASE -> GETTING_CLOSE -> SEARCH` sequence on track loss is forbidden.

## 9. NavigationManager goal intents and results

`NavigationManager` owns Nav2 `NavigateToPose` clients, yard-to-navigation-frame
transforms, moving-goal refresh, cancel/preemption, action correlation, path
viability, safe steering envelope publication, retries, and completion qualification.

### 9.1 Goal intent schema

Shared-state and telemetry expose each submitted goal intent as:

```json
{
  "goal_intent_id": "gi-0042",
  "objective_type": "GETTING_CLOSE",
  "target_id": "cat-17",
  "frame_id": "map",
  "x_m": 2.10,
  "y_m": -0.80,
  "yaw_rad": 0.00,
  "moving_goal": true,
  "requested_at_ms": 1785012400000,
  "action_goal_id": "nav2-goal-991",
  "refresh_count": 3,
  "last_refresh_ms": 1785012402500,
  "expected_replacement": false
}
```

Allowed `objective_type` values:

- `GETTING_CLOSE`
- `SEARCH`
- `SEARCH_OBSERVATION`
- `CHASE`
- `GOTO`
- `RETURN_HOME`

Rules:

- Moving cat goals default to at most `CAT_FOLLOW_NAV_MOVING_GOAL_MAX_HZ`.
- Normal refresh requires at least `CAT_FOLLOW_NAV_MOVING_GOAL_MIN_DISPLACEMENT_CM`
  displacement since the last submitted goal.
- Safety cancellation is immediate and bypasses rate limiting.
- Replacing a moving goal intentionally is neutral: cancellation/result from the
  replaced goal MUST NOT count as failure.
- Late action results with the wrong goal intent or correlation ID MUST be ignored
  and logged.

### 9.2 Navigation result schema

```json
{
  "goal_intent_id": "gi-0042",
  "action_goal_id": "nav2-goal-991",
  "status": "SUCCEEDED",
  "result_code": 4,
  "terminal": true,
  "failure_class": null,
  "completed_at_ms": 1785012500000,
  "pose_qualified": true,
  "dwell_qualified": true
}
```

Allowed terminal `status` values:

- `SUCCEEDED`
- `ABORTED`
- `CANCELED`
- `UNKNOWN`

Failure classes for retries and exhaustion reporting:

- `PLANNER_FAILURE`
- `CONTROLLER_FAILURE`
- `NO_PROGRESS`
- `PATH_BLOCKED`
- `LOCALIZATION_LOST`
- `PREEMPTED`
- `CORRELATION_MISMATCH`

Exhausted failure outcomes:

- `GOTO -> IDLE`
- `GETTING_CLOSE`, `SEARCH`, or `CHASE -> RETURN_HOME` if safe, otherwise `FAILSAFE`
- `RETURN_HOME -> FAILSAFE`

Nav2 BackUp MUST remain disabled.

### 9.3 Path viability and safe steering envelope

`NavigationManager` publishes:

```json
{
  "path_viable": true,
  "safe_steering_min": -0.35,
  "safe_steering_max": 0.35,
  "speed_cap_mps": 0.25,
  "no_progress": false,
  "dead_end": false
}
```

`DecisionEngine` applies camera pursuit by clamping the camera steering request into
the published safe envelope. Camera and Nav2 steering MUST NOT be added or combined
by weighted sum.

Applied speed policy:

```text
applied_speed_mps = min(
    pursuit_speed_request_mps,
    nav2_speed_cap_mps,
    alignment_speed_cap_mps,
    obstacle_speed_cap_mps,
    thermal_speed_cap_mps
)
```

### 9.4 Completion contract

`GOTO` and `RETURN_HOME` complete only when:

1. the correlated Nav2 result is `SUCCEEDED`;
2. a fresh authoritative local pose is within `CAT_FOLLOW_NAV_COMPLETION_XY_CM` XY and
   `CAT_FOLLOW_NAV_COMPLETION_YAW_RAD` yaw of the destination;
3. both tolerances remain satisfied continuously for
   `CAT_FOLLOW_NAV_COMPLETION_DWELL_SEC`.

An action result alone is insufficient.

## 10. Geofence contract

Two separate concepts MUST remain distinct in protocol, shared state, and telemetry:

| Concept | Owner | Meaning |
|---|---|---|
| Car geofence | Local runtime | Inner safe polygon for the localized `base_link` center |
| Cat perimeter | Overhead system | Boundary used to declare cat exit via mission event |

Car geofence rules:

- Crossing the configured car geofence boundary is an immediate `FAILSAFE`.
- There is no predictive path veto based solely on a planned path approaching or
  crossing the polygon.
- Loss of sufficient localization to determine containment is a health failure, not
  proof of remaining inside.

Shared-state geofence group:

```json
{
  "car_geofence_id": "yard-inner-v1",
  "car_inside": true,
  "car_distance_to_boundary_cm": 84.2,
  "localization_valid_for_containment": true,
  "breach_confirmed": false,
  "breach_at_ms": null
}
```

Cat perimeter status comes from overhead observations and mission events, not from the
car geofence.

## 11. Dual lidar and ultrasonic health contract

Both lidar and ultrasonic are required for autonomous motion and remain direct
`DecisionEngine` safety inputs even when integrated with Nav2.

Shared state MUST expose independent health for each source:

```json
{
  "lidar": {
    "fresh": true,
    "valid": true,
    "faulted": false,
    "distance_cm": 42.0,
    "stale_ms": 120,
    "backend": "rplidar_c1"
  },
  "ultrasonic": {
    "fresh": true,
    "valid": true,
    "faulted": false,
    "distance_cm": 118.0,
    "stale_ms": 80,
    "frame_id": "ultrasonic_link",
    "costmap_layer_enabled": true
  },
  "required_for_motion": true,
  "hold_active": false,
  "hold_started_ms": null,
  "hold_reason": null,
  "recovery_deadline_ms": null
}
```

Rules:

- Either source stale, invalid, or faulted in a driving state starts a hold with
  zero motion and preserves objective until `CAT_FOLLOW_SENSOR_RECOVERY_SEC`.
- Recovery requires both sources fresh and valid.
- Either source unhealthy during `BRAKE_REVERSE` enters `FAILSAFE` immediately.
- Ultrasonic MUST be published as `sensor_msgs/Range` with validated topic, frame,
  radiation type, field of view, min/max range, finite range value, timestamp, and
  transform into the local costmap frame.
- The local costmap MUST integrate ultrasonic through a validated `RangeSensorLayer`
  when `CAT_FOLLOW_NAV_ULTRASONIC_COSTMAP=1`.
- Disabling the costmap layer for diagnosis MUST NOT disable ultrasonic direct safety.

Close-obstacle policy:

- Fresh valid reading strictly below `CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM` enters
  `BRAKE_REVERSE`.
- There is no `10 cm` immediate `FAILSAFE` close-obstacle rule in the target contract.

## 12. Perception, recording, and stream lifecycle

### 12.1 Ownership

`PerceptionLifecycleManager` owns named consumers and reference counts:

- `detector`
- `recording`
- `stream`

Consumers are independent. Detector and recording never depend on stream clients.
Stream reference count is the actual connected-client count.

Mission policy overrides legacy PhaseMachine in `SEARCH`, `CHASE`, and a `GOTO` with
`request_yolo=true`.

### 12.2 Lifecycle status schema

```json
{
  "detector": {
    "requested": true,
    "active": true,
    "consumer_refcount": 1,
    "reason": "SEARCH_REQUIRED"
  },
  "recording": {
    "requested": true,
    "active": true,
    "consumer_refcount": 1,
    "segment_path": "/var/lib/cat_follow/recordings/20260726-001.mkv",
    "postroll_deadline_ms": null,
    "degraded_reason": null
  },
  "stream": {
    "requested_clients": 1,
    "active_clients": 1,
    "encoder_ready": true,
    "forced_off": false,
    "degraded_reason": null
  },
  "camera": {
    "hardware_state": "active",
    "streamoff_capable": true,
    "last_revalidation_ms": 1785012400000,
    "fatal_fault": false
  }
}
```

Allowed `camera.hardware_state` values:

- `closed`
- `ready_inactive`
- `active`
- `faulted`

Lifecycle table:

| FSM state | Detector | Recording | Stream | Camera policy |
|---|---|---|---|---|
| `HOME` | Forced off | Forced off | Forced off | Ready-inactive/closed |
| `IDLE` | Off | Post-roll/handoff only | Actual clients unless forced off | Active only with a consumer |
| `GETTING_CLOSE` | Off unless diagnostic policy | Chase mission request | Actual clients | Active if recording/stream need frames |
| `SEARCH` | Required on | Chase mission request | Actual clients | Active |
| `CHASE` | Required on | Chase mission request | Actual clients | Active |
| `BRAKE_REVERSE` | Inherit saved objective unless camera failed | Inherit saved request | Actual clients | Based on references |
| `GOTO` | Exactly `request_yolo` | Exactly `request_recording` | Actual clients | Active if any consumer |
| `RETURN_HOME` | Off | Retain active mission/post-roll request | Actual clients | Active if recording/stream need frames |
| `FAILSAFE` | Forced off | Forced off | Forced off | Ready-inactive/closed |

Recording rules:

- segmented, crash-tolerant Matroska via hardware H.264 encoder;
- quota and minimum free-space reserve enforced;
- oldest finalized segments deleted first;
- active segment never deleted as quota cleanup;
- low-space stop leaves mission running;
- automatic resume while still requested after health/space recovery;
- chase recording post-roll lasts `CAT_FOLLOW_RECORDING_POSTROLL_SEC`.

Monitoring stream rules:

- H.264 hardware encoding only;
- runs only while at least one actual client is connected and FSM does not force it off;
- no MJPEG or software-encoding fallback;
- recording encoder is independent from monitoring encoder/client rule.

Recording, encoder, storage, or monitoring-stream failure causes no FSM transition and
no motion veto. It produces degraded telemetry only.

## 13. Durable home contract

Home is not carried in lossy overhead observations. It is a durable, versioned,
calibration/map-associated record.

```json
{
  "home_version": 4,
  "checksum": "sha256:ab12...",
  "calibration_version": 7,
  "map_id": "yard-map-v3",
  "frame_id": "map",
  "x_m": 0.00,
  "y_m": 0.00,
  "yaw_rad": 0.00,
  "persisted_at_ms": 1785012000000,
  "source_command_id": "cmd-home-01",
  "frozen_for_mission": true,
  "mission_home_version": 4
}
```

Rules:

- `SET_HOME` succeeds only after durable commit.
- Active missions freeze `mission_home_version` at acceptance.
- `RETURN_HOME` uses the frozen mission home, not a newly edited home record.
- Persistence failure is command rejection, not in-memory success.

## 14. Physical m/s versus normalized reverse

Nav2 policies, caps, requests, and navigation telemetry use physical `m/s`.

Conversion to the PiCar-X normalized motor command uses a calibrated approximate
mapping derived from `speed_time_distance` measurements. Calibration MUST cover the
production surface, payload, representative battery range, and forward speed range.

`BRAKE_REVERSE` is the sole specified direct normalized/time-bounded maneuver:

- settle: `CAT_FOLLOW_BRAKE_REVERSE_SETTLE_MS`
- reverse command: `CAT_FOLLOW_BRAKE_REVERSE_NORMALIZED`
- reverse duration: `CAT_FOLLOW_BRAKE_REVERSE_DURATION_SEC`
- steering centered during settle and reverse

Decision and telemetry MUST record at least:

- requested physical speed in `m/s`;
- all applied physical speed caps;
- requested steering and Nav2 safe envelope;
- final applied normalized drive command;
- final applied steering command;
- mapping/calibration version;
- whether the command came from Nav2 policy or direct reverse;
- every zero-motion veto reason.

## 15. SharedState contract

`SharedState` is the only cross-thread runtime data contract. Each group has exactly
one authoritative writer.

### 15.1 Group ownership

| Group | Authoritative writer |
|---|---|
| `overhead` | `CommsManager` |
| `home` | Home persistence service / command commit path |
| `mission` | FSM / control loop |
| `vision` | Detector / tracker |
| `lidar` | Lidar adapter |
| `ultrasonic` | Ultrasonic adapter |
| `sensor_health` | `DecisionEngine` or safety aggregator |
| `navigation` | `NavigationManager` |
| `geofence` | Localization / safety aggregator |
| `perception_lifecycle` | `PerceptionLifecycleManager` |
| `recording` | Recording service |
| `stream` | Web stream / H.264 route owner |
| `system` | Runtime / health monitor |
| `fsm` | FSM |
| `command` | Command handler |
| `decision` | `DecisionEngine` |

Rules:

- Only the authoritative writer may update its group.
- `DecisionEngine` is the sole drivetrain decision authority.
- No camera, detector, communications component, Nav2 adapter, or web route writes
  motor or steering commands directly.
- `DecisionEngine` reads one coherent snapshot per control tick and MUST NOT use the
  previous `decision` group as input to future decisions.

### 15.2 Target SharedState schema

```json
{
  "overhead": {
    "received_ms": 0,
    "fresh": false,
    "observation_seq": 0,
    "observed_at_ms": 0,
    "perimeter_id": null,
    "calibration_version": null,
    "selected_target_id": null,
    "invalid": true,
    "retention_deadline_ms": null,
    "car": {
      "x_cm": 0.0,
      "y_cm": 0.0,
      "yaw_rad": 0.0,
      "confidence": 0.0
    },
    "cats": []
  },
  "home": {
    "valid": false,
    "home_version": null,
    "checksum": null,
    "calibration_version": null,
    "map_id": null,
    "frame_id": "map",
    "x_m": 0.0,
    "y_m": 0.0,
    "yaw_rad": 0.0,
    "persisted_at_ms": null,
    "source_command_id": null
  },
  "mission": {
    "mission_id": null,
    "objective_type": null,
    "target_id": null,
    "home_version_frozen": null,
    "handoff_deadline_ms": null,
    "recording_postroll_deadline_ms": null,
    "search_stage": 0,
    "association_count": 0,
    "local_track_id": null
  },
  "vision": {
    "received_ms": 0,
    "fresh": false,
    "track_id": null,
    "associated_target_id": null,
    "bearing_rad": null,
    "confidence": 0.0,
    "ambiguous": false
  },
  "lidar": {
    "received_ms": 0,
    "fresh": false,
    "valid": false,
    "faulted": false,
    "distance_cm": null
  },
  "ultrasonic": {
    "received_ms": 0,
    "fresh": false,
    "valid": false,
    "faulted": false,
    "distance_cm": null,
    "frame_id": null
  },
  "sensor_health": {
    "required_for_motion": true,
    "hold_active": false,
    "hold_started_ms": null,
    "hold_reason": null,
    "recovery_deadline_ms": null
  },
  "navigation": {
    "received_ms": 0,
    "fresh": false,
    "goal_intent": null,
    "last_result": null,
    "path_viable": false,
    "safe_steering_min": 0.0,
    "safe_steering_max": 0.0,
    "speed_cap_mps": 0.0,
    "no_progress": false,
    "dead_end": false
  },
  "geofence": {
    "car_geofence_id": null,
    "car_inside": null,
    "car_distance_to_boundary_cm": null,
    "localization_valid_for_containment": false,
    "breach_confirmed": false
  },
  "perception_lifecycle": {
    "detector": {},
    "recording": {},
    "stream": {},
    "camera": {}
  },
  "system": {
    "thermal_state": "unknown",
    "control_loop_seq": 0,
    "control_rate_hz": 0.0,
    "watchdog_ok": true,
    "threads": {}
  },
  "fsm": {
    "state": "IDLE",
    "previous_state": null,
    "state_entered_ms": 0,
    "saved_objective": null,
    "brake_reverse_phase": null,
    "brake_reverse_attempts": 0,
    "latched_failsafe_causes": []
  },
  "command": {
    "last_command_id": null,
    "last_message_type": null,
    "last_applied": null,
    "last_reason": null,
    "last_applied_control_seq": null
  },
  "decision": {
    "requested_state": "IDLE",
    "requested_speed_mps": 0.0,
    "applied_speed_mps": 0.0,
    "speed_caps_mps": [],
    "requested_steering": 0.0,
    "safe_steering_min": 0.0,
    "safe_steering_max": 0.0,
    "applied_steering": 0.0,
    "applied_drive_normalized": 0.0,
    "command_source": "none",
    "zero_motion_vetoes": [],
    "reason": "init"
  }
}
```

### 15.3 DecisionEngine dataclasses

`DecisionInput` is built from one immutable snapshot and excludes the previous
`decision` group:

```python
@dataclass(frozen=True)
class DecisionInput:
    now_ms: int
    overhead: OverheadState
    home: HomeState
    mission: MissionState
    vision: VisionState
    lidar: LidarState
    ultrasonic: UltrasonicState
    sensor_health: SensorHealthState
    navigation: NavigationState
    geofence: GeofenceState
    perception_lifecycle: PerceptionLifecycleState
    system: SystemState
    fsm: FSMSnapshot
    command: CommandState
```

```python
@dataclass(frozen=True)
class DecisionOutput:
    timestamp_ms: int
    requested_state: str
    requested_speed_mps: float
    applied_speed_mps: float
    speed_caps_mps: list[str]
    requested_steering: float
    safe_steering_min: float
    safe_steering_max: float
    applied_steering: float
    applied_drive_normalized: float
    command_source: str
    zero_motion_vetoes: list[str]
    reason: str
    rejected_transition: bool = False
```

Allowed `requested_state` values:

- `HOME`
- `IDLE`
- `GETTING_CLOSE`
- `SEARCH`
- `CHASE`
- `BRAKE_REVERSE`
- `GOTO`
- `RETURN_HOME`
- `FAILSAFE`

Allowed `command_source` values:

- `nav2_policy`
- `direct_reverse`
- `zero_motion`
- `none`

## 16. Telemetry contract

Telemetry is structured JSON Lines (`JSONL`). Each line is one event object.

Rules:

- logging uses a bounded async queue;
- low-priority events may be dropped if the queue is full;
- safety, failsafe, command, mission-event, transition, and control-sequence events
  are high priority and SHOULD be preserved whenever possible;
- systemd journal may receive human-readable service logs, but replay/tuning uses
  JSONL telemetry.

### 16.1 Common event envelope

```json
{
  "schema_version": 2,
  "event_id": "evt-000001",
  "event_type": "state_transition",
  "timestamp_ms": 123458000,
  "monotonic_ms": 987654321,
  "state": "GETTING_CLOSE",
  "source": "FSM",
  "severity": "info",
  "applied_control_seq": 88210,
  "data": {}
}
```

`monotonic_ms` is authoritative for ordering inside PiCar-X telemetry.

### 16.2 Required event types

- `state_transition`
- `transition_rejected`
- `decision`
- `command_received`
- `command_applied`
- `command_ack`
- `mission_event_received`
- `mission_event_applied`
- `mission_event_ack`
- `overhead_observation_received`
- `overhead_invalid`
- `vision_update`
- `lidar_update`
- `ultrasonic_update`
- `sensor_health_hold`
- `sensor_health_failsafe`
- `navigation_goal`
- `navigation_result`
- `geofence`
- `perception_lifecycle`
- `recording`
- `stream`
- `brake_reverse_phase`
- `failsafe`
- `thermal`
- `thread_health`
- `motor_command`
- `control_tick`

### 16.3 Example events

`state_transition`:

```json
{
  "event_type": "state_transition",
  "source": "FSM",
  "severity": "info",
  "data": {
    "from_state": "IDLE",
    "to_state": "GETTING_CLOSE",
    "reason": "START_CHASE_ACCEPTED",
    "target_id": "cat-17",
    "home_version_frozen": 4
  }
}
```

`mission_event_applied`:

```json
{
  "event_type": "mission_event_applied",
  "source": "CommsManager",
  "severity": "info",
  "data": {
    "event_id": "evt-31bd",
    "name": "PRIMARY_CAT_LEFT_PERIMETER",
    "target_id": "cat-17",
    "applied": true,
    "resulting_state": "IDLE",
    "reason": "PRIMARY_TARGET_EXIT_HANDOFF"
  }
}
```

`decision`:

```json
{
  "event_type": "decision",
  "source": "DecisionEngine",
  "severity": "debug",
  "data": {
    "requested_state": "CHASE",
    "requested_speed_mps": 0.22,
    "applied_speed_mps": 0.18,
    "speed_caps_mps": ["nav2_speed_cap", "alignment_speed_cap"],
    "requested_steering": 0.31,
    "safe_steering_min": -0.35,
    "safe_steering_max": 0.35,
    "applied_steering": 0.31,
    "applied_drive_normalized": 0.36,
    "command_source": "nav2_policy",
    "zero_motion_vetoes": []
  }
}
```

`navigation_goal`:

```json
{
  "event_type": "navigation_goal",
  "source": "NavigationManager",
  "severity": "info",
  "data": {
    "goal_intent_id": "gi-0042",
    "objective_type": "GETTING_CLOSE",
    "target_id": "cat-17",
    "moving_goal": true,
    "action_goal_id": "nav2-goal-991"
  }
}
```

Every transition, hold, retry, consumer reference, goal correlation, and applied
command MUST be reconstructable from telemetry.

## 17. Target configuration defaults

Environment names MUST use the `CAT_FOLLOW_*` namespace.

| Configuration | Default | Meaning |
|---|---:|---|
| `CAT_FOLLOW_SEARCH_ENTRY_DISTANCE_CM` | `200` | Enter SEARCH at or below this valid overhead distance |
| `CAT_FOLLOW_SEARCH_SPEED_CAP_MPS` | `0.10` | SEARCH and retained-goal speed cap |
| `CAT_FOLLOW_SEARCH_LOCK_OBSERVATIONS` | `3` | Consecutive unambiguous associated observations |
| `CAT_FOLLOW_SEARCH_INTERVAL_SEC` | `10` | Each of the two SEARCH intervals |
| `CAT_FOLLOW_LOCAL_TRACK_STALE_MS` | `350` | Local track expiration threshold |
| `CAT_FOLLOW_OVERHEAD_INVALID_MAX_SEC` | `10` | Last valid goal retention in GETTING_CLOSE/SEARCH |
| `CAT_FOLLOW_SENSOR_RECOVERY_SEC` | `2` | Required-sensor zero-motion recovery interval |
| `CAT_FOLLOW_HANDOFF_WAIT_SEC` | `10` | IDLE wait after primary target exit |
| `CAT_FOLLOW_RECORDING_POSTROLL_SEC` | `10` | Recording post-roll after chase stop/exit |
| `CAT_FOLLOW_NAV_MOVING_GOAL_MAX_HZ` | `2` | Maximum moving-target goal submission rate |
| `CAT_FOLLOW_NAV_MOVING_GOAL_MIN_DISPLACEMENT_CM` | `25` | Minimum displacement for a normal refresh |
| `CAT_FOLLOW_NAV_COMPLETION_XY_CM` | `20` | Local-pose XY completion tolerance |
| `CAT_FOLLOW_NAV_COMPLETION_YAW_RAD` | `0.3` | Local-pose yaw completion tolerance |
| `CAT_FOLLOW_NAV_COMPLETION_DWELL_SEC` | `1` | Continuous in-tolerance dwell |
| `CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM` | `15` | Trigger on fresh valid reading strictly below this |
| `CAT_FOLLOW_BRAKE_REVERSE_SETTLE_MS` | `100` | Centered, stopped settle before reverse |
| `CAT_FOLLOW_BRAKE_REVERSE_DURATION_SEC` | `0.5` | Bounded reverse duration |
| `CAT_FOLLOW_BRAKE_REVERSE_NORMALIZED` | `-0.30` | Direct normalized reverse command |
| `CAT_FOLLOW_BRAKE_REVERSE_MAX_ATTEMPTS` | `3` | Maximum actual reverse phases before failsafe |
| `CAT_FOLLOW_BRAKE_REVERSE_RESET_CM` | `20` | Both sources must be strictly above this to reset |
| `CAT_FOLLOW_BRAKE_REVERSE_RESET_SEC` | `2` | Continuous dual-source clearance duration |
| `CAT_FOLLOW_NAV_ULTRASONIC_COSTMAP` | `1` | Enable validated RangeSensorLayer integration |
| `CAT_FOLLOW_NAV2_BACKUP_ENABLED` | `0` | Nav2 BackUp disabled and MUST remain disabled |
| `CAT_FOLLOW_COMMAND_ACK_TIMEOUT_MS` | `200` | Command/event retry timeout |
| `CAT_FOLLOW_COMMAND_MAX_RETRIES` | `5` | Max command/event retries |
| `CAT_FOLLOW_COMMAND_ID_CACHE_SIZE` | `100` | Processed command/event cache size |
| `CAT_FOLLOW_OVERHEAD_MIN_CONFIDENCE` | deployment-calibrated | Minimum valid car/cat confidence |
| `CAT_FOLLOW_ASSOCIATION_BEARING_GATE_RAD` | deployment-calibrated | Base uncertainty gate for SEARCH association |
| `CAT_FOLLOW_RECORDING_QUOTA_BYTES` | deployment-required | Maximum retained finalized recording bytes |
| `CAT_FOLLOW_RECORDING_MIN_FREE_BYTES` | deployment-required | Free-space reserve below which recording stops |

Input freshness limits for lidar, ultrasonic, localization, overhead, and motor
feedback MUST also be explicit deployment configuration validated against measured
publication rates. No component may silently substitute a different timeout.

Superseded constants that MUST NOT be used as normative target behavior:

- `OVERHEAD_STALE_FAILSAFE_MS = 700`
- `OBSTACLE_TOO_CLOSE_CM = 10`
- `CAMERA_LOSS_FALLBACK_MS`-driven `TRACK_B -> CHASE_A` fallback
- bicycle/wheel odometry fallback constants

## 18. Current implementation gaps

The current repository does **not** implement this target contract. Known gaps
include:

1. **Resolved.** `cat_follow/control/types.py` and `cat_follow/control/fsm.py`
   implement the canonical states (`GETTING_CLOSE`, `SEARCH`, `CHASE`,
   `BRAKE_REVERSE`, `GOTO`, `RETURN_HOME`, `FAILSAFE`, `HOME`, `IDLE`) and this
   `FSM` is the live control authority instantiated in `cat_follow/runtime/app.py`.
   `CHASE_A`, `TRACK_B`, and `BRAKE` remain only as backward-compatible wire
   aliases for legacy V1 integrations (`FsmState._missing_`), not as the
   internal state representation. Full transition-table parity against this
   document's section 10 has not been independently re-audited.
2. **Resolved.** `DecisionEngine.close_obstacle_trigger_cm`
   (`cat_follow/control/decision_engine.py`) uses
   `max(target_config.brake_reverse_trigger_cm, safety_config.obstacle_too_close_cm)`
   (15 cm target threshold vs. the 10 cm operator-facing floor, whichever is more
   conservative) and triggers the recoverable `BRAKE_REVERSE` FSM state via
   `FsmEvent.BRAKE_REVERSE_TRIGGERED`; a routine close-obstacle event no longer
   latches `FAILSAFE`.
3. Current chase overhead expiry uses an approximately 700 ms failsafe rather than
   state-specific retained-goal, local-track, and return behavior.
4. The ROS navigation bridge lacks the required complete `NavigationManager`
   moving-goal output, refresh, cancel, correlation, and completion behavior.
5. **Partially resolved.** `DecisionEngine._navigation_drive_output` implements the
   non-additive fusion: in `CHASE` the camera request is clamped to the Nav2
   `safe_steering_min`/`safe_steering_max` envelope (never summed with
   `path_correction`), an inverted envelope stops the car, and speed is the
   minimum of the planner limit and every applied cap. The producer side of that
   envelope still depends on the `NavigationManager` work in gap 4.
6. The camera prototype is effectively always active rather than managed by named
   consumers, reference counts, and STREAMOFF/STREAMON readiness.
7. Detector activation is primarily PhaseMachine/motion-gated rather than
   mission-policy-required in SEARCH/CHASE and requested GOTO.
8. Hardware H.264 segmented Matroska recording, storage quota, reserve, crash
   recovery, post-roll, and degraded retry are not implemented.
9. The protocol lacks stable `target_id` on overhead cats/selection and chase
   commands.
10. The reliable ACKed `mission_event` envelope and
    `PRIMARY_CAT_LEFT_PERIMETER` transaction do not exist.
11. `PerceptionLifecycleManager` and the specified `NavigationManager` do not exist
    as target ownership boundaries.
12. Durable versioned home, active-mission home freezing, and transactional
    `SET_HOME` acceptance are incomplete or absent.
13. Startup overhead pose seeding/validation followed by local authoritative
    localization is not implemented as specified.
14. Ultrasonic is not fully published/validated as `sensor_msgs/Range` and
    integrated through a validated local-costmap `RangeSensorLayer`.
15. **Resolved.** `DecisionEngine.tick` holds zero motion while either required
    sensor is unhealthy in a normal driving state, escalates to `FAILSAFE` once
    the hold exceeds `CAT_FOLLOW_SENSOR_RECOVERY_SEC`, reports
    `sensor_health_degraded` without motion in stationary `HOME`/`IDLE`, and
    enters `FAILSAFE` immediately when either source is unhealthy during
    `BRAKE_REVERSE`.
16. Current completion does not require correlated Nav2 success plus fresh local
    pose tolerance for one second.
17. Current thermal, handoff, target-identity, SEARCH association/timeout, and
    direct CHASE-loss transitions are incomplete.
18. Existing documents and code may still claim bicycle odometry fallback, final
    `BRAKE`, MJPEG/software fallback, predictive geofence veto, or blanket inherited
    timing. Those claims are superseded by this target contract.

These are migration gaps, not permission to partially reinterpret the target.

## 19. Thread synchronization rules

`SharedState` is the only cross-thread data contract for runtime state.

Rules:

- each group has exactly one authoritative writer;
- writers publish complete group updates atomically;
- the control loop reads one coherent snapshot once per tick;
- readers MUST NOT hold locks while running control logic;
- command and mission-event deduplication caches MUST survive at least the max retry
  window;
- telemetry MUST NOT block the control loop;
- shutdown coordinates safe stop before process exit.

Recommended implementation:

- per-group immutable dataclass instances;
- writer builds a new group object outside the lock;
- writer acquires the group lock and swaps the object;
- snapshot reader copies references under brief lock acquisition before decisions.

## 20. Coordinate conventions

### 20.1 Yard frame

Overhead yard positions use a calibrated yard frame:

- units: centimeters;
- origin: fixed overhead-calibrated yard origin;
- `+X`: right in the yard;
- `+Y`: forward in the yard;
- coordinates are planar ground-plane coordinates.

The origin MUST NOT move during a run. Calibration changes produce a new
`calibration_version`.

### 20.2 Navigation frame

Nav2 destinations, localization, and completion tolerances use the configured ROS
navigation frame, default `map`, in meters and radians.

Yard-to-navigation-frame transforms MUST be explicit and versioned. Overhead yard
coordinates MUST NOT be treated as ROS navigation-frame coordinates without
calibration.

## 21. Control timing contract

The control loop target rate remains `50 Hz` with a `20 ms` period and a minimum
degraded rate of `20 Hz`.

Rules:

- commands and mission events are applied at control-loop boundaries;
- ACK commit occurs after application or rejection at that boundary;
- emergency stop remains synchronous;
- control-loop watchdog expiration enters `FAILSAFE`;
- telemetry enqueue MUST NOT run file I/O inside the control tick.

Control timing telemetry SHOULD include:

- `tick_duration_ms`
- `snapshot_ms`
- `decision_ms`
- `fsm_ms`
- `motor_ms`
- `telemetry_enqueue_ms`
- `overrun_count`
- `control_rate_hz`
- `applied_control_seq`

## 22. Enums and constants appendix

### 22.1 Protocol message types

- `overhead_observation`
- `command`
- `mission_event`
- `ack`

### 22.2 Command names

- `SET_HOME`
- `START_CHASE`
- `STOP_CHASE`
- `GO_TO`
- `RETURN_HOME`
- `CLEAR_FAILSAFE`
- `EMERGENCY_STOP`

### 22.3 Mission event names

- `PRIMARY_CAT_LEFT_PERIMETER`

### 22.4 FSM states

- `HOME`
- `IDLE`
- `GETTING_CLOSE`
- `SEARCH`
- `CHASE`
- `BRAKE_REVERSE`
- `GOTO`
- `RETURN_HOME`
- `FAILSAFE`

### 22.5 Navigation objective types

- `GETTING_CLOSE`
- `SEARCH`
- `SEARCH_OBSERVATION`
- `CHASE`
- `GOTO`
- `RETURN_HOME`

### 22.6 Perception camera hardware states

- `closed`
- `ready_inactive`
- `active`
- `faulted`

### 22.7 Thermal states

- `unknown`
- `normal`
- `warning`
- `speed_limited`
- `critical`

### 22.8 Telemetry severities

- `debug`
- `info`
- `warning`
- `error`
- `critical`
