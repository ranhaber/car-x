# Canonical Target Redesign: FSM and Runtime Behavior

**Project:** Autonomous Yard Navigator and Cat Tracker  
**Target hardware:** Radxa ROCK 4D with Radxa 4K IMX415 camera  
**Document version:** 2.0  
**Protocol version:** V1  
**Status:** Canonical approved target specification — **not implemented yet**  
**Date:** 2026-07-25

## 1. Purpose and authority

This document is the canonical target specification for the future mission
FSM, overhead/local target handoff, Nav2 integration, safety behavior,
perception lifecycle, recording, and H.264 monitoring stream.

Nothing in this document claims that the current executable implements the
target. Section 18 lists known implementation conflicts. Until the migration
is complete and validated, current runtime behavior remains different.

For the behavior covered here, this document supersedes conflicting
requirements in:

- `PRD_Autonomous_Yard_Navigator_Cat_Tracker.md`;
- `High_Level_Design_Autonomous_Yard_Navigator_Cat_Tracker.md`;
- `System_Architecture_Autonomous_Yard_Navigator_Cat_Tracker.md`;
- `Detailed_Software_Architecture_Autonomous_Yard_Navigator_Cat_Tracker.md`;
- `Interface_and_Data_Contract_Specification_Autonomous_Yard_Navigator_Cat_Tracker.md`;
- `Software_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md`;
- `Validation_Matrix_Autonomous_Yard_Navigator_Cat_Tracker.md`.

The following invariants are retained:

1. `DecisionEngine` is the sole drivetrain decision authority. No camera,
   detector, communications component, Nav2 adapter, or web route writes motor
   or steering commands directly.
2. Emergency stop and hard safety vetoes preempt mission behavior.
3. Detection and tracking are headless and never depend on a browser or
   monitoring stream client.
4. Commands remain identified, deduplicated, acknowledged, and observable.
5. Coordinate transforms are explicit; overhead yard coordinates are not
   treated as ROS navigation-frame coordinates without calibration.
6. Safety telemetry records the source, freshness, and reason for every motion
   inhibition.

The following prior behavior is explicitly superseded:

- `CHASE_A`, `TRACK_B`, and final-stop `BRAKE` state semantics;
- bicycle/wheel odometry support or fallback;
- a fixed 10 cm final stop at the tracked cat;
- immediate 700 ms overhead-stale failsafe during a chase;
- additive or weighted camera/Nav2 steering;
- always-on camera capture and PhaseMachine-owned mission inference;
- software/MJPEG monitoring fallback;
- predictive geofence path veto;
- any statement that lidar-only or ultrasonic-only autonomous motion is
  acceptable.

## 2. Normative language and units

`MUST`, `MUST NOT`, `SHOULD`, and `MAY` are normative. Configuration values
marked configurable MUST be validated at startup and exposed in effective
configuration telemetry.

- Yard positions and perimeter geometry use calibrated centimeters.
- Nav2 motion policy and navigation telemetry use physical meters per second.
- Direct reverse uses a normalized motor command and a bounded duration.
- Angles use radians in ROS interfaces unless a protocol field says otherwise.
- Durations use monotonic time internally.
- “Fresh and valid” means within the configured age limit, structurally valid,
  finite, in range, and not faulted.
- “Safe return is possible” means localization, geofence status,
  `NavigationManager`, control loop, lidar, ultrasonic, and the stored mission
  home are valid enough to execute `RETURN_HOME`.

## 3. Production platform and authority boundaries

### 3.1 Supported production configuration

The production target is:

- Radxa ROCK 4D;
- Radxa 4K IMX415 onboard camera;
- RKNN inference only;
- ROS 2/Nav2 local navigation;
- RF2O lidar odometry only;
- lidar plus forward ultrasonic sensing;
- overhead yard camera/service for strategic car and cat observations.

Bicycle/wheel odometry is unsupported. There is no stale-data bicycle
odometry fallback. Loss of RF2O/localization is handled by the health and
failsafe rules in this specification.

### 3.2 Authority boundaries

- The overhead system owns yard-level car/cat observations, selected
  `target_id`, and declaration that the selected cat left its perimeter.
- Startup MAY use the overhead car pose to seed and validate ROS localization.
  After startup acceptance, local Nav2/SLAM localization is authoritative for
  motion and completion.
- `NavigationManager` owns Nav2 `NavigateToPose` clients and goal lifecycle.
- `PerceptionLifecycleManager` owns camera/detector/recording/stream consumer
  references and hardware lifecycle.
- `DecisionEngine` consumes mission intent, navigation output, perception, and
  safety inputs and alone determines the applied drivetrain command.
- Nav2 BackUp recovery MUST remain disabled. `BRAKE_REVERSE` is separate,
  bounded, and owned by `DecisionEngine`.

## 4. FSM state and mission context

### 4.1 States

| State | Normative purpose |
|---|---|
| `HOME` | Stopped at the durable home pose within completion tolerance. |
| `IDLE` | Stopped, ready for an operator/overhead command, or waiting during target handoff. |
| `GETTING_CLOSE` | Follow the selected overhead target with Nav2; onboard target detection is not required. |
| `SEARCH` | Move slowly near the selected overhead target while acquiring a verified local lock. |
| `CHASE` | Pursue the associated local track inside Nav2 safety/path constraints. |
| `BRAKE_REVERSE` | Formal bounded close-obstacle recovery with saved interrupted objective and phase. |
| `GOTO` | Execute an explicit user Nav2 destination, with independently requested YOLO and recording. |
| `RETURN_HOME` | Navigate to the frozen mission home using the critical-return policy when applicable. |
| `FAILSAFE` | Latched zero-motion safety state requiring operator-confirmed, cause-specific clearance. |

There is no “arrived at cat” or mission-success state. Close proximity to a
tracked cat uses the same `BRAKE_REVERSE` policy as any close obstacle.

### 4.2 Required mission context

The FSM context MUST contain at least:

- current state and state-entry monotonic time;
- command ID and mission ID;
- active objective type;
- active `target_id`, or null for non-chase objectives;
- frozen home record and home version for the active mission;
- active overhead observation sequence and timestamp;
- local track ID and association evidence;
- `NavigationManager` goal intent ID and action correlation;
- sensor-health hold start and reason;
- overhead-invalid retention start and last valid moving-target goal;
- SEARCH timeout stage and observation-waypoint status;
- handoff wait deadline and recording post-roll deadline;
- `BRAKE_REVERSE` saved objective, phase, attempt count, and clearance-reset
  timer;
- thermal profile;
- requested and active perception consumers;
- latched failsafe causes.

An active mission freezes the durable home version at mission acceptance.
Updating home while a mission is active is forbidden.

### 4.3 State groups

- Chase states: `GETTING_CLOSE`, `SEARCH`, `CHASE`.
- Normal autonomous driving states: `GETTING_CLOSE`, `SEARCH`, `CHASE`,
  `GOTO`, `RETURN_HOME`.
- Stationary states: `HOME`, `IDLE`, `FAILSAFE`.
- “Any non-failsafe objective” includes any chase state, `GOTO`, and an
  interrupted objective in `BRAKE_REVERSE`.

## 5. Control precedence

Every control tick MUST apply the following precedence, highest first:

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
Recording and streaming never authorize motion. Emergency stop remains
synchronous; all other mission commands/events are applied transactionally by
the control loop as specified in Section 13.

## 6. Global safety and degradation rules

### 6.1 Unconditional failsafe causes

From every state, each of the following enters `FAILSAFE` immediately:

- emergency stop;
- confirmed crossing of the configured car geofence;
- motor/control fatal error;
- control-loop watchdog expiration.

Entry MUST command zero motion immediately, center steering when safely
possible, cancel Nav2, stop reverse output, clear pending non-safety motion,
and latch all active causes.

### 6.2 Required lidar and ultrasonic health

Both lidar and ultrasonic are required for autonomous motion and remain direct
`DecisionEngine` safety inputs even when integrated with Nav2.

In any normal autonomous driving state, if either source becomes stale,
invalid, or faulted:

1. command zero motion and cancel/pause active velocity output immediately;
2. retain the current FSM state and objective;
3. start a configurable recovery timer;
4. resume only if both sources become fresh and valid within 2 seconds and all
   other permissions remain valid;
5. enter `FAILSAFE` if the timer reaches 2 seconds.

During `BRAKE_REVERSE`, loss of either required source enters `FAILSAFE`
immediately; there is no recovery hold while reversing.

`HOME` and `IDLE` remain stopped and report degraded health without automatic
failsafe escalation from sensor staleness alone. A later motion command MUST
be rejected until both sensors are healthy. `FAILSAFE` remains latched.

### 6.3 Onboard camera or RKNN fatal failure

Fatal onboard camera/RKNN failure does not, by itself, enter `FAILSAFE`:

- `SEARCH` or `CHASE` transitions directly to `GETTING_CLOSE` and continues
  overhead-only at valid Nav2/safety limits;
- `GETTING_CLOSE` continues overhead/Nav2;
- `GOTO` and `RETURN_HOME` continue when navigation and safety remain valid;
- `HOME` and `IDLE` remain stopped and degraded;
- `BRAKE_REVERSE` completes only if required non-vision safety remains valid,
  then restoration is re-evaluated under these rules.

If overhead is also unavailable and the objective depends on overhead, the car
MUST enter `RETURN_HOME` when safe return is possible, otherwise `FAILSAFE`.
An onboard-perception failure MUST NOT silently associate a different target.

### 6.4 Recording or H.264 failure

Recording, encoder, storage, or monitoring-stream failure causes no FSM
transition and no motion veto. It produces degraded telemetry. Recording MUST
resume automatically when its requested consumer is still active and the
recording pipeline becomes healthy.

### 6.5 Localization and navigation failure

Loss of valid localization, geofence observability, or
`NavigationManager`/Nav2 viability while moving commands an immediate stop.
If safe return cannot be established, enter `FAILSAFE`. Exhausted
objective-specific navigation failures follow Section 11.5.

### 6.6 Critical thermal behavior

- Any active objective other than `RETURN_HOME` transitions to
  `RETURN_HOME` if safe return is possible, otherwise `FAILSAFE`.
- `RETURN_HOME` continues under a configured critical-return speed/compute
  profile; a thermal condition that makes even that profile unsafe enters
  `FAILSAFE`.
- `HOME` and `IDLE` remain stopped and degraded.
- `FAILSAFE` remains stopped.
- If detected during `BRAKE_REVERSE`, motion stops immediately and the
  interrupted objective is replaced by `RETURN_HOME` only after clearance and
  health are re-evaluated; otherwise enter `FAILSAFE`.

## 7. Commands and acceptance rules

### 7.1 `SET_HOME`

`SET_HOME` is accepted only in `HOME` or `IDLE` and only when:

- ROS localization, map, and all required transforms are valid;
- yard/ROS calibration is valid and versioned;
- the candidate pose is inside the car geofence;
- the car is stopped;
- durable persistence succeeds.

The home record MUST be versioned, checksummed, calibration/map-associated,
and durably committed before ACK. Failure to persist is rejection, not
in-memory success. Active missions use their frozen version.

### 7.2 `START_CHASE(target_id)`

Accepted only from `HOME` or `IDLE`. Acceptance requires:

- valid durable home;
- nonempty selected `target_id`;
- a fresh overhead car observation with acceptable confidence;
- a fresh overhead cat observation for the same `target_id` with acceptable
  confidence;
- valid localization/map/calibration and car inside geofence;
- healthy `NavigationManager`;
- fresh, valid lidar and ultrasonic;
- a chaseable target inside the configured cat perimeter.

On acceptance, freeze the home version, request chase recording, and enter
`GETTING_CLOSE`.
Onboard perception or recording MAY be unavailable at acceptance if
overhead-only pursuit remains safe; the mission starts degraded and reports
the unavailable facilities. A new `START_CHASE` received during handoff IDLE
MUST identify the intended new target and enters `GETTING_CLOSE`.

### 7.3 `STOP_CHASE`

- From `GETTING_CLOSE`, `SEARCH`, or `CHASE`: stop immediately, cancel the
  active Nav2 goal, enter `IDLE`, and retain recording for a configurable
  10-second post-roll.
- From `BRAKE_REVERSE`: accepted only when the saved objective is a chase
  objective; stop reverse immediately, cancel it, enter `IDLE`, and start the
  same post-roll.
- From `HOME` or `IDLE`: idempotent success with zero motion. If an IDLE
  handoff is pending, it is canceled; any already-started post-roll ends at
  its existing deadline.
- From `GOTO`, `RETURN_HOME`, or `FAILSAFE`: reject with `INVALID_STATE`.

### 7.4 `GO_TO`

Accepted only from `HOME` or `IDLE` after destination, localization, geofence,
`NavigationManager`, lidar, and ultrasonic validation. The command contains
independent `request_yolo` and `request_recording` booleans. Neither is
implicitly enabled by `GO_TO`, and monitoring streaming remains client-driven.

### 7.5 `RETURN_HOME`

- In `HOME` while within home tolerance: idempotent success.
- From `IDLE`, any chase state, or `GOTO`: immediate stop/cancel followed by
  `RETURN_HOME` after safety validation.
- In `RETURN_HOME`: idempotent success and retain the correlated goal.
- In `BRAKE_REVERSE`: stop reverse immediately, cancel the saved objective,
  set `RETURN_HOME` as the objective, and re-evaluate clearance before motion.
- In `FAILSAFE`: reject.

If return is requested but safe return cannot be established, enter
`FAILSAFE`; do not remain in a motion-capable state.

### 7.6 `CLEAR_FAILSAFE`

Acceptance requires all of:

- explicit operator confirmation;
- cause-specific clearance for every latched cause;
- stopped motor feedback;
- healthy control loop and watchdog;
- fresh, valid lidar and ultrasonic;
- valid motion-inhibition output.

Acceptance enters a clean `IDLE`, cancels/discards all Nav2 goals, latches,
handoff context, and interrupted objectives. It does not restart a mission,
detector, recording, or stream.

The reverse attempt count does not reset merely because `CLEAR_FAILSAFE` was
accepted. It resets only after both sensors report greater than the configured
clearance continuously for the configured duration.

## 8. Reliable mission event and target handoff

### 8.1 Authority and matching

Only the overhead system may declare:

```text
PRIMARY_CAT_LEFT_PERIMETER(target_id)
```

The declaration MUST use the reliable ACKed `mission_event` envelope in
Section 13. A matching event is one whose `target_id` equals the active chase
target and whose event ID/observation sequence is new for the current mission.
Wrong-target, duplicate, regressive-sequence, and stale events are logged and
ACKed as rejected; they do not change state.

Local camera visibility never overrides a valid matching event. The car
geofence and cat perimeter are separate concepts and telemetry.

### 8.2 Result and handoff timer

A matching event in `GETTING_CLOSE`, `SEARCH`, `CHASE`, or a
`BRAKE_REVERSE` whose saved objective is chase:

1. commands immediate zero motion;
2. cancels Nav2/reverse;
3. enters `IDLE`;
4. starts a configurable 10-second handoff wait and recording post-roll.

During the handoff:

- a valid `START_CHASE(new_target_id)` enters `GETTING_CLOSE`;
- the exited target cannot be restarted from the stale event/observation;
- an explicit `RETURN_HOME` enters `RETURN_HOME`;
- timeout enters `RETURN_HOME` if safe return is possible, otherwise
  `FAILSAFE`.

The car does not promote a local secondary track. Overhead selects and names
the next target.

## 9. Formal transition model

### 9.1 Global transition rules

These rows apply before state-specific rows:

| From | Event/condition | To | Required action |
|---|---|---|---|
| Any state | Emergency stop | `FAILSAFE` | Synchronous stop, cancel all motion, latch cause. |
| Any state | Actual car geofence crossing | `FAILSAFE` | Immediate stop, cancel all motion, latch cause. |
| Any state | Motor/control fatal or watchdog | `FAILSAFE` | Immediate stop, cancel all motion, latch cause. |
| Any normal driving state | Lidar or ultrasonic unhealthy for less than 2 s | Same state | Zero-motion hold; preserve objective. |
| Any normal driving state | Required sensor does not recover by 2 s | `FAILSAFE` | Cancel objective and latch sensor cause. |
| `BRAKE_REVERSE` | Lidar or ultrasonic unhealthy | `FAILSAFE` | Immediate stop; no recovery interval. |
| `HOME` or `IDLE` | Lidar or ultrasonic unhealthy | Same state | Stay stopped and degraded; reject motion starts. |
| Any active objective except `RETURN_HOME` | Critical thermal | `RETURN_HOME` or `FAILSAFE` | Use safe critical return, else fail. |
| `RETURN_HOME` | Critical thermal | `RETURN_HOME` or `FAILSAFE` | Continue critical-return profile, else fail. |
| Any state | Recording/H.264 failure | Same state | Degraded telemetry; retry while requested. |

### 9.2 Complete command/event transition matrix

“Reject” means no state or objective mutation and an ACK with a reason.
“Same” means a successful idempotent application unless noted.

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

Every accepted motion command is still subject to Section 7 validation.

### 9.3 Autonomous transition matrix

| From | Condition | To | Required action |
|---|---|---|---|
| `GETTING_CLOSE` | Valid target distance `<= SEARCH_ENTRY_DISTANCE_CM` | `SEARCH` | Request detector consumer and SEARCH speed policy. |
| `SEARCH` | Three consecutive unambiguous associated observations | `CHASE` | Bind local track to `target_id`; enable chase policy. |
| `SEARCH` | First SEARCH interval expires without lock | `SEARCH` | Ask `NavigationManager` for one collision-free observation waypoint; reset interval once. |
| `SEARCH` | Second SEARCH interval expires without lock | `RETURN_HOME` or `FAILSAFE` | Return if safe, else fail. |
| `CHASE` | Associated local track lost and fresh valid overhead distance `<= 200 cm` | `SEARCH` | Direct transition; no intermediate state. |
| `CHASE` | Associated local track lost and fresh valid overhead distance `> 200 cm` | `GETTING_CLOSE` | Direct transition; no intermediate state. |
| `CHASE` | Associated local track lost while overhead unavailable | `RETURN_HOME` or `FAILSAFE` | Return if safe, else fail. |
| `GETTING_CLOSE` or `SEARCH` | Overhead recovers with different `target_id` | `IDLE` | Stop; require new `START_CHASE`. |
| `GETTING_CLOSE` or `SEARCH` | Overhead invalid-retention timer expires | `RETURN_HOME` or `FAILSAFE` | Return if safe, else fail. |
| `CHASE` | Overhead unavailable but associated local track and all chase permissions valid | `CHASE` | Continue local chase without a fixed overhead timeout. |
| Any normal driving state | Close trigger from either required sensor | `BRAKE_REVERSE` | Save objective and begin STOP phase. |
| `BRAKE_REVERSE` | RECHECK clear and saved objective remains valid | Saved state | Re-submit/re-evaluate objective before motion. |
| `BRAKE_REVERSE` | RECHECK blocked and attempts remain | `BRAKE_REVERSE` | Repeat phases and increment attempt count. |
| `BRAKE_REVERSE` | Blocked with attempts exhausted | `FAILSAFE` | Stop and latch exhaustion. |
| `GOTO` | Correlated completion accepted | `IDLE` | Stop/cancel goal; release requested mission consumers. |
| `GOTO` | Navigation failures exhausted | `IDLE` | Stop; report failed objective. |
| Any chase state | Navigation failures exhausted | `RETURN_HOME` or `FAILSAFE` | Return if safe, else fail. |
| `RETURN_HOME` | Correlated completion accepted | `HOME` | Stop and force perception consumers off. |
| `RETURN_HOME` | Navigation failures exhausted | `FAILSAFE` | Stop and latch return failure. |
| `IDLE` handoff | Handoff timer expires | `RETURN_HOME` or `FAILSAFE` | Return if safe, else fail. |

### 9.4 Perception-fatal transitions

| Current state | Fatal onboard camera/RKNN result |
|---|---|
| `HOME`, `IDLE` | Same state, stopped degraded. |
| `GETTING_CLOSE` | Same state, overhead-only degraded. |
| `SEARCH`, `CHASE` | Directly `GETTING_CLOSE`, overhead-only degraded. |
| `GOTO`, `RETURN_HOME` | Same state if navigation/safety valid. |
| `BRAKE_REVERSE` | Continue only under non-vision safety; re-evaluate saved objective at RECHECK. |
| `FAILSAFE` | Remain `FAILSAFE`. |

If the resulting objective requires overhead and overhead is also unavailable,
the result is `RETURN_HOME` if safe, otherwise `FAILSAFE`.

## 10. Chase and overhead behavior

### 10.1 `GETTING_CLOSE`

- `NavigationManager` tracks the selected overhead `target_id` as a moving
  Nav2 goal.
- YOLO is not required by mission policy.
- Valid car-to-target distance is computed in calibrated yard space.
- At `<= 200 cm` by default, enter `SEARCH`.
- Onboard perception failure does not stop valid overhead-only navigation.

### 10.2 Overhead stale or fresh-but-invalid

Stale and fresh-but-invalid overhead data use the same policy. Invalid includes
bad confidence, impossible geometry, calibration mismatch, missing selected
target, or unusable observation sequence.

In `GETTING_CLOSE` or `SEARCH`:

1. retain the last valid Nav2 goal for at most 10 seconds;
2. cap motion at the SEARCH speed, default `0.10 m/s`;
3. continue only while localization/geofence, `NavigationManager`, lidar, and
   ultrasonic remain valid;
4. if the same `target_id` recovers, refresh the goal and continue the current
   state;
5. if a different `target_id` appears, stop and enter `IDLE`; a new
   `START_CHASE` is required;
6. on timeout, enter `RETURN_HOME` if safe, otherwise `FAILSAFE`.

This is controlled continuation toward the last validated goal, not blind
extrapolation of a cat position.

In `CHASE`, overhead loss has no fixed timeout while all of the following
remain valid:

- the associated local track;
- localization and car-geofence status;
- `NavigationManager` and path viability;
- lidar and ultrasonic.

If the local track is lost while overhead is unavailable, return home if safe,
otherwise enter `FAILSAFE`.

### 10.3 SEARCH acquisition and timeout

SEARCH uses YOLO continuously at the configured cadence; PhaseMachine motion
gating MUST NOT suppress it.

A local observation counts toward lock only when:

- it is unambiguous;
- it is fresh and track-consistent;
- it passes a calibrated association gate comparing the overhead bearing to
  `target_id` with the onboard camera track bearing and their combined
  uncertainty;
- no competing track also passes the gate ambiguously.

Three consecutive qualifying observations bind the local track to
`target_id` and enter `CHASE`. Any miss, ambiguity, or identity conflict
resets the consecutive count.

If no lock is obtained within the first configurable 10-second interval,
remain in `SEARCH` and allow `NavigationManager` to choose exactly one
collision-free observation waypoint. If no safe waypoint exists, skip it and
continue stationary SEARCH. If the second interval expires without lock,
return home if safe, otherwise enter `FAILSAFE`.

### 10.4 CHASE fusion

Normative look/drive modes, pan gating, and steering-envelope provenance are
defined in `Look_Drive_Path_Design.md`. This section summarizes the required
behavior.

Nav2 / `NavigationManager` produces a safe steering envelope (costmap sweep in
production), speed limit, and path-viability result. `DecisionEngine` selects
exactly one look/drive mode per tick:

- `LOOK_AT` / `PATH_FOLLOW`: chassis follows `path_correction` inside the
  envelope; pan may track the bound local cat; vision `x_offset_norm` MUST NOT
  drive the chassis.
- `PAN_RESET`: pan slews to calibrated forward; chassis vision steer is frozen.
- `BODY_STEER`: allowed only while pan is within the forward deadband; then

```text
applied_steering = clamp(
    camera_steering_request,
    nav2_safe_steering_min,
    nav2_safe_steering_max
)
```

```text
applied_speed_mps = min(
    pursuit_speed_request_mps,
    nav2_speed_cap_mps,
    alignment_speed_cap_mps,
    obstacle_speed_cap_mps,
    thermal_speed_cap_mps
)
```

Path viability and every safety veto remain mandatory. Camera and Nav2
steering MUST NOT be added or combined by weighted sum. `DecisionEngine`
remains the sole drivetrain authority and the sole pan command authority.
Stale or missing costmap MUST fail closed (`path_viable=false`), never widen
the envelope.

When the local track is lost with valid overhead, transition directly to
`SEARCH` at `<= 200 cm`, or directly to `GETTING_CLOSE` at `> 200 cm`. A
two-hop `CHASE -> GETTING_CLOSE -> SEARCH` sequence is forbidden.

## 11. Navigation and geofence

### 11.1 Car geofence

The car geofence is a configured inner safe polygon. The lidar/`base_link`
center may move anywhere inside it. Crossing the boundary is an immediate
`FAILSAFE`.

There is no predictive path veto based solely on a planned path approaching or
crossing the polygon. Nav2 costmaps/planners SHOULD be configured to avoid
leaving the polygon, but the safety transition is based on the actual
localized center crossing it. Loss of sufficient localization to determine
containment is a health failure, not proof of remaining inside.

Cat exit is declared only by the overhead mission event and never inferred
from the car geofence.

### 11.2 `NavigationManager`

`NavigationManager` MUST own:

- `NavigateToPose` action clients;
- state-derived goal intents;
- yard-to-navigation-frame transforms;
- moving-goal refresh rate and displacement filtering;
- cancel/preemption;
- action result and goal-intent correlation;
- expected replacement classification;
- path viability and safe steering envelope publication;
- retries and exhausted-failure reporting;
- completion qualification.

Moving cat goals default to at most `2 Hz` and require at least `25 cm`
displacement since the last submitted goal. Safety cancellation is immediate
and bypasses rate limiting.

Replacing a moving goal intentionally is neutral: cancellation/result from the
replaced goal MUST NOT count as a failure. Late results with the wrong goal
intent/correlation ID MUST be ignored and logged.

### 11.3 Completion

`GOTO` and `RETURN_HOME` complete only when:

1. the correlated Nav2 result is `SUCCEEDED`;
2. a fresh authoritative local pose is within `20 cm` XY and `0.3 rad` yaw of
   the destination;
3. both tolerances remain satisfied continuously for `1 second`.

An action result alone is insufficient. Completion is canceled if pose
freshness or tolerance is lost during the dwell.

### 11.4 Ultrasonic costmap integration

Ultrasonic MUST be published as `sensor_msgs/Range` with validated:

- topic and frame;
- radiation type;
- field of view;
- minimum and maximum range;
- finite range value and timestamp;
- transform into the local costmap frame.

The local costmap MUST integrate it through a validated `RangeSensorLayer`.
Lidar remains integrated independently. Regardless of costmap integration,
both lidar and ultrasonic remain direct safety inputs and both are required
for autonomous motion. Disabling the costmap layer for diagnosis MUST NOT
disable ultrasonic direct safety.

### 11.5 Failure outcomes

After configured Nav2 retries/recovery are exhausted:

- `GOTO -> IDLE`;
- `GETTING_CLOSE`, `SEARCH`, or `CHASE -> RETURN_HOME` if safe, otherwise
  `FAILSAFE`;
- `RETURN_HOME -> FAILSAFE`.

Nav2 BackUp MUST remain disabled.

## 12. `BRAKE_REVERSE`

### 12.1 Trigger and entry

In `GETTING_CLOSE`, `SEARCH`, `CHASE`, `GOTO`, or `RETURN_HOME`, a fresh valid
lidar or ultrasonic reading below
`CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM` enters `BRAKE_REVERSE`. The default is
strictly `< 15 cm`.

Entry saves the interrupted objective, goal intent, and detector/recording
policy. A tracked cat triggers identical behavior; there is no arrival state.

### 12.2 Formal phases

The phases are:

1. `STOP`: immediately command and confirm zero drive; pause/cancel Nav2
   velocity authority.
2. `CENTER`: command centered steering once.
3. `SETTLE`: remain stopped for `100 ms`.
4. `REVERSE`: drive straight for `0.5 s` at normalized `-0.30` (30% reverse).
5. `STOP`: command and confirm zero drive.
6. `RECHECK`: re-read both required sensors, health, commands, event queue,
   geofence, localization, and saved objective before any later motion.

Steering MUST remain centered during settle and reverse. The reverse is direct
normalized/time-bounded control; it is not a Nav2 physical-speed command.

### 12.3 Attempts, blocked result, and reset

- Maximum attempts: `3`.
- Each actual REVERSE phase increments the attempt count.
- If RECHECK remains blocked and attempts remain, repeat within
  `BRAKE_REVERSE`.
- If blocked after the third attempt, enter `FAILSAFE`.
- The count resets only after both lidar and ultrasonic are fresh/valid and
  each reports `> 20 cm` continuously for `2 seconds`.
- Stale/invalid data never proves clearance.
- `CLEAR_FAILSAFE` does not bypass the clearance rule.

### 12.4 Preemption

During any phase:

- emergency stop, actual geofence breach, motor/control fatal/watchdog, or
  loss of either required sensor enters `FAILSAFE`;
- accepted `STOP_CHASE`, when the saved objective is chase, stops immediately
  and enters `IDLE`;
- a matching `PRIMARY_CAT_LEFT_PERIMETER` for the saved chase target stops
  immediately and enters handoff `IDLE`;
- accepted `RETURN_HOME` stops immediately and replaces the objective with
  `RETURN_HOME`.

All preemptions require a fresh clearance and permission evaluation before
later motion. A non-chase `STOP_CHASE` is rejected after first ensuring the
reverse phase is not altered.

### 12.5 Accepted bounded risk

There is no rear-facing sensor. The target accepts the bounded risk of a
straight, centered, 0.5-second reverse at normalized `-0.30`, limited to three
attempts. Production release requires explicit hardware validation on the
actual car, surface, payload, battery range, and steering calibration.

## 13. Protocol V1 target contract

### 13.1 Versioning and transactional application

The protocol remains **V1** because no upstream overhead implementation has
been deployed. This redesign therefore redefines V1 before first deployment;
backward compatibility with the repository's incomplete draft schema is not a
requirement, and this document does not declare V2.

All commands except emergency stop:

1. are received and deduplicated by command/event ID;
2. are queued for the control loop;
3. are validated and applied atomically at a control-loop boundary;
4. are ACKed only after application or rejection is committed;
5. report the resulting state and reason.

An ACK MUST NOT claim a destination state before the control loop has applied
it. Duplicate IDs return the stored result. Emergency stop remains synchronous
and is not delayed for transactional mission processing.

Every target-scoped command MUST carry `target_id`. `START_CHASE` requires it;
`STOP_CHASE` SHOULD carry the expected active `target_id` and MUST be rejected
as `WRONG_TARGET` if a different active chase target is named. Commands with no
target semantics, including `SET_HOME`, `GO_TO`, and `RETURN_HOME`, omit it.

### 13.2 Overhead observation schema

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

Every cat MUST have a stable `target_id`; the selected target is named by
`selected_target_id`, never by array position.

### 13.3 Command envelope and examples

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

```json
{
  "protocol_version": 1,
  "type": "command",
  "command_id": "cmd-a103",
  "issued_at_ms": 1785012350000,
  "name": "GO_TO",
  "args": {
    "x_m": 2.1,
    "y_m": -0.8,
    "yaw_rad": 0.0,
    "frame_id": "map",
    "request_yolo": false,
    "request_recording": true
  }
}
```

### 13.4 Reliable mission-event envelope

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

`event_id`, `target_id`, `perimeter_id`, and `observation_seq` are mandatory.
The sender retries until an ACK is received. The receiver stores deduplication
results durably enough to prevent replay after a process restart within the
defined protocol retention window.

### 13.5 ACK schema

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

Rejected ACKs MUST use `applied: false`, retain the actual state, and include a
specific reason such as `WRONG_TARGET`, `STALE_OBSERVATION`,
`DUPLICATE_SUPERSEDED`, `INVALID_STATE`, `HOME_INVALID`, or
`SAFETY_HEALTH_INVALID`.

## 14. Perception, camera, recording, and stream lifecycle

### 14.1 Ownership model

`PerceptionLifecycleManager` owns named consumers and reference counts:

- `detector`;
- `recording`;
- `stream`.

Consumers are independent. Detector and recording never depend on stream
clients. Stream reference count is the actual connected-client count.

Mission policy overrides the legacy PhaseMachine in `SEARCH`, `CHASE`, and a
`GOTO` with `request_yolo=true`. PhaseMachine MAY optimize detector cadence in
other allowed states but MUST NOT suppress a mission-required consumer.

### 14.2 Camera hardware lifecycle

Ready-inactive means the device is initialized but frame production is paused
with IMX415/V4L2 `STREAMOFF` or an equivalent verified hardware pause.
Activation MUST revalidate the device and use `STREAMON`; failed revalidation
reports camera-fatal degradation. Ready-inactive MUST not busy-loop or process
frames.

`HOME` and `FAILSAFE` force all consumers off and the camera to ready-inactive
or closed. A stream client cannot override those states.

### 14.3 Lifecycle table

| FSM state | Detector consumer | Recording consumer | Stream consumer | Camera policy |
|---|---|---|---|---|
| `HOME` | Forced off | Forced off | Forced off | Ready-inactive/closed |
| `IDLE` | Off | Post-roll/handoff only | Actual clients, unless forced off by policy | Active only with a consumer |
| `GETTING_CLOSE` | Off, unless diagnostic policy separately allowed | Chase mission request | Actual clients | Active if recording/stream needs frames |
| `SEARCH` | Required on | Chase mission request | Actual clients | Active |
| `CHASE` | Required on | Chase mission request | Actual clients | Active |
| `BRAKE_REVERSE` | Inherit saved objective request unless camera failed | Inherit saved request | Actual clients | Based on references |
| `GOTO` | Exactly `request_yolo` | Exactly `request_recording` | Actual clients | Active if any consumer |
| `RETURN_HOME` | Off | Retain active mission/post-roll request | Actual clients | Active if recording/stream needs frames |
| `FAILSAFE` | Forced off | Forced off | Forced off | Ready-inactive/closed |

An overhead-only chase can therefore run in `GETTING_CLOSE` with no active
onboard camera consumer when recording and streaming are also off.

### 14.4 Recording

Recording uses a separate hardware H.264 encoder and writes segmented,
crash-tolerant Matroska files. Requirements:

- finalize segments atomically where possible and recover incomplete segments
  after restart;
- enforce a storage quota and a minimum free-space reserve;
- delete oldest finalized segments first when retention cleanup is required;
- never delete an active segment as quota cleanup;
- stop recording, report low space, and leave the mission running if reserve
  cannot be restored;
- automatically resume while still requested after health/space recovery;
- preserve requested versus actual state in telemetry.

Chase recording starts when mission policy requests it, continues through
SEARCH/CHASE/reverse/return and handoff, and receives a 10-second post-roll
after `STOP_CHASE` or the primary-left event. `HOME` and `FAILSAFE` force it
off. `GOTO` recording follows only `request_recording`.

### 14.5 H.264 monitoring stream

Monitoring video uses H.264 hardware encoding only and runs only while at
least one actual client is connected and the FSM does not force it off. If the
monitoring H.264 hardware path is unavailable, there is no monitoring video
stream. The target has no MJPEG or software-encoding fallback.

Recording H.264 is independent of the monitoring encoder/client rule:
recording does not require a stream client and may use a separate hardware
encoder instance. Failure or lack of monitoring capacity MUST NOT stop a
healthy recording instance, and recording demand MUST NOT be represented as a
fake stream client.

## 15. Speed and command units

Nav2 policies, caps, requests, and telemetry use physical `m/s`. Conversion to
the PiCar-X normalized motor command uses a calibrated approximate mapping
derived from `speed_time_distance` measurements. Calibration MUST cover the
production surface, payload, representative battery range, and forward speed
range; saturation and deadband are explicit.

`BRAKE_REVERSE` is the sole specified direct normalized/time-bounded maneuver:
normalized `-0.30` for `0.5 s` after the 100 ms settle.

Telemetry MUST record at least:

- requested physical speed in `m/s`;
- all applied physical speed caps;
- requested steering and Nav2 safe envelope;
- final applied normalized drive command;
- final applied steering command;
- mapping/calibration version;
- whether the command came from Nav2 policy or direct reverse;
- every zero-motion veto reason.

## 16. Target configuration defaults

Environment names MUST use the `CAT_FOLLOW_*` namespace. The following are
canonical defaults unless a row explicitly requires deployment calibration.

| Configuration | Default | Meaning |
|---|---:|---|
| `CAT_FOLLOW_SEARCH_ENTRY_DISTANCE_CM` | `200` | Enter SEARCH at or below this valid overhead distance. |
| `CAT_FOLLOW_SEARCH_SPEED_CAP_MPS` | `0.10` | SEARCH and retained-goal speed cap. |
| `CAT_FOLLOW_SEARCH_LOCK_OBSERVATIONS` | `3` | Consecutive unambiguous associated observations. |
| `CAT_FOLLOW_SEARCH_INTERVAL_SEC` | `10` | Each of the two SEARCH intervals. |
| `CAT_FOLLOW_LOCAL_TRACK_STALE_MS` | `350` | Local track expiration threshold. |
| `CAT_FOLLOW_OVERHEAD_INVALID_MAX_SEC` | `10` | Last valid goal retention in GETTING_CLOSE/SEARCH. |
| `CAT_FOLLOW_SENSOR_RECOVERY_SEC` | `2` | Required-sensor zero-motion recovery interval. |
| `CAT_FOLLOW_HANDOFF_WAIT_SEC` | `10` | IDLE wait after primary target exit. |
| `CAT_FOLLOW_RECORDING_POSTROLL_SEC` | `10` | Recording post-roll after chase stop/exit. |
| `CAT_FOLLOW_NAV_MOVING_GOAL_MAX_HZ` | `2` | Maximum moving-target goal submission rate. |
| `CAT_FOLLOW_NAV_MOVING_GOAL_MIN_DISPLACEMENT_CM` | `25` | Minimum displacement for a normal refresh. |
| `CAT_FOLLOW_NAV_COMPLETION_XY_CM` | `20` | Local-pose XY completion tolerance. |
| `CAT_FOLLOW_NAV_COMPLETION_YAW_RAD` | `0.3` | Local-pose yaw completion tolerance. |
| `CAT_FOLLOW_NAV_COMPLETION_DWELL_SEC` | `1` | Continuous in-tolerance dwell. |
| `CAT_FOLLOW_BRAKE_REVERSE_TRIGGER_CM` | `15` | Trigger on a fresh valid reading strictly below this. |
| `CAT_FOLLOW_BRAKE_REVERSE_SETTLE_MS` | `100` | Centered, stopped settle before reverse. |
| `CAT_FOLLOW_BRAKE_REVERSE_DURATION_SEC` | `0.5` | Bounded reverse duration. |
| `CAT_FOLLOW_BRAKE_REVERSE_NORMALIZED` | `-0.30` | Direct normalized reverse command. |
| `CAT_FOLLOW_BRAKE_REVERSE_MAX_ATTEMPTS` | `3` | Maximum actual reverse phases before failsafe. |
| `CAT_FOLLOW_BRAKE_REVERSE_RESET_CM` | `20` | Both sources must be strictly above this to reset. |
| `CAT_FOLLOW_BRAKE_REVERSE_RESET_SEC` | `2` | Continuous dual-source clearance duration. |
| `CAT_FOLLOW_NAV_ULTRASONIC_COSTMAP` | `1` | Enable validated RangeSensorLayer integration. |
| `CAT_FOLLOW_NAV2_BACKUP_ENABLED` | `0` | Nav2 BackUp is disabled and MUST remain disabled. |
| `CAT_FOLLOW_OVERHEAD_MIN_CONFIDENCE` | deployment-calibrated | Minimum valid car/cat confidence; startup fails if absent. |
| `CAT_FOLLOW_ASSOCIATION_BEARING_GATE_RAD` | deployment-calibrated | Base uncertainty gate; startup rejects SEARCH capability if absent. |
| `CAT_FOLLOW_RECORDING_QUOTA_BYTES` | deployment-required | Maximum retained finalized recording bytes. |
| `CAT_FOLLOW_RECORDING_MIN_FREE_BYTES` | deployment-required | Free-space reserve below which recording stops. |

Input freshness limits for lidar, ultrasonic, localization, overhead, and
motor feedback MUST also be explicit deployment configuration validated
against measured publication rates. No component may silently substitute a
different timeout.

## 17. Startup and durable-home sequence

Before accepting autonomous motion:

1. load and verify the durable home record, map, geofence, and calibration
   versions;
2. initialize RF2O lidar odometry and local Nav2/SLAM localization;
3. obtain a fresh overhead yard car pose;
4. transform it through the calibrated yard-to-map relationship;
5. seed or validate ROS localization within configured startup tolerances;
6. make local Nav2/SLAM localization authoritative;
7. validate `NavigationManager`, control loop, motor feedback, lidar, and
   ultrasonic;
8. publish readiness/degradation reasons.

Subsequent overhead car poses MAY be used for telemetry and bounded validation
but MUST NOT continuously overwrite authoritative local localization or hide
RF2O drift/failure.

## 18. Current implementation conflicts

The current repository does **not** implement this target. Known conflicts
include:

1. **Resolved.** `cat_follow/control/types.py` and `cat_follow/control/fsm.py`
   implement the canonical states (`GETTING_CLOSE`, `SEARCH`, `CHASE`,
   `BRAKE_REVERSE`, `GOTO`, `RETURN_HOME`, `FAILSAFE`, `HOME`, `IDLE`) and this
   `FSM` is the live control authority instantiated in `cat_follow/runtime/app.py`.
   `CHASE_A`, `TRACK_B`, and `BRAKE` remain only as backward-compatible wire
   aliases for legacy V1 integrations (`FsmState._missing_`), not as the
   internal state representation. Full transition-table parity against the
   Interface and Data Contract Specification section 10 has not been
   independently re-audited.
2. **Resolved.** `DecisionEngine.close_obstacle_trigger_cm`
   (`cat_follow/control/decision_engine.py`) uses
   `max(target_config.brake_reverse_trigger_cm, safety_config.obstacle_too_close_cm)`
   (15 cm target threshold vs. the 10 cm operator-facing floor, whichever is more
   conservative) and triggers the recoverable `BRAKE_REVERSE` FSM state via
   `FsmEvent.BRAKE_REVERSE_TRIGGERED`; a routine close-obstacle event no longer
   latches `FAILSAFE`.
3. Current chase overhead expiry uses an approximately 700 ms failsafe rather
   than state-specific retained-goal/local-track/return behavior.
4. The ROS navigation bridge consumes navigation output but currently has no
   mission Nav2 goal-output path; it does not provide the required complete
   `NavigationManager` moving-goal output, refresh, cancel, correlation, and
   completion behavior.
5. Current chase/navigation authority is advisory/incomplete and does not
   implement the camera-request clamp inside a Nav2 safe steering envelope.
6. The camera prototype is effectively always active rather than managed by
   named consumers, reference counts, and STREAMOFF/STREAMON readiness.
7. Detector activation is primarily PhaseMachine/motion-gated rather than
   mission-policy-required in SEARCH/CHASE and requested GOTO.
8. Hardware H.264 segmented Matroska recording, storage quota, reserve, crash
   recovery, post-roll, and degraded retry are not implemented.
9. The protocol lacks stable `target_id` on overhead cats/selection and chase
   commands.
10. The reliable ACKed `mission_event` envelope and
    `PRIMARY_CAT_LEFT_PERIMETER` transaction do not exist.
11. `PerceptionLifecycleManager` and the specified `NavigationManager` do not
    exist as target ownership boundaries.
12. Durable versioned home, active-mission home freezing, and transactional
    `SET_HOME` acceptance are incomplete or absent.
13. Startup overhead pose seeding/validation followed by local authoritative
    localization is not implemented as specified.
14. Ultrasonic is not fully published/validated as `sensor_msgs/Range` and
    integrated through a validated local-costmap `RangeSensorLayer`.
15. Current health handling does not implement the dual-required-sensor
    2-second hold, stationary degradation, and reverse-immediate-fail rules.
16. Current completion does not require correlated Nav2 success plus fresh
    local pose tolerance for one second.
17. Current thermal, handoff, target-identity, SEARCH association/timeout, and
    direct CHASE-loss transitions are incomplete.
18. Existing documents may still claim bicycle odometry fallback, final
    `BRAKE`, MJPEG/software fallback, predictive geofence veto, or blanket
    inherited timing. Those claims are superseded here and must be removed.

These are migration gaps, not permission to partially reinterpret the target.

## 19. Validation requirements

### 19.1 FSM and protocol

Tests MUST cover:

- every cell in the command/event matrix, including rejection and idempotency;
- each global failsafe source from every state;
- transactional ACK after control-loop application and duplicate replay;
- wrong target, stale sequence, duplicate event, and matching primary-left
  behavior;
- 10-second handoff with new target, explicit return, timeout return, and
  unsafe timeout failsafe;
- durable/versioned `SET_HOME`, persistence failure, and mission home freeze;
- START_CHASE validation, including degraded camera/recording acceptance;
- direct CHASE-loss transitions without a two-hop transient;
- all objective-specific navigation exhaustion outcomes;
- cause-specific `CLEAR_FAILSAFE` and clean-IDLE context disposal.

### 19.2 Health and degradation

Tests MUST cover lidar and ultrasonic independently and together:

- zero-motion hold and recovery before 2 seconds in every driving state;
- failsafe at 2 seconds;
- immediate failure during every reverse phase;
- stopped degradation in HOME/IDLE;
- blocked motion-command acceptance while degraded;
- clearance reset requiring both sources `>20 cm` continuously for 2 seconds.

Also cover camera/RKNN failure in every state, combined camera plus overhead
loss, recording/stream failure without FSM transition, localization loss, and
critical thermal return/failure.

### 19.3 Chase, navigation, and geofence

Tests MUST cover:

- 200 cm inclusive SEARCH entry;
- same-target overhead recovery, different-target stop, and 10-second timeout;
- CHASE continuation under overhead loss only while every listed permission
  remains valid;
- three-observation bearing/uncertainty association, ambiguity reset, and
  target identity preservation;
- first SEARCH timeout with at most one safe observation waypoint and second
  timeout return/fail;
- non-additive steering clamp, speed caps, path viability, and safety vetoes;
- moving-goal 2 Hz/25 cm filtering and immediate safety cancel;
- neutral expected replacements and ignored late action results;
- completion requiring action success plus pose/dwell;
- actual geofence crossing failsafe, legal movement everywhere inside, and no
  predictive path-veto transition;
- overhead-only authority for cat exit.

### 19.4 Reverse hardware validation

Bench and production-car tests MUST validate:

- STOP/CENTER/100 ms/REVERSE/STOP/RECHECK phase timing;
- normalized `-0.30` and 0.5-second travel across battery, payload, and surface
  ranges;
- centered steering and no steering updates during reverse;
- each permitted entry state and saved-objective restoration;
- STOP_CHASE, primary-left, RETURN_HOME, and hard-fault preemption in every
  phase;
- three blocked attempts and exhaustion;
- low obstacles seen only by ultrasonic;
- explicit acceptance evidence for the no-rear-sensor bounded risk.

### 19.5 Lifecycle, recording, and headless operation

Tests MUST cover every lifecycle-table row, reference-count races, STREAMOFF
idle behavior, STREAMON revalidation failure, and forced-off HOME/FAILSAFE.
SEARCH/CHASE detector operation MUST work with no web UI or stream clients.

Recording tests MUST cover segmented Matroska crash recovery, quota deletion
of oldest finalized files, active-file protection, low-space stop, mission
continuation, health recovery/resume, and post-roll. Monitoring tests MUST
prove client-only H.264 behavior and absence of MJPEG/software fallback.

### 19.6 Long-duration and fault-injection validation

Run long-duration missions with dropped/reordered overhead messages, ROS action
replacement races, storage exhaustion, detector restart, thermal throttling,
sensor staleness, localization drift, and process restart. Telemetry MUST make
every transition, hold, retry, consumer reference, goal correlation, and
applied command reconstructable.

## 20. Migration and documentation impact

Expected implementation areas include, but are not limited to:

- `cat_follow/control/types.py`
- `cat_follow/control/fsm.py`
- `cat_follow/control/decision_engine.py`
- `cat_follow/comms/messages.py`
- `cat_follow/comms/comms_manager.py`
- `cat_follow/runtime/app.py`
- `cat_follow/runtime/control_loop.py`
- `cat_follow/runtime/shared_state.py`
- `cat_follow/threads/camera.py`
- `cat_follow/threads/detector.py`
- `cat_follow/perception/phase.py`
- `cat_follow/perception/vision_adapter.py`
- `cat_follow/perception/h264_encoder.py`
- new `PerceptionLifecycleManager`, recording, and storage components
- new `NavigationManager` and ROS action/goal-correlation components
- `cat_follow/navigation/ros_bridge.py`
- `cat_follow/safety_config.py`
- `cat_follow/web_ui/routes_h264.py`
- `ros_ws/cat_follow_bringup/config/nav2_params.yaml`

The corrected Nav2 configuration impact path is
`ros_ws/cat_follow_bringup/config/nav2_params.yaml`.

Migration MUST update the PRD, HLD, system architecture, detailed software
architecture, interface/data contract, integration guide, validation matrix,
service environment, telemetry schema, operator UI, deployment configuration,
and overhead implementation together. Protocol remains V1 until the overhead
implementation is delivered and interoperability testing passes.

Implementation should be staged behind explicit readiness criteria. No partial
deployment may advertise this document as current behavior until the complete
FSM, protocol, safety, navigation, lifecycle, and validation requirements are
implemented and accepted.
