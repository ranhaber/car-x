# High-Level Design (HLD)
**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)  
**Based on:** `PRD_Autonomous_Yard_Navigator_Cat_Tracker.md`  
**Version:** 1.1  
**Status:** Draft for Architecture Review

## 1. System Overview
The system is a multi-layer real-time robotics control architecture for a PiCar-X platform.

It combines:
- Overhead global tracking for strategic guidance.
- Onboard camera tracking for local cat alignment.
- Local navigation/SLAM for obstacle-aware movement.
- Lidar C1 and ultrasonic range sensing for obstacle veto and final range sensing.
- A central `DecisionEngine` as the only module that produces final motion decisions.

The design follows a conservative safety model. The car keeps chasing until the overhead system sends `stop_chase`, unless a higher-priority safety condition overrides the mission. `stop_chase` stops the chase and transitions to `IDLE`; `return_home` is a separate command that transitions to `HOME` only after return-home completes.

## 2. Core Design Principle
Only one module decides final motion: `DecisionEngine`.

All other modules either:
- observe the environment
- report state
- provide constraints
- execute a validated command

The FSM stores and validates the active mode, but it does not independently choose motion.

## 3. High-Level Pipeline
```text
Overhead Camera -> CommsManager -> Shared State

Onboard Camera -> VisionTracker  -> Shared State
Lidar C1 + US  -> RangeSafety    -> Shared State
Local Nav/SLAM -> Navigation     -> Shared State

Shared State -> DecisionEngine -> FSM validation -> MotorInterface
                          |
                          v
                    Telemetry/Logs
```

The web UI is optional and must not be required for detection, tracking, or control.

## 4. Authority Hierarchy
Every control tick applies this fixed priority:
1. **Failsafe**
2. **Obstacle veto**
3. **DecisionEngine pursuit decision**
4. **MotorInterface execution**

Sensor modules do not own authority. They provide inputs and constraints to the `DecisionEngine`.

If `stop_chase`, `return_home`, and a safety condition occur together, safety wins first. `RETURN_HOME` is allowed only when the vehicle is not in an unrecoverable safety condition.

## 5. Hardware Layer
- **Compute:** Raspberry Pi 4B (4GB)
- **Chassis:** SunFounder PiCar-X
- **Global tracking:** overhead camera system
- **Local vision:** RPi Camera Module (5MP)
- **Local depth/obstacle:** Slamtec RPLIDAR C1 (2D dToF lidar) plus ultrasonic (HC-SR04) hardware. _Note: the ams OSRAM TMF8829 dToF sensor is on hold; Lidar C1 + ultrasonic are the current range hardware._
- **Actuation:** PiCar-X drive motors and steering servo
- **Power:** 2x 18650 high-discharge cells

## 6. Coordinate and Timing Assumptions
- Coordinates and distances are expressed in centimeters (`cm`) throughout the system.
- Yard frame:
  - `+X` = right in yard
  - `+Y` = forward in yard
- Overhead nominal update rate is approximately 10 Hz.
- Global position error from overhead is up to +/- 25 cm.
- Control loop target frequency is 50 Hz for `DecisionEngine`; 20 Hz is the degraded minimum before reducing speed or stopping.
- Overhead host and PiCar-X should use NTP/Chrony for cross-device log correlation.
- Safety/control freshness uses PiCar-X local monotonic receive time, not synchronized wall-clock time.

## 7. Runtime Modules

### 7.1 `CommsManager`
Receives overhead data and command messages.

Responsibilities:
- receive overhead updates at approximately 10 Hz
- validate packet freshness
- maintain latest global car/cat position
- track accepted command state, including `stop_chase` and `return_home`
- expose stale/wifi status

Outputs:
- `car_x`, `car_y`, `car_heading` when available
- `cat_x`, `cat_y`
- packet timestamp and packet age
- command state

`car_heading` from overhead is optional and non-authoritative. Local navigation/SLAM owns authoritative car-local heading (`theta`).

### 7.2 `VisionTracker`
Runs onboard cat detection/tracking.

Responsibilities:
- detect cat in the car camera FOV
- report stable visual lock
- report horizontal offset from frame center
- report detection confidence and last-seen time

Outputs:
- `cat_visible`
- `cat_visible_stable`
- `x_offset_norm`
- `confidence`

### 7.3 `RangeSafety`
Reads the Lidar C1 and ultrasonic sensors and produces local range/safety facts.

Responsibilities:
- detect near obstacles
- provide critical obstacle veto
- provide valid range data during final approach
- expose range confidence

Important constraint: local range sensing (Lidar C1 / ultrasonic) alone does not identify the cat. Final approach uses range only when visual tracking confirms the cat is the active local target.

### 7.4 `Navigation`
Runs as a dedicated navigation thread.

The implementation is pluggable and remains TBD. The HLD requires the interface and responsibility boundary:
- consume global pursuit direction or local visual target cue
- account for obstacles and dead ends
- output steering constraints or avoidance recommendation
- report `no_progress` / `dead_end`
- own authoritative car-local heading (`theta`) and local path/map state when the selected implementation supports it

### 7.5 `DecisionEngine`
The central decision maker.

Responsibilities:
- select requested state transitions
- perform sensor arbitration
- enforce safety precedence
- compute final speed/steering/brake request
- choose between global chase, local chase, final approach, return-home, and failsafe behavior
- emit decision reasons for telemetry

Nominal rate: 50 Hz target, with 20 Hz degraded minimum.

Inputs:
- overhead state from `CommsManager`
- local visual state from `VisionTracker`
- range/obstacle state from `RangeSafety`
- navigation constraints from `Navigation`
- current FSM state
- thermal/system health state

Output:
```json
{
  "requested_state": "IDLE | CHASE_A | TRACK_B | BRAKE | GOTO | RETURN_HOME | HOME | FAILSAFE",
  "steering": 0.0,
  "speed": 0.0,
  "brake": false,
  "reason": "string"
}
```

### 7.6 `FSM`
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
| `HOME` | At the home position after return-home completion |
| `IDLE` | Stationary, safe, not chasing, not necessarily at home |
| `CHASE_A` | Overhead-guided pursuit with local navigation constraints |
| `TRACK_B` | Onboard camera-based local chase |
| `BRAKE` | Final range-based stop behavior |
| `GOTO` | Navigate to a supplied coordinate |
| `RETURN_HOME` | Return to supplied home coordinates after `return_home` |
| `FAILSAFE` | Emergency stop / zero-motion state |

### 7.7 `MotorInterface`
The only module that writes to drivetrain hardware.

Responsibilities:
- execute validated speed/steering commands
- perform controlled stop
- perform emergency stop
- prevent direct motor access from perception or navigation modules

### 7.8 `TelemetryLogger`
Records state, sensor, decision, and safety events.

Logs should go to systemd journal for service visibility and to structured JSONL telemetry for replay/tuning.

## 8. Control Strategy

### 8.1 Stage A: Global Chase
Active when:
- overhead has a valid cat/car stream
- local camera has not yet established stable cat detection

Behavior:
- overhead provides cat target position and global guidance
- `DecisionEngine` computes desired pursuit intent
- `Navigation` constrains movement around obstacles/dead ends
- overhead never directly commands steering or motors

### 8.2 Stage B: Local Chase
Active when:
- cat is detected in the onboard camera FOV for the required stability window

Behavior:
- `VisionTracker` provides horizontal cat offset
- `DecisionEngine` keeps cat centered in frame
- speed decreases when cat alignment is poor
- local navigation and obstacle veto remain active
- overhead is retained for fallback/context but ignored for direct steering

### 8.3 Stage C: Final Approach
Active when:
- local visual lock is active
- Lidar C1 range is valid for the current target direction
- final approach condition is met

Behavior:
- dToF range controls the final stop decision
- visual tracking preserves target identity and direction
- obstacle veto remains active
- final stop target is 10 cm per PRD

Local range sensing alone cannot identify the cat. Final approach and braking require a fresh visual lock so the ranged object is still associated with the tracked cat.

## 9. Control Fusion
The `DecisionEngine` may blend non-critical steering influences:
```text
steering_request =
    w_camera * camera_error
  + w_navigation * navigation_correction
```

Critical obstacle handling is never blended. It is a veto:
```text
if failsafe:
    emergency_stop()
elif obstacle_distance_cm < 10:
    enter_failsafe("obstacle_too_close")
elif obstacle_critical:
    stop_or_escape()
else:
    apply_pursuit_decision()
```

Speed is bounded by:
```text
speed = min(
    pursuit_speed,
    alignment_speed_limit,
    obstacle_distance_limit,
    thermal_speed_limit
)
```

## 10. Handover Logic
Global-to-local handover occurs only when the onboard camera confirms the cat in the local FOV.

Entry to `TRACK_B`:
- `cat_visible_stable >= 3 frames`

Distance estimates may reduce speed or increase caution, but distance alone does not trigger local handover.

Exit from `TRACK_B`:
- cat not detected for more than 350 ms -> return to `CHASE_A`

## 11. Timeout Policy
- **Overhead stale warning:** `> 300 ms` -> reduce speed to safe crawl.
- **Overhead stale failsafe:** `> 700 ms` -> enter hard-stop `FAILSAFE`.
- **Camera-loss fallback:** `> 350 ms` without local cat detection in `TRACK_B` -> return to `CHASE_A`.
- **No-progress/dead-end detection:** `> 2.0 s` -> trigger recovery behavior.
- **Recovery-to-failsafe escalation:** `> 5.0 s` blocked/no progress -> enter `FAILSAFE`.

## 12. Thermal Policy
The system uses conservative thermal defaults for V1:
- **75C:** thermal warning; log event and reduce nonessential workload where possible.
- **80C:** speed-limit active; reduce maximum chase speed.
- **85C:** stop chase behavior and transition to `RETURN_HOME` when safe and home is known; enter `FAILSAFE` if safe return is not possible.

Active cooling is required for daytime operation.

## 13. Shared State Model
Modules publish into a synchronized snapshot consumed by the `DecisionEngine`.

The exact timestamp, freshness, authority, and confidence fields for each group are defined in the Detailed Software Architecture and the future Interface & Data Contract Specification. The HLD schema below is illustrative only.

All `x`, `y`, and distance-like fields in shared state use centimeters (`cm`).

```json
{
  "car": {
    "x": 0.0,
    "y": 0.0,
    "heading": 0.0,
    "confidence": 1.0
  },
  "home": {
    "x": 0.0,
    "y": 0.0,
    "set": true
  },
  "cat_global": {
    "x": 0.0,
    "y": 0.0,
    "confidence": 1.0
  },
  "cat_local": {
    "visible": true,
    "stable": true,
    "x_offset_norm": 0.0,
    "confidence": 1.0
  },
  "obstacle": {
    "detected": false,
    "critical": false,
    "severity": 0.0
  },
  "system": {
    "overhead_packet_age_ms": 0,
    "wifi_ok": true,
    "thermal_c": 0.0,
    "timestamp_ms": 0
  }
}
```

## 14. ACK Movement Protocol
The vehicle sends movement/transition acknowledgements to the overhead system for stationary-to-motion transitions and movement-type events.

This HLD requires ACK behavior but does not define the packet schema. The exact ACK/status schema is deferred to the detailed interface specification.

## 15. Observability
The system logs:
- state transitions
- sensor snapshots
- overhead packet freshness
- local cat detection/loss
- obstacle veto events
- thermal events
- decision reasons
- motor command outputs
- failsafe triggers
- `stop_chase` and return-home events

Logs should be written to:
- systemd journal for service-level operations
- structured JSONL telemetry for replay and tuning

## 16. Open Issues
- Exact `Navigation` / SLAM implementation remains TBD.
- ACK/status packet schema is deferred to the detailed interface specification.

## 17. Next Documents
- Detailed Interface Specification
- Local Navigation / SLAM design
- Control tuning guide
- Test execution plan based on the validation matrix
