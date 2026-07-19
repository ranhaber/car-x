# System Architecture Document
**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)  
**Based on:** PRD v1.1  
**Version:** 1.0  
**Status:** Baseline Architecture (Ready for Implementation)

## 1. Purpose
This document defines the implementation architecture for the autonomous chase system described in the PRD. It translates product behavior into runtime modules, control loops, packet contracts, state transitions, and safety guards.

## 2. Architecture Principles
- Detection and tracking must continue without web UI dependency.
- Global sensing (overhead camera) is strategic guidance, not direct steering.
- Local sensing/planning owns tactical movement and all obstacle decisions.
- Safety posture is conservative and has hard priority over pursuit.
- Control precedence is fixed: `failsafe > obstacle veto > pursuit logic`.

## 3. Runtime Topology

### 3.1 Process Model
- Single Python application process on Raspberry Pi 4.
- Dedicated processing/control thread for real-time loops.
- Optional web UI process/thread for monitoring/configuration only.

### 3.2 Threads and Responsibilities
- **Main thread**
  - Bootstraps modules, loads config, starts worker threads.
  - Handles lifecycle (start/stop/restart).
- **Control thread (real-time)**
  - Runs state machine.
  - Executes control arbitration and motor commands.
  - Target loop frequency: 50-100 Hz.
- **Overhead comm thread**
  - Receives overhead packets (~10 Hz nominal).
  - Maintains latest validated global snapshot.
  - Tracks staleness/heartbeat timing.
- **Onboard vision thread**
  - Detects cat in local camera FOV.
  - Outputs target centroid and detection confidence.
- **Range/obstacle thread**
  - Polls the Lidar C1 and ultrasonic sensors.
  - Produces obstacle veto and range-to-target data.
- **SLAM/navigation thread**
  - Computes local obstacle-aware steering path.
  - Accepts global goal vector or local camera target cue.
- **Telemetry logger thread**
  - Logs state transitions, control outputs, packet timing, and safety events.

## 4. Core Modules

### 4.1 `StateMachine`
- Owns canonical state: `HOME`, `IDLE`, `CHASE_A`, `TRACK_B`, `BRAKE`, `GOTO`, `RETURN_HOME`, `FAILSAFE`.
- Applies timeout rules and transition guards.

### 4.2 `OverheadClient`
- Receives and validates overhead packets.
- Maintains:
  - latest packet
  - packet age
  - home position
  - stop_chase command status

### 4.3 `PerceptionLocal`
- Wraps local camera target detection.
- Provides:
  - `cat_detected: bool`
  - `cat_center_x_norm: float [-1..1]`
  - `detection_confidence: float`
  - `last_seen_ms`

### 4.4 `RangeSafety`
- Wraps Lidar C1 and ultrasonic sampling and filtering. _(TMF8829 dToF is on hold; not used in the current build.)_
- Provides:
  - `target_distance_cm` (when target measurable)
  - `obstacle_veto: bool`
  - `range_confidence`

### 4.5 `NavigatorLocal`
- Consumes desired heading/goal and obstacle map.
- Produces safe steering command.
- Can issue "no progress" / "dead-end" status.

### 4.6 `PursuitController`
- Implements stage-specific logic:
  - Stage A: overhead-guided pursuit intent
  - Stage B: visual centering
  - Stage C: dToF final brake

### 4.7 `SafetySupervisor`
- Enforces hard rules:
  - packet staleness thresholds
  - boundary checks
  - no-progress escalation
  - fail-safe motor cut

### 4.8 `MotorInterface`
- Single output boundary to motion hardware.
- Exposes:
  - `set_motion(speed, steering)`
  - `stop()`
  - `emergency_stop()`

## 5. Data Flow

### 5.1 High-Level Flow
1. Overhead packet arrives -> validated and timestamped.
2. Control loop computes pursuit intent from global data.
3. Local SLAM adjusts path to avoid obstacles.
4. If cat detected in local camera, switch to visual centering.
5. dToF handles final proximity braking.
6. Safety supervisor can override any stage at any time.

### 5.2 Overhead Packet Schema
```json
{
  "timestamp": 1234567890,
  "cat": { "x": 0.0, "y": 0.0, "confidence": 1.0 },
  "car": { "x": 0.0, "y": 0.0, "heading": 0.0, "confidence": 1.0 },
  "command": "none"
}
```

`command` values:
- `none`
- `stop_chase`

## 6. Control Design

### 6.1 Stage A - Global Guidance + Local Navigation
- Input: overhead car/cat positions.
- Pursuit computes desired heading toward cat.
- Local SLAM converts desired heading into obstacle-safe steering.
- Overhead does not directly command steering actuators.

### 6.2 Stage B - Local Visual Tracking
- Entry: cat detected in onboard camera FOV.
- Control objective: keep cat centered in frame.
- Steering from normalized horizontal error:
  - `error = (cat_x - center_x) / image_width`
  - dead zone around zero to reduce jitter.
- Speed scales inversely with alignment error.
- Obstacle veto remains active.

### 6.3 Stage C - Final Braking
- Distance control transfers to the Lidar C1 (with ultrasonic support).
- Vehicle decelerates and stops at target proximity.
- Final stop command issued by local range logic, not overhead.

## 7. State Machine and Transitions

### 7.1 Primary Transitions
- `HOME -> CHASE_A`: `start_chase` accepted with valid car/cat tracking.
- `IDLE -> CHASE_A`: `start_chase` accepted with valid car/cat tracking.
- `CHASE_A -> TRACK_B`: local camera acquires cat.
- `TRACK_B -> CHASE_A`: camera lost for longer than fallback timeout.
- `TRACK_B -> BRAKE`: final approach condition met by local logic.
- `BRAKE -> TRACK_B`: if target moves away before stop is complete.
- `HOME -> GOTO`: `go_to` accepted.
- `IDLE -> GOTO`: `go_to` accepted.
- `GOTO -> IDLE`: go-to target reached.
- `ANY_CHASE_STATE -> IDLE`: `stop_chase` accepted.
- `ANY_NON_FAILSAFE -> RETURN_HOME`: `return_home` accepted with valid home coordinates.
- `RETURN_HOME -> HOME`: return-home completed successfully.
- `ANY -> FAILSAFE`: obstacle distance <10 cm or critical obstacle condition.
- `ANY -> FAILSAFE`: safety violation or unrecoverable fault.

### 7.2 Failsafe/Timeout Rules
- Overhead stale warning: `> 300 ms` -> reduce speed.
- Overhead stale fail: `> 700 ms` -> hard-stop in `FAILSAFE`.
- Camera-loss fallback in `TRACK_B`: `> 350 ms` -> return to `CHASE_A`.
- No-progress detect: `> 2.0 s` -> recovery behavior.
- Recovery timeout: `> 5.0 s` blocked -> `FAILSAFE`.

## 8. Arbitration Logic (Authoritative)
At each control tick:
1. If failsafe condition true -> emergency stop.
2. Else if obstacle veto true -> obstacle-safe command only.
3. Else -> stage pursuit command (A/B/C).

This precedence is mandatory and not configurable in V1.

## 9. Configuration
Configuration lives in versioned config files and is loaded at startup.

Required runtime parameters:
- loop rates (control, vision, range)
- timeout values from PRD
- camera centering gains and dead-zone width
- brake target distance
- speed caps per stage
- boundary polygon for perimeter safety

## 10. Observability and Logging
Log at minimum:
- state transitions with timestamp
- overhead packet age and drop/stale events
- local detection confidence and loss events
- obstacle veto assertions
- control outputs (speed/steering)
- failsafe triggers and reason codes

Log format should be machine-parseable (JSON lines recommended).

## 11. Failure Handling
- **Comms degradation:** speed reduction then failsafe by thresholds.
- **Camera detection loss:** fallback from `TRACK_B` to `CHASE_A`.
- **Dead-end/no progress:** recovery maneuver then failsafe escalation.
- **Sensor confidence low:** conservative speed limiting; no aggressive actuation.
- **Thermal constraints:** throttle-aware speed limiting (detailed values TBD).

## 12. Implementation Sequence
1. Build and test state machine skeleton with simulated events.
2. Integrate overhead packet client and stale-time watchdog.
3. Integrate local camera tracker output interface.
4. Integrate Lidar C1 / ultrasonic range and obstacle veto.
5. Wire arbitration and motor output boundary.
6. Add telemetry and replay tooling.
7. Run calibration and tuning passes.

## 13. Open Issues (Carried from PRD)
- Validation matrix completed in `Validation_Matrix_Autonomous_Yard_Navigator_Cat_Tracker.md`.
- Exact SLAM implementation choice remains `TBD` (interface defined, algorithm pluggable).
- Braking calibration constants remain `TBD` (to be measured and tuned).
- Thermal thresholds/fan policy values remain `TBD`.

## 14. Ready-for-Next Document
With this architecture baseline, the next technical artifacts are:
- Detailed interface spec (module APIs and message enums)
- Control tuning guide (gains, filters, brake calibration table)
- Validation matrix and acceptance test protocol
