# Product Requirements Document (PRD)
**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)  
**Version:** 1.1  
**Status:** Finalized for Architecture Phase

## 1. Executive Summary
The goal is to develop a high-speed autonomous pursuit robot using a tiered global-to-local sensing architecture. The overhead "god camera" provides global target and vehicle positions for strategic guidance, while onboard camera, dToF, and local navigation/SLAM provide tactical movement, obstacle handling, and final approach behavior.

## 2. Target Environment and Constraints
- **Surface:** Synthetic grass (consistent friction, low height noise).
- **Lighting:** Daylight only (optimized for high ambient light up to 100k lux).
- **Global Accuracy:** +/- 25 cm from overhead camera system.
- **Operating Window:** Daytime; no IR-cut switching or external illumination required.
- **Coordinate Frame:**  
  - `+X` = right in yard  
  - `+Y` = forward in yard  
  - Units = centimeters (`cm`) for all coordinates and distances

## 3. Functional Requirements

### 3.1 Navigation and Pursuit
- **FR-1: High-Speed Chase:** The vehicle prioritizes velocity when a cat target is identified, with predictive guidance based on overhead updates.
- **FR-2: 10 cm Proximity Stop:** Final stopping is controlled locally using Lidar C1 (and ultrasonic) distance data.
- **FR-2a: Final Target Identity:** Local range data (Lidar C1 / ultrasonic) may authorize final stop only while onboard vision maintains a fresh cat lock; range alone must not be treated as proof that the ranged object is the cat.
- **FR-3: Dual-Stage Handover:**  
  - **Stage A:** Overhead-guided target pursuit with local navigation handling actual steering/pathing.  
  - **Stage B:** Once cat is detected in onboard camera FOV, vehicle keeps the cat centered in frame (unless obstacle veto applies).  
- **FR-4: ACK Movement Protocol:** Vehicle verifies orientation/motion transitions through overhead ACK exchange on movement-type events.
- **FR-5: Chase Stop Condition:** Car continues chase until overhead sends `stop_chase`; then stops motion, stops cat chase/tracking behavior, and transitions to `IDLE`.
- **FR-6: Heading Authority:** Car-local navigation/SLAM owns authoritative heading (`theta`). Overhead heading, if provided, is an optional global observation and is not authoritative for local steering.
- **FR-7: Return Home Command:** `return_home` is a separate reliable command and must include home coordinates. The car transitions to `HOME` only after return-home completes successfully.

### 3.2 Safety and Obstacle Avoidance
- **FR-8: Daytime Range Sensing:** Use the Lidar C1 plus ultrasonic hardware for local obstacle/range sensing in daytime conditions. _(The TMF8829 dToF sensor originally specified for this role is on hold.)_
- **FR-9: Emergency Veto:** Local obstacle detection and local planner (SLAM/navigation) have veto power over pursuit commands.
- **FR-10: Boundary and Comms Safety:** Car hard-stops on perimeter breach or prolonged comms staleness.
- **FR-11: Conservative Safety Posture:** Safety-first behavior is mandatory for V1 operation.

### 3.3 Control Arbitration (Authoritative Priority)
Final control precedence is fixed as:
1. **Failsafe**
2. **Obstacle veto**
3. **Pursuit logic**

## 4. Data Contracts (In Scope Interface)
Detection algorithm internals are out of scope. The system assumes overhead provides cat/car positions.

All position and distance fields use centimeters (`cm`) unless explicitly documented otherwise.

### 4.1 Overhead to Car Packet
```json
{
  "timestamp": 1234567890,
  "cat": {
    "x": 0.0,
    "y": 0.0,
    "confidence": 1.0
  },
  "car": {
    "x": 0.0,
    "y": 0.0,
    "heading": 0.0,
    "confidence": 1.0
  }
}
```

`car.heading` is optional/non-authoritative. It may be used as a global observation, but car-local navigation/SLAM owns authoritative heading for local control.

## 5. Timing and Timeout Requirements
- **Overhead nominal update:** Approximately 10 Hz.
- **Overhead stale warning threshold:** `> 300 ms` (reduce speed to safe crawl).
- **Overhead stale failsafe threshold:** `> 700 ms` (enter hard-stop FAILSAFE).
- **Camera-loss fallback (during local tracking):** `> 350 ms` without cat detection -> return to Stage A.
- **No-progress/dead-end detection:** `> 2.0 s` -> trigger recovery behavior.
- **Recovery-to-failsafe escalation:** `> 5.0 s` blocked/no progress -> FAILSAFE.
- **Clock synchronization:** Overhead host and PiCar-X should use NTP/Chrony for cross-device log correlation.
- **Freshness authority:** Control and failsafe freshness calculations must use PiCar-X local monotonic receive time, not synchronized wall-clock time.

## 6. Hardware Specification
- **Chassis:** SunFounder PiCar-X.
- **Compute:** Raspberry Pi 4B (4GB).
- **Local Depth/Range:** Slamtec RPLIDAR C1 (2D dToF lidar) plus ultrasonic (HC-SR04). _(ams OSRAM TMF8829 dToF is on hold.)_
- **Local Vision:** RPi Camera Module (5MP).
- **Power:** 2x 18650 high-discharge cells (targeting >5A burst for maneuvers).

## 7. Software State Machine
| State | Description | Transition Condition |
|---|---|---|
| **HOME** | Car has completed return-home and is at the home position. | Start command received or home updated. |
| **IDLE** | Stationary, safe, not chasing, not necessarily at home. | Start command, return-home command, or manual/dev command. |
| **CHASE_A** | Overhead-guided chase with local pathing and obstacle handling. | Cat enters local FOV or fallback/timeout logic triggers. |
| **TRACK_B** | Local camera-based target centering with obstacle veto active. | Cat lost timeout, stop command, or final approach condition. |
| **BRAKE** | Local dToF-based final proximity stop logic. | Distance target reached or safety override. |
| **GOTO** | Navigate to a supplied coordinate. | `go_to` command accepted. |
| **RETURN_HOME** | Return to supplied home coordinates. | `return_home` command accepted. |
| **FAILSAFE** | Zero-power/hard stop. | Comms loss, boundary breach, obstacle too close, or unrecoverable safety condition. |

## 8. Success Criteria
- Vehicle continues active chase until overhead explicitly sends `stop_chase`.
- On `stop_chase`, vehicle stops motion, stops cat chase/tracking behavior, and transitions to `IDLE`.
- On `return_home`, vehicle navigates to the supplied home coordinates and transitions to `HOME` only after completion.
- Safety precedence is never violated: failsafe and obstacle veto always override pursuit intent.

## 9. Open Issues and Future Considerations
- **[CLOSED]** Heading ambiguity addressed through explicit ownership: navigation/SLAM owns authoritative car-local heading.
- **[CLOSED]** Global-vs-local role separation defined (overhead guidance vs local tactical control).
- **[CLOSED]** Heading ownership clarified: navigation/SLAM owns authoritative car-local heading; overhead heading is optional/non-authoritative.
- **[CLOSED]** Conservative safety posture and timeout thresholds defined.
- **[CLOSED]** Validation matrix completed in `Validation_Matrix_Autonomous_Yard_Navigator_Cat_Tracker.md`.


