# Validation Matrix
**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)  
**Version:** 1.0  
**Status:** Baseline Acceptance Matrix

## 1. Purpose
This document defines the minimum test matrix, pass/fail criteria, and evidence required to validate PRD v1.1 and the System Architecture baseline.

## 2. Test Conditions
- Environment: daylight only, synthetic grass, configured perimeter active.
- Hardware: PiCar-X, Radxa ROCK 4D, Robot HAT, ultrasonic (HC-SR04),
  onboard camera, overhead camera link, and Slamtec RPLIDAR C1 when available.
  _(TMF8829 dToF is on hold; C1 and camera validation are pending.)_
- Build: release-equivalent runtime configuration.
- Safety posture: conservative.

## 3. Evidence Required Per Test
- Timestamped run log.
- State transition log.
- Key telemetry: packet age, cat detection status, veto events, motor command outputs.
- Final verdict: pass/fail with reason code.

## 4. Acceptance Criteria Summary
- Chase continues until overhead sends `stop_chase`.
- On `stop_chase`, system stops chase/tracking behavior and transitions to `IDLE`.
- On `return_home`, system transitions to `RETURN_HOME` and then `HOME` after completion.
- Control precedence is never violated: `failsafe > obstacle veto > pursuit logic`.
- Timeout actions occur within configured thresholds.
- No collision events in validation suite.

## 5. Validation Matrix

| Test ID | Scenario | Setup | Expected Behavior | Pass Criteria |
|---|---|---|---|---|
| VM-01 | Boot to standby | System powered, no chase command | Enters `HOME` then ready state without motor motion | No unexpected motion; no failsafe |
| VM-02 | Start chase path | Valid overhead stream and start command | `HOME -> CHASE_A` or `IDLE -> CHASE_A` | Correct transition and stable control loop |
| VM-03 | Stage A behavior | Cat tracked by overhead, not yet in local FOV | Uses overhead for target intent and local planner for steering | Steering comes from local planner; no direct overhead motor command |
| VM-04 | Stage A to B handover | Cat appears in local FOV | `CHASE_A -> TRACK_B` | Transition occurs within 250 ms after stable detection |
| VM-05 | Local visual centering | Cat visible off-center in frame | Vehicle steers to reduce horizontal error | Error magnitude decreases over time; no unstable oscillation |
| VM-06 | Camera-loss fallback | In `TRACK_B`, suppress local cat detection for >350 ms | Returns to `CHASE_A` | Transition occurs within timeout + 1 control cycle |
| VM-07 | Final approach and stop | `TRACK_B` with decreasing dToF distance | Enters `BRAKE`, stops at local proximity target | Stop command issued at target threshold; no forward creep after stop |
| VM-08 | Obstacle veto during chase | Place obstacle ahead while in `CHASE_A` | Obstacle logic overrides pursuit | Veto asserted; command reflects avoidance or stop |
| VM-09 | Obstacle veto during local track | Place obstacle ahead while in `TRACK_B` | Obstacle logic still overrides tracking | No collision; veto precedence preserved |
| VM-10 | Overhead stale warning | Delay overhead packets to >300 ms and <700 ms | Speed reduced to safe crawl | Speed reduction logged before failsafe |
| VM-11 | Overhead stale failsafe | Delay overhead packets to >700 ms | Enter `FAILSAFE` hard-stop | Hard-stop executed and reason logged |
| VM-12 | Dead-end recovery | Create blocked route with no progress >2 s | Recovery behavior triggered | Recovery mode entered and logged |
| VM-13 | Recovery escalation | Keep blocked condition >5 s | Escalates to `FAILSAFE` | Failsafe entered with dead-end reason |
| VM-14 | Boundary enforcement | Drive toward perimeter boundary | Boundary breach triggers safety action | Stops before/at boundary with failsafe reason |
| VM-15 | stop_chase command | Send `stop_chase` during active chase | Stops chase/tracking and transitions to `IDLE` | Transition occurs within 200 ms of command receipt |
| VM-16 | Return home command and completion | Send `return_home` with valid home coordinates | Transitions to `RETURN_HOME`, navigates home, then enters `HOME` | Reaches home tolerance zone and stops safely |
| VM-17 | go_to command and completion | Send `go_to` with valid target coordinates | Transitions to `GOTO`, navigates to target, then enters `IDLE` | Reaches target tolerance zone and stops safely |
| VM-18 | Obstacle too close | Place obstacle <10 cm from car | Enters `FAILSAFE` with `obstacle_too_close` reason | Hard-stop executed; no collision |
| VM-19 | Precedence audit | Inject simultaneous pursuit and hazard commands | Arbitration order maintained | No cycle violates precedence rule |
| VM-20 | Endurance run | 20-minute mixed scenario run | Stable operation without unsafe event | No collision, no unhandled exception, deterministic recovery |

### 5.1 ROCK 4D platform bring-up evidence (2026-07-19)

| Test | Result | Evidence |
|------|--------|----------|
| I2C8 / Robot HAT MCU | Pass | `0x14` detected; non-root ADC transaction completed |
| Direct GPIO | Pass | Motor direction, MCU reset, ultrasonic trigger/echo verified |
| MCU PWM | Pass | P0/P1/P2 servos and P12/P13 motors exercised |
| Motor backend | Pass | Elevated-wheel forward, reverse, stop, and emergency stop completed |
| Runtime service | Pass | Real `PiCarXBackend` service started and stopped cleanly |
| Power stability | Pass (bench) | Dual-rail test completed without ROCK reset |
| MIPI camera | Pending | Camera hardware/software bring-up not yet run |
| RPLidar C1 | Pending | Hardware not yet available |

## 6. Quantitative Thresholds
- Stage transition reaction target: <= 250 ms unless timeout-governed.
- `stop_chase` reaction target: <= 200 ms.
- Overhead stale thresholds:
  - warning at `>300 ms`
  - failsafe at `>700 ms`
- Camera-loss fallback threshold: `>350 ms`.
- Dead-end trigger: `>2.0 s`; escalation at `>5.0 s`.

## 7. Failure Classification
- **Critical fail:** collision, missing failsafe, precedence violation.
- **Major fail:** incorrect state transition, timeout action missing/late.
- **Minor fail:** non-critical oscillation, delayed but safe response.

Any critical fail blocks release.

## 8. Test Execution Order (Recommended)
1. Functional flow tests: VM-01 through VM-07.
2. Safety/timeout tests: VM-08 through VM-14.
3. Command/return tests: VM-15 through VM-17.
4. Robustness tests: VM-18 through VM-20.

## 9. Exit Criteria
Validation suite is considered complete when:
- All critical tests pass.
- Zero critical fails.
- Any major/minor failures have approved disposition.
- Results are archived with logs and configuration snapshot.
