# Canonical Target Validation Matrix
**Project:** Autonomous Yard Navigator and Cat Tracker  
**Target platform:** Radxa ROCK 4D, Radxa 4K IMX415, RKNN, ROS 2/Nav2, RF2O  
**Document version:** 2.0  
**Status:** Target acceptance matrix — target behavior pending implementation and validation  
**Authority:** `Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`

## 1. Scope and status rules

This matrix validates the canonical target, not the current executable. The
target FSM, protocol, `NavigationManager`, perception lifecycle, recording,
and reverse behavior are not implemented yet. Every target test below is
**Pending** until implementation evidence is attached. Section 12 preserves
existing hardware bring-up evidence; a hardware pass must not be reported as a
target-behavior pass.

Production validation uses the ROCK 4D, IMX415 camera, RKNN inference,
RPLIDAR C1 with RF2O lidar odometry, Nav2, and forward HC-SR04 ultrasonic.
Bicycle/wheel odometry, MJPEG/software video fallback, and lidar-only or
ultrasonic-only autonomous motion are outside the supported target.

## 2. Evidence required for every target test

- build/configuration identity, calibration/map/home versions, and monotonic
  timestamped logs;
- command/event ID, mission ID, `target_id`, observation sequence, ACK,
  applied control sequence, and resulting-state reason where applicable;
- state entry, objective, goal-intent/action correlation, health/freshness,
  geofence, consumer references, reverse phase/attempt, and thermal profile;
- requested speed/steering, every cap/envelope/veto, normalized motor output,
  and confirmation of zero motion;
- pass/fail result with retained artifacts. Hardware tests also require surface,
  payload, battery, steering calibration, and measured travel.

## 3. State coverage

| ID | State/scenario | Required result | Status |
|---|---|---|---|
| FSM-01 | `HOME` entry/completion | Stopped within frozen-home tolerance; all perception consumers forced off | Pending |
| FSM-02 | `IDLE` normal and handoff | Stopped; accepts only valid commands; handoff context/timer observable | Pending |
| FSM-03 | `GETTING_CLOSE` | Selected overhead `target_id` drives filtered moving Nav2 goals; detector not required | Pending |
| FSM-04 | `SEARCH` | SEARCH speed; detector continuously required without UI/stream; timeout stage retained | Pending |
| FSM-05 | `CHASE` | Bound local track; camera steering clamped to Nav2 safe envelope; all vetoes retained | Pending |
| FSM-06 | `BRAKE_REVERSE` | Saved objective/policy, formal phases, attempts, and preemption are observable | Pending |
| FSM-07 | `GOTO` | Explicit Nav2 destination; YOLO and recording exactly match independent request flags | Pending |
| FSM-08 | `RETURN_HOME` | Uses mission-frozen durable home and critical-return policy; completes only after qualified dwell | Pending |
| FSM-09 | `FAILSAFE` | Latched zero motion; goals/reverse canceled; cause-specific operator clearance required | Pending |

## 4. Complete command/event transition coverage

Each row means one test for every listed source state. Acceptance still
requires the canonical validation gates; rejection must preserve state and
objective and return a committed reason.

| ID | Input | Source states and expected result | Status |
|---|---|---|---|
| CMD-01 | `SET_HOME` | `HOME`,`IDLE`: durable update/same state; all other states: reject | Pending |
| CMD-02 | `START_CHASE(target_id)` | `HOME`,`IDLE`: `GETTING_CLOSE`; all other states: reject | Pending |
| CMD-03 | `STOP_CHASE` | chase states: `IDLE`+post-roll; reverse: `IDLE` only for saved chase; `HOME`,`IDLE`: idempotent; `GOTO`,`RETURN_HOME`,`FAILSAFE`: reject | Pending |
| CMD-04 | `GO_TO` | `HOME`,`IDLE`: `GOTO`; every other state: reject | Pending |
| CMD-05 | `RETURN_HOME` | `HOME`: idempotent in tolerance; `IDLE`, chase states, `GOTO`: `RETURN_HOME`; reverse: stop/replace objective/recheck; `RETURN_HOME`: idempotent correlated goal; `FAILSAFE`: reject | Pending |
| CMD-06 | matching primary-left event | chase states and reverse with matching saved chase: handoff `IDLE`; all other contexts: reject/no active target | Pending |
| CMD-07 | `CLEAR_FAILSAFE` | `FAILSAFE`: clean `IDLE` only after every clearance check; all other states: reject | Pending |
| CMD-08 | `START_CHASE` acceptance gates | Validate durable home, nonempty target, fresh confident matching car/cat, calibration/localization/geofence, NavigationManager, both sensors, and cat perimeter | Pending |
| CMD-09 | degraded chase acceptance | Camera/RKNN or recording unavailable is accepted only when safe overhead-only pursuit remains valid and degradation is reported | Pending |
| CMD-10 | `GO_TO` acceptance/flags | Reject invalid destination/navigation/geofence/sensors; prove `request_yolo` and `request_recording` are independent and stream remains client-driven | Pending |
| CMD-11 | unsafe return request | Any return request without safe-return viability enters `FAILSAFE`, never a motion-capable state | Pending |
| CMD-12 | wrong-target stop | `STOP_CHASE` naming a different active target returns `WRONG_TARGET` without mutation | Pending |

## 5. Autonomous transitions, target identity, and handoff

| ID | Scenario | Required result | Status |
|---|---|---|---|
| AUTO-01 | Target distance crosses to exactly `200 cm` | `GETTING_CLOSE -> SEARCH`; inclusive threshold | Pending |
| AUTO-02 | SEARCH association succeeds | Three consecutive fresh, unambiguous, track-consistent bearing-gated observations bind the track and enter `CHASE` | Pending |
| AUTO-03 | SEARCH miss/ambiguity/conflict | Consecutive-lock count resets; no identity promotion | Pending |
| AUTO-04 | First SEARCH 10 s timeout | Remain `SEARCH`; request at most one collision-free observation waypoint; reset interval once | Pending |
| AUTO-05 | No safe SEARCH waypoint | Remain stationary in SEARCH for second interval | Pending |
| AUTO-06 | Second SEARCH timeout | `RETURN_HOME` if safe, otherwise `FAILSAFE` | Pending |
| AUTO-07 | CHASE local-track loss, overhead `<=200 cm` | Direct `CHASE -> SEARCH`; no two-hop transient | Pending |
| AUTO-08 | CHASE local-track loss, overhead `>200 cm` | Direct `CHASE -> GETTING_CLOSE` | Pending |
| AUTO-09 | CHASE local-track loss, overhead unavailable | `RETURN_HOME` if safe, otherwise `FAILSAFE` | Pending |
| AUTO-10 | Same-target overhead recovery | Refresh goal and retain current `GETTING_CLOSE`/`SEARCH` state | Pending |
| AUTO-11 | Different target appears | Stop and enter `IDLE`; require a new `START_CHASE` | Pending |
| AUTO-12 | Matching primary-left event | Immediate stop/cancel, handoff `IDLE`, 10 s handoff and recording post-roll | Pending |
| AUTO-13 | Wrong/stale/duplicate/regressive primary-left | ACK rejected and logged; no state change; local visibility cannot override matching event | Pending |
| AUTO-14 | Handoff receives new target | Valid `START_CHASE(new_target_id) -> GETTING_CLOSE`; exited target cannot restart from stale data | Pending |
| AUTO-15 | Handoff return/timeout | Explicit return enters `RETURN_HOME`; timeout returns if safe, else `FAILSAFE` | Pending |
| AUTO-16 | GOTO completion/failure | Qualified completion -> `IDLE`; exhausted navigation failures -> `IDLE` with failed objective | Pending |
| AUTO-17 | Chase navigation exhaustion | `RETURN_HOME` if safe, otherwise `FAILSAFE` | Pending |
| AUTO-18 | Return completion/failure | Qualified completion -> `HOME`; exhausted failures -> `FAILSAFE` | Pending |

## 6. Overhead validity, navigation, and geofence

| ID | Scenario | Required result | Status |
|---|---|---|---|
| NAV-01 | Stale overhead in `GETTING_CLOSE`/`SEARCH` | Retain last valid goal for at most 10 s, cap at `0.10 m/s`, and continue only with all listed permissions | Pending |
| NAV-02 | Fresh-but-invalid overhead | Same policy as stale for confidence, geometry, calibration, missing target, and sequence failures | Pending |
| NAV-03 | Invalid-retention timeout | `RETURN_HOME` if safe, otherwise `FAILSAFE`; no blind extrapolation | Pending |
| NAV-04 | Overhead loss in `CHASE` | Continue without fixed timeout only while associated track, localization/geofence, NavigationManager/path, lidar, and ultrasonic remain valid | Pending |
| NAV-05 | Moving-goal filtering | At most 2 Hz and only after at least 25 cm displacement; safety cancellation bypasses limits | Pending |
| NAV-06 | Goal replacement race | Expected replacement is neutral; late/wrong-correlation results ignored and logged | Pending |
| NAV-07 | NavigationManager ownership | Owns transforms, intents, action clients, refresh, cancellation, correlation, path viability, steering envelope, retries, and completion | Pending |
| NAV-08 | Steering/speed fusion | Camera steering is clamped—not added/weighted—to Nav2 envelope; speed is minimum of pursuit and all caps | Pending |
| NAV-09 | Completion qualification | Correlated `SUCCEEDED` plus fresh local pose within 20 cm/0.3 rad continuously for 1 s | Pending |
| NAV-10 | Completion dwell interruption | Stale/out-of-tolerance pose cancels dwell; action result alone never completes | Pending |
| NAV-11 | Actual car geofence crossing | Immediate `FAILSAFE` from every state; center containment is authoritative | Pending |
| NAV-12 | Legal inside-geofence path | Motion anywhere inside is not failed solely because a planned path approaches/crosses boundary | Pending |
| NAV-13 | Geofence observability loss | Immediate stop while moving; fail if safe return cannot be established | Pending |
| NAV-14 | Cat perimeter authority | Only reliable overhead event declares cat exit; never infer it from car geofence | Pending |
| NAV-15 | Ultrasonic costmap | Valid `sensor_msgs/Range` and RangeSensorLayer integration; disabling layer does not disable direct safety | Pending |
| NAV-16 | RF2O/startup authority | Seed/validate from overhead once, then local Nav2/SLAM is authoritative; no continuous overwrite or bicycle fallback | Pending |
| ENV-01 | Costmap sweep envelope | Local costmap sweep publishes contiguous free `[safe_steering_min, max]` containing path_correction when viable; `envelope_source=costmap_sweep` | Pending |
| ENV-02 | Stale/missing costmap | `path_viable=false`; zero/empty envelope; no silent `[-1,1]` synthesis; no motion on that envelope | Pending |
| ENV-03 | Point envelope fallback | `envelope_source=point` only when explicitly configured for test/fallback; never production default with ROS nav | Pending |
| LOOK-01 | LOOK_AT eligibility | CHASE + fresh bound track + error ≤ N_enter + pan can center → LOOK_AT; chassis uses path_correction only | Pending |
| LOOK-02 | Vision chassis gate | Vision `x_offset_norm` never steers chassis while pan outside forward deadband | Pending |
| LOOK-03 | PAN_RESET before BODY_STEER | Leaving LOOK_AT for vision body steer always passes through PAN_RESET to calibrated forward | Pending |
| LOOK-04 | BODY_STEER clamp | With pan at forward, applied steer = clamp(x_offset_norm, envelope); never summed with path_correction | Pending |
| LOOK-05 | Mode hysteresis | N_exit > N_enter and mode dwell prevent LOOK_AT↔BODY_STEER chatter | Pending |
| LOOK-06 | Safety pan forward | BRAKE_REVERSE / FAILSAFE / HOME / IDLE / GOTO / RETURN_HOME command pan to calibrated forward | Pending |
| LOOK-07 | Ambiguous track | Association ambiguity disables look chase; no pan tracking of unbound boxes | Pending |

## 7. Safety, dual sensors, thermal, and degraded perception

| ID | Scenario | Required result | Status |
|---|---|---|---|
| SAFE-01 | E-stop from every state | Synchronous zero, cancel all motion, latch `FAILSAFE` | Pending |
| SAFE-02 | Geofence breach from every state | Immediate zero/cancel and latched `FAILSAFE` | Pending |
| SAFE-03 | Motor/control fatal or watchdog from every state | Immediate zero/cancel and latched `FAILSAFE` | Pending |
| SAFE-04 | Lidar stale/invalid/faulted in each normal driving state | Immediate zero hold, retain objective; resume only if both sensors healthy before 2 s | Pending |
| SAFE-05 | Ultrasonic stale/invalid/faulted in each normal driving state | Same independent test and result as SAFE-04 | Pending |
| SAFE-06 | Either/both sensors fail for 2 s | Cancel objective and latch `FAILSAFE` | Pending |
| SAFE-07 | Sensor degradation in `HOME`/`IDLE` | Remain stopped/degraded; reject later motion until both healthy | Pending |
| SAFE-08 | Required sensor loss during every reverse phase | Immediate `FAILSAFE`; no recovery hold | Pending |
| SAFE-09 | Clearance reset | Attempt count resets only after both fresh/valid sensors are strictly `>20 cm` for 2 s | Pending |
| SAFE-10 | `CLEAR_FAILSAFE` | Requires operator confirmation, cause clearance, stopped feedback, healthy loop/watchdog/sensors/inhibition; discards objectives/goals/consumers | Pending |
| SAFE-11 | Camera/RKNN fatal by state | `SEARCH`/`CHASE -> GETTING_CLOSE`; other state outcomes exactly match canonical perception-fatal table | Pending |
| SAFE-12 | Camera failure plus overhead unavailable | Overhead-dependent objective returns if safe, else `FAILSAFE`; never changes target identity | Pending |
| SAFE-13 | Recording/H.264/storage failure | No FSM transition or motion veto; degraded telemetry and automatic retry while requested | Pending |
| SAFE-14 | Localization/NavManager viability loss | Immediate stop while moving; `FAILSAFE` if safe return cannot be established | Pending |
| SAFE-15 | Critical thermal, active non-return objective | `RETURN_HOME` under safe critical profile, otherwise `FAILSAFE` | Pending |
| SAFE-16 | Critical thermal in `RETURN_HOME` | Continue critical-return profile, or `FAILSAFE` if unsafe | Pending |
| SAFE-17 | Critical thermal in `HOME`/`IDLE`/`FAILSAFE` | Remain stopped with appropriate degraded/latched status | Pending |
| SAFE-18 | Critical thermal during reverse | Stop immediately; replace with return only after clearance/health recheck, else fail | Pending |
| SAFE-19 | Precedence fault injection | Exercise all 15 canonical precedence levels simultaneously/in pairs; no lower input reauthorizes vetoed motion | Pending |

## 8. `BRAKE_REVERSE` production validation

| ID | Scenario | Required result | Status |
|---|---|---|---|
| REV-01 | Entry from each normal driving state | Either fresh valid sensor strictly `<15 cm` enters reverse and saves exact objective/goal/consumer policy | Pending |
| REV-02 | Formal phase timing | STOP/confirm, CENTER once, SETTLE 100 ms, REVERSE, STOP/confirm, RECHECK in order | Pending |
| REV-03 | Reverse output | Centered steering, no steering updates, normalized `-0.30` for 0.5 s; not a Nav2 m/s command | Pending |
| REV-04 | Clear recheck | Revalidate permissions and re-submit/re-evaluate saved objective before restoring saved state motion | Pending |
| REV-05 | Blocked retries | Each actual reverse increments count; at most three attempts | Pending |
| REV-06 | Third blocked recheck | Immediate stopped `FAILSAFE` with exhaustion latched | Pending |
| REV-07 | `STOP_CHASE` preemption in every phase | Saved chase stops immediately -> `IDLE`+post-roll; non-chase reject leaves phase unaltered | Pending |
| REV-08 | Primary-left preemption in every phase | Matching saved target stops -> handoff `IDLE`; wrong target rejected | Pending |
| REV-09 | `RETURN_HOME` preemption in every phase | Stop, replace objective, and require fresh clearance before return motion | Pending |
| REV-10 | Hard-fault preemption in every phase | E-stop/geofence/control/watchdog/either sensor immediately stops -> `FAILSAFE` | Pending |
| REV-11 | Hardware travel envelope | Measure 0.5 s travel over production surface, payload, battery, and steering calibration ranges | Pending |
| REV-12 | Low obstacle/no rear sensor | Ultrasonic-only low obstacle triggers; explicitly accept bounded straight-reverse risk | Pending |

## 9. Protocol, durable home, and transactional ACKs

| ID | Scenario | Required result | Status |
|---|---|---|---|
| PROTO-01 | Command transaction | Deduplicate, queue, atomically apply/reject at control boundary, then ACK actual state/reason/control sequence | Pending |
| PROTO-02 | Duplicate command/event replay | Return stored result without reapplication, including within restart retention window | Pending |
| PROTO-03 | ACK ordering race | No ACK claims destination state before control-loop commit | Pending |
| PROTO-04 | Emergency stop | Remains synchronous and bypasses mission transaction queue | Pending |
| PROTO-05 | Observation identity/schema | Stable target IDs, selected ID, finite/ranged fields, calibrated coordinates, monotonic usable sequence | Pending |
| PROTO-06 | Mission-event reliability | Mandatory IDs/perimeter/sequence; sender retries; receiver deduplicates and gives specific rejection reasons | Pending |
| HOME-01 | `SET_HOME` durable commit | Accept only stopped in `HOME`/`IDLE` with valid transforms/map/calibration/geofence; checksum/version and commit before ACK | Pending |
| HOME-02 | Persistence failure/corruption | Reject; no in-memory success; detect checksum/calibration/map mismatch at startup | Pending |
| HOME-03 | Mission home freeze | Freeze version at mission acceptance; forbid updates during mission; return to frozen pose | Pending |
| HOME-04 | Startup readiness | Verify home/map/geofence/calibration, RF2O, overhead seed, local authority, NavigationManager, control, motor, and both sensors before motion | Pending |

## 10. Perception lifecycle, recording, H.264, and headless operation

| ID | Scenario | Required result | Status |
|---|---|---|---|
| LIFE-01 | `HOME` | Detector/recording/stream forced off; camera ready-inactive or closed | Pending |
| LIFE-02 | `IDLE` | Detector off; recording only for post-roll/handoff; stream equals actual clients; camera active only with consumer | Pending |
| LIFE-03 | `GETTING_CLOSE` | Detector off except diagnostics; chase recording request and actual stream clients independently drive camera | Pending |
| LIFE-04 | `SEARCH` | Detector required, chase recording requested, actual stream clients independent, camera active | Pending |
| LIFE-05 | `CHASE` | Detector required, chase recording requested, actual stream clients independent, camera active | Pending |
| LIFE-06 | `BRAKE_REVERSE` | Inherit saved detector/recording requests; stream remains actual clients; camera follows references | Pending |
| LIFE-07 | `GOTO` | Detector exactly `request_yolo`; recording exactly `request_recording`; stream actual clients | Pending |
| LIFE-08 | `RETURN_HOME` | Detector off; retain mission/post-roll recording; stream actual clients; camera active only if needed | Pending |
| LIFE-09 | `FAILSAFE` | Detector/recording/stream forced off; camera ready-inactive or closed | Pending |
| LIFE-10 | Reference-count races | Concurrent connect/disconnect and mission changes never leak, underflow, or suppress required consumers | Pending |
| LIFE-11 | Ready-inactive camera | Verified IMX415/V4L2 STREAMOFF or equivalent; no frame processing/busy loop | Pending |
| LIFE-12 | Activation/revalidation | STREAMON revalidates device; failure reports camera-fatal degradation | Pending |
| LIFE-13 | Headless SEARCH/CHASE | Detection/tracking continue with no browser or stream client; PhaseMachine cannot suppress required detector | Pending |
| REC-01 | H.264 recording | Separate hardware encoder writes segmented crash-tolerant Matroska and recovers incomplete segments | Pending |
| REC-02 | Storage retention | Enforce quota/reserve; oldest finalized first; never delete active segment | Pending |
| REC-03 | Low space/failure recovery | Stop recording only, mission continues, requested/actual telemetry differs, resume automatically on recovery | Pending |
| REC-04 | Mission/post-roll policy | Chase recording spans search/chase/reverse/return/handoff and 10 s post-roll; GOTO follows exact flag | Pending |
| STR-01 | Monitoring clients | Hardware H.264 runs only for actual clients and allowed states; zero clients stops encode | Pending |
| STR-02 | No fallback/independence | No MJPEG/software fallback; monitoring failure does not stop recording and recording is not a fake stream client | Pending |

## 11. Endurance and release gates

Run long-duration fault injection with reordered/dropped overhead packets,
action replacement races, sensor and planner silence, localization drift,
storage exhaustion, camera/detector restart, thermal throttling, and process
restart. Telemetry must reconstruct every transition, hold, retry, consumer,
goal correlation, ACK, and applied command. Any collision, missed hard stop,
authority/precedence violation, target-identity substitution, or motion based
on stale/invalid required data is a release-blocking critical failure.

## 12. Preserved current hardware and implementation evidence

The following evidence predates the canonical target matrix. It remains useful
for platform readiness only and does **not** pass any Pending target test above.
References to legacy runtime behavior describe what was measured, not required
target behavior.

### 12.1 ROCK 4D platform bring-up evidence (2026-07-19)

| Test | Result | Evidence |
|------|--------|----------|
| I2C8 / Robot HAT MCU | Pass | `0x14` detected; non-root ADC transaction completed |
| Direct GPIO | Pass | Motor direction, MCU reset, ultrasonic trigger/echo verified; production uses libgpiod edge worker on `gpiochip2:16` / `gpiochip1:21` |
| MCU PWM | Pass | P0/P1/P2 servos and P12/P13 motors exercised |
| Motor backend | Pass | Elevated-wheel forward, reverse, stop, and emergency stop completed |
| Runtime service | Pass | Real `PiCarXBackend` service started and stopped cleanly |
| Power stability | Pass (bench) | Dual-rail test completed without ROCK reset |
| MIPI camera | Pass | Radxa Camera 4K / IMX415 detected at I2C5 `0x1a`; RKISP captured 30 frames at 30 FPS and produced a visible image |
| Current perception implementation | Pass (host, non-target) | Motion-gated detector, RKNN-only idle unload/reload, adaptive OpenCV threads + affinity, lores motion contract, event-driven HC-SR04, camera fatal escalation, and safety regressions landed; `python -m pytest tests -q`: **511 passed** on 2026-07-26. Canonical lifecycle and board hardware tests remain Pending. |
| Edge ultrasonic (ROCK 4D) | Pass (bench, 2026-07-23) | `CatFollow-UltrasonicIRQ` active on core 3; busy-wait CPU spike eliminated; ranging via cached edge timestamps | Journal shows edge worker ready; VM-ULTRA-1..4 criteria met on deployed board |
| ROS 2 / Nav2 / C1 integration | Hardware bring-up pass; canonical navigation validation pending | `cat_follow_bringup` package implemented; C1 health OK and `/scan` steady at ~10 Hz on ROCK 4D. The canonical `NavigationManager`, goal output/correlation, completion, costmap, and safety tests remain Pending. |
| Current contract web UI monitoring | Pass (host, non-target) | `--web-ui` on `runtime.app`; `/api/status` exposes SharedSnapshot + perception diagnostics; Control page shows constraints/lidar/nav/phase and `/api/map`. Canonical client-only H.264/no-fallback tests remain Pending. |
| RPLidar C1 | Pass (bring-up) | C1 detected on `/dev/rplidar` -> `ttyUSB0`; firmware 1.02, hardware rev 18, health OK; Standard mode `/scan` measured at ~10 Hz on 2026-07-22 |
| LiDAR odometry (RF2O) | Stationary sanity pass; motion/target validation pending | 2026-07-22: lidar-source mapping launch showed one RF2O `/odom` publisher at ~10 Hz, `odom->base_link`, ~1-3 cm drift over 25 s stationary, and active slam_toolbox. Motion, stress, startup-authority, and canonical failure-policy tests remain Pending. |

### 12.2 Current repair-plan evidence (not canonical acceptance)

| Gate | Current evidence | Status |
|------|------------------|--------|
| Optional web monitoring isolation | Runtime catches Web UI import/init/thread/server/TLS failures; H.264 dependency or encoder failure disables video; headless core remains active | Pass (host contracts) |
| Mutating control fail-closed | Incomplete production token pair returns `503` on guarded mutations and prevents UDP ingress; bad configured web token returns `401`; stop/e-stop remain open | Pass (host contracts) |
| Camera transient/fatal tiers | Open/read/dequeue events retry to configured limits; no-publish, persistent camera, self-test/QBUF, and detector fatal paths e-stop, latch `FAILSAFE`, stop the app, and rely on systemd `Restart=on-failure` | Pass (host contracts); board fault injection Pending |
| H.264 ownership | One admitted viewer; one pending camera lease; matching-PTS release; delayed access units polled without a newer frame; teardown clears counters | Pass (host contracts); ROCK 4D direct MPP 30/30 access units; WebSocket/browser and long soak Pending |
| Native zero-copy startup/ownership | Startup dequeue→RGA/RKNN→QBUF self-test; RAII cleanup; queued/dequeued state; infer/copy/requeue serialization; camera-owner-thread close | Pass (source/host contracts); staged ROCK 4D build and 10-frame native run Pass; deployment/long soak Pending |
| Native crop and ABI | Even/in-bounds center-bottom crop must match RKNN input; C ABI and `ctypes` model lifecycle, offsets, detections, and frame metadata align | Pass (host contracts and staged ROCK 4D ABI load) |
| Idle unload | NumPy and native paths unload RKNN/model only; native camera/RGA/DMA resources remain; reload failure is fatal | Pass (host contracts and staged ROCK 4D unload/reload); long soak Pending |
| DMA-BUF motion source | With motion gating enabled, a real lores/luma source such as `/dev/video12` is required; an fd is never treated as motion | Pass (host contracts); RKISP lores board validation Pending |
| Option A/B parity thresholds | `compare_zerocopy_vs_numpy.py` requires at least one compared frame, aggregate detection-count delta `0`, median top-box IoU `>=0.90` when boxes are comparable, steady fd delta 0, and repeat-session lifecycle fd delta 0 | Threshold logic Pass (host); staged ROCK 4D 10-frame count/fd gate Pass with no comparable boxes; detected-scene IoU Pending |
| Device permissions/credentials | Rockchip media devices use group `video`, mode `0660`; board helper scripts contain no password | Source inspection and ROCK 4D `/dev/rga`, `/dev/video11`, `/dev/video12` mode/group Pass |

The **511 passing** result is the completed host test gate. On 2026-07-26 an
isolated ROCK 4D build additionally passed native compile/ABI load, 10-frame
RGA/RKNN execution, model unload/reload, repeat-session fd cleanup, 10-frame
count parity, and 30/30 DMA-BUF MPP/H.264 output; the installed service was
restored active. Deployment, detected-scene IoU, long fd/QBUF and concurrent
detector/stream soak, lores motion, camera-fault injection, and browser playback
remain Pending.

## 13. Exit criteria

Target acceptance requires every matrix row to pass on release-equivalent
hardware, all deployment-calibrated values to be recorded, no unresolved
critical failure, and all logs/configuration/calibration artifacts to be
archived. Existing bring-up evidence may be reused only where its setup and
pass criteria exactly satisfy a target row; otherwise that row remains Pending.
