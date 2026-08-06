# Look / Drive / Path Design

**Project:** Autonomous Yard Navigator and Cat Tracker  
**Status:** Normative addendum for look/drive fusion (v1)  
**Authority:** For look/drive modes, pan gating, and steering-envelope
provenance, this document supersedes conflicting CHASE-fusion wording in
`Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`
§10.4 and
`Interface_and_Data_Contract_Specification_Autonomous_Yard_Navigator_Cat_Tracker.md`
§9.3. FSM states, Protocol V1, dual-sensor safety, and NavigationManager goal
lifecycle remain owned by those documents.

## 1. Purpose

v1 separates three concerns that must not fight each other:

1. **Path** — overhead cat+car yard pose → Nav2 moving goals on the lidar map.
2. **Envelope** — local costmap short-horizon sweep → real
   `[safe_steering_min, safe_steering_max]` around the planned path.
3. **Look vs drive** — camera pan keeps the bound cat near frame center when
   that is enough; chassis vision steering is allowed only after pan returns to
   calibrated forward.

## 2. Authorities

| Output | Owner | Rule |
|---|---|---|
| Chassis speed / steer / brake | `DecisionEngine` via `MotorInterface` | Sole drivetrain authority |
| Pan angle | `DecisionEngine` look output via `MotorInterface.apply_look` | Sole pan writer in runtime |
| Nav goals / envelope | `NavigationManager` | Publishes envelope + `path_viable`; does not drive motors |
| Target identity / cat exit | Overhead Protocol V1 | Unchanged |

Tilt stays at the calibrated chase default in v1 (no active tilt tracking).

## 3. Look/drive modes

Exactly one mode is active per control tick.

| Mode | Pan | Chassis steer source |
|---|---|---|
| `PATH_FOLLOW` | Hold or ease to calibrated forward | `path_correction` inside envelope |
| `LOOK_AT` | Track bound cat to ±N px of frame center | Nav2 path only — **no** vision chassis steer |
| `PAN_RESET` | Slew to calibrated forward; chassis steer frozen | No new vision steer |
| `BODY_STEER` | Must be within forward deadband | `clamp(x_offset_norm, envelope)` |
| `HOLD` | Freeze, then forward on exit | Zero motion |

Safety / non-chase exits (`BRAKE_REVERSE`, `FAILSAFE`, `HOME`, `IDLE`, `GOTO`,
`RETURN_HOME`, `GETTING_CLOSE`, `SEARCH`) force pan to calibrated forward and
use existing FSM drive policy (`PATH_FOLLOW` or zero).

### 3.1 Hard invariants

1. `BODY_STEER` is illegal unless `|pan_deg - pan_forward_deg| <= deadband`.
2. `LOOK_AT` never writes chassis steering from `VisionState.x_offset_norm`.
3. Leaving `LOOK_AT` for body vision steer MUST pass through `PAN_RESET`.
4. Missing/stale costmap or invalid envelope → `path_viable=false` (fail
   closed). Never invent a wide envelope.
5. Pan tracks only the bound `target_id` local track; ambiguity → no look chase.
6. Mode edges use hysteresis (`N_exit > N_enter`), minimum dwell, and pan
   slew-rate limits.

### 3.2 Mode selection sketch

```text
if not CHASE or sensors/nav HOLD:
    HOLD or PATH_FOLLOW (pan → forward)
elif LOOK_AT eligible (track fresh, error <= N_enter, pan can center, envelope OK):
    LOOK_AT
elif need body vision steer or leave LOOK_AT:
    PAN_RESET until forward deadband (timeout → PATH_FOLLOW/HOLD)
elif pan in forward deadband and vision chase needed:
    BODY_STEER = clamp(x_offset_norm, envelope)
else:
    PATH_FOLLOW
```

## 4. Path authority

When overhead provides fresh cat and car observations for the active
`target_id`, `NavigationManager` refreshes moving `NavigateToPose` goals using
the calibrated yard→map transform. Local RF2O/Nav2 pose remains authoritative
for robot motion after startup. Lidar/costmap shape the path; overhead does not
continuously overwrite localization.

## 5. Steering envelope

`NavigationManager` publishes envelope fields on `NavigationState`:

- `path_viable`
- `safe_steering_min` / `safe_steering_max` (normalized)
- `speed_cap_mps`
- `envelope_source` (`costmap_sweep` | `point` | `none`)
- `costmap_age_ms`

Production with `--ros-nav` uses `CostmapSweepEnvelopeProvider`: sample
normalized steer angles, roll a short bicycle-model arc, test the inflated
footprint against the local costmap, and publish the contiguous free band
that contains `path_correction`. If no free run contains the planned steer,
fail closed (`path_viable=false`; do not substitute a nearest free band).
`PointEnvelopeProvider` (`min = max = path_correction`) is test/fallback
only and MUST NOT silently widen to `[-1, 1]`.

If the costmap is missing or older than the configured TTL, publish
`path_viable=false` and a zero/empty envelope.

## 6. Applied fusion

```text
# LOOK_AT / PATH_FOLLOW (no vision chassis):
applied_steering = clamp(path_correction, safe_min, safe_max)

# BODY_STEER only (pan at forward):
applied_steering = clamp(x_offset_norm, safe_min, safe_max)

# PAN_RESET / HOLD:
applied_steering = held_or_zero

applied_speed_mps = min(
    pursuit_speed_request_mps,
    nav2_speed_cap_mps,
    alignment_speed_cap_mps,
    obstacle_speed_cap_mps,
    thermal_speed_cap_mps
)
```

Camera and Nav2 steering MUST NOT be added or weighted-summed.

## 7. Configuration (startup-validated)

| Knob | Meaning | Default intent |
|---|---|---|
| `look_n_enter_px` | Enter LOOK_AT when \|pixel error\| ≤ this | tighter |
| `look_n_exit_px` | Exit LOOK_AT when \|pixel error\| ≥ this | `> n_enter` |
| `look_pan_slew_deg_s` | Max pan rate | hardware-safe |
| `look_pan_forward_deadband_deg` | Forward gate for BODY_STEER | small |
| `look_pan_reset_timeout_ms` | PAN_RESET deadline | fail → PATH_FOLLOW/HOLD |
| `look_mode_dwell_ms` | Min time in a mode before leaving | 300–500 |
| `envelope_lookahead_m` | Sweep arc length | ~0.4–0.8 |
| `envelope_sample_count` | Steer samples in [-1, 1] | odd, ≥9 |
| `envelope_stale_ttl_ms` | Costmap freshness | fail closed |
| `envelope_max_half_width` | Cap band half-width | safety |

`pan_forward_deg` is the calibrated pan center (mechanical forward), not raw
PWM zero.

## 8. Telemetry (every control tick)

Record: `look_drive_mode`, `pan_deg`, `pan_forward_deg`, `pixel_error_px`,
`camera_request`, `safe_steering_min/max`, `path_correction`, applied chassis
`steering`, `envelope_source`, `costmap_age_ms`, `look_reason`.

## 9. Scenario coverage

### A — Sensing / authority

| ID | Scenario | Stable solution |
|---|---|---|
| A1 | Overhead sees cat and car | Moving Nav2 goal; costmap envelope |
| A2 | Overhead cat OK, car pose bad | Do not overwrite RF2O; health/containment rules |
| A3 | Overhead cat stale | Retention policy; no blind extrapolation |
| A4 | Overhead lost, local track OK | Continue CHASE; LOOK_AT/BODY_STEER as below |
| A5 | Overhead and track lost | RETURN_HOME or FAILSAFE; pan→forward |
| A6 | Different target_id | IDLE; new START_CHASE; pan→forward |
| A7 | PRIMARY_CAT_LEFT match | Stop; handoff IDLE; pan→forward |
| A8 | Camera/RKNN fatal | GETTING_CLOSE; pan→forward |
| A9 | Lidar/ultrasonic unhealthy | Existing hold/failsafe; pan freeze→forward |
| A10 | Costmap/envelope stale | path_viable=false; HOLD |
| A11 | Localization/geofence lost | Stop / failsafe; pan→forward |

### B — Mission phase

| ID | Scenario | Stable solution |
|---|---|---|
| B1 | GETTING_CLOSE | PATH_FOLLOW; pan forward |
| B2 | SEARCH | PATH_FOLLOW; no BODY_STEER from partial detections |
| B3 | SEARCH→CHASE lock | PAN_RESET then LOOK_AT or BODY_STEER |
| B4–B5 | Cat centerable by pan | LOOK_AT |
| B6 | Pan saturates / still off-center | PAN_RESET→BODY_STEER |
| B7 | Path requires body turn | Exit LOOK_AT→PAN_RESET→PATH_FOLLOW/BODY_STEER |
| B8 | Narrow envelope | LOOK_AT preferred; chassis clamp only in BODY_STEER |
| B9 | Empty envelope | HOLD; nav exhaustion / return rules |
| B10–B11 | Track lost with overhead | Direct SEARCH (≤200 cm) or GETTING_CLOSE (>200 cm); pan→forward |
| B12 | Ambiguous multi-cat | No look chase; reset association |
| B13–B15 | GOTO/RETURN/BRAKE/HOME/IDLE/FAILSAFE | Pan forward; existing drive policy |

### C — Geometry / conflict

| ID | Scenario | Stable solution |
|---|---|---|
| C1 | Pan≠forward + vision chassis | Forbidden |
| C2 | Fast cat motion | Slew limits; escalate to PAN_RESET→BODY_STEER |
| C3–C4 | Mode / pan oscillation | Hysteresis, dwell, deadband |
| C5 | Overhead vs local disagree | Association miss; no blend |
| C6 | Envelope forbids needed nudge | Prefer LOOK_AT; else stop/SEARCH |
| C7 | Cat outside pan FOV | PATH_FOLLOW on overhead or track-loss |
| C8 | Tilt | Fixed default in v1 |
| C9 | Stuck pan | Watchdog → PATH_FOLLOW/HOLD |
| C10 | Calibrated forward | Use pan center cal as forward |

## 10. Board validation checklist

See [`Board_Checklist_Look_Drive_Path.md`](Board_Checklist_Look_Drive_Path.md)
for ROCK 4D scenes LOOK-01…LOOK-05 and ENV-01.

| ID | Scene | Pass criteria |
|---|---|---|
| LOOK-01 | Open-yard CHASE | Enters LOOK_AT; pan tracks; chassis follows path |
| LOOK-02 | Obstacle one side | Asymmetric envelope; BODY_STEER stays inside; pan resets before body steer |
| LOOK-03 | Pan off-forward + vision offset | No vision chassis steer until PAN_RESET completes |
| ENV-01 | Kill/stale local costmap | path_viable=false; stop; no wide fake envelope |
| LOOK-04 | Track loss / primary-left / BRAKE_REVERSE | Pan to forward; FSM outcomes unchanged |
| LOOK-05 | Soak | No LOOK_AT↔BODY_STEER chatter under dwell/hysteresis |

## 11. Out of scope (v1)

- Simultaneous pan+steer with bearing transform
- Active tilt tracking
- Custom Nav2 corridor plugin
- Protocol / FSM state-set changes

## 12. Implementation map

| Piece | Location |
|---|---|
| Types / LookCommand | `cat_follow/control/types.py` |
| Mode selector | `cat_follow/control/look_drive.py` |
| Fusion | `cat_follow/control/decision_engine.py` |
| Pan actuator | `cat_follow/motion/motor_interface.py`, `picarx_backend.py` |
| Envelope providers | `cat_follow/navigation/steering_envelope.py` |
| Costmap ingest | `cat_follow/navigation/ros_bridge.py` |
| Config | `cat_follow/target_config.py` |
