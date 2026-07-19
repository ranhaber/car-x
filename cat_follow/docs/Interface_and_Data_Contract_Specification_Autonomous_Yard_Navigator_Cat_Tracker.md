# Interface and Data Contract Specification
**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)  
**Based on:** PRD v1.1, HLD v1.1, Detailed Software Architecture v1.0  
**Version:** 1.0  
**Status:** Ready for Implementation

## 1. Purpose
This document defines the concrete message schemas, shared-state contracts, command semantics, timestamp rules, and synchronization rules used by the autonomous yard navigator.

The goal is to prevent implementation drift between modules and between the overhead system and the PiCar-X runtime.

All coordinates and distances in this specification use centimeters (`cm`) unless explicitly documented otherwise.

## 2. Protocol Model

### 2.1 Message Classes
The production protocol uses separate message types:
- `tracking`
- `command`
- `ack`

Future message types may be added with a schema version bump or backward-compatible extension.

### 2.2 Reliability Model
Tracking and command messages use different reliability semantics.

| Message Type | Frequency | ACK Required | Retry | Semantics |
|---|---:|---|---|---|
| `tracking` | ~10 Hz | No | No | lossy, latest-wins |
| `command` | event-driven | Yes | Yes | reliable via ACK/retry |
| `ack` | response | No | No | acknowledges received command packet |

Non-command messages sent at 10 Hz do not wait for ACK and do not retry. They are inspected using sequence numbers and freshness rules only.

Commands wait for ACK. If no ACK is received before the retry timeout, the sender retries the same command using the same `command_id` and a new packet `sequence`.

## 3. Common Message Envelope
Every protocol message contains:

```json
{
  "type": "tracking | command | ack",
  "schema_version": 1,
  "sequence": 0,
  "timestamp_ms": 0
}
```

### 3.1 Common Fields
- `type`: message type.
- `schema_version`: integer schema version. V1 uses `1`.
- `sequence`: monotonically increasing packet number from the sending side.
- `timestamp_ms`: sender-side timestamp in milliseconds. This is for cross-device log correlation and latency analysis.

### 3.2 Clock and Freshness Rules
Overhead host and PiCar-X should use NTP/Chrony so `timestamp_ms` values can be compared during logs and post-run analysis.

Control logic must not depend on synchronized wall-clock time.

For every incoming message, the PiCar-X runtime records:
- `timestamp_ms`: producer/sender timestamp from the message.
- `received_ms`: PiCar-X local monotonic receive timestamp.

Freshness is computed using local monotonic time:

```text
age_ms = now_monotonic_ms - received_ms
```

Rules:
- Use `timestamp_ms` for cross-device log correlation and latency debugging.
- Use `received_ms` for freshness, timeout, and failsafe decisions.
- Do not compute safety freshness from producer `timestamp_ms`.
- If NTP/Chrony is unavailable or clocks drift, control behavior must remain safe because freshness uses monotonic receive time.

### 3.3 Sequence Rules
- Every message has a `sequence`.
- Each sender owns its own independent sequence counter.
- Sequence numbers are used for diagnostics, duplicate detection, out-of-order detection, and ACK correlation.
- Tracking sequence gaps do not trigger retries.
- Command retry packets use a new `sequence` but keep the same `command_id`.

## 4. Step 1: Command, Tracking, and ACK Reliability Contract
**Status:** Done

### 4.1 Final Decision
Use separate message types:
- `tracking`
- `command`
- `ack`

All messages include:
- `type`
- `schema_version`
- `sequence`
- `timestamp_ms`

Commands additionally include:
- `command_id`
- `command`

ACKs include:
- their own `sequence`
- `ack_sequence`
- `ack_type`
- `command_id` when ACKing a command
- `status`
- current car `state`
- `reason`

### 4.2 Tracking Reliability
Tracking messages are fast, lossy, latest-wins updates.

Rules:
- No ACK.
- No retry.
- Duplicate tracking packets are ignored.
- Out-of-order tracking packets are ignored.
- Missing tracking packets are tolerated until freshness thresholds are exceeded.
- Packet age `>300 ms` causes stale-warning behavior.
- Packet age `>700 ms` causes overhead-expired failsafe behavior.

### 4.3 Command Reliability
Command messages are reliable command transactions.

Rules:
- Commands require ACK.
- Sender retries command if ACK is not received.
- Retry uses the same `command_id`.
- Retry uses a new packet `sequence`.
- Commands must be idempotent.
- Receiving the same `command_id` more than once must not re-execute side effects.
- Duplicate command retries receive an ACK with the original accepted/rejected result.

Recommended retry defaults:
- Retry timeout: `200 ms`
- Max retries: `5`

### 4.4 ACK Semantics
ACKs always identify the exact packet received.

ACKs include:
- `sequence`: ACK packet sequence from the ACK sender.
- `ack_sequence`: sequence of the packet being acknowledged.
- `ack_type`: type of packet being acknowledged.
- `command_id`: included when ACKing a command.
- `status`: `accepted` or `rejected`.
- `state`: current car FSM state.
- `reason`: machine-readable reason code.
- `cause`: always present; `null` for accepted ACKs and a machine-readable cause for rejected ACKs.

ACK status values are limited to:
- `accepted`
- `rejected`

`accepted` / `rejected` describe the command result, not whether the packet was the first send or a retry.

### 4.5 ACK Example
Command packet:

```json
{
  "type": "command",
  "schema_version": 1,
  "sequence": 503,
  "timestamp_ms": 100200,
  "command_id": "cmd-42",
  "command": "stop_chase"
}
```

ACK packet:

```json
{
  "type": "ack",
  "schema_version": 1,
  "sequence": 9001,
  "timestamp_ms": 100230,
  "ack_sequence": 503,
  "ack_type": "command",
  "command_id": "cmd-42",
  "status": "accepted",
  "state": "IDLE",
  "reason": "stop_chase_accepted",
  "cause": null
}
```

If packet `502` was handled but its ACK was lost, and retry packet `503` arrives with the same `command_id`, the car does not execute the command again. It ACKs packet `503` with the original result.

## 5. Step 2: Tracking Message Schema
**Status:** Done

### 5.1 Final Decision
Tracking messages carry overhead global observations only. They are sent at approximately 10 Hz and use lossy latest-wins semantics.

`tracking` messages do not include `home`. Home position is mission-critical and must be set only by reliable commands such as `set_home` or `return_home`.

### 5.2 Schema
```json
{
  "type": "tracking",
  "schema_version": 1,
  "sequence": 1001,
  "timestamp_ms": 123456789,
  "frame_id": "yard",
  "car": {
    "x": 0.0,
    "y": 0.0,
    "heading": 0.0,
    "heading_valid": false,
    "confidence": 1.0
  },
  "cat": {
    "x": 0.0,
    "y": 0.0,
    "confidence": 1.0
  }
}
```

### 5.3 Field Rules
- `frame_id`: coordinate frame name. V1 uses `yard`.
- `car.x`, `car.y`: global car position in centimeters.
- `cat.x`, `cat.y`: global cat position in centimeters.
- `car.heading`: optional overhead heading observation in radians; non-authoritative.
- `car.heading_valid`: whether overhead believes `car.heading` is usable.
- `confidence`: binary confidence value in V1.

### 5.4 Confidence Rules
For V1, confidence is binary:
- `1.0`: recognized by overhead camera and usable.
- `0.0`: not recognized / unusable.

`car` and `cat` objects are always required in every tracking packet.

If overhead loses either target:
- Keep the object present.
- Set `confidence` to `0.0`.
- Receiver must ignore that object's position for control.
- Position fields may contain last known values or `0.0`, but they have no authority while confidence is `0.0`.

If `car.confidence == 0.0`, then `car.heading_valid` must be `false`.

### 5.5 Receiver Rules
- Ignore duplicate or out-of-order `tracking` sequences.
- Do not request retry for missing tracking packets.
- Use latest valid packet only.
- Treat object data with `confidence == 0.0` as unusable / weight zero.
- Compute velocity locally from timestamped position updates if needed; velocity is not included in V1 tracking packets.

## 6. Step 3: Command Message Schema
**Status:** Done

### 6.1 Final Decision
Command messages are reliable, ACKed, retried messages. Commands are separate from the 10 Hz `tracking` stream.

V1 commands:
- `set_home`
- `start_chase`
- `stop_chase`
- `return_home`
- `go_to`
- `emergency_stop`
- `clear_failsafe`

### 6.2 Base Schema
```json
{
  "type": "command",
  "schema_version": 1,
  "sequence": 2001,
  "timestamp_ms": 123456900,
  "command_id": "cmd-0001",
  "command": "start_chase",
  "params": {}
}
```

### 6.3 Command Fields
- `command_id`: stable command transaction ID. Retries reuse the same `command_id`.
- `command`: command name.
- `params`: command-specific parameters. Use `{}` when no parameters are required.

### 6.4 `set_home`
Sets or updates the home position. `set_home` is allowed at any time.

```json
{
  "type": "command",
  "schema_version": 1,
  "sequence": 2001,
  "timestamp_ms": 123456900,
  "command_id": "cmd-0001",
  "command": "set_home",
  "params": {
    "home": {
      "x": 0.0,
      "y": 0.0,
      "frame_id": "yard"
    }
  }
}
```

Acceptance rules:
- Accept if `home.x`, `home.y`, and `home.frame_id` are valid.
- Reject if coordinates are missing, invalid, or outside the known yard frame.
- `home.x` and `home.y` are centimeters in the `yard` frame.

### 6.5 `start_chase`
Starts cat chase.

```json
{
  "type": "command",
  "schema_version": 1,
  "sequence": 2002,
  "timestamp_ms": 123457000,
  "command_id": "cmd-0002",
  "command": "start_chase",
  "params": {}
}
```

Acceptance rules:
- Accept only after the car has received valid current tracking data for both car and cat.
- `car.confidence` must be `1.0`.
- `cat.confidence` must be `1.0`.
- Reject if car position is unknown.
- Reject if cat position is unknown.
- Reject if the latest tracking packet is stale or expired.

The overhead system may already be sending tracking packets before `start_chase`. The car only ACKs `start_chase` as `accepted` after both positions are valid.

### 6.6 `stop_chase`
Stops the car and stops cat chase/tracking behavior. It does not return home.

```json
{
  "type": "command",
  "schema_version": 1,
  "sequence": 2003,
  "timestamp_ms": 123457100,
  "command_id": "cmd-0003",
  "command": "stop_chase",
  "params": {}
}
```

Acceptance rules:
- Accept unless the car is in an unrecoverable failsafe condition.
- On acceptance, stop motors, disable chase behavior, stop cat tracking behavior, and transition to `IDLE`.

### 6.7 `return_home`
Returns the car to home. `return_home` must include home coordinates for brownout/restart robustness.

```json
{
  "type": "command",
  "schema_version": 1,
  "sequence": 2004,
  "timestamp_ms": 123457200,
  "command_id": "cmd-0004",
  "command": "return_home",
  "params": {
    "home": {
      "x": 0.0,
      "y": 0.0,
      "frame_id": "yard"
    }
  }
}
```

Acceptance rules:
- Accept if home coordinates are valid.
- Reject if home coordinates are missing, invalid, or outside the known yard frame.
- On acceptance, transition toward `RETURN_HOME`.
- Transition to `HOME` only after return-home completes successfully.
- `home.x` and `home.y` are centimeters in the `yard` frame.

### 6.8 `go_to`
Moves the car to a specified yard coordinate.

```json
{
  "type": "command",
  "schema_version": 1,
  "sequence": 2005,
  "timestamp_ms": 123457300,
  "command_id": "cmd-0005",
  "command": "go_to",
  "params": {
    "target": {
      "x": 0.0,
      "y": 0.0,
      "frame_id": "yard"
    }
  }
}
```

Acceptance rules:
- Accept if target coordinates are valid and safety state allows motion.
- Reject if target is missing, invalid, outside the known yard frame, or motion is unsafe.
- `target.x` and `target.y` are centimeters in the `yard` frame.

### 6.9 `emergency_stop`
Immediately stops the car and enters `FAILSAFE`.

```json
{
  "type": "command",
  "schema_version": 1,
  "sequence": 2006,
  "timestamp_ms": 123457400,
  "command_id": "cmd-0006",
  "command": "emergency_stop",
  "params": {}
}
```

### 6.10 `clear_failsafe`
Allows the car to leave `FAILSAFE` when operator confirmation and system safety checks allow it.

```json
{
  "type": "command",
  "schema_version": 1,
  "sequence": 2007,
  "timestamp_ms": 123457500,
  "command_id": "cmd-0007",
  "command": "clear_failsafe",
  "params": {
    "operator_confirmed": true
  }
}
```

Acceptance rules:
- Accept only if `operator_confirmed` is `true` and safety checks pass.
- Reject otherwise.

### 6.11 Rejection Cause
When a command ACK has `status: "rejected"`, the ACK must include `cause`.

```json
{
  "type": "ack",
  "schema_version": 1,
  "sequence": 9002,
  "timestamp_ms": 123457530,
  "ack_sequence": 2002,
  "ack_type": "command",
  "command_id": "cmd-0002",
  "status": "rejected",
  "state": "IDLE",
  "reason": "start_chase_rejected",
  "cause": "cat_position_invalid"
}
```

Recommended V1 rejection causes:
- `car_position_invalid`
- `cat_position_invalid`
- `tracking_stale`
- `home_missing`
- `home_invalid`
- `target_invalid`
- `motion_unsafe`
- `failsafe_active`
- `operator_confirmation_required`
- `invalid_command`
- `invalid_params`

For accepted ACKs, `cause` must be present with value `null`.

## 7. Step 4: ACK Schema and Retry Behavior
**Status:** Done

### 7.1 Final Decision
ACK messages acknowledge reliable command packets. Tracking packets do not receive ACKs.

ACKs always reference:
- the exact received packet sequence
- the command transaction ID when ACKing a command
- the command result
- the current car state

### 7.2 Schema
```json
{
  "type": "ack",
  "schema_version": 1,
  "sequence": 9001,
  "timestamp_ms": 123457530,
  "ack_sequence": 2002,
  "ack_type": "command",
  "command_id": "cmd-0002",
  "status": "accepted",
  "state": "CHASE_A",
  "reason": "start_chase_accepted",
  "cause": null
}
```

### 7.3 Field Rules
- `sequence`: ACK packet's own sequence.
- `ack_sequence`: exact received packet sequence being acknowledged.
- `ack_type`: message type being acknowledged. V1 uses `command`.
- `command_id`: required when `ack_type == "command"`.
- `status`: command result. Must be `accepted` or `rejected`.
- `state`: current FSM state after command processing.
- `reason`: machine-readable result reason.
- `cause`: always present.
  - `null` when `status == "accepted"`.
  - machine-readable cause string when `status == "rejected"`.

### 7.4 Accepted ACK Example
```json
{
  "type": "ack",
  "schema_version": 1,
  "sequence": 9001,
  "timestamp_ms": 123457530,
  "ack_sequence": 2002,
  "ack_type": "command",
  "command_id": "cmd-0002",
  "status": "accepted",
  "state": "CHASE_A",
  "reason": "start_chase_accepted",
  "cause": null
}
```

### 7.5 Rejected ACK Example
```json
{
  "type": "ack",
  "schema_version": 1,
  "sequence": 9002,
  "timestamp_ms": 123457730,
  "ack_sequence": 2003,
  "ack_type": "command",
  "command_id": "cmd-0003",
  "status": "rejected",
  "state": "IDLE",
  "reason": "start_chase_rejected",
  "cause": "cat_position_invalid"
}
```

### 7.6 Retry Behavior
Overhead sender behavior:
- Send command.
- Wait `200 ms` for ACK.
- If no ACK arrives, retry the command.
- Retry uses the same `command_id`.
- Retry uses a new packet `sequence`.
- Retry up to `5` times.
- If still no ACK, mark the car command channel unhealthy.
- Do not assume the command was not executed after max retries; require operator/system decision before sending a conflicting command.

Car receiver behavior:
- If command is new, process it once, store the command result, and send ACK.
- If the same `command_id` is received again, do not re-execute side effects.
- For duplicate command retries, send ACK for the newly received packet's `ack_sequence`.
- Duplicate retries reuse the original command result (`accepted` or `rejected`), `reason`, and `cause`.

### 7.7 ACK Scope
For V1:
- ACKs are required for `command` messages only.
- ACKs are not sent for `tracking` messages.
- ACK messages themselves are not ACKed.

## 8. Step 5: SharedState Schema
**Status:** Done

### 8.1 Finalized Decisions
- `decision` is included in `SharedState`.
- Only `DecisionEngine` writes `decision`.
- UI, telemetry, tests, and `MotorInterface` may read `decision`.
- `DecisionEngine` must not use the previous `decision` group as input to future decisions.
- All coordinate and distance fields use centimeters (`cm`).
- `distance_cm` uses centimeters.
- `decision.speed` and `decision.steering` are normalized values.
- `navigation.speed_limit` is normalized `0.0..1.0`.
- `obstacle_severity` is normalized `0.0..1.0`.
- `vision.x_offset_norm` uses `-1.0` for left, `0.0` for centered, and `1.0` for right.
- `home.fresh` does not expire; `home.set` determines whether home is usable.

### 8.2 Normalized Motion Fields
- `decision.speed`: normalized drive request.
  - `-1.0`: full reverse
  - `0.0`: stop/no throttle
  - `1.0`: full forward
- `decision.steering`: normalized steering request.
  - `-1.0`: full left
  - `0.0`: centered
  - `1.0`: full right

`MotorInterface` converts normalized values to hardware-specific PWM/servo commands.

Additional normalized fields:
- `navigation.speed_limit`: `0.0` means no motion allowed, `1.0` means no navigation-imposed speed reduction.
- `range.obstacle_severity`: `0.0` means no obstacle risk, `1.0` means maximum/critical obstacle risk.
- `vision.x_offset_norm`: `-1.0` means cat is at/near left edge, `0.0` means centered, `1.0` means at/near right edge.

Home validity:
- `home.fresh` does not expire.
- `home.set == true` means a home position is available.
- `home` remains valid until replaced by another accepted `set_home` or `return_home` command.

### 8.3 Schema
```json
{
  "overhead": {
    "timestamp_ms": 0,
    "received_ms": 0,
    "fresh": false,
    "authority": "CommsManager",
    "sequence": 0,
    "frame_id": "yard",
    "car": {
      "x": 0.0,
      "y": 0.0,
      "heading": 0.0,
      "heading_valid": false,
      "confidence": 0.0
    },
    "cat": {
      "x": 0.0,
      "y": 0.0,
      "confidence": 0.0
    }
  },
  "home": {
    "timestamp_ms": 0,
    "received_ms": 0,
    "fresh": true,
    "authority": "CommsManager",
    "set": false,
    "x": 0.0,
    "y": 0.0,
    "frame_id": "yard",
    "source_command_id": null
  },
  "vision": {
    "timestamp_ms": 0,
    "received_ms": 0,
    "fresh": false,
    "authority": "VisionTracker",
    "cat_visible": false,
    "cat_visible_stable": false,
    "x_offset_norm": 0.0,
    "confidence": 0.0,
    "last_seen_ms": 0
  },
  "range": {
    "timestamp_ms": 0,
    "received_ms": 0,
    "fresh": false,
    "authority": "RangeSafety",
    "backend": "ultrasonic | lidar_c1 | tmf8829",
    "distance_cm": null,
    "confidence": 0.0,
    "obstacle_detected": false,
    "obstacle_critical": false,
    "obstacle_severity": 0.0,
    "zone": null
  },
  "navigation": {
    "timestamp_ms": 0,
    "received_ms": 0,
    "fresh": false,
    "authority": "Navigation",
    "heading": 0.0,
    "heading_valid": false,
    "steering_constraint": 0.0,
    "speed_limit": 0.0,
    "path_correction": 0.0,
    "no_progress": false,
    "dead_end": false
  },
  "system": {
    "timestamp_ms": 0,
    "received_ms": 0,
    "fresh": true,
    "authority": "Runtime",
    "thermal_c": null,
    "thermal_state": "unknown | normal | warning | speed_limited | critical",
    "battery_voltage": null,
    "brownout_detected": false,
    "threads": {
      "comms_alive": false,
      "vision_alive": false,
      "range_alive": false,
      "navigation_alive": false,
      "control_alive": false
    }
  },
  "fsm": {
    "timestamp_ms": 0,
    "received_ms": 0,
    "fresh": true,
    "authority": "FSM",
    "state": "IDLE",
    "previous_state": null,
    "last_transition_ms": 0,
    "last_transition_reason": "init",
    "last_rejected_transition": null
  },
  "command": {
    "timestamp_ms": 0,
    "received_ms": 0,
    "fresh": true,
    "authority": "CommsManager",
    "last_command_id": null,
    "last_command": null,
    "last_status": null,
    "last_reason": null,
    "last_cause": null
  },
  "decision": {
    "timestamp_ms": 0,
    "received_ms": 0,
    "fresh": true,
    "authority": "DecisionEngine",
    "requested_state": "IDLE",
    "speed": 0.0,
    "steering": 0.0,
    "brake": false,
    "reason": "init",
    "active_constraints": []
  }
}
```

## 9. Step 6: DecisionEngine Input/Output Dataclasses
**Status:** Done

### 9.1 Final Decision
`DecisionEngine` reads one immutable snapshot each control tick and emits one decision output.

`decision` from `SharedState` is not included in `DecisionInput`. This prevents feedback loops from previous decisions.

`DecisionOutput` does not include `cause`. Decision explanations use `reason` and `active_constraints`. Command rejection details remain in ACK `cause`.

`DecisionOutput` includes optional target debug fields:
- `target_x`
- `target_y`
- `target_source`

Target fields are for observability, telemetry, UI, and tests. They do not create additional control authority.

### 9.2 DecisionInput
```python
@dataclass(frozen=True)
class DecisionInput:
    now_ms: int
    overhead: OverheadState
    home: HomeState
    vision: VisionState
    range: RangeState
    navigation: NavigationState
    system: SystemState
    fsm: FSMSnapshot
    command: CommandState
```

`FSMSnapshot` is the dataclass that wraps the `fsm` shared-state group. It is named distinctly from the `FsmState` enum to avoid case-only collisions in code.

Rules:
- `DecisionInput` is immutable for the duration of a control tick.
- It is built from a coherent `SharedState` snapshot.
- It must not include previous `decision` output.

### 9.3 DecisionOutput
```python
@dataclass(frozen=True)
class DecisionOutput:
    timestamp_ms: int
    requested_state: str
    speed: float
    steering: float
    brake: bool
    reason: str
    active_constraints: list[str]
    target_x: float | None = None
    target_y: float | None = None
    target_source: str | None = None
    rejected_transition: bool = False
```

### 9.4 Field Rules
- `requested_state`: requested FSM state.
- `speed`: normalized `-1.0..1.0`.
- `steering`: normalized `-1.0..1.0`.
- `brake`: hard stop / braking request.
- `reason`: machine-readable decision reason.
- `active_constraints`: list of active limiters, fallbacks, or vetoes.
- `target_x`, `target_y`: optional debug target coordinates in centimeters.
- `target_source`: optional target source identifier.
- `rejected_transition`: set when FSM rejects a requested transition.

Allowed `requested_state` values:
- `HOME`
- `IDLE`
- `CHASE_A`
- `TRACK_B`
- `BRAKE`
- `GOTO`
- `RETURN_HOME`
- `FAILSAFE`

Allowed `target_source` values:
- `cat_global`
- `cat_local`
- `home`
- `go_to`
- `none`

### 9.5 Example Active Constraints
- `overhead_stale`
- `obstacle_veto`
- `thermal_speed_limit`
- `camera_lost`
- `range_stale`
- `navigation_stale`
- `home_missing`
- `tracking_invalid`
- `failsafe_active`

### 9.6 Example Output
```json
{
  "timestamp_ms": 123458000,
  "requested_state": "CHASE_A",
  "speed": 0.5,
  "steering": 0.2,
  "brake": false,
  "reason": "global_chase",
  "active_constraints": ["navigation_constraint"],
  "target_x": 230.0,
  "target_y": 410.0,
  "target_source": "cat_global",
  "rejected_transition": false
}
```

## 10. Step 7: FSM States, Events, Transition Rules, and Reason Codes
**Status:** Done

### 10.1 Final Decision
`SEARCH` is not part of the V1 FSM. `start_chase` is accepted only after both car and cat tracking are valid, so the car can transition directly to `CHASE_A`.

`GOTO` is a separate state for the `go_to` command. It is distinct from `RETURN_HOME`.

Obstacle distance `<10 cm` is not a `BRAKE` condition. It is a safety condition that transitions to `FAILSAFE` with reason `obstacle_too_close`.

### 10.2 States
- `HOME`: car is at home position after successful return-home.
- `IDLE`: stopped, safe, not chasing, not necessarily at home.
- `CHASE_A`: global overhead-guided chase.
- `TRACK_B`: local camera-guided chase.
- `BRAKE`: final dToF-based cat stopping phase.
- `GOTO`: navigating to a supplied coordinate.
- `RETURN_HOME`: navigating to supplied home coordinates.
- `FAILSAFE`: emergency stop / unrecoverable safety state.

### 10.3 Events
- `start_chase_accepted`
- `stop_chase_accepted`
- `return_home_accepted`
- `go_to_accepted`
- `emergency_stop_accepted`
- `clear_failsafe_accepted`
- `cat_visible_stable`
- `cat_lost`
- `final_approach_ready`
- `brake_aborted_cat_moved`
- `go_to_complete`
- `return_home_complete`
- `failsafe_triggered`
- `obstacle_too_close`
- `transition_rejected`

### 10.4 Transition Rules
| From | To | Trigger |
|---|---|---|
| `HOME` | `CHASE_A` | `start_chase_accepted` |
| `IDLE` | `CHASE_A` | `start_chase_accepted` |
| `CHASE_A` | `TRACK_B` | `cat_visible_stable` |
| `TRACK_B` | `CHASE_A` | `cat_lost` |
| `TRACK_B` | `BRAKE` | `final_approach_ready` |
| `BRAKE` | `TRACK_B` | `brake_aborted_cat_moved` |
| `HOME` | `GOTO` | `go_to_accepted` |
| `IDLE` | `GOTO` | `go_to_accepted` |
| `GOTO` | `IDLE` | `go_to_complete` |
| any chase state | `IDLE` | `stop_chase_accepted` |
| any non-failsafe state | `RETURN_HOME` | `return_home_accepted` |
| `RETURN_HOME` | `HOME` | `return_home_complete` |
| any state | `FAILSAFE` | `obstacle_too_close` |
| any state | `FAILSAFE` | `failsafe_triggered` |
| `FAILSAFE` | `IDLE` | `clear_failsafe_accepted` |

Any transition not listed above is rejected by the FSM.

### 10.5 Chase State Set
The chase state set is:
- `CHASE_A`
- `TRACK_B`
- `BRAKE`

`stop_chase_accepted` from any chase state transitions to `IDLE`.

### 10.6 Reason Codes
Recommended V1 reason codes:
- `start_chase_accepted`
- `start_chase_rejected`
- `global_chase`
- `local_track`
- `final_approach`
- `brake_complete`
- `brake_aborted_cat_moved`
- `cat_lost_fallback`
- `stop_chase_accepted`
- `return_home_accepted`
- `return_home_complete`
- `go_to_accepted`
- `go_to_complete`
- `obstacle_too_close`
- `obstacle_veto`
- `overhead_stale`
- `overhead_expired`
- `camera_lost`
- `tracking_invalid`
- `home_missing`
- `failsafe_triggered`
- `clear_failsafe_accepted`
- `transition_rejected`

### 10.7 Rejected Transition Behavior
When a transition is rejected:
- FSM records `last_rejected_transition`.
- `DecisionOutput.rejected_transition` is set to `true`.
- Telemetry logs the rejected transition.
- Motor output must hold the current safe command or safe-stop.
- Rejected transitions must never produce raw or stale motor output.

## 11. Step 8: Telemetry Event Schema
**Status:** Done

### 11.1 Final Decision
Telemetry is written as structured JSON Lines (`JSONL`). Each line is one event object.

Telemetry must be safe for real-time operation:
- logging uses a bounded async queue
- low-priority events may be dropped if the queue is full
- safety, failsafe, command, and transition events are high priority and should be preserved whenever possible

Systemd journal may receive human-readable service logs, but replay/tuning uses JSONL telemetry.

### 11.2 Common Event Envelope
Every telemetry event uses this envelope:

```json
{
  "schema_version": 1,
  "event_id": "evt-000001",
  "event_type": "decision",
  "timestamp_ms": 123458000,
  "monotonic_ms": 987654321,
  "state": "CHASE_A",
  "source": "DecisionEngine",
  "severity": "info",
  "data": {}
}
```

Field rules:
- `schema_version`: telemetry schema version. V1 uses `1`.
- `event_id`: unique event ID generated by the car runtime.
- `event_type`: event category.
- `timestamp_ms`: wall/sender timestamp when available.
- `monotonic_ms`: local monotonic timestamp; required for ordering.
- `state`: current FSM state at event time.
- `source`: module that emitted the event.
- `severity`: `debug`, `info`, `warning`, `error`, or `critical`.
- `data`: event-specific payload.

`monotonic_ms` is the authoritative ordering and elapsed-time field inside PiCar-X telemetry. `timestamp_ms` is used for cross-device correlation with overhead logs.

### 11.3 Required Event Types
V1 telemetry event types:
- `state_transition`
- `transition_rejected`
- `decision`
- `command_received`
- `command_ack`
- `tracking_received`
- `tracking_stale`
- `vision_update`
- `range_update`
- `obstacle_veto`
- `failsafe`
- `thermal`
- `thread_health`
- `motor_command`

### 11.4 `state_transition`
```json
{
  "event_type": "state_transition",
  "source": "FSM",
  "severity": "info",
  "data": {
    "from_state": "IDLE",
    "to_state": "CHASE_A",
    "reason": "start_chase_accepted"
  }
}
```

### 11.5 `transition_rejected`
```json
{
  "event_type": "transition_rejected",
  "source": "FSM",
  "severity": "warning",
  "data": {
    "from_state": "IDLE",
    "to_state": "BRAKE",
    "reason": "transition_rejected",
    "cause": "invalid_transition"
  }
}
```

### 11.6 `decision`
```json
{
  "event_type": "decision",
  "source": "DecisionEngine",
  "severity": "debug",
  "data": {
    "requested_state": "CHASE_A",
    "speed": 0.5,
    "steering": 0.2,
    "brake": false,
    "reason": "global_chase",
    "active_constraints": ["navigation_constraint"],
    "target_x": 230.0,
    "target_y": 410.0,
    "target_source": "cat_global"
  }
}
```

### 11.7 `command_received`
```json
{
  "event_type": "command_received",
  "source": "CommsManager",
  "severity": "info",
  "data": {
    "sequence": 2002,
    "command_id": "cmd-0002",
    "command": "start_chase"
  }
}
```

### 11.8 `command_ack`
```json
{
  "event_type": "command_ack",
  "source": "CommsManager",
  "severity": "info",
  "data": {
    "ack_sequence": 2002,
    "command_id": "cmd-0002",
    "status": "accepted",
    "reason": "start_chase_accepted",
    "cause": null
  }
}
```

### 11.9 `tracking_received`
```json
{
  "event_type": "tracking_received",
  "source": "CommsManager",
  "severity": "debug",
  "data": {
    "sequence": 1001,
    "packet_age_ms": 35,
    "car_confidence": 1.0,
    "cat_confidence": 1.0
  }
}
```

### 11.10 `tracking_stale`
```json
{
  "event_type": "tracking_stale",
  "source": "CommsManager",
  "severity": "warning",
  "data": {
    "packet_age_ms": 350,
    "threshold_ms": 300,
    "reason": "overhead_stale"
  }
}
```

### 11.11 `vision_update`
```json
{
  "event_type": "vision_update",
  "source": "VisionTracker",
  "severity": "debug",
  "data": {
    "cat_visible": true,
    "cat_visible_stable": true,
    "x_offset_norm": -0.15,
    "confidence": 1.0
  }
}
```

### 11.12 `range_update`
```json
{
  "event_type": "range_update",
  "source": "RangeSafety",
  "severity": "debug",
  "data": {
    "backend": "lidar_c1",
    "distance_cm": 42.0,
    "confidence": 1.0,
    "obstacle_detected": true,
    "obstacle_critical": false,
    "obstacle_severity": 0.4
  }
}
```

### 11.13 `obstacle_veto`
```json
{
  "event_type": "obstacle_veto",
  "source": "RangeSafety",
  "severity": "warning",
  "data": {
    "distance_cm": 8.0,
    "obstacle_severity": 1.0,
    "reason": "obstacle_too_close"
  }
}
```

### 11.14 `failsafe`
```json
{
  "event_type": "failsafe",
  "source": "SafetySupervisor",
  "severity": "critical",
  "data": {
    "reason": "obstacle_too_close",
    "previous_state": "TRACK_B",
    "motor_command": "emergency_stop"
  }
}
```

### 11.15 `thermal`
```json
{
  "event_type": "thermal",
  "source": "Runtime",
  "severity": "warning",
  "data": {
    "thermal_c": 80.5,
    "thermal_state": "speed_limited",
    "reason": "thermal_speed_limit"
  }
}
```

### 11.16 `thread_health`
```json
{
  "event_type": "thread_health",
  "source": "Runtime",
  "severity": "warning",
  "data": {
    "thread": "CatFollow-Vision",
    "alive": false,
    "last_seen_ms": 123457000,
    "reason": "thread_stale"
  }
}
```

### 11.17 `motor_command`
```json
{
  "event_type": "motor_command",
  "source": "MotorInterface",
  "severity": "debug",
  "data": {
    "speed": 0.4,
    "steering": -0.2,
    "brake": false,
    "reason": "global_chase"
  }
}
```

### 11.18 Severity Rules
- `debug`: high-volume diagnostics, may be dropped first.
- `info`: normal state/command events.
- `warning`: degraded behavior, stale data, veto conditions.
- `error`: recoverable runtime errors.
- `critical`: failsafe and emergency-stop events.

High-priority events:
- `state_transition`
- `transition_rejected`
- `command_received`
- `command_ack`
- `obstacle_veto`
- `failsafe`

These should be preserved ahead of `debug` telemetry when the queue is under pressure.

## 12. Step 9: Thread Synchronization Rules
**Status:** Done

### 12.1 Final Decision
`SharedState` is the only cross-thread data contract for runtime state.

Each `SharedState` group has exactly one authoritative writer. Readers may consume snapshots but must not mutate state groups they do not own.

### 12.2 Writer Ownership
| SharedState Group | Authoritative Writer |
|---|---|
| `overhead` | `CommsManager` |
| `home` | `CommsManager` |
| `vision` | `VisionTracker` |
| `range` | `RangeSafety` |
| `navigation` | `Navigation` |
| `system` | `Runtime` / health monitor |
| `fsm` | `FSM` |
| `command` | `CommsManager` / command handler |
| `decision` | `DecisionEngine` |

Rules:
- Only the authoritative writer may update its group.
- Cross-group writes are forbidden unless explicitly listed above.
- `DecisionEngine` may request FSM transitions, but only `FSM` writes `fsm`.
- `DecisionEngine` writes `decision`, but does not directly mutate sensor/input groups.

### 12.3 Snapshot Read Rules
The control loop must read a coherent snapshot once per tick.

Rules:
- `DecisionEngine` reads one `SharedState` snapshot at the start of a control tick.
- That snapshot is immutable for the duration of the tick.
- `DecisionEngine` must not read individual groups again mid-tick.
- `DecisionEngine` input excludes the previous `decision` group.
- `MotorInterface` consumes the validated `decision` output, not raw perception state.

### 12.4 Update Atomicity
Writers must publish complete group updates atomically.

Rules:
- No partial group updates are visible to readers.
- Metadata and payload update together.
- `timestamp_ms`, `received_ms`, `fresh`, `authority`, and payload fields must describe the same sample/update.
- If a group update fails validation, the previous valid group value remains active and an error/warning telemetry event is emitted.

### 12.5 Locking Policy
Implementation may use per-group locks, copy-on-write snapshots, or immutable dataclass replacement.

Required behavior:
- Writers hold locks only long enough to replace one group.
- Readers must not hold locks while running control logic.
- No thread may hold multiple group locks at once unless a later implementation doc defines a lock ordering rule.
- Avoid calling external I/O, camera APIs, motor APIs, or telemetry flushing while holding a state lock.

Recommended implementation:
- Use per-group immutable dataclass instances.
- Writer builds a new group object outside the lock.
- Writer acquires the group lock and swaps the object.
- Snapshot reader briefly acquires locks or a snapshot lock, copies references, then releases locks before running decisions.

### 12.6 Freshness Calculation
Freshness is computed by the reader or shared-state helper using PiCar-X local monotonic time:

```text
age_ms = now_monotonic_ms - received_ms
```

Rules:
- Do not compute safety freshness from producer `timestamp_ms`.
- `timestamp_ms` is for log correlation and latency analysis.
- `received_ms` is authoritative for timeout/failsafe decisions.
- `fresh` may be stored for convenience, but it must be recomputed or validated against current monotonic time when consumed.

### 12.7 Command Idempotency Cache
The command handler must keep a bounded cache of processed command results keyed by `command_id`.

Rules:
- Duplicate command retries must not re-execute side effects.
- Duplicate command retries must send ACK for the newly received `ack_sequence`.
- Duplicate command retries reuse the original `status`, `reason`, and `cause`.
- Cache entries may expire after a safe retention window, but the window must be longer than the max command retry window.

V1 retention:
- Keep the latest `100` command IDs.
- When the cache exceeds `100` entries, remove the oldest entry.

### 12.8 Telemetry Queue Synchronization
Telemetry must not block control.

Rules:
- Telemetry uses a bounded async queue.
- Producers enqueue event objects without blocking the control loop.
- If queue is full, drop lower-priority `debug` events before high-priority events.
- `critical` failsafe events should be preserved whenever possible.
- Telemetry writer thread owns file I/O.

### 12.9 Shutdown Rules
Shutdown must be coordinated through a shared stop event or lifecycle controller.

Rules:
- Control thread commands safe stop before process exit.
- Worker threads exit without holding state locks.
- Telemetry thread flushes high-priority events if possible.
- Motor output must be stopped before process termination completes.

## 13. Step 10: Enums and Constants Appendix
**Status:** Done

### 13.1 Message Types
- `tracking`
- `command`
- `ack`

### 13.2 Command Names
- `set_home`
- `start_chase`
- `stop_chase`
- `return_home`
- `go_to`
- `emergency_stop`
- `clear_failsafe`

### 13.3 ACK Statuses
- `accepted`
- `rejected`

### 13.4 ACK Types
- `command`

### 13.5 FSM States
- `HOME`
- `IDLE`
- `CHASE_A`
- `TRACK_B`
- `BRAKE`
- `GOTO`
- `RETURN_HOME`
- `FAILSAFE`

### 13.6 FSM Events
- `start_chase_accepted`
- `stop_chase_accepted`
- `return_home_accepted`
- `go_to_accepted`
- `emergency_stop_accepted`
- `clear_failsafe_accepted`
- `cat_visible_stable`
- `cat_lost`
- `final_approach_ready`
- `brake_aborted_cat_moved`
- `go_to_complete`
- `return_home_complete`
- `failsafe_triggered`
- `obstacle_too_close`
- `transition_rejected`

### 13.7 Command Rejection Causes
- `car_position_invalid`
- `cat_position_invalid`
- `tracking_stale`
- `home_missing`
- `home_invalid`
- `target_invalid`
- `motion_unsafe`
- `failsafe_active`
- `operator_confirmation_required`
- `invalid_command`
- `invalid_params`

### 13.8 Decision Reason Codes
- `start_chase_accepted`
- `start_chase_rejected`
- `global_chase`
- `local_track`
- `final_approach`
- `brake_complete`
- `brake_aborted_cat_moved`
- `cat_lost_fallback`
- `stop_chase_accepted`
- `return_home_accepted`
- `return_home_complete`
- `go_to_accepted`
- `go_to_complete`
- `obstacle_too_close`
- `obstacle_veto`
- `overhead_stale`
- `overhead_expired`
- `camera_lost`
- `tracking_invalid`
- `home_missing`
- `failsafe_triggered`
- `clear_failsafe_accepted`
- `transition_rejected`

### 13.9 Target Sources
- `cat_global`
- `cat_local`
- `home`
- `go_to`
- `none`

### 13.10 Range Backends
- `ultrasonic`
- `lidar_c1`
- `tmf8829` (on hold; retained for backward compatibility, not used in the current build)

### 13.11 Thermal States
- `unknown`
- `normal`
- `warning`
- `speed_limited`
- `critical`

### 13.12 Telemetry Event Types
- `state_transition`
- `transition_rejected`
- `decision`
- `command_received`
- `command_ack`
- `tracking_received`
- `tracking_stale`
- `vision_update`
- `range_update`
- `obstacle_veto`
- `failsafe`
- `thermal`
- `thread_health`
- `motor_command`

### 13.13 Telemetry Severities
- `debug`
- `info`
- `warning`
- `error`
- `critical`

### 13.14 Numeric Constants
| Name | Value | Notes |
|---|---:|---|
| `TRACKING_RATE_HZ` | `10` | Nominal overhead tracking rate |
| `OVERHEAD_STALE_WARNING_MS` | `300` | Reduce speed to safe crawl |
| `OVERHEAD_STALE_FAILSAFE_MS` | `700` | Enter `FAILSAFE` |
| `CAMERA_LOSS_FALLBACK_MS` | `350` | `TRACK_B -> CHASE_A` |
| `NO_PROGRESS_RECOVERY_MS` | `2000` | Trigger recovery behavior |
| `RECOVERY_FAILSAFE_MS` | `5000` | Escalate to `FAILSAFE` |
| `OBSTACLE_TOO_CLOSE_CM` | `10` | Immediate `FAILSAFE` |
| `COMMAND_ACK_TIMEOUT_MS` | `200` | Retry command if no ACK |
| `COMMAND_MAX_RETRIES` | `5` | Mark command channel unhealthy after this |
| `COMMAND_ID_CACHE_SIZE` | `100` | Latest processed command IDs |
| `THERMAL_WARNING_C` | `75` | Log thermal warning |
| `THERMAL_SPEED_LIMIT_C` | `80` | Limit chase speed |
| `THERMAL_CRITICAL_C` | `85` | Return-home or failsafe |
| `CONTROL_TARGET_RATE_HZ` | `50` | Target control loop rate |
| `CONTROL_TARGET_PERIOD_MS` | `20` | Target control loop period |
| `CONTROL_MIN_DEGRADED_RATE_HZ` | `20` | Below this, reduce speed or stop |
| `CONTROL_OVERRUN_MS` | `20` | One tick exceeded target budget |
| `CONTROL_CONSECUTIVE_OVERRUN_LIMIT` | `3` | Apply conservative speed limiting |
| `CONTROL_CRITICAL_OVERRUN_MS` | `100` | Safe stop / critical telemetry |

### 13.15 Normalized Ranges
| Field | Range | Meaning |
|---|---|---|
| `decision.speed` | `-1.0..1.0` | reverse to forward |
| `decision.steering` | `-1.0..1.0` | left to right |
| `vision.x_offset_norm` | `-1.0..1.0` | left edge to right edge |
| `navigation.speed_limit` | `0.0..1.0` | blocked to unrestricted |
| `range.obstacle_severity` | `0.0..1.0` | no risk to critical |
| `confidence` | `0.0 or 1.0` | V1 binary unusable/usable |

## 14. Coordinate Convention Contract
**Status:** Done

### 14.1 Yard Frame
All global positions use the `yard` coordinate frame.

V1 convention:
- Origin: fixed overhead-calibrated yard origin.
- Recommended physical origin: bottom-left yard corner from the overhead/operator view.
- Units: centimeters (`cm`).
- `+X`: right in the yard.
- `+Y`: forward in the yard.
- Coordinates are planar ground-plane coordinates.

The origin must not move during a run. If overhead calibration changes, the runtime must treat that as a new coordinate frame/configuration.

### 14.2 Heading Convention
Heading fields use radians.

V1 convention:
- `heading = 0` points along `+X`.
- Positive heading rotates counter-clockwise toward `+Y`.
- Heading is normalized to `[-pi, pi)`.
- Overhead `car.heading` is optional and non-authoritative.
- `Navigation` owns authoritative car-local heading (`theta`).

Examples:
- `0 rad`: facing `+X`.
- `pi / 2 rad`: facing `+Y`.
- `pi rad` or `-pi rad`: facing `-X`.
- `-pi / 2 rad`: facing `-Y`.

### 14.3 Coordinate Consumers
All modules must use this same convention:
- `CommsManager`
- `DecisionEngine`
- `Navigation`
- `MotorInterface`
- telemetry/replay tools
- web UI visualization
- validation tests

Any coordinate transform from camera pixels, local camera frame, dToF zone frame, or motor/robot frame must explicitly convert into this convention before publishing to `SharedState`.

## 15. Control Timing Contract
**Status:** Done

### 15.1 Control Loop Rate
The `CatFollow-Control` thread runs the `DecisionEngine` and FSM validation.

V1 timing:
- Target rate: `50 Hz`.
- Target period: `20 ms`.
- Minimum degraded rate: `20 Hz`.
- Timing is measured with PiCar-X local monotonic time.

### 15.2 Per-Tick Budget
| Operation | Budget |
|---|---:|
| Snapshot acquire | `<1 ms` |
| Freshness evaluation | `<1 ms` |
| `DecisionEngine` | `<5 ms` |
| FSM validation | `<1 ms` |
| Motor command write | `<2 ms` |
| Telemetry enqueue | `<1 ms` |
| Total control tick | `<20 ms` |

Telemetry file I/O must not run inside the control tick. It is handled by the async telemetry thread.

### 15.3 Tick Overrun Behavior
An overrun occurs when one control tick exceeds `20 ms`.

Rules:
- Single overrun: emit `control_tick_overrun` telemetry with measured duration.
- `3` consecutive overruns: apply conservative speed limiting and emit warning telemetry.
- Any tick over `100 ms`: command safe stop and emit critical telemetry.
- Repeated overrun while already degraded may escalate to `FAILSAFE` if motion cannot be considered safe.

### 15.4 Timing Telemetry
Control timing telemetry should include:
- `tick_duration_ms`
- `snapshot_ms`
- `decision_ms`
- `fsm_ms`
- `motor_ms`
- `telemetry_enqueue_ms`
- `overrun_count`
- `control_rate_hz`

### 15.5 Missed Tick Safety
If the control loop cannot run at or above the minimum degraded rate (`20 Hz`), the system must reduce speed or stop.

Safety rule:
- stale control output must not continue driving indefinitely.
- if fresh decisions cannot be produced, `MotorInterface` must receive a safe stop or emergency stop depending on current safety state.

## 16. Final Consistency Pass
**Status:** Done

A consistency scan was performed across:
- PRD
- HLD
- Detailed Software Architecture
- Validation Matrix
- System Architecture
- Interface and Data Contract Specification

Verified:
- `SEARCH` is not part of the V1 FSM.
- `GOTO` is included where `go_to` is supported.
- `stop_chase` transitions chase behavior to `IDLE`, not `RETURN_HOME`.
- `return_home` transitions to `RETURN_HOME` and then `HOME` after completion.
- Coordinates and distances use centimeters (`cm`).
- Coordinate convention is fixed: `yard` frame, `+X` right, `+Y` forward, heading `0` along `+X`, positive CCW, radians normalized to `[-pi, pi)`.
- Control timing contract is defined: 50 Hz target, 20 ms tick budget, overrun behavior, and timing telemetry.
- ACK status values are only `accepted` and `rejected`.
- Tracking messages are latest-wins and do not use ACK/retry.
- Command messages use ACK/retry with stable `command_id`.
- Freshness and failsafe timing use PiCar-X local monotonic receive time.
