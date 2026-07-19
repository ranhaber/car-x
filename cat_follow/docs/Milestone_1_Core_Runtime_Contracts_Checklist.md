# Milestone 1: Core Runtime Contracts Checklist
**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)  
**Status:** Ready for Implementation

## 1. Goal
Build the contract-driven runtime foundation without breaking the current prototype behavior.

Milestone 1 creates the new core architecture pieces:
- contract-level `SharedState`
- bounded async telemetry skeleton
- target FSM validator
- `DecisionEngine` shell
- unit tests for the above

This milestone should not yet integrate motors, overhead comms, range hardware (Lidar C1 / ultrasonic; TMF8829 is on hold), or the full control loop.

## 2. Implementation Strategy
Create the new target modules alongside the existing prototype, then migrate gradually.

Do not rewrite these existing files in-place as the first step:
- `cat_follow/main_loop.py`
- `cat_follow/memory/shared_state.py`
- `cat_follow/state_machine.py`

Reason:
- `cat_follow/memory/shared_state.py` is currently a vision/frame shared buffer for camera/tracker/detector. It is not the new contract-level `SharedState`.
- `cat_follow/state_machine.py` is the prototype FSM with `IDLE`, `GOTO_TARGET`, `SEARCH`, `APPROACH`, `TRACK`, and `LOST_SEARCH`. It conflicts with the target FSM.
- `cat_follow/main_loop.py` currently owns control in the main thread at about 30 Hz. The target architecture uses a dedicated `CatFollow-Control` thread at 50 Hz.

Building the new core alongside the current prototype avoids breaking working code while the contract-driven architecture is implemented.

## 3. Target New Modules
Recommended Milestone 1 structure:

```text
cat_follow/
  control/
    types.py
    fsm.py
    decision_engine.py
  runtime/
    shared_state.py
  telemetry/
    async_logger.py
```

The existing modules remain in place during this milestone:
- keep `cat_follow/memory/shared_state.py` for frame/bbox buffers
- keep `cat_follow/state_machine.py` until replacement integration
- keep `cat_follow/main_loop.py` untouched initially

## 4. Checklist

### 4.1 Runtime SharedState
- [x] Create immutable dataclasses for all contract state groups in `cat_follow/control/types.py`:
  - `overhead` (`OverheadState`)
  - `home` (`HomeState`)
  - `vision` (`VisionState`)
  - `range` (`RangeState`)
  - `navigation` (`NavigationState`)
  - `system` (`SystemState`)
  - `fsm` (`FSMSnapshot`)
  - `command` (`CommandState`)
  - `decision` (`DecisionState`)
  - aggregate (`SharedSnapshot`)
- [x] Create `cat_follow/runtime/shared_state.py` with single-writer group update methods.
- [x] Implement coherent snapshot API for `DecisionEngine` (`get_snapshot()`).
- [x] Implement freshness helpers using local monotonic `received_ms` (`now_monotonic_ms`, `is_fresh`).
- [x] Add tests for atomic updates and immutable snapshots (`tests/test_runtime_shared_state.py`).

### 4.2 Telemetry Skeleton
- [x] Create bounded async telemetry queue in `cat_follow/telemetry/async_logger.py`.
- [x] Support JSONL event envelope per Interface spec section 11.2 (`event_id`, `monotonic_ms`, `state`, `source`, `severity`, `data`).
- [x] Preserve high-priority events ahead of `debug` events (priority-aware drop policy).
- [x] Add no-blocking behavior for producers (`AsyncLogger.log()` only enqueues).
- [x] Add tests for queue overflow behavior (`tests/test_telemetry_async_logger.py`).
- [x] Batched flush with critical-event force-flush (`flush_interval_s`, `flush_batch_size`, immediate flush on `critical`).
- [x] Pluggable sinks: `JsonlFileSink`, `CallableSink`.

### 4.3 FSM Validator
- [x] Implement target states (enumerated as `FsmState` enum in `cat_follow/control/types.py`):
  - `HOME`
  - `IDLE`
  - `CHASE_A`
  - `TRACK_B`
  - `BRAKE`
  - `GOTO`
  - `RETURN_HOME`
  - `FAILSAFE`
- [x] Implement transition table from the interface spec (section 10.4) in `cat_follow/control/fsm.py`.
- [x] Reject all unlisted transitions.
- [x] Record rejected transition info into `FSMSnapshot.last_rejected_transition`.
- [x] Add tests for all valid and representative invalid transitions (`tests/test_control_fsm.py`).
- [x] Provide `is_transition_allowed()` helper for callers that want to query before applying.

### 4.4 DecisionEngine Shell
- [x] Define `DecisionInput` dataclass.
- [x] Define `DecisionOutput` dataclass.
- [x] Exclude previous `decision` from `DecisionInput`.
- [x] Implement `DecisionEngine.tick(decision_input) -> DecisionOutput` shell in `cat_follow/control/decision_engine.py`.
- [x] Return safe default decision for initial state (zero speed/steering, current FSM state).
- [x] Add simple decision reasons and active constraints from `ReasonCode` enum.
- [x] Add tests for basic safety/default behavior (`tests/test_control_decision_engine.py`).
- [x] Enforce safety precedence: failsafe > obstacle veto > pursuit logic.
- [x] Detect overhead staleness (`>300 ms`) and expiry (`>700 ms`) in chase states.
- [x] Detect obstacle distance below `OBSTACLE_TOO_CLOSE_CM` and trigger `FAILSAFE`.
- [x] Inject `FSM` for transition requests (no direct `MotorInterface` coupling).

### 4.5 Integration Boundaries
- [x] Do not connect the new `DecisionEngine` to motors yet.
- [x] Do not replace the current `main_loop.py` yet.
- [x] Do not remove the existing prototype FSM yet.
- [x] Ensure imports do not break existing runtime (new `cat_follow/control/` package is isolated).

## 5. Completion Criteria
Milestone 1 is complete when:
- New core modules exist alongside current prototype code.
- Unit tests pass for `SharedState`, telemetry, FSM, and `DecisionEngine` shell.
- No existing prototype behavior is broken.
- The new modules conform to the Interface and Data Contract Specification.

## 5a. Test Environment Reminder
> [!REMINDER]
> `pytest` is currently not installed in the active Python environment.
> Tests for `cat_follow/control/types.py` (`tests/test_control_types.py`) compile but cannot be executed yet.
>
> When the environment can be modified, run:
>
> ```bash
> python -m pip install pytest
> python -m pytest tests/test_control_types.py -v
> ```
>
> Add follow-up tests for `runtime/shared_state.py`, `telemetry/async_logger.py`, `control/fsm.py`, and `control/decision_engine.py` as those modules land.

## 6. Next Milestone Preview
Milestone 2 should wire the new core into runtime incrementally:
- create `CatFollow-Control` thread
- connect `DecisionEngine` to snapshot reads
- add `CommsManager` skeleton
- add `MotorInterface` boundary
- keep motor execution in safe/no-op mode until validated
