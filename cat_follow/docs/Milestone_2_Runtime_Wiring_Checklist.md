## Milestone 2: Runtime Wiring Checklist

**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)
**Status:** Complete

## 1. Goal
Wire the contract-driven Milestone 1 modules (`SharedState`, `FSM`,
`DecisionEngine`, `AsyncLogger`) into a runnable end-to-end control stack
without touching real motors and without modifying the existing prototype
runtime.

The deliverable is a standalone process that can be started, exercises one
real control tick, observes shared state, requests transitions, emits
telemetry, and stops cleanly.

## 2. Implementation Strategy
Add new modules alongside the existing prototype. Do not modify
`cat_follow/main_loop.py`, `cat_follow/state_machine.py`, or any prototype
threads in this milestone.

## 3. Target New Modules
```
cat_follow/
  motion/
    motor_interface.py          # NEW (sibling of prototype driver.py)
  comms/
    __init__.py                 # NEW
    messages.py                 # NEW
    comms_manager.py            # NEW
  runtime/
    control_loop.py             # NEW
    app.py                      # NEW
```

## 4. Checklist

### 4.1 MotorInterface boundary
- [x] `cat_follow/motion/motor_interface.py` defines `MotorInterface`,
  `MotorBackend` protocol, and `NoOpMotorBackend`.
- [x] `apply(decision)` accepts a `DecisionOutput`, clamps normalized
  motion fields, and returns a `MotorCommand`.
- [x] Backend is pluggable via the `MotorBackend` protocol.
- [x] Unit tests cover clamping, change-only logging, brake severity, and
  emergency-stop behavior (`tests/test_motor_interface.py`).

### 4.2 Comms message dataclasses
- [x] `cat_follow/comms/messages.py` defines `TrackingMessage`,
  `CommandMessage`, and `AckMessage` matching Interface spec sections 4-7.
- [x] Each dataclass exposes `to_dict()` / `from_dict()` round-trip helpers.
- [x] `SchemaVersionError` raised on unsupported schema versions.
- [x] Unit tests verify round-trip equality and schema enforcement
  (`tests/test_comms_messages.py`).

### 4.3 CommsManager skeleton
- [x] `cat_follow/comms/comms_manager.py` is an in-process dispatcher.
- [x] Owns a bounded `command_id` cache (default 100 entries).
- [x] Validates commands and emits `AckMessage` with `cause` always present
  (`null` when accepted).
- [x] Updates `SharedState.overhead`, `SharedState.home`, and
  `SharedState.command`.
- [x] Drops duplicate / out-of-order tracking packets.
- [x] Emits `tracking_received`, `command_received`, and `command_ack`
  telemetry.
- [x] Unit tests cover acceptance, rejection, idempotent retry, and cache
  bounds (`tests/test_comms_manager.py`).

### 4.4 DecisionEngine command consumption
- [x] DecisionEngine consumes the latest accepted command from
  `SharedState.command` and translates it into the corresponding FSM event.
- [x] Each `command_id` is consumed exactly once.
- [x] Rejected commands are marked consumed without firing an FSM event.
- [x] `set_home` is intentionally not an FSM event.
- [x] Unit tests cover all command-driven transitions
  (`tests/test_control_decision_engine.py`).

### 4.5 ControlLoop
- [x] `cat_follow/runtime/control_loop.py` owns its `CatFollow-Control`
  thread and exposes `tick()` for tests.
- [x] Per tick: snapshot read, `DecisionEngine.tick()`, FSM snapshot
  publish, `DecisionState` publish, `MotorInterface.apply()`, decision
  telemetry.
- [x] Overrun detection per Interface spec section 15: warning on a single
  overrun, critical telemetry + emergency stop above the critical
  threshold.
- [x] Unit tests cover tick assembly, command-driven transitions,
  obstacle veto, overrun, critical overrun, and start/stop lifecycle
  (`tests/test_runtime_control_loop.py`).

### 4.6 Runtime app
- [x] `cat_follow/runtime/app.py` exposes `build_app()` and `main()`.
- [x] `main()` installs SIGINT/SIGTERM handlers (signals only).
- [x] Uses `JsonlFileSink` with a runtime-configurable path
  (`--log-path`).
- [x] Cleanly stops the control loop and logger on shutdown.
- [x] Smoke tests bring the app up briefly and verify JSONL telemetry,
  command processing, and FSM advancement (`tests/test_runtime_app.py`).

### 4.7 Integration boundaries
- [x] No changes to `cat_follow/main_loop.py`.
- [x] No changes to prototype FSM, threads, or web UI.
- [x] No real motor I/O (NoOpMotorBackend).
- [x] No real network sockets (in-process API only).

## 5. Completion Criteria
Milestone 2 is complete when:
- `python -m cat_follow.runtime.app` starts the new stack, runs at 50 Hz,
  writes JSONL telemetry, and exits cleanly on SIGINT/SIGTERM.
- All Milestone 1 + Milestone 2 unit tests pass.
- The existing prototype runtime still imports without errors.

**Status:** All criteria met. 103 unit tests passing across both
milestones (52 Milestone 1 + 51 Milestone 2).

## 6. Next Milestone Preview
Milestone 3 will replace the no-op backends with real I/O:
- UDP transport for `CommsManager`
- PiCar-X backend for `MotorInterface`
- Adapter from prototype vision threads into `SharedState.vision`
- Adapter from existing range sensor into `SharedState.range`
