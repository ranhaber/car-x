## Milestone 3: Real I/O Checklist

**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)
**Status:** In Progress

## 1. Goal
Replace the no-op backends from Milestone 2 with real I/O implementations
so the contract-driven runtime can drive the actual PiCar-X chassis,
receive overhead packets over UDP, and surface prototype perception data
into ``SharedState``.

## 2. Implementation Strategy
Add the new I/O modules as siblings of the existing prototype code.
The Milestone 2 stack already accepts pluggable backends, so only the
adapter layers need to land.

## 3. Target New Modules
```
cat_follow/
  motion/
    picarx_backend.py           # NEW (sibling of motor_interface.py / driver.py)
  comms/
    udp_receiver.py             # NEW (Milestone 3)
    udp_sender.py               # NEW (ACK transport)
  perception/
    vision_adapter.py           # NEW (prototype threads -> SharedState.vision)
    range_adapter.py            # NEW (prototype range_sensor -> SharedState.range)
  runtime/
    app.py                      # extend with --picarx / --udp flags
```

## 4. Checklist

### 4.1 PiCar-X motor backend
- [x] `cat_follow/motion/picarx_backend.py` implements the
  ``MotorBackend`` protocol with normalized-to-SDK scaling.
- [x] Calibration is constructor-injected (``max_steer_deg``,
  ``max_speed_pct``).
- [x] Adapter talks to ``Picarx`` directly so we don't rely on the
  prototype ``driver.py`` module-level state.
- [x] Unit tests cover direction, brake, steering scaling, repeated-write
  suppression, and emergency stop using a hardware-free fake (no
  ``picarx`` import in tests).
- [ ] Wire ``--picarx`` flag into ``cat_follow/runtime/app.py`` so the
  real backend can be selected at startup.
- [ ] Manual smoke test on the actual PiCar-X (run for a few seconds and
  verify wheels respond to a fake command).

### 4.2 UDP transport
- [x] `cat_follow/comms/udp_receiver.py` listens on a configurable port
  and parses incoming JSON packets.
- [x] Each packet is converted into the appropriate
  ``TrackingMessage`` / ``CommandMessage`` and forwarded to
  ``CommsManager.submit_*``.
- [x] `cat_follow/comms/udp_sender.py` ships outgoing ``AckMessage``
  packets via ``CommsManager.ack_sink``.
- [x] Schema-version errors / malformed packets are logged via
  ``thread_health`` telemetry without killing the receiver.
- [x] Unit tests use loopback sockets to verify tracking/command
  ingress, error handling, and full receiver→CommsManager→sender round
  trip (`tests/test_comms_udp.py`, `tests/test_runtime_app.py`).
- [x] `cat_follow/runtime/app.py` exposes `--udp-listen-port`,
  `--udp-listen-host`, `--udp-target-host`, `--udp-target-port` flags
  to enable transport at runtime.

### 4.3 Vision adapter
- [x] `cat_follow/perception/vision_adapter.py` reads the prototype
  tracker bbox via duck-typed ``get_bbox_tracker()`` and publishes
  ``VisionState`` into the contract ``SharedState.vision``.
- [x] Translates pixel-space ``(x, y, w, h)`` bboxes into normalized
  horizontal offset ``x_offset_norm`` in ``[-1.0, 1.0]``.
- [x] Tracks ``cat_visible_stable`` after N consecutive frames (default
  3, matching Interface spec section 10.4).
- [x] Tracks ``last_seen_ms`` and emits ``vision_update`` telemetry.
- [x] Unit tests cover invisible/visible transitions, offset math,
  clamping, stability, last-seen tracking, telemetry, and the polling
  thread (`tests/test_perception_vision_adapter.py`).

### 4.4 Range adapter
- [x] `cat_follow/perception/range_adapter.py` polls a configurable
  ``read_distance`` callable (default usage:
  ``cat_follow.range_sensor.get_distance_cm``) and publishes
  ``RangeState`` into the contract ``SharedState.range``.
- [x] Linear severity ramp from ``obstacle_detected_cm`` to
  ``obstacle_critical_cm`` (default 50 cm -> 10 cm).
- [x] Emits ``range_update`` telemetry per update; ``thread_health`` on
  exceptions.
- [x] Sensor failures (``None`` reads or callable exceptions) yield a
  ``confidence=0.0`` state so DecisionEngine's veto logic short-circuits.
- [x] Unit tests cover normal/obstacle/critical distances, severity
  ramp, sensor failures, telemetry, and the polling thread
  (`tests/test_perception_range_adapter.py`).
- [x] App-level integration test verifies a close-distance reading
  drives the FSM into FAILSAFE through the full stack
  (`tests/test_runtime_app.py::test_app_with_range_adapter_triggers_failsafe_on_obstacle`).
- [x] ``build_app`` accepts ``range_read_distance`` and constructs the
  ``RangeAdapter`` automatically.

### 4.5 Runtime app extensions
- [x] ``--picarx`` flag selects ``PiCarXBackend`` instead of
  ``NoOpMotorBackend``.
- [x] ``--udp-listen-host`` / ``--udp-listen-port`` /
  ``--udp-target-host`` / ``--udp-target-port`` flags enable UDP
  ingress/egress alongside the in-process API.
- [x] ``--with-prototype-perception`` flag spins up the prototype
  camera/tracker/detector threads and wires their bbox + the range
  sensor through ``VisionAdapter`` / ``RangeAdapter``.  A single shared
  ``Picarx`` instance is reused between the motor backend and the range
  sensor wiring so both reference the same hardware object.
- [x] App lifecycle starts and stops prototype perception threads as
  part of ``App.start()`` / ``App.stop()``; integration test verifies
  the lifecycle (`tests/test_runtime_app.py::test_app_with_prototype_perception_threads_lifecycle`).

## 5. Completion Criteria
Milestone 3 is complete when:
- A real PiCar-X driven by ``cat_follow.runtime.app --picarx`` responds
  to commands sent over UDP and emits JSONL telemetry.
- All Milestone 1 + 2 + 3 unit tests pass on a CI machine without
  hardware.
- Manual smoke test on the Pi confirms forward/backward/steering and
  obstacle-too-close failsafe behavior.

## 6. Status
- 4.1 PiCar-X motor backend — code + 10 tests landed; CLI flag wired in.
- 4.2 UDP transport — receiver, sender, integration tests, and CLI
  flags landed. 8 unit tests + 1 app-level round-trip test pass.
- 4.3 Vision adapter — code + 12 unit tests + 1 app-level integration
  test landed. ``build_app`` accepts ``prototype_vision_shared_state``
  + ``vision_image_width``/``vision_image_height`` and constructs a
  ``VisionAdapter`` automatically. Polling thread starts/stops with the
  rest of the app.
- 4.4 Range adapter — code + 11 unit tests + 1 app-level integration
  test landed. ``build_app`` accepts ``range_read_distance`` and
  constructs a ``RangeAdapter`` automatically. Failsafe on
  obstacle-too-close confirmed end-to-end.
- 4.5 Runtime app extensions — all CLI flags landed:
  ``--picarx``, ``--with-prototype-perception``,
  ``--udp-listen-host``, ``--udp-listen-port``,
  ``--udp-target-host``, ``--udp-target-port``.  Single-command Pi
  bring-up is now possible.

**Total tests:** 148 passing across all milestones.

**Remaining open item:** Manual smoke test on the actual PiCar-X
hardware.  All software pieces are in place and verified end-to-end on
Windows via fakes; the next step is a live run on the Pi.
