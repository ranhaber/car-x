# Code Review Plan

Use this checklist for every code review in this repository. Review the requested
diff (branch, pull request, or uncommitted changes); review the whole repository
only when explicitly requested.

## Deliverable

Report findings first, sorted by severity, with:

`Severity | Dimension | Area | Location | Finding`

Use these dimensions: Correctness/Safety, Concurrency, Memory, Naming, Errors,
Timing, Security, Hardware, Validation, Observability, Degradation, Contract,
CPU, Wiring/Config, and Tests. Include coverage gaps after concrete findings.
Do not change code unless the user separately asks for fixes.

Bugbot may be used as a high-confidence seed, but it does not replace this
review.

## Cross-cutting checks

Apply every check below to every relevant file and pipeline.

### C1 — Concurrency

- Verify actual single-writer and single-reader ownership.
- Protect compound read-modify-write operations and multi-field snapshots.
- Do not share mutable buffers, lists, or dictionaries while they are mutated.
- Do not hold locks across camera, inference, network, ROS, or disk I/O.
- Check callback re-entrancy, thread shutdown, joins, and lock ordering.
- Do not rely on the GIL where NumPy, RKNN, OpenCV, or other native code may
  release it.

### C2 — Memory and zero-copy

- Check frame-ring and inference handoff for avoidable copies and allocations.
- Prefer preallocated buffers, indexes, memory views, and bounded pools.
- Zero-copy buffers must use ownership/generation handoff so readers never see
  writes in progress.
- Check caches, queues, telemetry, detections, and tracks for bounded growth.
- Release models, frames, files, sockets, and native resources promptly.

### C3 — Naming

- Functions, arguments, variables, constants, configuration keys, routes, and
  ROS parameters must describe their purpose.
- Include units in physical and timing values (`_cm`, `_ms`, `_hz`, `_rad`).
- Boolean names should read as predicates.
- Use consistent vocabulary across Python, JSON, environment variables, UI,
  ROS topics, and documentation.
- Reject misleading names and unexplained magic numbers.

### C4 — Error handling

- Determine whether every failure path should fail closed or degrade safely.
- Unexpected control, motor, sequence, sensor, camera, NPU, and ROS failures
  must not leave motion active.
- Flag swallowed exceptions and errors without actionable context.

### C5 — Timing and real-time behavior

- Use monotonic clocks for freshness, aging, deadlines, and heartbeats.
- Verify clock and unit consistency across producer and consumer boundaries.
- Check control-loop overruns, blocking work, priority inversion, and mismatched
  producer/consumer rates.

### C6 — Security and secrets

- Authenticate all mutating HTTP and UDP operations.
- Do not expose or log credentials and tokens.
- Validate payload sizes and prevent path or command injection.
- Never publish repository-local development secrets.

### C7 — Hardware ownership

- Verify exclusive, coordinated ownership of motors, Picarx, ultrasonic sensor,
  cameras, RKNN/NPU, and lidar serial port.
- Check open/close lifecycle and concurrent access during startup, shutdown,
  reload, and failure.

### C8 — Input validation

- Validate HTTP JSON, ROS messages, environment values, and calibration data
  before persistence or runtime application.
- Handle NaN, infinity, empty messages, invalid geometry, malformed plans, and
  out-of-range physical values.
- Avoid partially initialized operation after configuration errors.

### C9 — Observability

- Log safety transitions, emergency stops, sequence aborts, and fatal detector
  failures.
- Keep telemetry bounded and off critical control paths.
- Ensure status endpoints show the real FSM, sequence, sensor, and safety state.
- Check log rate and disk-growth risks on the target device.

### C10 — Graceful degradation

- Detection and tracking must work headlessly without Flask or video clients.
- Missing ROS, map, lidar, odometry, camera, or detector data must have explicit
  safe behavior.
- Check temporary dependency absence and service restart ordering.

### C11 — Contracts and dead code

- Verify shared-state, message, route, ROS topic, and configuration contracts
  across every producer/consumer pair.
- Find dangling imports and references after removals or renames.
- Flag legacy or unwired modules that appear authoritative.

### C12 — CPU and performance

- Preserve idle CPU reductions, detector/model lifecycle, and stream-off
  behavior.
- Check hot-loop algorithmic cost at required rates.
- Avoid synchronous disk I/O, large serialization, and expensive allocation on
  control and perception loops.

## Architecture passes

### 1. Process and service topology

Map service entry points, threads, process boundaries, hardware owners,
startup/shutdown ordering, and optional modes. Verify every deployed service
uses the intended runtime.

### 2. Pipelines

Review each complete producer-to-consumer pipeline:

1. Camera, motion gate, detector, post-processing, and publish.
2. Multi-target association, prediction, roles, and primary-track publish.
3. Shared-state adapters, freshness, and generation handling.
4. FSM, decision engine, safety precedence, and motor actuation.
5. Movement plans, sequence executor, heartbeat, abort, and emergency stop.
6. Ultrasonic/lidar ranging, fusion, thresholds, and stale-data policy.
7. Navigation, odometry, map, command velocity, and TF.

### 3. ROS2

Review package dependencies, launch composition, topic and message types, QoS,
frame IDs, TF tree, timestamps, odometry publisher exclusivity, environment
gates, and Python bridge consumption.

### 4. UI and HTTP control plane

Review Flask registration, mutation authentication, request validation,
templates/JavaScript, background-tab heartbeat behavior, status accuracy, and
independence of perception from MJPEG/H264 clients.

### 5. Messages and IPC

Trace shared-state groups, generation counters, queues, callbacks, UDP, HTTP,
ROS DDS, acknowledgements, idempotency, units, freshness, ownership, and
backpressure between every producer and consumer.

### 6. Configuration and calibration

Check environment, JSON, runtime, UI, service, and documentation consistency.
Verify precedence, validation-before-save, live-apply claims, restart behavior,
defaults, and secret handling.

### 7. Tests

Map changed behavior to tests. Identify untested integration boundaries,
failure paths, concurrency, clock jumps, auth matrices, malformed input,
hardware ownership, graceful degradation, and startup/shutdown behavior.

## Review order

1. Existing high-confidence findings and safety/control paths.
2. Shared-state and concurrency boundaries.
3. Tracking and movement sequences.
4. ROS2/navigation and range fusion.
5. Vision/detection and memory/CPU behavior.
6. UI, IPC, configuration, security, and validation.
7. Dedicated C1–C12 sweep to catch subjects missed in pipeline reviews.
8. Test coverage gaps and consolidated severity-sorted report.
