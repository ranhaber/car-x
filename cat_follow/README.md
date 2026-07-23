# cat_follow

**Version:** 0.7.1

Modular cat-follow feature for PiCar-X. Camera stays straight; car steers and drives to keep the cat in the middle of the frame.

## Layout

- **state_machine.py** — States and events; `dispatch(event, payload)`.
- **commands.py** — Stub: `set_cat_location(x,y)`, `set_stop_command()`; `poll_commands(on_cat_location, on_stop)`.
- **calibration/** — `loader.py` + JSONs: speed–time–distance, steering limits (incl. target approach distance). Stored in `cat_follow/calibration/*.json`; loaded once at startup.
- **motion/** — `driver`, `center_cat_control()`, `limits`, `goto_xy` (runtime goto), `search`. Runtime goto uses **motion/goto_xy.py**; **calibration/goto_xy.py** is for calibration runs only.
- **vision/** — YOLOv8n COCO RKNN backend (`rknn_backend.py`) with 9-tensor model-zoo DFL/NMS decoding (`yolo_postprocess.py`). P1 publishes the best cat into the existing single-bbox contract; RKNN is the only backend.
- **threads/** — Camera, role-aware `PredictiveTracker` (constant velocity, two-stage high/low-confidence association), and RKNN detector. The tracker keeps sticky `PRIMARY_CAT`/`SECONDARY_CAT` identities but publishes only the primary through the existing chase bbox. Camera capture and detection remain independent of the web UI.
- **odometry.py** — Bicycle-model dead reckoning (position, heading). Used via **location/** facade.
- **range_sensor.py** — Throttled/cached distance facade; `set_reader()` for edge worker or `set_car()` for legacy polling.
- **perception/edge_ultrasonic.py** — libgpiod v1 HC-SR04 edge worker (`CatFollow-UltrasonicIRQ`); production ROCK 4D path.
- **perception/range_adapter.py** — Polls `range_sensor.get_distance_cm()` → `SharedState.range` (~20 Hz).
- **runtime/app.py** — Contract runtime (`--picarx`, `--with-prototype-perception`, optional `--ros-nav`, `--web-ui`); wires edge ultrasonic + adapters.
- **main_loop.py** — Legacy tick loop (polling ultrasonic via `set_car`); production uses `runtime.app`.
- **web_ui/** — Flask app (`app.py` factory + Blueprint route modules). Live UI: `templates/main.html`. Starts from `main_loop` or `runtime.app --web-ui`.

## Run (stub mode, no hardware)

From **car-x** root:

```bash
python -m cat_follow.main_loop
```

Then from another terminal or in code:

```python
from cat_follow.commands import set_cat_location, set_stop_command
set_cat_location(100, 50)   # state -> GOTO_TARGET then SEARCH
# set_stop_command()        # state -> IDLE
```

Ctrl+C stops the loop.

## ROCK 4D hardware deployment

The validated ROCK 4D installation uses:

- application: `/opt/car-x`
- virtual environment: `/opt/car-x/venv`
- runtime environment: `/etc/car-x/car-x.env`
- calibration: `/opt/picar-x/picar-x.conf`
- service: `/etc/systemd/system/cat-follow.service`

Run the real-hardware contract runtime manually:

```bash
cd /opt/car-x
set -a
. /etc/car-x/car-x.env
set +a
/opt/car-x/venv/bin/python -m cat_follow.runtime.app --picarx --with-prototype-perception
```

Optional monitoring UI (non-authoritative) on the contract runtime:

```bash
/opt/car-x/venv/bin/python -m cat_follow.runtime.app \
  --picarx --with-prototype-perception --web-ui --web-ui-port 5000
```

Then open `http://<rock-ip>:5000/`. The Control page shows contract FSM,
DecisionEngine constraints, lidar/ultrasonic, navigation fusion, perception
phase, a live occupancy map + robot pose (from ROS `/map` + TF when
`--ros-nav` is running), and optional H.264 when Rockchip MPP is available.
Disconnecting the browser stops stream encode (VM-24) while detection continues.

### Control-channel authentication

The web UI and UDP receiver are reachable beyond localhost in the normal
deployment. Configure both shared secrets in `/etc/car-x/car-x.env`:

```bash
CAT_FOLLOW_WEB_CONTROL_TOKEN=<strong-random-secret>
CAT_FOLLOW_COMMS_TOKEN=<strong-random-secret>
```

`CAT_FOLLOW_WEB_CONTROL_TOKEN` protects motion-causing control, calibration,
and stream-resolution routes. Supply it as the `X-Control-Token` header
(preferred) or a JSON/form `token` field. Query-string tokens are not accepted
(they leak into access logs). Stop and emergency-stop routes intentionally
remain open so any operator can halt the vehicle.

`CAT_FOLLOW_COMMS_TOKEN` protects UDP command datagrams. Senders must include a
matching top-level JSON `"token"` field; missing or invalid tokens are dropped.
Tracking datagrams are not gated.

Production requires both tokens to be non-empty. For explicit unauthenticated
bench/dev operation only, set:

```bash
CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL=1
```

Without that override, missing tokens cause control endpoints to fail closed.
Do not commit real token values to git; keep them in the deployment env file.

The ROCK 4D deployment is configured for the Radxa Camera 4K (IMX415):

- sensor: Sony IMX415, Type 1/2.8, 8.29 MP, 1.45 µm pixels
- output: four-lane MIPI CSI-2, RAW10/RAW12
- lens: M12 × P0.5, 2.95 mm EFL
- field of view: 88.2° diagonal, 75° horizontal, 59° vertical; 15° CRA
- capture device: `/dev/video11` (RKISP main path)
- capture format: 640×480 NV12 at 30 FPS, scaled by RKISP in hardware
- processing format: converted to the existing 640×480 BGR frame pool
- sensor controls: `/dev/v4l-subdev2`, exposure 900, analogue gain 48

Full sensor, lens, and optical specifications are recorded in
`docs/Hardware_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md`.

These values live in `scripts/car-x.env`. The service applies the sensor
controls once before startup and launches the headless camera, tracker, and
detector threads with `--with-prototype-perception`. The web UI remains
optional.

### Ultrasonic (event-driven HC-SR04)

Production on ROCK 4D uses `EdgeTimedUltrasonic` (`CatFollow-UltrasonicIRQ`)
instead of `robot_hat.Ultrasonic` GPIO polling. TRIG/ECHO are owned exclusively
by the edge worker; `Picarx(enable_ultrasonic=False)` keeps motors/servos on
the shared Picarx instance without claiming D2/D3.

Key env vars in `/etc/car-x/car-x.env`:

```text
CAT_FOLLOW_ULTRASONIC_CPU_CORE=3
CAT_FOLLOW_ULTRASONIC_RT_PRIORITY=70
CAT_FOLLOW_ULTRASONIC_REQUIRE_REALTIME=0
CAT_FOLLOW_PERCEPTION_DETECTOR_CORES=2
```

`REQUIRE_REALTIME=0` is the current stable production setting: core-3 affinity
works even when `SCHED_FIFO` is denied. Install `scripts/99-cat-follow-rtprio.conf`
to `/etc/security/limits.d/` if pursuing full RT scheduling. See
`docs/Software_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md` §4.7.

The systemd unit is installed but intentionally disabled until camera and
floor-drive testing are complete. Start and stop it explicitly:

```bash
sudo systemctl start cat-follow.service
sudo systemctl status cat-follow.service
sudo systemctl stop cat-follow.service
```

### ROS 2 deployment status

ROS 2 Jazzy, `slam_toolbox`, and Nav2 are installed on the ROCK 4D. Because
the Jazzy ARM64 repository does not publish `ros-jazzy-sllidar-ros2`, the
official Slamtec `sllidar_ros2` driver was built successfully from source in
`/opt/car-x/ros_ws`, together with `cat_follow_bringup`. The udev rule, WiFi
power-save configuration, runtime environment, and systemd units have been
installed. Both ROS services remain disabled and inactive until the C1 is
connected and a yard map exists.

Hardware validation remains pending: the C1 was not connected during
installation. The stable `/dev/rplidar` rule is installed, but device creation
and `/scan` output cannot be verified until the hardware arrives.

Build or rebuild the ROS workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd /opt/car-x/ros_ws
rosdep install --from-paths . --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source /opt/car-x/ros_ws/install/setup.bash
```

Detailed installation, service deployment, and first-lidar validation commands
are in `docs/Software_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md`.

See `docs/Hardware_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md`,
`docs/Software_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md`, and
`docs/Radxa_ROCK_4D_Robot_HAT_Power_Problem.md`.

## Calibration (save and load)

- **Web UI → Calibration tab:** Run speed/steer tests (Start/Stop), measure distance or radius, enter values in the table/fields, then click **Save calibration** to write to disk.
- **Storage:** `cat_follow/calibration/speed_time_distance.json`, `steering_limits.json`.
- **On startup:** `main_loop` creates `Calibration()`, which loads these JSONs. Odometry and goto use `get_cm_per_sec(speed)`; steering uses `get_max_steer_angle_deg()` and turn radii. Saving from the Web UI also updates the in-memory calibration for the current run (no restart needed for that session).

## Tests (no pytest required)

From **car-x** root:

```bash
python -c "from cat_follow.state_machine import StateMachine, State, Event; sm=StateMachine(); sm.dispatch(Event.CAT_LOCATION_RECEIVED, (10,10)); assert sm.state == State.GOTO_TARGET; print('OK')"
python -c "from cat_follow.calibration import Calibration; c=Calibration(); assert c.get_cm_per_sec(30)==12.0; print('OK')"
```

Or install pytest and run: `python -m pytest tests/ -v`. The current host
baseline is **376 passing tests** (includes `test_edge_ultrasonic.py` and
`test_range_sensor.py`).

## Next steps

1. **Validate the C1 lidar** — Connect it over USB, install/verify the `/dev/rplidar` udev rule, launch `sllidar_ros2` at 460800 baud, and confirm `/scan`.
2. **Complete ROCK 4D validation** — Run floor-drive and thermal tests and tune `LOST_THRESHOLD`, `DETECT_EVERY_K`, `APPROACH_TRACK_MARGIN_CM`, and calibration JSONs.
3. **RKNN model** — YOLOv8n COCO 320×320 for rk3576 is provisioned as
   `models/yolov8n_coco_320_rk3576.rknn` (built with
   `scripts/convert_yolo_to_rknn.py --platform rk3576 --no-quant`). Point
   `CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH` at it. On the ROCK 4D a missing model
   is a hard error (no CPU fallback). Decoding uses `vision/yolo_postprocess.py`
   (9-tensor model-zoo layout); chase-start warmup preloads the NPU when
   `CAT_FOLLOW_PERCEPTION_WARMUP_ON_START=0`.

## 📝 Version History

- **0.7.1** — Ops/monitoring hardening (medium-priority review items) plus
  ROCK 4D event-driven HC-SR04 acquisition. Legacy `robot_hat.Ultrasonic` GPIO
  polling replaced in the contract runtime by `EdgeTimedUltrasonic`
  (`CatFollow-UltrasonicIRQ`, libgpiod both-edge on `gpiochip2:16` /
  `gpiochip1:21`, core 3, best-effort `SCHED_FIFO`). `range_sensor.set_reader()`
  injects cached nonblocking reads; `Picarx(enable_ultrasonic=False)` avoids
  D2/D3 double-ownership. Legacy `main_loop.py` still uses the polling path.
  Telemetry no longer loses already-dequeued events (CRITICAL failsafe
  forensics in particular) when the sink write fails: failed batches are
  re-buffered and retried on the next drain, bounded and dropping the
  lowest-severity events first, and a detector escalation / e-stop now emits a
  CRITICAL telemetry event. Web-map freshness is recomputed at read time from
  `received_ms` + TTL (map/pose/scan), and an odom-frame fallback pose (TF
  failure) is flagged `pose_on_map=false` so it — and its scan rays — are no
  longer drawn over the map grid. UDP command datagrams are dropped unless they
  carry a matching `CAT_FOLLOW_COMMS_TOKEN` (when configured; tracking packets
  are unaffected), and the receiver warns at startup when command auth is off.
- **0.7.0** — Cross-pipeline safety/authority hardening from a full review.
  Freshness is now recomputed at decision time from `received_ms` + TTL (the
  sticky `fresh` flag is advisory only), so a dead ultrasonic/lidar/nav thread
  fails closed instead of staying authoritative forever; the ROS bridge ages
  `/cmd_vel` independently of `/odom` (a silent planner drops the drive terms).
  A detector escalation now stops the whole vehicle (motor e-stop + FAILSAFE
  latch + app teardown), and the legacy loop observes the stop event.
  Control-loop critical/consecutive overruns and tick exceptions latch FAILSAFE
  (inhibited until operator `clear_failsafe`) instead of allowing an immediate
  re-drive; comms `emergency_stop` actuates synchronously. Perception: the
  sticky detector bbox is cleared on IDLE, the tracker (re)inits on the exact
  frame the detector inferred on (frame-generation handoff) and fuses the
  post-update box, and vision now drives the FSM (`CAT_VISIBLE_STABLE` /
  `CAT_LOST`, stability keyed off tracker-frame generation, aged from genuine
  observations). Nav2 is demoted to advisory (cap/bias) in `CHASE_A`; `GOTO`
  still navigates. Ops: motion-causing web endpoints require
  `CAT_FOLLOW_WEB_CONTROL_TOKEN` when set (stops always open), calibration motor
  tests are serialized by a hardware arbiter, the H.264 encoder-fail path no
  longer leaks client counters, and a faulty/stuck ultrasonic fails closed.
- **0.6.1** — Detector production-safety hardening. The deterministic no-NPU
  stub is now opt-in via `CAT_FOLLOW_PERCEPTION_ALLOW_STUB` (default off); in
  production a missing/broken `rknnlite` hard-fails instead of masquerading as
  valid detection. The detector worker validates its backend once and reports
  readiness to the supervisor through a startup handshake, so a failed init
  aborts startup rather than silently killing the daemon thread. Failed NPU
  reloads (after idle-unload) and repeated inference failures now escalate
  (stopping the app; `CAT_FOLLOW_PERCEPTION_MAX_INFER_FAILURES`) instead of
  returning empty detections forever, and surface via `perception.error` on
  `/api/status`. Output-contract validation now checks box/score/class shape
  alignment and score range, not just the boxes tensor.
- **0.6.0** — RKNN NPU became the only production detection backend. Removed
  the legacy software backend and model downloader. The detector hard-fails
  when the RKNN runtime is present but the model is missing, and only runs the
  deterministic stub on machines without the RKNN runtime (dev/CI). Added
  `CAT_FOLLOW_PERCEPTION_RKNN_INPUT`; `scripts/benchmark_detector.py` now
  benchmarks the RKNN backend.
- **0.5.2** — Live ROS occupancy map in the web UI. `ros_bridge` subscribes to
  `/map` and TF `map→base_link` (odom fallback), publishes a downsampled
  snapshot with scan-ray overlay; Control page polls `/api/map` and draws the
  grid + robot pose on a canvas.
- **0.5.1** — Contract-runtime web UI adaptation. Added `--web-ui` /
  `--web-ui-port` to `runtime.app`, extended `/api/status` with SharedSnapshot
  (FSM, DecisionEngine constraints, lidar, navigation, vision) plus perception
  phase diagnostics, stream capabilities / H.264 toggle, CommsManager command
  routing from the Control page, and a read-only config panel. Removed orphaned
  legacy `web_ui/main.html` / `main.js` / `style.css`.
- **0.5.0** — Perception resource optimization + ROS 2 navigation integration.
  Added motion-gated detection with a perception phase FSM, lazy model load,
  boot warmup and idle unload (`malloc_trim` reclaim), adaptive image-processing threads and CPU
  affinity, an optional hardware-scaled RKISP lores motion stream, decoupled
  MJPEG (encode only with viewers, `simplejpeg` when available), and an
  optional Rockchip MPP H.264 WebSocket stream. Added the ROS 2 Jazzy track:
  `ros_ws/cat_follow_bringup` (C1 lidar, TF/URDF, slam_toolbox mapping +
  localization, Nav2 composed launch and embedded-tuned params), a
  `navigation.ros_bridge` + `odom_publisher`, a `--ros-nav` runtime flag, and
  DecisionEngine fusion of `NavigationState` (path_correction / speed_limit)
  and a lidar-critical obstacle veto — safety precedence unchanged.
- **0.4.0** — Added environment-configured V4L2 capture for the ROCK 4D
  Radxa Camera 4K (IMX415), including NV12 conversion/downscaling, camera
  permissions and startup controls, and headless perception service startup.
- **0.3.0** — Established the standalone control runtime and prototype
  perception-thread integration.

Design: see **DESIGN_CAT_FOLLOW_CLARIFICATIONS_AND_FILE_PLAN.md** and **DESIGN_CAT_FOLLOW_STATE_MACHINE.md**.
