# Software Integration
**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X chassis)  
**Production platform:** Radxa ROCK 4D + Radxa 4K IMX415  
**Navigation:** ROS 2 Jazzy hybrid (Nav2 + slam_toolbox) + `cat_follow` runtime  
**Version:** 2.0  
**Status:** ROCK 4D/C1/RKNN/ultrasonic bring-up partially validated; canonical target integration is pending

## 1. Purpose
This document records verified current bring-up facts and the pending software
integration roadmap for the production ROCK 4D. The canonical target behavior
is defined by
`Target_Redesign_FSM_and_Runtime_Autonomous_Yard_Navigator_Cat_Tracker.md`.
Where this guide conflicts with that document, the canonical target redesign
wins.

The production configuration is ROCK 4D, Radxa 4K IMX415, RKNN inference only,
RF2O lidar odometry only, ROS 2/Nav2, C1 lidar, and forward ultrasonic. There
is no Raspberry Pi operational mode, no Raspberry Pi camera path, no TFLite
backend, and no bicycle/wheel-odometry fallback.

Hardware wiring is in
`Hardware_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md`.

## 2. Integration authority and status
- **Only `DecisionEngine` commands motors** (safety precedence preserved).
- **Detection works without web UI.**
- Web UI import/start/TLS, H.264 dependency/encoder, and incomplete control-auth
  configuration degrade optional monitoring or mutating control availability;
  they do not stop the headless camera/detector/tracker/control core.
- Mutating web routes fail closed (`503` for incomplete production token
  configuration, `401` for a missing/invalid configured token). Stop and
  emergency-stop remain intentionally unauthenticated.
- Emergency stop and hard safety vetoes preempt mission behavior.
- Both lidar and ultrasonic are required for autonomous motion and remain
  direct `DecisionEngine` safety inputs.
- `NavigationManager` will own Nav2 goal lifecycle, correlation, transforms,
  moving-goal refresh, completion qualification, and safe steering envelopes.
- `PerceptionLifecycleManager` will own detector, recording, and stream
  consumers independently.

The last two managers and the canonical FSM/protocol are **target components
that are not implemented yet**. Existing `ros_bridge`, `NavigationState`, and
old-FSM behavior are current migration inputs, not normative target behavior.

```text
┌─────────────────────────────────────────────────────────────┐
│  cat_follow (Python) — main process                         │
│  CommsManager, adapters, DecisionEngine, MotorInterface     │
│  Pending: NavigationManager + PerceptionLifecycleManager    │
│  Current bridge → SharedState; target manager → Nav2 goals  │
└───────────────────────────┬─────────────────────────────────┘
                            │ ROS 2 topics (DDS)
┌───────────────────────────▼─────────────────────────────────┐
│  ROS 2 Jazzy                                                │
│  sllidar_ros2 (C1) → /scan                                  │
│  slam_toolbox → localize on yard_map.yaml                   │
│  nav2 (composed) → goals, paths, constraints, /cmd_vel      │
└─────────────────────────────────────────────────────────────┘
```

## 3. Recommended operating system

### 3.1 Primary choice: Armbian Ubuntu 24.04 Noble — vendor kernel (6.1 BSP)
| Requirement | Why this OS |
|-------------|-------------|
| GPIO / I2C for Robot HAT | Device-tree overlays on Armbian; `libgpiod` |
| ROS 2 Jazzy | Native Ubuntu 24.04 target (`apt install ros-jazzy-*`) |
| NPU (future RKNN vision) | Vendor kernel exposes `/dev/rknpu` |
| Headless robot | **Armbian Minimal / CLI** — no desktop |

Download: [Armbian ROCK 4D](https://armbian.com/boards/radxa-rock-4d) —
select **Ubuntu 24.04** with **vendor / legacy 6.1** kernel, not edge mainline.

### 3.2 Development/deployment alternatives
| OS | Use when |
|----|----------|
| **RadxaOS (Debian 12)** | ROCK 4D BSP evaluation; run Jazzy in Docker |
| **RadxaOS + Docker `ros:jazzy-ros-base`** | ROCK 4D NPU on host, ROS isolated in container |
| **Armbian edge kernel** | Avoid for NPU + stable I2C on day one |

### 3.3 ROS 2 distribution
| Distro | Ubuntu | Support |
|--------|--------|---------|
| **Jazzy** | 24.04 | Recommended on ROCK 4D (LTS to 2029) |

Do **not** install Humble on Ubuntu 24.04.

### 3.4 First-boot software checklist
```bash
# After flash — enable I2C overlay via armbian-config, then:
sudo i2cdetect -y <bus>          # expect 0x14 (Robot HAT MCU)
sudo apt update
sudo apt install -y python3-libgpiod python3-smbus2 i2c-tools
sudo groupadd -f gpio
sudo usermod -aG gpio "$USER"
sudo install -m 0644 /opt/car-x/cat_follow/scripts/99-rock4d-gpio.rules \
  /etc/udev/rules.d/99-rock4d-gpio.rules
sudo install -m 0644 /opt/car-x/cat_follow/scripts/99-cat-follow-rtprio.conf \
  /etc/security/limits.d/99-cat-follow-rtprio.conf
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=gpio

# ROS 2 Jazzy (after adding the official Ubuntu 24.04 ROS repository)
sudo apt install -y ros-jazzy-ros-base ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup ros-dev-tools

# Jazzy ARM64 does not publish ros-jazzy-sllidar-ros2. Build the official
# Slamtec driver in the application workspace.
source /opt/ros/jazzy/setup.bash
cd /opt/car-x/ros_ws
git clone https://github.com/Slamtec/sllidar_ros2.git sllidar_ros2
sudo rosdep init                    # omit if already initialized
rosdep update
rosdep install --from-paths . --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source /opt/car-x/ros_ws/install/setup.bash

# Install runtime configuration. Preserve the previous environment first.
sudo install -d /etc/car-x /etc/NetworkManager/conf.d
sudo cp /etc/car-x/car-x.env /etc/car-x/car-x.env.pre-ros
sudo install -m 0644 /opt/car-x/cat_follow/scripts/car-x.env \
  /etc/car-x/car-x.env
sudo install -m 0644 /opt/car-x/cat_follow/scripts/99-rplidar.rules \
  /etc/udev/rules.d/99-rplidar.rules
sudo install -m 0644 \
  /opt/car-x/cat_follow/scripts/default-wifi-powersave-off.conf \
  /etc/NetworkManager/conf.d/default-wifi-powersave-off.conf
sudo install -m 0644 /opt/car-x/cat_follow/scripts/ros-nav.service \
  /etc/systemd/system/ros-nav.service
sudo install -m 0644 /opt/car-x/cat_follow/scripts/cat-follow-ros.service \
  /etc/systemd/system/cat-follow-ros.service
sudo udevadm control --reload-rules
sudo systemctl daemon-reload

# Keep both units disabled until the C1 is connected and the map exists.
sudo systemctl disable ros-nav.service cat-follow-ros.service
```

The runtime user must be in `video` for `/dev/video*`, `/dev/rga`,
`/dev/mpp_service`, and required DMA heaps. Rockchip media device nodes are
provisioned as group `video`, mode `0660`; do not make them world-writable.
`scripts/board_fix_mpp_h264.sh` installs the MPP plugin and persistent `0660`
udev rules when run explicitly as root. Board helper scripts contain no
password and must not embed, echo, or persist one; invoke them through the
operator's normal `sudo`/root procedure.

#### Provision the RKNN detection model (required)
Detection is RKNN-only; the runtime has no software inference fallback, so a fresh
deployment is not functional until a `.rknn` model is present. Build it once on
an x86 workstation / WSL with `rknn-toolkit2`, copy it to the board, and point
the env at it. Production uses calibrated **YOLOv8n COCO INT8 320×320**
(airockchip 9-tensor head) targeting **rk3576**. FP models remain alongside
INT8 models for accuracy comparisons.

```bash
# On x86 Linux / WSL (rknn-toolkit2 installed); ONNX must be the 9-tensor
# model-zoo head (not a plain Ultralytics single-tensor export):
python scripts/convert_yolo_to_rknn.py \
  --onnx yolov8n_320.onnx \
  --output models/yolov8n_coco_320_rk3576_int8.rknn \
  --platform rk3576 \
  --dataset calib_catft.txt

# Copy to the board (path must match CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH,
# resolved relative to /opt/car-x):
scp models/yolov8n_coco_320_rk3576_int8.rknn picarx@<board>:/opt/car-x/models/
```

The board runs the lightweight `rknnlite` runtime (install it into the venv on
the ROCK 4D). At startup the detector worker validates the backend and reports
readiness to the main thread through a **startup handshake**: the supervisor
blocks until the worker confirms a usable model (one real inference + output
contract check), so a missing/corrupt model or a failed init aborts startup and
the service exits non-zero (visible in `journalctl`) instead of the daemon
thread silently dying. Each service unit also has an `ExecStartPre`
file-existence guard for a clearer message. Ensure
`CAT_FOLLOW_PERCEPTION_RKNN_INPUT` matches the converted model's input geometry
(320,320 for the documented model).

Perception CPU affinity uses **A53 cores only** (`0–3`): camera `0,1`,
detector `2`, ultrasonic edge worker `3`. Leave A72 `4–7` for ROS/Nav2/control.
**Priority todo:** time ROS 2 modules (Nav2 / lidar / slam_toolbox / bridge)
during an active chase to measure A72 load and decide whether core
distribution/affinity can be rebalanced — detector `invoke` is ~2× slower on
A53 than A72 because `rknn.inference()` includes host-side CPU work.

### 3.5 Deployment progress (2026-07-20)
- [x] ROS 2 Jazzy `ros-base` installed on ROCK 4D.
- [x] `slam_toolbox` and Nav2 bringup installed.
- [x] Official `sllidar_ros2` and `cat_follow_bringup` built successfully in `/opt/car-x/ros_ws`.
- [x] ROS environment, udev rule, WiFi tuning, and systemd units deployed.
- [x] Launch-file argument loading and systemd unit syntax validated.
- [x] `picarx` user confirmed in the `dialout` group.
- [x] C1 connected; installed udev rule creates `/dev/rplidar` -> `ttyUSB0` (verified 2026-07-22).
- [x] C1 launched at 460800 baud; health OK and `/scan` measured at a steady ~10 Hz (verified 2026-07-22).

### 3.6 NPU / RKNN (sole detection backend)
Detection runs exclusively on the RK3576 NPU via RKNN; there is no software
inference fallback. When the RKNN runtime (`rknnlite`) is present but the `.rknn` model
is missing or fails to load, startup hard-fails with a `RuntimeError`. The
deterministic stub is **opt-in**: it only runs on machines that both lack the
RKNN runtime *and* set `CAT_FOLLOW_PERCEPTION_ALLOW_STUB=1` (dev laptop / CI).
In production `ALLOW_STUB` stays `0`, so a broken/uninstalled `rknnlite` never
masquerades as valid detection. At runtime, a failed NPU reload after
idle-unload or `CAT_FOLLOW_PERCEPTION_MAX_INFER_FAILURES` consecutive inference
failures escalate: the detector records the error in `perception.error` on
`/api/status` and stops the app (systemd restarts the unit) rather than
returning empty detections forever.

In `ZEROCOPY=dmabuf` mode the camera startup handshake runs one complete
dequeue/RGA/RKNN/requeue self-test before readiness. Native resources use RAII,
buffer ownership serializes infer/copy/requeue, crop geometry is validated
against the RKNN input, and the Python `ctypes` ABI includes the native model
lifecycle and crop/detection metadata. Idle unload releases RKNN/model
resources only; camera streaming, exported camera DMA-BUFs, RGA imports, and
the crop DMA-BUF remain alive for reload.

This changed native implementation has passed host contract tests but has not
been rebuilt, deployed, or validated on the ROCK 4D. Earlier fd-path results
remain historical bring-up evidence only.

- Use **Python 3.12 venv** (`/opt/car-x/venv`) with RKNN-Toolkit-Lite2 (`rknnlite`) on the board.
- Convert YOLOv8n (9-tensor ONNX) on an x86 workstation/WSL with
  `rknn-toolkit2` and the representative calibration list, then set
  `CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH=models/yolov8n_coco_320_rk3576_int8.rknn`
  (and `CAT_FOLLOW_PERCEPTION_RKNN_INPUT=320,320`). Keep the `--no-quant` FP
  build under its distinct filename for accuracy A/B.
- Pin perception to A53 cores `0–3` (camera `0,1`, detector `2`, ultrasonic `3`); leave A72 `4–7` for ROS/Nav2/control.
- Add udev rule for `/dev/rknpu` group access.
- Consider `cma=512M` or `1024M` in boot env for large camera buffers.

### 3.7 Current safety baseline and control-channel security

The 0.7.x safety review established the following **current implementation**
rules. They are preserved as verified baseline facts but do not replace the
pending canonical target rules:

- `DecisionEngine` recomputes range, lidar, navigation, and vision freshness
  from local monotonic `received_ms`; stored `fresh=True` values are advisory.
- `/cmd_vel` and `/odom` age independently. Navigation drive data is usable
  only while both are within TTL, and stale planner speed/steering terms are
  cleared.
- The current navigation drive path fails closed unless at least one obstacle
  sensor is fresh and valid. This is a migration gap: the target requires
  **both** lidar and ultrasonic to be fresh and valid for autonomous motion,
  with the canonical two-second hold/escalation behavior.
- Current persistent camera/RKNN failures differ from the target. Individual
  camera open/dequeue/drop events are transient and retried. Persistent open
  failure (default 5), persistent dequeue/read failure (default 30), no
  published frame for 5 seconds, zerocopy self-test/requeue failure, failed
  RKNN reload, or repeated inference failure is fatal: synchronously e-stop,
  latch runtime `FAILSAFE`, stop the app, and let `Restart=on-failure` restart
  the systemd unit. Optional Web UI/H.264/auth failures do not enter this path.
  The canonical target later replaces this migration behavior with its
  state-specific perception-fatal fallback.
- Map, pose, and scan freshness is recomputed when `/api/map` is read. A fresh
  odom-frame fallback pose is not overlaid on a map-frame occupancy grid when
  TF localization fails.
- Failed telemetry sink writes are re-buffered and retried; bounded overflow
  evicts lower-severity records before CRITICAL failsafe forensics.

Configure motion-channel authentication in `/etc/car-x/car-x.env`:

```text
CAT_FOLLOW_WEB_CONTROL_TOKEN=<strong-random-secret>
CAT_FOLLOW_COMMS_TOKEN=<strong-random-secret>
```

The web token protects motion-causing control/calibration routes; stop and
emergency-stop remain intentionally unauthenticated. UDP command packets must
carry a matching top-level JSON `token` when `CAT_FOLLOW_COMMS_TOKEN` is set.
Tracking packets are unaffected. Unset tokens retain development compatibility
only with the explicit bench override. Without that override, incomplete
tokens leave monitoring available but every guarded mutation returns `503` and
UDP ingress is not opened; the core runtime continues.

## 4. `robot_hat` port for ROCK 4D

### 4.1 Why the ROCK 4D port is required
`robot_hat` uses:
- **gpiozero** (Raspberry Pi only)
- **BCM GPIO numbers** in `pin.py`
- **I2C bus 1** (`smbus2`) for MCU @ 0x14

The port selects Radxa `libgpiod` line mappings and I2C bus 8 through:

```bash
export ROBOT_HAT_GPIO_BACKEND=rock4d
export ROBOT_HAT_I2C_BUS=8
```

I2C8_M1 must be enabled on physical pins 3 and 5.

### 4.2 Files to adapt
| Module | Change |
|--------|--------|
| `robot_hat/pin.py` | Map D2,D3,D4,D5,MCURST to ROCK lines (phys 13,15,16,18,29) |
| `robot_hat/i2c.py` | Default bus → Radxa I2C device (post-overlay) |
| `robot_hat/pwm.py` | Unchanged protocol to MCU 0x14 if I2C works |
| `robot_hat/adc.py` | Unchanged protocol to MCU 0x14 |
| `picarx/picarx.py` | Add `enable_ultrasonic=False` for contract runtime so D2/D3 are not claimed by `robot_hat` polling |

### 4.3 Verified GPIO map (libgpiod)
| HAT signal | Phys pin | ROCK 4D line | libgpiod (production) |
|------------|----------|-------------|------------------------|
| I2C SDA | 3 | I2C8_SDA_M1 / GPIO1_C7 | — |
| I2C SCL | 5 | I2C8_SCL_M1 / GPIO1_C6 | — |
| D2 TRIG | 13 | GPIO2_C0 | `gpiochip2` line **16** |
| D3 ECHO | 15 | GPIO1_C5 | `gpiochip1` line **21** |
| D4 motor L | 16 | GPIO2_B6 | via `robot_hat` |
| D5 motor R | 18 | GPIO2_B7 | via `robot_hat` |
| MCURST | 29 | GPIO3_A2 | via `robot_hat` |

The map was verified on the target Armbian image with `gpioinfo`. Motor
direction lines and MCU reset were exercised with the motors disconnected;
the MCU returned at `0x14` after reset. Ultrasonic TRIG/ECHO were verified
with a successful range reading during bring-up; production now uses
`cat_follow/perception/edge_ultrasonic.py` (libgpiod v1 both-edge events) instead
of `robot_hat.Ultrasonic` busy-wait polling.

### 4.4 Bring-up test sequence
1. `i2cdetect` → `0x14` — **done**
2. `robot_hat` PWM test on P0/P1/P2 (servo twitch ±10°) — **done** (motors disconnected)
3. Motor PWM P12/P13 with wheels off ground — **done**
4. `cat_follow` with `PiCarXBackend` / `--picarx` — **done**

### 4.5 Motor direction calibration (ROCK 4D bring-up)

Observed on the bench:

| Commanded | Observed |
|-----------|----------|
| Right forward (`D5` low) | Right reverse |
| Right reverse (`D5` high) | Right forward |
| Left forward / reverse | Matched |

The raw GPIO directions differ because the motors are mirrored. The PiCar-X
`forward()` implementation already sends opposite signed commands to the two
motors, so the verified high-level calibration is:

```text
picarx_dir_motor = [1, 1]
```

Using `[1, -1]` would double-invert the right motor when driving through
`Picarx.forward()` or `PiCarXBackend`.

### 4.6 Validated deployment

The live ROCK 4D deployment uses:

| Item | Path / value |
|------|--------------|
| Application source | `/opt/car-x` |
| Python environment | `/opt/car-x/venv` |
| Hardware environment | `/etc/car-x/car-x.env` |
| PiCar-X calibration | `/opt/picar-x/picar-x.conf` |
| systemd unit | `/etc/systemd/system/cat-follow.service` |
| GPIO permissions | `/etc/udev/rules.d/99-rock4d-gpio.rules` |
| RT priority limits | `/etc/security/limits.d/99-cat-follow-rtprio.conf` |

The environment file sets (see `scripts/car-x.env` for the full template):

```text
ROBOT_HAT_GPIO_BACKEND=rock4d
ROBOT_HAT_I2C_BUS=8
CAT_FOLLOW_PERCEPTION_CAMERA_CORES=0,1
CAT_FOLLOW_PERCEPTION_DETECTOR_CORES=2
CAT_FOLLOW_PERCEPTION_ZEROCOPY=dmabuf
CAT_FOLLOW_PERCEPTION_MOTION_GATING=1
CAT_FOLLOW_CAMERA_LORES_DEVICE=/dev/video12
CAT_FOLLOW_ULTRASONIC_CPU_CORE=3
CAT_FOLLOW_ULTRASONIC_RT_PRIORITY=70
CAT_FOLLOW_ULTRASONIC_REQUIRE_REALTIME=0
CAT_FOLLOW_ULTRASONIC_PING_INTERVAL_S=0.060
CAT_FOLLOW_ULTRASONIC_ECHO_TIMEOUT_S=0.040
CAT_FOLLOW_ULTRASONIC_STALE_AFTER_S=0.250
CAT_FOLLOW_WEB_CONTROL_TOKEN=<deployment secret>
CAT_FOLLOW_COMMS_TOKEN=<deployment secret>
PYTHONPATH=/opt/car-x
PYTHONUNBUFFERED=1
```

DMA-BUF frames do not expose CPU luma and an fd alone is not motion evidence.
Therefore `ZEROCOPY=dmabuf` plus `MOTION_GATING=1` requires a working RKISP
lores/luma source (normally `/dev/video12`). If the board cannot provide that
stream, set `MOTION_GATING=0` explicitly; leaving lores absent while expecting
motion-gated DMA-BUF detection is not a valid production configuration.

`CAT_FOLLOW_ULTRASONIC_REQUIRE_REALTIME=0` is the current production default:
the worker still pins to core 3 but degrades to affinity-only scheduling when
`SCHED_FIFO` is denied by the kernel despite `CAP_SYS_NICE` / `LimitRTPRIO=80`.
Set it to `1` only after validating RT scheduling on the target image.

### 4.7 Event-driven HC-SR04 acquisition (ROCK 4D production)

Legacy `robot_hat.Ultrasonic` polls the ECHO GPIO in a tight loop and can
saturate one CPU core. The contract runtime (`runtime.app
--with-prototype-perception`) uses a dedicated libgpiod edge-event worker instead:

```text
EdgeTimedUltrasonic (CatFollow-UltrasonicIRQ, libgpiod v1 both-edge, core 3)
 └─ latest_distance_cm()  [nonblocking cache; kernel event timestamps]
 └─ range_sensor.set_reader(...)
 └─ range_sensor.get_distance_cm()  [60 ms throttle]
 └─ RangeAdapter (CatFollow-RangeAdapter, ~20 Hz)
 └─ SharedState.range → DecisionEngine
Picarx(enable_ultrasonic=False)  # motors/servos only; avoids D2/D3 conflict
App.start/stop → range_source.start/stop
```

GPIO ownership: only one consumer may request D2/D3. Production constructs
`Picarx(enable_ultrasonic=False)` and lets `EdgeTimedUltrasonic` own TRIG/ECHO
with consumer `"cat-follow-ultrasonic"`. The legacy prototype path
`main_loop.py` still uses `range_sensor.set_car(px)` and the polling
`Picarx.get_distance()` path; it has **not** been migrated to the edge reader.

The persisted hardware calibration is:

```text
picarx_dir_motor = [1, 1]
picarx_dir_servo = -6
picarx_cam_pan_servo = 8
picarx_cam_tilt_servo = -3
```

Look/drive treats commanded pan `0` (after this servo offset) as mechanical
forward. Runtime `look_pan_forward_deg` carries that calibrated forward into
`LookDriveController`. Polarity contract: positive `x_offset` / pan = cat /
camera toward the **right** (matches `stare_at_you` / `bull_fight`); no
software flip knob unless board evidence contradicts the contract. See
`Look_Drive_Path_Design.md` §6.1.

The service was successfully started with the real `PiCarXBackend`, observed
active, and stopped cleanly. It remains deliberately **disabled** until camera
and floor-drive validation are complete:

```bash
sudo systemctl start cat-follow.service
sudo systemctl status cat-follow.service
sudo systemctl stop cat-follow.service
```

## 5. ROS 2 navigation stack

### 5.1 Packages
| Package | Role |
|---------|------|
| **sllidar_ros2** | C1 USB driver → `sensor_msgs/LaserScan` on `/scan` |
| **slam_toolbox** | Build map (once) + localize on saved map |
| **nav2_bringup** | Global plan, local avoid, dead-end recovery |
| **tf2** | `map` → `odom` → `base_link` → `laser` |

Launch C1 (example):
```bash
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py serial_port:=/dev/ttyUSB0
```

### 5.2 Mapping vs navigation modes
| Mode | When | Command concept |
|------|------|-----------------|
| **Mapping (live SLAM)** | One-time yard survey | `slam_toolbox` online_async + teleop |
| **Localization** | Every chase session | `slam_toolbox` on saved `yard_map.yaml` |
| **Navigation** | Canonical `GOTO`, `GETTING_CLOSE`, `SEARCH`, `CHASE`, and `RETURN_HOME` | Nav2 on saved map |

**Do not** run live mapping during cat chase — use a **pre-saved map**.

### 5.3 Nav2 scope
**Full Nav2** = map_server + AMCL or slam_toolbox localize + planner_server +
controller_server + behavior_server + bt_navigator + costmaps + lifecycle_manager.

Use **composition mode** on ROCK 4D to reduce CPU (~28% savings per ROS
composition benchmarks on Pi-class ARM).

Tune for embedded:
- Lower costmap update rates (1–5 Hz global, 5 Hz local).
- 2D lidar costmap only (no voxel/3D layers).
- No RViz on board (run on PC).
- `online_async` slam_toolbox, not sync.

### 5.4 Odometry contract
PiCar-X has **no wheel encoders**, and the current hardware has **no IMU
installed**. Nav2 and slam_toolbox expect `/odom`.

| Source | Quality |
|--------|---------|
| ROS 2 lidar-odometry scan matcher (RF2O — production/default) | Local `odom -> base_link` from consecutive C1 scans; may degrade in feature-poor or dynamic scenes |
| slam_toolbox scan matching | Corrects pose on saved map |
| Overhead `car` pose | Global hint only; not authoritative for local steering |
| IMU (future; not installed) | Better heading after hardware is added and validated |

The architecture uses a ROS 2 lidar-odometry node (RF2O) to publish `/odom`
and the dynamic `odom -> base_link` transform. It is the production/default and
only supported odometry source.

RF2O is the only supported production odometry source. Bicycle/wheel odometry
is unsupported and has no stale-data or startup fallback. Configuration or
code paths requesting it must be rejected rather than substituted. Loss of
RF2O/localization follows the canonical health and failsafe rules.
Exactly one component may publish `odom -> base_link`.

Current software must not require, consume, or assume an IMU measurement for
startup, localization, navigation, or safety decisions. Any future IMU fusion
must remain disabled until the hardware is installed and validated.

### 5.5 TF / URDF
Provide minimal URDF:
- `base_link` — chassis center
- `laser` — C1 mount offset (x, y, yaw)
- `camera_link` — optional

Align yard frame with architecture convention (+X right, +Y forward, heading
radians per Interface Specification §14).

## 6. Target ROS 2 integration (pending)

### 6.1 Current bridge versus target manager
The current `navigation/ros_bridge.py` reads ROS data into `NavigationState`.
It does not yet implement the complete target `NavigationManager`.

`NavigationManager` must own:

- `NavigateToPose` action clients and state-derived goal intents;
- calibrated yard-to-navigation-frame transforms;
- moving-target refresh (default maximum 2 Hz and minimum 25 cm displacement);
- cancel/preemption and goal-intent/action-result correlation;
- neutral classification of expected moving-goal replacement;
- path viability, safe steering envelope, retries, and exhausted failures;
- completion only after correlated Nav2 success plus fresh local pose within
  20 cm XY and 0.3 rad yaw continuously for one second.

Nav2 `BackUp` remains disabled. Reverse is not a Nav2 fallback: the sole target
reverse is `DecisionEngine`-owned `BRAKE_REVERSE`, triggered by a fresh valid
lidar or ultrasonic reading strictly below 15 cm. Its phases are
STOP, CENTER, 100 ms SETTLE, straight normalized `-0.30` REVERSE for 0.5 s,
STOP, and dual-sensor RECHECK, with at most three attempts.

### 6.2 Target ultrasonic and dual-sensor integration
The existing edge-driven HC-SR04 reader and `RangeAdapter` are verified current
building blocks. The target integration is still pending:

- publish validated ultrasonic data as `sensor_msgs/Range`, including frame,
  radiation type, field of view, min/max range, finite value, and timestamp;
- provide a valid transform into the local costmap frame;
- integrate the topic through Nav2 local-costmap `RangeSensorLayer`;
- keep lidar integrated independently;
- retain both sources as direct `DecisionEngine` safety inputs even when the
  costmap layer is active.

Both sensors are mandatory for every autonomous motion start. In normal driving,
loss of either commands zero motion, retains the objective for up to two
seconds, resumes only if both recover, and otherwise enters `FAILSAFE`. During
`BRAKE_REVERSE`, loss of either enters `FAILSAFE` immediately.

### 6.3 Planned module layout
```text
cat_follow/
  navigation/
    __init__.py
    ros_bridge.py          # current ROS input bridge; migration input
    navigation_manager.py  # target Nav2 actions/goals/correlation/completion
  runtime/
    app.py
  perception/
    lifecycle_manager.py   # target named consumers and camera lifecycle

ros_ws/                    # optional colcon workspace
  cat_follow_bringup/
    launch/rock4d_nav.launch.py
    config/nav2_params.yaml
    config/slam_mapper.yaml
    urdf/picarx_lidar.urdf
```

### 6.4 Target DecisionEngine fusion
```text
drive gate = fresh localization/navigation AND fresh valid lidar AND ultrasonic
CHASE steering = clamp(camera request, Nav2 safe steering envelope)
speed = min(pursuit request, Nav2, alignment, obstacle, and thermal caps)
```
Camera and Nav2 steering are never added or combined by weighted sum.
`DecisionEngine` remains the sole drivetrain authority. This fusion and the
complete `NavigationManager` output path are not implemented yet.

### 6.5 What not to vendor into Python
Do **not** copy Nav2 or slam_toolbox sources into `cat_follow`. Run them as
ROS 2 packages and bridge topics only.

## 7. Lidar SDK
`../rplidar_sdk-master/` (a sibling of the `car-x` repository, at
`C:\Users\rahaber\my_projects\rplidar_sdk-master`) is the official Slamtec
C++ SDK.
Use it for:
- Windows/desktop protocol testing (`app/ultra_simple`)
- Reference when debugging serial issues

The ROCK 4D production path uses **sllidar_ros2**; a custom ctypes lidar
wrapper is not an operational fallback.

## 8. Runtime topology on ROCK 4D

| Thread / process | Responsibility | Rate |
|------------------|----------------|------|
| `cat_follow` main | FSM, DecisionEngine, adapters | 50 Hz control |
| `CatFollow-Proc` | Camera + RKNN NPU detection | independent |
| `CatFollow-Comms` | Overhead UDP | ~10 Hz |
| `CatFollow-UltrasonicIRQ` | libgpiod HC-SR04 edge worker (cached distance) | ~17 Hz ping (60 ms min interval) |
| `CatFollow-RangeAdapter` | Polls `range_sensor.get_distance_cm()` → `SharedState.range` | ~20 Hz |
| ROS 2 `sllidar_ros2` | `/scan` | ~10 Hz |
| ROS 2 `slam_toolbox` | localize | async |
| ROS 2 Nav2 (composed) | plan + avoid + recover | 5–20 Hz |
| `ros_bridge` | NavigationState publisher | 10–20 Hz |

Expect **high CPU** when all layers run; use composition, saved map, and no
on-board RViz. Camera + NPU + Nav2 still requires ROCK 4D profiling and tuning.

### 8.1 Current optional monitoring behavior
The contract runtime builds and runs Flask on a daemon thread after the
headless perception/control core is ready. Web import, route initialization,
TLS certificate, bind/server, Flask-Sock, GStreamer, or `mpph264enc` failure
logs a degraded monitoring state and leaves the core active. H.264 has no
MJPEG/software fallback.

The current H.264 route admits one viewer at a time. Admission reserves the
single encoder owner atomically; later viewers are closed. DMA-BUF submission
holds at most one camera lease until the matching encoded PTS arrives, drops
additional pending inputs, and polls the appsink on every loop so delayed
access units are delivered even when no new camera frame is published. Encoder
start/stop errors and disconnect cleanup cannot leave the stream-client count
latched.

### 8.2 Target perception and media lifecycle (pending)
`PerceptionLifecycleManager` will own independent, reference-counted
`detector`, `recording`, and `stream` consumers. Detector and recording must
work headlessly and must never depend on a browser or stream client.

With no consumer, the IMX415 must become ready-inactive using verified V4L2
`STREAMOFF` (or an equivalent hardware pause), without frame processing or a
busy loop. Reactivation uses `STREAMON` and device revalidation. `HOME` and
`FAILSAFE` force all consumers off. The current effectively always-active
camera path does not satisfy this target.

Target recording uses hardware H.264 and segmented, crash-tolerant Matroska
(`.mkv`) files with quota, free-space reserve, finalized-segment cleanup,
restart recovery, requested-versus-actual telemetry, automatic retry, and
chase post-roll. Recording is independent of monitoring clients.

Target monitoring is hardware H.264 only and exists only while an actual client
is connected and the FSM permits it. If a monitoring hardware H.264 path is
unavailable, there is **no monitoring stream**. MJPEG and software encoding are
not target fallbacks.

## 9. Software milestones

### M4a — Platform bring-up
- [x] Armbian Ubuntu 24.04 vendor image flashed.
- [x] I2C MCU `0x14` detected.
- [x] `robot_hat` port: GPIO + I2C verified with motors disconnected.
- [x] `picarx` and `PiCarXBackend` forward/stop on elevated bench.

### M4b — Lidar
- [x] ROS 2 Jazzy, Nav2, and `slam_toolbox` installed on the ROCK 4D.
- [x] Official `sllidar_ros2` built from source in `/opt/car-x/ros_ws`.
- [x] `sllidar_ros2` C1 launch authored (`cat_follow_bringup/launch/sllidar_c1.launch.py`, 460800, `/dev/rplidar` udev symlink).
- [x] `/dev/rplidar` udev rule installed on the ROCK 4D.
- [x] Verified that the rule creates `/dev/rplidar` -> `ttyUSB0` on hardware.
- [x] `/scan` visible on hardware at ~10 Hz; C1 reports health OK (RViz on PC optional).

### M4c — Mapping session
- [x] `slam_toolbox` online_async mapping launch + params authored (`mapping.launch.py`, `config/slam_mapper.yaml`).
- [ ] Teleop-map the yard and save `yard_map.yaml` + pgm into `cat_follow_bringup/maps/`. _(C1 ready; mapping session pending)_
- [x] Map origin alignment procedure documented (`cat_follow_bringup/maps/README.md`).

### M4d — Navigation
- [x] Nav2 composed bringup + embedded-tuned params authored (`rock4d_nav.launch.py`, `config/nav2_params.yaml`, `config/slam_localization.yaml`).
- [ ] Localize on saved map; Nav2 reaches goals; dead-end recovery observed. _(pending saved yard map)_

### M4e — Current `cat_follow` bridge baseline
- [x] `navigation/ros_bridge.py` publishes `NavigationState` + lidar `RangeState` (LIDAR_C1); `navigation/odom_publisher.py` publishes `/odom` + `odom->base_link`.
- [x] Current `DecisionEngine` fuses `path_correction`/`speed_limit` and a
  lidar-critical veto in legacy modes. This is verified migration baseline,
  not the canonical target fusion.
- [x] `--ros-nav` runtime flag starts the bridge + odom publisher in-process.
- [x] Validation Matrix extended (VM-21 scan health, VM-22 lidar veto, VM-23 lidar-assisted nav, VM-24 headless efficiency).

### M4f — Production hardening
- [x] WiFi power-save-off config authored (`scripts/default-wifi-powersave-off.conf`).
- [x] systemd units authored (`scripts/ros-nav.service`, `scripts/cat-follow-ros.service`).
- [x] Host regression suite complete: `python -m pytest tests -q` — **511
  passing** on 2026-07-26. This is host contract evidence, not a ROCK 4D
  hardware pass.
- [x] Event-driven HC-SR04 reader (`edge_ultrasonic.py`) wired into contract runtime; GPIO exclusivity via `enable_ultrasonic=False`.
- [x] Web/UDP motion paths support shared-secret authentication and warn when open.
- [x] CRITICAL telemetry survives transient sink failure through bounded retry.
- [ ] **Priority:** profile ROS 2 modules (Nav2 / lidar / slam / bridge) CPU
  usage during an active chase; use the data to decide whether A53/A72 affinity
  can be rebalanced (detector invoke is ~11 ms unpinned/A72 vs ~22 ms on A53).
- [ ] **Priority:** prove `CAT_FOLLOW_PERCEPTION_DETECT_FPS=30` is stable in the
  production app during a complete chase with ROS 2/Nav2 active. Soak for
  several minutes and record actual end-to-end detection cadence, 33.3 ms
  deadline misses/dropped frames, RKNN stage latency, thermal throttling, and
  per-core CPU utilization/affinity.
- [ ] Thermal/fan policy documented.
- [ ] JSONL telemetry includes navigation + scan health end-to-end on hardware.
- [x] Current frame-ring slot pinning implemented for detector and legacy
  media consumers: four-slot refcounted leases + per-slot generation,
  center-bottom 320×320 detector crop, and legacy detector staging removed.
  See [`Frame_Ring_Ownership_Audit.md`](Frame_Ring_Ownership_Audit.md).
- [x] Native RKISP NV12 software ring path: retain `/dev/video11` 640×480 NV12
  through capture/ring/motion. The 320×320 AI crop converts directly from NV12
  into the preallocated RGB RKNN tensor without an intermediate BGR image.
  Legacy viewers and injection convert at their explicit boundaries; H.264
  submits packed NV12 directly to `mpph264enc`. This raw RKISP source does not
  require `mppjpegdec`.
- [x] ROCK 4D native NV12 + real RKNN headless smoke: contiguous 460,800-byte
  `/dev/video11` frames, crop/full conversion equality, camera generation 28,
  detector generation 27, and ~21–22 ms NPU invoke without web/ROS/motors.
- [x] Direct NV12→RGB RKNN input benchmark at 30 FPS: 354 headless inferences,
  color conversion p50 2.08 ms versus 3.63 ms for NV12→BGR→RGB, detector p50
  16.75 ms/p95 19.57 ms, and zero 33.3 ms detector-processing deadline misses.
- [ ] Soak frame-drop behavior under concurrent detector + stream leases and
  active ROS 2/Nav2 load.
- [x] Zero-copy repair implementation landed: startup self-test, native RAII
  and queued/dequeued ownership, validated crop/model geometry, synchronized C
  ABI/`ctypes`, RKNN-only idle unload/reload, H.264 one-viewer ownership and
  delayed-AU polling, and Option A/B parity gate defaults (aggregate count
  delta 0; median top-box IoU ≥0.90).
- [x] Build the changed native library in an isolated ROCK 4D staging tree and
  run 10 native RGA/RKNN frames, model unload/reload, repeat-session fd cleanup,
  10 same-input parity frames (count delta 0), and 30/30 DMA-BUF MPP/H.264
  access units. The installed service was restored active afterward.
- [ ] Deploy the repaired revision and complete comparable-box IoU, long
  fd/QBUF and detector/H.264 concurrency soak, `/dev/video12` motion, forced
  camera-fault, and secure-context browser playback gates.

### M4g — RF2O lidar odometry (no encoders / no IMU)
- [x] Define odometry-source contract: RF2O lidar scan matching is the sole
  supported production local odometry source; no bicycle fallback exists.
- [x] Require exclusive ownership of `/odom` and `odom -> base_link`.
- [x] Selected `rf2o_laser_odometry` (`ros2` branch) and built it successfully
  from source for ROS 2 Jazzy on the aarch64 ROCK 4D.
- [x] Add configurable lidar-odometry launch and parameter files
  (`cat_follow_bringup/launch/lidar_odom.launch.py`,
  `cat_follow_bringup/config/lidar_odom.yaml`).
- [x] Disable and reject the `cat_follow` bicycle odometry source because it
  would publish a frozen `/odom`. RF2O is the production source
  (`CAT_FOLLOW_ODOM_SOURCE=lidar`).
- [x] Require a saved map for localization/Nav2: `CAT_FOLLOW_MAP_FILE` (or the
  `map_file` launch argument) is validated at launch (`rock4d_nav.launch.py`)
  and by `ros-nav.service`; localization fails clearly when unset or when the
  serialized `<basename>.posegraph` / `.data` files are missing. Mapping
  (`mapping.launch.py`) remains map-free.
- [x] Integrate lidar odometry into mapping and navigation launch files, with
  RF2O enablement tied to ``CAT_FOLLOW_ODOM_SOURCE`` and slam_toolbox lifecycle
  autostart.
- [x] Validate scan-matching odometry and its failure behavior on C1 hardware
  (2026-07-22: mapping launch with `CAT_FOLLOW_ODOM_SOURCE=lidar` shows RF2O as
  sole `/odom` publisher, slam_toolbox lifecycle `active`, `/map` publishing).
- [ ] Keep IMU fusion disabled until an IMU is installed and validated.

### Deployment units (added in 0.5.0)
| Unit | Role |
|------|------|
| `scripts/ros-nav.service` | External ROS stack: C1 lidar + slam_toolbox localization + Nav2 composed |
| `scripts/cat-follow-ros.service` | cat_follow runtime + in-process `--ros-nav` bridge/odom (use instead of `cat-follow.service` when navigating) |
| `scripts/99-rplidar.rules` | Stable `/dev/rplidar` symlink for the C1 |
| `scripts/default-wifi-powersave-off.conf` | Disable WiFi power save for DDS stability |
| `scripts/99-cat-follow-rtprio.conf` | PAM rtprio/memlock limits for `picarx` (install to `/etc/security/limits.d/`) |
| `scripts/cat-follow.service` | `AmbientCapabilities=CAP_SYS_NICE`, `LimitRTPRIO=80`; ultrasonic RT is best-effort when `REQUIRE_REALTIME=0` |

The venv must expose `rclpy` for `--ros-nav` (create with
`python3 -m venv --system-site-packages /opt/car-x/venv` and keep
`/opt/ros/jazzy` sourced in the unit).

The deployed units are intentionally disabled until the C1 is connected and a
yard map has been saved. After connecting the lidar, reload udev and validate
the device before starting ROS:

```bash
sudo udevadm trigger
ls -l /dev/rplidar
source /opt/ros/jazzy/setup.bash
source /opt/car-x/ros_ws/install/setup.bash
ros2 launch cat_follow_bringup sllidar_c1.launch.py
# In another shell:
ros2 topic hz /scan
```

Only after `/scan` is healthy should `ros-nav.service` and
`cat-follow-ros.service` be enabled or started.

## 10. Canonical target integration roadmap (pending)

The following work must land and be validated together before this guide may
describe the canonical target as implemented:

1. Replace the legacy states with `HOME`, `IDLE`, `GETTING_CLOSE`, `SEARCH`,
   `CHASE`, `BRAKE_REVERSE`, `GOTO`, `RETURN_HOME`, and `FAILSAFE`, including
   the complete canonical transition and control-precedence rules.
2. Implement `NavigationManager`, including moving overhead goals, transforms,
   action correlation, safe steering envelope, retries, and qualified
   completion.
3. Publish ultrasonic as validated `sensor_msgs/Range`, integrate
   `RangeSensorLayer`, and enforce the dual-required lidar/ultrasonic health
   contract independently of costmap integration.
4. Implement Protocol V1 extensions: stable `target_id`, transactional
   command/event application, durable deduplication, post-application ACKs,
   and reliable `PRIMARY_CAT_LEFT_PERIMETER` handoff.
5. Implement the formal strictly-below-15-cm `BRAKE_REVERSE` maneuver and
   validate its bounded no-rear-sensor risk on the actual car.
6. Implement `PerceptionLifecycleManager`, named consumers, IMX415
   `STREAMOFF`/`STREAMON`, hardware-H.264 segmented MKV recording, post-roll,
   retention, recovery, and client-counted hardware-H.264 monitoring.
7. Remove target reliance on legacy PhaseMachine mission inference and prove
   SEARCH/CHASE detection and recording with no web UI or stream client.
8. Validate the saved yard map, RF2O-only localization, Nav2 goals, thermal
   behavior, full-chase performance, failure injection, and long-duration
   headless operation.

There is no dual-board or Raspberry Pi operational fallback. Deployment remains
the single ROCK 4D production architecture.

## 11. Glossary
| Term | Meaning |
|------|---------|
| **RViz** | PC tool to visualize map, laser, paths (not on-robot) |
| **Live SLAM** | Building map while driving (mapping day only) |
| **AMCL** | Particle-filter localization on known map |
| **Full Nav2** | Complete navigation stack with recovery behaviors |
| **Hybrid** | ROS 2 navigates; `cat_follow` owns safety and motors |
| **Composition mode** | Multiple Nav2 nodes in one process (less CPU) |

## 12. References
- ROS 2 Jazzy install: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- sllidar_ros2: https://github.com/Slamtec/sllidar_ros2
- Nav2: https://navigation.ros.org/
- slam_toolbox: https://github.com/SteveMacenski/slam_toolbox
- Radxa ROCK 4D docs: https://docs.radxa.com/en/rock4/rock4d
- Repo contracts: `Interface_and_Data_Contract_Specification_*.md`
- Repo runtime: `cat_follow/runtime/app.py`, `cat_follow/control/types.py`
