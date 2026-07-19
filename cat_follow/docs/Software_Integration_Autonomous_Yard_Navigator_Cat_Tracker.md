# Software Integration
**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)  
**Compute target:** Radxa ROCK 4D (primary)  
**Navigation:** ROS 2 Jazzy hybrid (Nav2 + slam_toolbox) + `cat_follow` runtime  
**Version:** 1.0  
**Status:** Integration baseline (pre-implementation)

## 1. Purpose
This document defines how software components integrate on the ROCK 4D:
operating system choice, `robot_hat` porting, ROS 2 navigation stack, and the
bridge into existing `cat_follow` contracts (`SharedState`, `NavigationState`,
`DecisionEngine`).

Hardware wiring is in
`Hardware_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md`.

## 2. Design principles (unchanged)
From the architecture docs:
- **Only `DecisionEngine` commands motors** (safety precedence preserved).
- **Detection works without web UI.**
- Precedence: `failsafe > obstacle veto > pursuit logic`.
- ROS 2 provides **navigation constraints**, not direct motor authority in
  production.

```text
┌─────────────────────────────────────────────────────────────┐
│  cat_follow (Python) — main process                         │
│  CommsManager, VisionAdapter, RangeAdapter, DecisionEngine  │
│  FSM, MotorInterface → picarx / robot_hat                   │
│  ros_bridge → reads/writes SharedState.navigation           │
└───────────────────────────┬─────────────────────────────────┘
                            │ ROS 2 topics (DDS)
┌───────────────────────────▼─────────────────────────────────┐
│  ROS 2 Jazzy                                                │
│  sllidar_ros2 (C1) → /scan                                  │
│  slam_toolbox → localize on yard_map.yaml                   │
│  nav2 (composed) → path, recoveries, /cmd_vel              │
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

### 3.2 Alternatives
| OS | Use when |
|----|----------|
| **RadxaOS (Debian 12)** | Fastest vendor BSP bring-up; run Jazzy in Docker |
| **RadxaOS + Docker `ros:jazzy-ros-base`** | NPU on host, ROS isolated in container |
| **Armbian edge kernel** | Avoid for NPU + stable I2C on day one |

### 3.3 ROS 2 distribution
| Distro | Ubuntu | Support |
|--------|--------|---------|
| **Jazzy** | 24.04 | Recommended on ROCK 4D (LTS to 2029) |
| Humble | 22.04 | Use only if staying on Pi 4 without ROCK 4D |

Do **not** install Humble on Ubuntu 24.04.

### 3.4 First-boot software checklist
```bash
# After flash — enable I2C overlay via armbian-config, then:
sudo i2cdetect -y <bus>          # expect 0x14 (Robot HAT MCU)
sudo apt update
sudo apt install -y python3-libgpiod python3-smbus2 i2c-tools
sudo groupadd -f gpio
sudo usermod -aG gpio "$USER"
sudo install -m 0644 cat_follow/scripts/99-rock4d-gpio.rules \
  /etc/udev/rules.d/99-rock4d-gpio.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=gpio

# ROS 2 Jazzy (follow official Ubuntu 24.04 debs guide)
sudo apt install -y ros-jazzy-ros-base ros-jazzy-sllidar-ros2 \
  ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup

# Optional: disable WiFi power save (DDS stability)
# /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf → wifi.powersave = 2
```

### 3.5 NPU / RKNN (later milestone)
If adding NPU-accelerated vision on native Ubuntu 24.04:
- Use **Python 3.11 venv** (Miniforge) for RKNN-Toolkit-Lite2 wheels.
- Add udev rule for `/dev/rknpu` group access.
- Consider `cma=512M` or `1024M` in boot env for large camera buffers.

## 4. `robot_hat` port for ROCK 4D (Option B)

### 4.1 Why a port is required
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
| `picarx/picarx.py` | No pin label changes if `robot_hat` API preserved |

### 4.3 Verified GPIO map (libgpiod)
| HAT signal | Phys pin | ROCK 4D line |
|------------|----------|-------------|
| I2C SDA | 3 | I2C8_SDA_M1 / GPIO1_C7 |
| I2C SCL | 5 | I2C8_SCL_M1 / GPIO1_C6 |
| D2 TRIG | 13 | GPIO2_C0 |
| D3 ECHO | 15 | GPIO1_C5 |
| D4 motor L | 16 | GPIO2_B6 |
| D5 motor R | 18 | GPIO2_B7 |
| MCURST | 29 | GPIO3_A2 |

The map was verified on the target Armbian image with `gpioinfo`. Motor
direction lines and MCU reset were exercised with the motors disconnected;
the MCU returned at `0x14` after reset. The ultrasonic trigger and echo lines
were also verified with a successful range reading.

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
| **Navigation** | GOTO / CHASE_A pathing | Nav2 on saved map |

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

### 5.4 Odometry gap (PiCar-X)
PiCar-X has **no wheel encoders**. Nav2 and slam_toolbox expect `/odom`.

| Source | Quality |
|--------|---------|
| `cat_follow/odometry.py` (bicycle model) | Drifty; OK for short paths |
| slam_toolbox scan matching | Corrects pose on saved map |
| Overhead `car` pose | Global hint only; not authoritative for local steering |
| IMU (future) | Better heading |

Publish `/odom` from a small ROS node wrapping `odometry.py` until encoders
or IMU are added.

### 5.5 TF / URDF
Provide minimal URDF:
- `base_link` — chassis center
- `laser` — C1 mount offset (x, y, yaw)
- `camera_link` — optional

Align yard frame with architecture convention (+X right, +Y forward, heading
radians per Interface Specification §14).

## 6. Bridge: ROS 2 → `cat_follow`

### 6.1 Contract target
`NavigationState` in `cat_follow/control/types.py`:

| Field | ROS 2 source (conceptual) |
|-------|---------------------------|
| `heading` | Localized pose yaw (rad) |
| `heading_valid` | Localization active + fresh |
| `path_correction` | Lateral/heading delta from Nav2 `/cmd_vel` or local plan |
| `speed_limit` | Scale from obstacle proximity / Nav2 |
| `no_progress` | Nav2 progress checker |
| `dead_end` | Nav2 goal failed after recoveries |

### 6.2 Planned module layout
```text
cat_follow/
  navigation/
    __init__.py
    ros_bridge.py          # rclpy node: /scan, /odom, nav2 → NavigationState
    odom_publisher.py      # cat_follow odometry → nav_msgs/Odometry
  runtime/
    app.py                 # optional --ros-nav flag to start bridge

ros_ws/                    # optional colcon workspace
  cat_follow_bringup/
    launch/rock4d_nav.launch.py
    config/nav2_params.yaml
    config/slam_mapper.yaml
    urdf/picarx_lidar.urdf
```

### 6.3 DecisionEngine fusion (CHASE_A / GOTO)
```text
pursuit_steer  ← overhead cat or go_to target
path_correction ← NavigationState (from Nav2)
final_steer = clamp(pursuit_steer + path_correction)
final_speed = min(pursuit_speed, navigation.speed_limit × max_speed)
```
Obstacle veto from ultrasonic/range and lidar-critical sectors unchanged.

### 6.4 What not to vendor into Python
Do **not** copy Nav2 or slam_toolbox sources into `cat_follow`. Run them as
ROS 2 packages and bridge topics only.

## 7. Lidar SDK in repo
`rplidar_sdk-master/` (repo root) is the official Slamtec C++ SDK v2.1.0.
Use it for:
- Windows/desktop protocol testing (`app/ultra_simple`)
- Reference when debugging serial issues

On ROCK 4D production path, prefer **sllidar_ros2** over a custom ctypes
wrapper unless ROS is unavailable.

## 8. Runtime topology on ROCK 4D

| Thread / process | Responsibility | Rate |
|------------------|----------------|------|
| `cat_follow` main | FSM, DecisionEngine, adapters | 50 Hz control |
| `CatFollow-Proc` | Camera + TFLite (or NPU later) | independent |
| `CatFollow-Comms` | Overhead UDP | ~10 Hz |
| `CatFollow-Range` | Ultrasonic adapter | ~20 Hz |
| ROS 2 `sllidar_ros2` | `/scan` | ~10 Hz |
| ROS 2 `slam_toolbox` | localize | async |
| ROS 2 Nav2 (composed) | plan + avoid + recover | 5–20 Hz |
| `ros_bridge` | NavigationState publisher | 10–20 Hz |

Expect **high CPU** when all layers run; use composition, saved map, and no
on-board RViz. Pi 4 user reports showed 90–100% CPU for multi-process Nav2;
ROCK 4D has more headroom but camera + NPU + Nav2 still requires tuning.

## 9. Software milestones

### M4a — Platform bring-up
- [ ] Armbian Ubuntu 24.04 vendor image flashed.
- [x] I2C MCU `0x14` detected.
- [x] `robot_hat` port: GPIO + I2C verified with motors disconnected.
- [x] `picarx` and `PiCarXBackend` forward/stop on elevated bench.

### M4b — Lidar
- [ ] `sllidar_ros2` C1 launch on USB.
- [ ] `/scan` visible; RViz on PC optional.

### M4c — Mapping session
- [ ] Teleop map yard with `slam_toolbox` online_async.
- [ ] Save `yard_map.yaml` + pgms.
- [ ] Document map origin alignment with overhead yard frame.

### M4d — Navigation
- [ ] Localize on saved map.
- [ ] Nav2 composed bringup reaches goals in yard.
- [ ] Dead-end recovery observed (backup / replan).

### M4e — cat_follow bridge
- [ ] `ros_bridge` publishes `NavigationState`.
- [ ] `DecisionEngine` consumes constraints in GOTO / CHASE_A.
- [ ] Validation: VM entries for lidar-assisted navigation (extend Validation Matrix).

### M4f — Production hardening
- [ ] WiFi power save off for DDS.
- [ ] Thermal/fan policy documented.
- [ ] JSONL telemetry includes navigation + scan health.

## 10. Dual-board fallback (Option A software)
If `robot_hat` port is delayed:
- Run stock `cat_follow` + `picarx` on **Raspberry Pi 4** (Robot HAT).
- Run ROS 2 + C1 + bridge on **ROCK 4D**.
- UDP channel: `MotorCommand` / `NavigationState` between boards.

Reuse existing `CommsManager` patterns for the inter-board link.

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
