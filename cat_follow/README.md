# cat_follow

**Version:** 0.5.0

Modular cat-follow feature for PiCar-X. Camera stays straight; car steers and drives to keep the cat in the middle of the frame.

## Layout

- **state_machine.py** — States and events; `dispatch(event, payload)`.
- **commands.py** — Stub: `set_cat_location(x,y)`, `set_stop_command()`; `poll_commands(on_cat_location, on_stop)`.
- **calibration/** — `loader.py` + JSONs: speed–time–distance, steering limits (incl. target approach distance). Stored in `cat_follow/calibration/*.json`; loaded once at startup.
- **motion/** — `driver`, `center_cat_control()`, `limits`, `goto_xy` (runtime goto), `search`. Runtime goto uses **motion/goto_xy.py**; **calibration/goto_xy.py** is for calibration runs only.
- **vision/** — `get_cat_bbox(image)` uses TFLite (`tflite_common.py` + `detector.py`). Optional API for single-frame detection.
- **threads/** — Camera, tracker (OpenCV single-object tracker, re-init via IoU), detector (TFLite loop; writes to SharedState). Camera writes into a pre-allocated frame ring; main loop copies to detector frame every K frames.
- **odometry.py** — Bicycle-model dead reckoning (position, heading). Used via **location/** facade.
- **main_loop.py** — Tick loop: commands → state machine → motion.
- **web_ui/** — Flask app (`app.py` factory + Blueprint route modules). Live UI: `templates/main.html`; static assets in `web_ui/static/`.

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
`~/ros2_ws`. Both `/opt/ros/jazzy/setup.bash` and the workspace setup are
loaded from the `picarx` user's `.bashrc`, and that user belongs to the
`dialout` group.

Hardware validation remains pending: the C1 was not connected during
installation, so the stable `/dev/rplidar` udev symlink and `/scan` output
have not yet been verified.

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

Or install pytest and run: `python -m pytest tests/ -v`

## Next steps

1. **Validate the C1 lidar** — Connect it over USB, install/verify the `/dev/rplidar` udev rule, launch `sllidar_ros2` at 460800 baud, and confirm `/scan`.
2. **Complete ROCK 4D validation** — Run floor-drive and thermal tests and tune `LOST_THRESHOLD`, `DETECT_EVERY_K`, `APPROACH_TRACK_MARGIN_CM`, and calibration JSONs.
3. **TFLite models** — Place a compatible `.tflite` model (e.g. SSD MobileNet V2) in `models/` so the detector thread and `vision.get_cat_bbox()` can use it when not in stub mode.

## 📝 Version History

- **0.5.0** — Perception resource optimization + ROS 2 navigation integration.
  Added motion-gated detection with a perception phase FSM, a pluggable
  detection backend (CPU TFLite / RK3576 NPU RKNN) with lazy load, boot warmup
  and idle unload (`malloc_trim` reclaim), adaptive OpenCV threads and CPU
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
