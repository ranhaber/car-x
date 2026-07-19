# cat_follow

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

1. **Run on hardware** — Test on Pi with real Picar-X, camera, and ultrasonic; tune `LOST_THRESHOLD`, `DETECT_EVERY_K`, `APPROACH_TRACK_MARGIN_CM`, and calibration JSONs.
2. **TFLite models** — Place a compatible `.tflite` model (e.g. SSD MobileNet V2) in `models/` so the detector thread and `vision.get_cat_bbox()` can use it when not in stub mode.
3. **Optional** — Add tests for `vision.get_cat_bbox()` with a fixture image; extend calibration UI if you add more steering/speed parameters.

Design: see **DESIGN_CAT_FOLLOW_CLARIFICATIONS_AND_FILE_PLAN.md** and **DESIGN_CAT_FOLLOW_STATE_MACHINE.md**.
