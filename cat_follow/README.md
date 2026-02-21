# cat_follow

Modular cat-follow feature for PiCar-X. Camera stays straight; car steers and drives to keep the cat in the middle of the frame.

## Layout

- **state_machine.py** — States and events; `dispatch(event, payload)`.
- **commands.py** — Stub: `set_cat_location(x,y)`, `set_stop_command()`; `poll_commands(on_cat_location, on_stop)`.
- **calibration/** — `loader.py` + JSONs: speed–time–distance, steering limits, bbox–distance.
- **motion/** — `driver` (stop, forward, backward, set_steer), `center_cat_control()`, `limits`.
- **vision/** — Stub `get_cat_bbox()`; replace with vilib/TFLite.
- **vision/** — Detector + tracker. Camera now writes into a small rotating
	frame ring (pre-allocated) and tracker uses an OpenCV single-object
	tracker with improved re-init logic (IoU, temporal confirmation).
- **odometry.py** — Stub (x,y), heading; replace with time-based dead reckoning.
- **main_loop.py** — Tick loop: commands → state machine → motion.

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

## Tests (no pytest required)

From **car-x** root:

```bash
python -c "from cat_follow.state_machine import StateMachine, State, Event; sm=StateMachine(); sm.dispatch(Event.CAT_LOCATION_RECEIVED, (10,10)); assert sm.state == State.GOTO_TARGET; print('OK')"
python -c "from cat_follow.calibration import Calibration; c=Calibration(); assert c.get_cm_per_sec(30)==12.0; print('OK')"
```

Or install pytest and run: `python -m pytest tests/ -v`

## Next steps

1. Wire real picar-x in `main_loop` (uncomment `set_car(Picarx())`).
2. Implement `motion/goto_xy.py` (drive toward target using odometry + calibration).
3. Implement `motion/search.py` (arc using steering limits).
4. Replace `vision/detector.get_cat_bbox()` with vilib COCO (class 16 = cat).
5. Add 30 FPS tracker in `vision/tracker.py` (OpenCV KCF/CSRT; detector every K frames).
6. Replace odometry stub with time + speed + steer integration.

Design: see **DESIGN_CAT_FOLLOW_CLARIFICATIONS_AND_FILE_PLAN.md** and **DESIGN_CAT_FOLLOW_STATE_MACHINE.md**.
