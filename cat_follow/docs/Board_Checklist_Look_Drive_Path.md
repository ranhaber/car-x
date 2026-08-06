# Look / Drive / Path — ROCK 4D board validation checklist

Run on the production ROCK 4D with `--ros-nav`, a saved yard map when
available, and `CAT_FOLLOW_ENVELOPE_PROVIDER=costmap_sweep` (default).

Record telemetry JSONL for each scene (`look_drive_mode`, `pan_deg`,
`safe_steering_*`, `path_correction`, `applied` steer).

| ID | Scene | Pass criteria | Status |
|---|---|---|---|
| LOOK-01 | Open-yard CHASE, cat near center | Enters `LOOK_AT`; pan tracks; chassis follows path_correction; vision offset does not drive wheels | Pending |
| LOOK-02 | Obstacle one side | Envelope asymmetric (`costmap_sweep`); `BODY_STEER` stays inside band; pan returns to calibrated forward before body vision steer | Pending |
| LOOK-03 | Inject pan off-forward + vision offset | No vision chassis steer until `PAN_RESET` completes (forward deadband) | Pending |
| ENV-01 | Stop/kill `local_costmap/costmap` during CHASE | `path_viable=false`; stop; `envelope_source=none`; no silent `[-1,1]` band | Pending |
| LOOK-04 | Track loss / PRIMARY_CAT_LEFT / BRAKE_REVERSE | Pan commanded to calibrated forward; FSM outcomes unchanged | Pending |
| LOOK-05 | Multi-minute soak | No LOOK_AT↔BODY_STEER chatter under dwell/hysteresis | Pending |

Host evidence (not a board pass): `tests/test_look_drive_mode.py`,
`tests/test_steering_envelope.py`, `tests/test_decision_navigation_fusion.py`.

See also: [`Look_Drive_Path_Design.md`](Look_Drive_Path_Design.md) §10.
