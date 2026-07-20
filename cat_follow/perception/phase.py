"""Perception-internal phase machine that gates AI inference.

This is intentionally distinct from the *control* FSM in
:mod:`cat_follow.control.fsm` (which owns robot behaviour and motor safety).
This machine only decides *when the detector should actually invoke the
model*, mirroring the cat_ball_tracker phase design:

- ``IDLE``: no motion; the model stays unloaded and is never invoked.
- ``ACQUISITION``: motion seen but no confirmed target; run the detector
  every frame to lock on quickly.
- ``TRACKING``: a target was detected; run the detector at a reduced cadence
  (the tracker carries the bbox between detections).
- ``WATCH``: the target was lost but motion recently stopped; keep sampling at
  the reduced cadence before falling back to IDLE.

Timestamps are injected so the machine is deterministic and unit-testable.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional


class Phase(str, Enum):
    IDLE = "IDLE"
    ACQUISITION = "ACQUISITION"
    TRACKING = "TRACKING"
    WATCH = "WATCH"


class PhaseMachine:
    """Motion/detection-driven gate for the detector cadence."""

    def __init__(
        self,
        *,
        acquisition_timeout_s: float = 10.0,
        detection_timeout_s: float = 30.0,
        tracking_interval: int = 2,
        watch_interval: int = 2,
    ) -> None:
        self._acquisition_timeout = float(acquisition_timeout_s)
        self._detection_timeout = float(detection_timeout_s)
        self._tracking_interval = max(1, int(tracking_interval))
        self._watch_interval = max(1, int(watch_interval))

        self._phase = Phase.IDLE
        self._last_motion_s: Optional[float] = None
        self._last_detection_s: Optional[float] = None

    @property
    def phase(self) -> Phase:
        return self._phase

    def update(self, *, now_s: float, motion: bool, detected: bool) -> Phase:
        """Advance the machine and return the new phase.

        Parameters
        ----------
        now_s:
            Monotonic time in seconds.
        motion:
            Whether the cheap motion detector fired this tick.
        detected:
            Whether the detector produced a valid target this tick.
        """
        if motion:
            self._last_motion_s = now_s
        if detected:
            self._last_detection_s = now_s

        if self._phase is Phase.IDLE:
            if motion:
                self._phase = Phase.ACQUISITION
        elif self._phase is Phase.ACQUISITION:
            if detected:
                self._phase = Phase.TRACKING
            elif self._idle_for(self._last_motion_s, now_s, self._acquisition_timeout):
                self._phase = Phase.IDLE
        elif self._phase is Phase.TRACKING:
            if self._idle_for(self._last_detection_s, now_s, self._detection_timeout):
                self._phase = Phase.IDLE
            elif not motion and not detected:
                self._phase = Phase.WATCH
        elif self._phase is Phase.WATCH:
            if detected:
                self._phase = Phase.TRACKING
            elif self._idle_for(self._last_detection_s, now_s, self._detection_timeout):
                self._phase = Phase.IDLE

        return self._phase

    def should_detect(self, frame_index: int) -> bool:
        """Return True when the detector should invoke the model this frame."""
        if self._phase is Phase.IDLE:
            return False
        if self._phase is Phase.ACQUISITION:
            return True
        interval = (
            self._tracking_interval
            if self._phase is Phase.TRACKING
            else self._watch_interval
        )
        return frame_index % interval == 0

    @property
    def is_active(self) -> bool:
        """True whenever the pipeline is doing real work (not IDLE)."""
        return self._phase is not Phase.IDLE

    @staticmethod
    def _idle_for(
        last_event_s: Optional[float], now_s: float, timeout_s: float
    ) -> bool:
        if last_event_s is None:
            return True
        return (now_s - last_event_s) >= timeout_s


__all__ = ["Phase", "PhaseMachine"]
