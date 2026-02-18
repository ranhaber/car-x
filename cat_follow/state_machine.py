"""
State machine for cat-follow behavior.
States: IDLE, GOTO_TARGET, SEARCH, APPROACH, TRACK, LOST_SEARCH.
Events drive transitions; no hardware dependency.
"""
from enum import Enum
from typing import Optional, Tuple, Any


class State(Enum):
    IDLE = "idle"
    GOTO_TARGET = "goto_target"
    SEARCH = "search"
    APPROACH = "approach"
    TRACK = "track"
    LOST_SEARCH = "lost_search"


class Event(Enum):
    CAT_LOCATION_RECEIVED = "cat_location_received"
    AT_TARGET = "at_target"
    TIMEOUT = "timeout"
    CAT_FOUND = "cat_found"
    CAT_LOST = "cat_lost"
    DISTANCE_AT_15CM = "distance_at_15cm"
    STOP_COMMAND = "stop_command"
    SEARCH_CYCLE_DONE = "search_cycle_done"  # full circle done, no cat found


# Transition table: (state, event) -> (new_state, payload_to_keep)
_TRANSITIONS = {
    (State.IDLE, Event.CAT_LOCATION_RECEIVED): State.GOTO_TARGET,
    (State.IDLE, Event.STOP_COMMAND): State.IDLE,
    (State.GOTO_TARGET, Event.AT_TARGET): State.SEARCH,
    (State.GOTO_TARGET, Event.CAT_FOUND): State.APPROACH,
    (State.GOTO_TARGET, Event.TIMEOUT): State.SEARCH,
    (State.GOTO_TARGET, Event.STOP_COMMAND): State.IDLE,
    (State.SEARCH, Event.CAT_FOUND): State.APPROACH,
    (State.SEARCH, Event.SEARCH_CYCLE_DONE): State.IDLE,
    (State.SEARCH, Event.STOP_COMMAND): State.IDLE,
    (State.APPROACH, Event.DISTANCE_AT_15CM): State.TRACK,
    (State.APPROACH, Event.CAT_LOST): State.LOST_SEARCH,
    (State.APPROACH, Event.STOP_COMMAND): State.IDLE,
    (State.TRACK, Event.CAT_LOST): State.LOST_SEARCH,
    (State.TRACK, Event.STOP_COMMAND): State.IDLE,
    (State.LOST_SEARCH, Event.CAT_FOUND): State.APPROACH,
    (State.LOST_SEARCH, Event.SEARCH_CYCLE_DONE): State.IDLE,
    (State.LOST_SEARCH, Event.TIMEOUT): State.SEARCH,
    (State.LOST_SEARCH, Event.STOP_COMMAND): State.IDLE,
}


class StateMachine:
    """Single source of truth for cat-follow state. No hardware."""

    def __init__(self):
        self._state = State.IDLE
        self._target_xy: Optional[Tuple[float, float]] = None  # from CAT_LOCATION_RECEIVED
        self._last_bbox: Optional[Tuple[float, float, float, float]] = None  # (x,y,w,h) from CAT_FOUND

    @property
    def state(self) -> State:
        return self._state

    @property
    def target_xy(self) -> Optional[Tuple[float, float]]:
        return self._target_xy

    @property
    def last_bbox(self) -> Optional[Tuple[float, float, float, float]]:
        return self._last_bbox

    def dispatch(self, event: Event, payload: Any = None) -> State:
        """
        Process event; update state and optional payload. Returns new state.
        Unknown (state, event) leaves state unchanged.
        """
        key = (self._state, event)
        new_state = _TRANSITIONS.get(key)
        if new_state is not None:
            self._state = new_state
            if event == Event.CAT_LOCATION_RECEIVED and payload is not None:
                self._target_xy = tuple(payload[:2])
            if event == Event.CAT_FOUND and payload is not None:
                self._last_bbox = tuple(payload[:4]) if len(payload) >= 4 else None
        return self._state

    def reset_to_idle(self) -> None:
        """Force IDLE and clear target/bbox."""
        self._state = State.IDLE
        self._target_xy = None
        self._last_bbox = None
