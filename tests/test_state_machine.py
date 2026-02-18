"""
Test state machine transitions. Run from car-x: python -m pytest tests/test_state_machine.py -v
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.state_machine import StateMachine, State, Event


def test_idle_to_goto_on_cat_location():
    sm = StateMachine()
    assert sm.state == State.IDLE
    sm.dispatch(Event.CAT_LOCATION_RECEIVED, (100, 50))
    assert sm.state == State.GOTO_TARGET
    assert sm.target_xy == (100, 50)


def test_goto_to_search_on_at_target():
    sm = StateMachine()
    sm.dispatch(Event.CAT_LOCATION_RECEIVED, (10, 10))
    assert sm.state == State.GOTO_TARGET
    sm.dispatch(Event.AT_TARGET)
    assert sm.state == State.SEARCH


def test_search_to_approach_on_cat_found():
    sm = StateMachine()
    sm.dispatch(Event.CAT_LOCATION_RECEIVED, (0, 0))
    sm.dispatch(Event.AT_TARGET)
    sm.dispatch(Event.CAT_FOUND, (100, 100, 80, 120))
    assert sm.state == State.APPROACH
    assert sm.last_bbox == (100, 100, 80, 120)


def test_approach_to_track_on_distance_15cm():
    sm = StateMachine()
    sm.dispatch(Event.CAT_LOCATION_RECEIVED, (0, 0))
    sm.dispatch(Event.AT_TARGET)
    sm.dispatch(Event.CAT_FOUND, (0, 0, 50, 50))
    sm.dispatch(Event.DISTANCE_AT_15CM)
    assert sm.state == State.TRACK


def test_goto_to_approach_on_cat_found():
    """Cat found while driving to target -> go straight to APPROACH."""
    sm = StateMachine()
    sm.dispatch(Event.CAT_LOCATION_RECEIVED, (1.0, 0.5))
    assert sm.state == State.GOTO_TARGET
    sm.dispatch(Event.CAT_FOUND, (200, 150, 60, 80))
    assert sm.state == State.APPROACH
    assert sm.last_bbox == (200, 150, 60, 80)


def test_search_cycle_done_goes_to_idle():
    """Full circle search with no cat -> stop (IDLE)."""
    sm = StateMachine()
    sm.dispatch(Event.CAT_LOCATION_RECEIVED, (0, 0))
    sm.dispatch(Event.AT_TARGET)
    assert sm.state == State.SEARCH
    sm.dispatch(Event.SEARCH_CYCLE_DONE)
    assert sm.state == State.IDLE


def test_lost_search_cycle_done_goes_to_idle():
    sm = StateMachine()
    sm.dispatch(Event.CAT_LOCATION_RECEIVED, (0, 0))
    sm.dispatch(Event.AT_TARGET)
    sm.dispatch(Event.CAT_FOUND, (0, 0, 50, 50))
    sm.dispatch(Event.CAT_LOST)
    assert sm.state == State.LOST_SEARCH
    sm.dispatch(Event.SEARCH_CYCLE_DONE)
    assert sm.state == State.IDLE


def test_stop_from_any_state_goes_to_idle():
    sm = StateMachine()
    sm.dispatch(Event.CAT_LOCATION_RECEIVED, (0, 0))
    sm.dispatch(Event.STOP_COMMAND)
    assert sm.state == State.IDLE

    sm.dispatch(Event.CAT_LOCATION_RECEIVED, (0, 0))
    sm.dispatch(Event.AT_TARGET)
    sm.dispatch(Event.STOP_COMMAND)
    assert sm.state == State.IDLE
