"""Tests for cat_follow.range_sensor."""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cat_follow.range_sensor as range_sensor


def setup_function():
    with range_sensor._lock:
        range_sensor._car = None
        range_sensor._reader = None
        range_sensor._last_distance_cm = None
        range_sensor._last_read_time = 0.0


def test_set_reader_returns_cached_distance_without_hardware():
    readings = iter([42.0, 30.0])

    def read():
        return next(readings, 30.0)

    range_sensor.set_reader(read)
    assert range_sensor.get_distance_cm() == 42.0
    assert range_sensor.get_last_distance_cm() == 42.0
    # Throttled: second call within 60 ms returns cache without re-reading.
    assert range_sensor.get_distance_cm() == 42.0


def test_set_reader_clears_legacy_car_backend():
    class _Car:
        def get_distance(self):
            return 99.0

    range_sensor.set_car(_Car())
    range_sensor.set_reader(lambda: 12.0)
    assert range_sensor.get_distance_cm() == 12.0


def test_set_reader_invalid_type_raises():
    try:
        range_sensor.set_reader("not-callable")  # type: ignore[arg-type]
        raised = False
    except TypeError:
        raised = True
    assert raised


def test_set_reader_allows_eventual_refresh_after_throttle(monkeypatch):
    now = 100.0
    monkeypatch.setattr(range_sensor.time, "monotonic", lambda: now)

    readings = iter([40.0, 25.0])
    range_sensor.set_reader(lambda: next(readings, 25.0))
    assert range_sensor.get_distance_cm() == 40.0

    now = 100.061
    assert range_sensor.get_distance_cm() == 25.0
