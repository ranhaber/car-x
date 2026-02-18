"""
Tests for cat_follow.location — modular car location (pluggable providers).

Run: python tests/test_location.py
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.location import (
    get_position,
    get_heading_deg,
    update,
    reset,
    set_provider,
    OdometryProvider,
)


def test_odometry_provider_default():
    """Default provider is odometry; reset and read position/heading."""
    reset(0, 0, 0)
    x, y = get_position()
    assert x == 0 and y == 0
    assert get_heading_deg() == 0.0


def test_odometry_provider_reset():
    reset(100, 200, 45.0)
    x, y = get_position()
    assert x == 100 and y == 200
    assert abs(get_heading_deg() - 45.0) < 0.01


def test_odometry_provider_update():
    reset(0, 0, 0)
    # One tick: forward at 22 cm/s for 1 sec -> (22, 0), heading 0
    update(1.0, 50, 0, 22.0)
    x, y = get_position()
    assert abs(x - 22.0) < 1.0 and abs(y) < 0.5
    assert abs(get_heading_deg()) < 1.0


def test_set_provider():
    """Setting provider to a new OdometryProvider keeps interface."""
    set_provider(OdometryProvider())
    reset(10, 20, 90.0)
    x, y = get_position()
    assert x == 10 and y == 20
    assert abs(get_heading_deg() - 90.0) < 0.01


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"  PASS  {fn.__name__}")
    print(f"\nAll {len(tests)} tests passed.")
