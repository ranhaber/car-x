"""
Unit tests for cat_follow.motion.search — search arc steer/speed.

Run:
    python -m pytest tests/test_search.py -v
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.motion.search import (
    compute_search_tick,
    compute_full_circle_tick,
    ARC_DURATION_SEC,
    SEARCH_SPEED,
)


class MockCalib:
    def get_max_steer_angle_deg(self):
        return 25.0


def test_speed_constant():
    steer, speed = compute_search_tick(0.0, None)
    assert speed == SEARCH_SPEED
    assert speed == 20


def test_arc_duration_constant():
    assert ARC_DURATION_SEC == 2.0


def test_no_calib_uses_30_deg():
    steer, speed = compute_search_tick(0.0, None)
    assert steer == 30.0
    steer2, _ = compute_search_tick(2.5, None)
    assert steer2 == -30.0


def test_calib_clamps_steer():
    calib = MockCalib()
    steer, speed = compute_search_tick(0.0, calib)
    assert steer == 25.0
    steer2, _ = compute_search_tick(2.5, calib)
    assert steer2 == -25.0


def test_first_arc_left():
    """0 to 2s: steer left (positive)."""
    steer0, _ = compute_search_tick(0.0, None)
    steer1, _ = compute_search_tick(0.5, None)
    steer2, _ = compute_search_tick(1.99, None)
    assert steer0 == 30.0 and steer1 == 30.0 and steer2 == 30.0


def test_second_arc_right():
    """2 to 4s: steer right (negative)."""
    steer2, _ = compute_search_tick(2.0, None)
    steer3, _ = compute_search_tick(3.0, None)
    steer4, _ = compute_search_tick(3.99, None)
    assert steer2 == -30.0 and steer3 == -30.0 and steer4 == -30.0


def test_third_arc_left_again():
    """4 to 6s: left again."""
    steer4, _ = compute_search_tick(4.0, None)
    steer5, _ = compute_search_tick(5.0, None)
    assert steer4 == 30.0 and steer5 == 30.0


def test_returns_tuple_steer_speed():
    out = compute_search_tick(1.0, None)
    assert isinstance(out, tuple)
    assert len(out) == 2
    assert isinstance(out[0], (int, float))
    assert isinstance(out[1], (int, float))


def test_full_circle_steer_left():
    """Full circle search: max left steer, constant speed."""
    steer, speed = compute_full_circle_tick(None)
    assert steer == 30.0
    assert speed == SEARCH_SPEED


def test_full_circle_respects_calib():
    calib = MockCalib()
    steer, speed = compute_full_circle_tick(calib)
    assert steer == 25.0
    assert speed == SEARCH_SPEED


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"  PASS  {fn.__name__}")
    print(f"\nAll {len(tests)} tests passed.")
