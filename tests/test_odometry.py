"""
Unit tests for cat_follow.odometry — bicycle-model dead reckoning.

Run:
    python -m pytest tests/test_odometry.py -v
or:
    python tests/test_odometry.py
"""

import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow import odometry


def _reset():
    odometry.reset(0, 0, 0)


# ── straight line ────────────────────────────────────────────────────────

def test_straight_forward():
    """Drive straight at 10 cm/s for 1 s -> x ~ 10 cm, y ~ 0."""
    _reset()
    odometry.update(dt_sec=1.0, speed=50, steer_deg=0.0, cm_per_sec=10.0)
    x, y = odometry.get_position()
    assert abs(x - 10.0) < 0.01, f"x={x}"
    assert abs(y) < 0.01, f"y={y}"
    assert abs(odometry.get_heading_deg()) < 0.01


def test_straight_backward():
    """Drive backward at 10 cm/s for 1 s -> x ~ -10."""
    _reset()
    odometry.update(dt_sec=1.0, speed=-50, steer_deg=0.0, cm_per_sec=10.0)
    x, y = odometry.get_position()
    assert abs(x - (-10.0)) < 0.01, f"x={x}"
    assert abs(y) < 0.01, f"y={y}"


def test_straight_multiple_steps():
    """10 steps of 0.1 s at 20 cm/s -> x ~ 20 cm."""
    _reset()
    for _ in range(10):
        odometry.update(dt_sec=0.1, speed=50, steer_deg=0.0, cm_per_sec=20.0)
    x, y = odometry.get_position()
    assert abs(x - 20.0) < 0.1, f"x={x}"
    assert abs(y) < 0.1, f"y={y}"


# ── turning ──────────────────────────────────────────────────────────────

def test_turn_left_heading_increases():
    """Steering left (positive) should increase heading (CCW)."""
    _reset()
    odometry.update(dt_sec=1.0, speed=50, steer_deg=15.0, cm_per_sec=10.0)
    h = odometry.get_heading_deg()
    assert h > 0, f"heading={h}, expected positive (left turn)"


def test_turn_right_heading_decreases():
    """Steering right (negative) should decrease heading (CW)."""
    _reset()
    odometry.update(dt_sec=1.0, speed=50, steer_deg=-15.0, cm_per_sec=10.0)
    h = odometry.get_heading_deg()
    assert h < 0, f"heading={h}, expected negative (right turn)"


def test_turn_symmetric():
    """Left and right turns at same angle/speed should be symmetric."""
    _reset()
    odometry.update(dt_sec=1.0, speed=50, steer_deg=20.0, cm_per_sec=10.0)
    x_left, y_left = odometry.get_position()
    h_left = odometry.get_heading_deg()

    _reset()
    odometry.update(dt_sec=1.0, speed=50, steer_deg=-20.0, cm_per_sec=10.0)
    x_right, y_right = odometry.get_position()
    h_right = odometry.get_heading_deg()

    assert abs(x_left - x_right) < 0.01, f"x_left={x_left}, x_right={x_right}"
    assert abs(y_left + y_right) < 0.01, f"y_left={y_left}, y_right={y_right}"
    assert abs(h_left + h_right) < 0.01, f"h_left={h_left}, h_right={h_right}"


# ── heading normalization ────────────────────────────────────────────────

def test_heading_wraps():
    """After many left turns, heading should stay in [-180, 180)."""
    _reset()
    for _ in range(200):
        odometry.update(dt_sec=0.1, speed=50, steer_deg=25.0, cm_per_sec=20.0)
    h = odometry.get_heading_deg()
    assert -180 <= h < 180, f"heading={h}, not in [-180, 180)"


# ── zero speed or zero dt ────────────────────────────────────────────────

def test_zero_speed_no_movement():
    _reset()
    odometry.update(dt_sec=1.0, speed=0, steer_deg=15.0, cm_per_sec=0.0)
    x, y = odometry.get_position()
    assert x == 0 and y == 0


def test_zero_dt_no_movement():
    _reset()
    odometry.update(dt_sec=0.0, speed=50, steer_deg=15.0, cm_per_sec=10.0)
    x, y = odometry.get_position()
    assert x == 0 and y == 0


# ── reset ────────────────────────────────────────────────────────────────

def test_reset_to_custom():
    odometry.reset(100, 200, 45)
    x, y = odometry.get_position()
    assert x == 100 and y == 200
    assert odometry.get_heading_deg() == 45


def test_reset_clears_previous():
    _reset()
    odometry.update(dt_sec=1.0, speed=50, steer_deg=10.0, cm_per_sec=10.0)
    _reset()
    x, y = odometry.get_position()
    assert x == 0 and y == 0 and odometry.get_heading_deg() == 0


# ── fallback (no cm_per_sec) ────────────────────────────────────────────

def test_fallback_velocity():
    """Without cm_per_sec, uses speed * 0.5 as rough cm/s."""
    _reset()
    odometry.update(dt_sec=1.0, speed=20, steer_deg=0.0)
    x, y = odometry.get_position()
    expected = 20 * 0.5  # 10 cm
    assert abs(x - expected) < 0.01, f"x={x}, expected ~{expected}"


# ── arc geometry ─────────────────────────────────────────────────────────

def test_full_circle_returns_near_origin():
    """Driving in a circle should bring the car roughly back to the start."""
    _reset()
    steer = 20.0
    v = 15.0
    steer_rad = math.radians(steer)
    turn_radius = odometry.WHEELBASE_CM / math.tan(steer_rad)
    circumference = 2 * math.pi * abs(turn_radius)
    total_time = circumference / v  # seconds to complete circle

    steps = 500
    dt = total_time / steps
    for _ in range(steps):
        odometry.update(dt_sec=dt, speed=50, steer_deg=steer, cm_per_sec=v)

    x, y = odometry.get_position()
    dist_from_origin = math.sqrt(x**2 + y**2)
    assert dist_from_origin < 2.0, (
        f"After full circle: ({x:.2f}, {y:.2f}), dist={dist_from_origin:.2f}"
    )

    h = odometry.get_heading_deg()
    assert abs(h) < 5.0 or abs(abs(h) - 360) < 5.0, (
        f"Heading after full circle: {h:.2f}, expected near 0"
    )


# ── run as script ────────────────────────────────────────────────────────

if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"  PASS  {fn.__name__}")
    print(f"\nAll {len(tests)} tests passed.")
