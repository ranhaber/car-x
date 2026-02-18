"""
Unit tests for cat_follow.motion.goto_xy — proportional heading controller.

Run:
    python -m pytest tests/test_goto_xy.py -v
or:
    python tests/test_goto_xy.py
"""

import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.motion.goto_xy import (
    compute_bearing_deg,
    normalize_angle,
    compute_heading_error,
    compute_distance,
    compute_goto,
    ARRIVAL_THRESHOLD_CM,
    KP,
    CRUISE_SPEED,
    SLOW_SPEED,
    SLOW_ERROR_DEG,
)


# ── bearing ──────────────────────────────────────────────────────────────

def test_bearing_east():
    assert abs(compute_bearing_deg(0, 0, 100, 0) - 0.0) < 0.01

def test_bearing_north():
    assert abs(compute_bearing_deg(0, 0, 0, 100) - 90.0) < 0.01

def test_bearing_west():
    b = compute_bearing_deg(0, 0, -100, 0)
    assert abs(b - 180.0) < 0.01 or abs(b - (-180.0)) < 0.01

def test_bearing_south():
    assert abs(compute_bearing_deg(0, 0, 0, -100) - (-90.0)) < 0.01

def test_bearing_northeast():
    b = compute_bearing_deg(0, 0, 100, 100)
    assert abs(b - 45.0) < 0.01

def test_bearing_from_nonzero():
    b = compute_bearing_deg(50, 50, 150, 50)
    assert abs(b - 0.0) < 0.01


# ── normalize_angle ──────────────────────────────────────────────────────

def test_normalize_zero():
    assert normalize_angle(0) == 0

def test_normalize_positive():
    assert abs(normalize_angle(270) - (-90)) < 0.01

def test_normalize_negative():
    assert abs(normalize_angle(-270) - 90) < 0.01

def test_normalize_360():
    assert abs(normalize_angle(360)) < 0.01

def test_normalize_minus_180():
    assert abs(normalize_angle(-180) - (-180)) < 0.01


# ── heading_error ────────────────────────────────────────────────────────

def test_heading_error_zero():
    assert abs(compute_heading_error(45, 45)) < 0.01

def test_heading_error_left():
    """Target is 30 deg to the left -> positive error."""
    err = compute_heading_error(30, 0)
    assert abs(err - 30) < 0.01

def test_heading_error_right():
    """Target is 30 deg to the right -> negative error."""
    err = compute_heading_error(-30, 0)
    assert abs(err - (-30)) < 0.01

def test_heading_error_wraparound():
    """Heading 170, target at -170 -> shortest turn is 20 deg (not 340)."""
    err = compute_heading_error(-170, 170)
    assert abs(err - 20) < 0.01

def test_heading_error_wraparound_other():
    """Heading -170, target at 170 -> shortest turn is -20 deg."""
    err = compute_heading_error(170, -170)
    assert abs(err - (-20)) < 0.01


# ── distance ─────────────────────────────────────────────────────────────

def test_distance_zero():
    assert compute_distance(10, 20, 10, 20) == 0

def test_distance_horizontal():
    assert abs(compute_distance(0, 0, 100, 0) - 100) < 0.01

def test_distance_diagonal():
    d = compute_distance(0, 0, 100, 100)
    assert abs(d - math.sqrt(20000)) < 0.1


# ── compute_goto ─────────────────────────────────────────────────────────

def test_arrived_when_close():
    steer, speed, arrived = compute_goto(0, 0, 0, 5, 0)
    assert arrived is True
    assert speed == 0

def test_not_arrived_when_far():
    steer, speed, arrived = compute_goto(0, 0, 0, 200, 0)
    assert arrived is False
    assert speed > 0

def test_steer_zero_when_facing_target():
    """Car at origin, heading 0 (east), target at (200, 0) -> steer ~0."""
    steer, speed, arrived = compute_goto(0, 0, 0, 200, 0)
    assert abs(steer) < 1.0
    assert speed == CRUISE_SPEED

def test_steer_left_when_target_is_left():
    """Car heading 0, target at (100, 100) -> bearing 45 -> steer positive (left)."""
    steer, speed, arrived = compute_goto(0, 0, 0, 100, 100)
    assert steer > 0, f"Expected positive steer (left), got {steer}"

def test_steer_right_when_target_is_right():
    """Car heading 0, target at (100, -100) -> bearing -45 -> steer negative (right)."""
    steer, speed, arrived = compute_goto(0, 0, 0, 100, -100)
    assert steer < 0, f"Expected negative steer (right), got {steer}"

def test_steer_clamped():
    """Car heading 0, target directly to the left (0, 200) -> bearing 90 -> steer clamped.
    Without calibration, default clamp is +/-30 (from limits.py fallback)."""
    steer, speed, arrived = compute_goto(0, 0, 0, 0, 200)
    assert abs(steer) <= 30.1  # default clamp without calibration is +/-30

def test_slow_on_big_error():
    """Car heading 0, target behind (bearing ~180) -> big error -> slow speed."""
    steer, speed, arrived = compute_goto(0, 0, 0, -200, 0)
    assert speed == SLOW_SPEED, f"Expected SLOW_SPEED={SLOW_SPEED}, got {speed}"

def test_cruise_on_small_error():
    """Car heading 0, target ahead -> small error -> cruise speed."""
    steer, speed, arrived = compute_goto(0, 0, 0, 200, 10)
    assert speed == CRUISE_SPEED

def test_arrival_threshold():
    """Just outside threshold -> not arrived; just inside -> arrived."""
    steer, speed, arrived = compute_goto(0, 0, 0, ARRIVAL_THRESHOLD_CM + 1, 0)
    assert arrived is False

    steer, speed, arrived = compute_goto(0, 0, 0, ARRIVAL_THRESHOLD_CM - 1, 0)
    assert arrived is True


# ── simulation: drive straight to target ─────────────────────────────────

def test_simulation_straight():
    """Simulate driving straight east to (200, 0). Should arrive."""
    from cat_follow import odometry

    odometry.reset(0, 0, 0)
    tx, ty = 200.0, 0.0
    dt = 1.0 / 30.0
    arrived = False

    for tick in range(300):  # 10 seconds max
        x, y = odometry.get_position()
        h = odometry.get_heading_deg()
        steer, speed, arrived = compute_goto(x, y, h, tx, ty)
        if arrived:
            break
        cm_per_sec = speed * 0.5  # rough
        odometry.update(dt, speed, steer, cm_per_sec)

    assert arrived, f"Did not arrive after 300 ticks. Pos=({x:.1f}, {y:.1f})"


def test_simulation_diagonal():
    """Simulate driving to (150, 150). Should arrive."""
    from cat_follow import odometry

    odometry.reset(0, 0, 0)
    tx, ty = 150.0, 150.0
    dt = 1.0 / 30.0
    arrived = False

    for tick in range(600):  # 20 seconds max
        x, y = odometry.get_position()
        h = odometry.get_heading_deg()
        steer, speed, arrived = compute_goto(x, y, h, tx, ty)
        if arrived:
            break
        cm_per_sec = speed * 0.5
        odometry.update(dt, speed, steer, cm_per_sec)

    assert arrived, f"Did not arrive after 600 ticks. Pos=({x:.1f}, {y:.1f})"


def test_simulation_behind():
    """Target is behind the car (bearing ~180). Car should turn and reach it."""
    from cat_follow import odometry

    odometry.reset(0, 0, 0)
    tx, ty = -100.0, 0.0
    dt = 1.0 / 30.0
    arrived = False

    for tick in range(900):  # 30 seconds max (needs to turn around)
        x, y = odometry.get_position()
        h = odometry.get_heading_deg()
        steer, speed, arrived = compute_goto(x, y, h, tx, ty)
        if arrived:
            break
        cm_per_sec = speed * 0.5
        odometry.update(dt, speed, steer, cm_per_sec)

    assert arrived, f"Did not arrive after 900 ticks. Pos=({x:.1f}, {y:.1f})"


# ── run as script ────────────────────────────────────────────────────────

if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"  PASS  {fn.__name__}")
    print(f"\nAll {len(tests)} tests passed.")
