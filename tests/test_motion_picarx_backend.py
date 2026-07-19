"""Tests for the PiCar-X motor backend.

A ``FakePicarx`` records every SDK call so the tests stay hardware-free.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.motion.motor_interface import MotorInterface  # noqa: E402
from cat_follow.motion.picarx_backend import (  # noqa: E402
    DEFAULT_MAX_SPEED_PCT,
    DEFAULT_MAX_STEER_DEG,
    PiCarXBackend,
)
from cat_follow.control.types import (  # noqa: E402
    DecisionOutput,
    FsmState,
    ReasonCode,
    TargetSource,
)


class FakePicarx:
    """Records every SDK call.  Mirrors the subset of methods PiCarXBackend uses."""

    def __init__(self) -> None:
        self.calls: list = []

    def forward(self, speed: int) -> None:
        self.calls.append(("forward", int(speed)))

    def backward(self, speed: int) -> None:
        self.calls.append(("backward", int(speed)))

    def stop(self) -> None:
        self.calls.append(("stop",))

    def set_dir_servo_angle(self, angle_deg: float) -> None:
        self.calls.append(("set_dir_servo_angle", float(angle_deg)))


def _decision(*, speed=0.0, steering=0.0, brake=False):
    return DecisionOutput(
        timestamp_ms=0,
        requested_state=FsmState.CHASE_A,
        speed=speed,
        steering=steering,
        brake=brake,
        reason=ReasonCode.GLOBAL_CHASE,
        active_constraints=(),
        target_x=None,
        target_y=None,
        target_source=TargetSource.NONE,
        rejected_transition=False,
    )


# ── construction ────────────────────────────────────────────────────


def test_constructor_rejects_invalid_calibration():
    px = FakePicarx()
    with pytest.raises(ValueError):
        PiCarXBackend(px, max_steer_deg=0)
    with pytest.raises(ValueError):
        PiCarXBackend(px, max_speed_pct=0)


# ── direction ──────────────────────────────────────────────────────


def test_positive_speed_drives_forward():
    px = FakePicarx()
    backend = PiCarXBackend(px, max_speed_pct=100)
    backend.apply(speed=0.5, steering=0.0, brake=False)
    assert ("forward", 50) in px.calls


def test_negative_speed_drives_backward():
    px = FakePicarx()
    backend = PiCarXBackend(px, max_speed_pct=100)
    backend.apply(speed=-0.4, steering=0.0, brake=False)
    assert ("backward", 40) in px.calls


def test_zero_speed_calls_stop():
    px = FakePicarx()
    backend = PiCarXBackend(px)
    backend.apply(speed=0.0, steering=0.0, brake=False)
    assert ("stop",) in px.calls


def test_brake_overrides_forward_speed():
    px = FakePicarx()
    backend = PiCarXBackend(px)
    backend.apply(speed=0.9, steering=0.0, brake=True)
    drive_calls = [c for c in px.calls if c[0] in ("forward", "backward", "stop")]
    assert drive_calls == [("stop",)]


# ── steering ───────────────────────────────────────────────────────


def test_steering_scales_to_max_steer_deg():
    px = FakePicarx()
    backend = PiCarXBackend(px, max_steer_deg=25.0)
    backend.apply(speed=0.0, steering=1.0, brake=False)
    assert ("set_dir_servo_angle", 25.0) in px.calls

    backend = PiCarXBackend(FakePicarx(), max_steer_deg=20.0)
    backend.apply(speed=0.0, steering=-0.5, brake=False)
    # The new backend instance has its own px reference; check state via
    # the backend's last_steering attribute.
    assert backend._last_steering == -10.0  # type: ignore[attr-defined]


def test_steering_clamped_outside_normalized_range():
    px = FakePicarx()
    backend = PiCarXBackend(px, max_steer_deg=25.0)
    backend.apply(speed=0.0, steering=2.0, brake=False)
    assert ("set_dir_servo_angle", 25.0) in px.calls
    backend.apply(speed=0.0, steering=-3.0, brake=False)
    assert ("set_dir_servo_angle", -25.0) in px.calls


def test_repeated_same_steering_only_writes_once():
    px = FakePicarx()
    backend = PiCarXBackend(px)
    backend.apply(speed=0.5, steering=0.3, brake=False)
    backend.apply(speed=0.5, steering=0.3, brake=False)
    backend.apply(speed=0.5, steering=0.3, brake=False)
    angle_calls = [c for c in px.calls if c[0] == "set_dir_servo_angle"]
    assert len(angle_calls) == 1


# ── emergency stop ─────────────────────────────────────────────────


def test_emergency_stop_stops_drivetrain_and_centers_wheels():
    px = FakePicarx()
    backend = PiCarXBackend(px)
    backend.apply(speed=0.5, steering=0.4, brake=False)
    backend.emergency_stop()
    # The last-three calls should include a stop and a centered steering.
    assert ("stop",) in px.calls
    assert ("set_dir_servo_angle", 0.0) in px.calls
    assert backend._last_steering == 0.0  # type: ignore[attr-defined]
    assert backend._last_drive == "stop"  # type: ignore[attr-defined]


# ── integration with MotorInterface ────────────────────────────────


def test_motor_interface_with_picarx_backend_round_trip():
    px = FakePicarx()
    backend = PiCarXBackend(px, max_steer_deg=DEFAULT_MAX_STEER_DEG)
    iface = MotorInterface(backend=backend)

    iface.apply(_decision(speed=0.6, steering=-0.5))
    iface.apply(_decision(speed=0.0, steering=0.0, brake=True))

    forwards = [c for c in px.calls if c[0] == "forward"]
    stops = [c for c in px.calls if c[0] == "stop"]
    assert forwards == [("forward", int(0.6 * DEFAULT_MAX_SPEED_PCT))]
    assert stops, "MotorInterface should have routed brake -> backend.stop()"
    angle_calls = [c for c in px.calls if c[0] == "set_dir_servo_angle"]
    # One on the first apply (-0.5 -> -12.5 deg), one on brake (steering 0 -> 0 deg).
    assert angle_calls == [
        ("set_dir_servo_angle", -0.5 * DEFAULT_MAX_STEER_DEG),
        ("set_dir_servo_angle", 0.0),
    ]
