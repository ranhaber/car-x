"""Tests for RangeAdapter."""

import os
import sys
import time
from typing import List, Optional

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.control.decision_engine import OBSTACLE_TOO_CLOSE_CM  # noqa: E402
from cat_follow.control.types import (  # noqa: E402
    RangeBackend,
    TelemetryEventType,
)
from cat_follow.perception.range_adapter import (  # noqa: E402
    DEFAULT_OBSTACLE_DETECTED_CM,
    RangeAdapter,
)
from cat_follow.runtime.shared_state import SharedState  # noqa: E402
from cat_follow.telemetry.async_logger import AsyncLogger, CallableSink  # noqa: E402


class _FakeRange:
    """Replays a configured sequence of distance readings."""

    def __init__(self, readings: List[Optional[float]]) -> None:
        self._readings = list(readings)
        self._idx = 0

    def __call__(self) -> Optional[float]:
        if not self._readings:
            return None
        if self._idx >= len(self._readings):
            return self._readings[-1]
        value = self._readings[self._idx]
        self._idx += 1
        return value


def _make_adapter(readings, **kwargs):
    contract = SharedState()
    fake = _FakeRange(readings)
    adapter = RangeAdapter(
        contract_shared_state=contract,
        read_distance=fake,
        **kwargs,
    )
    return adapter, contract, fake


# ── construction ────────────────────────────────────────────────────


def test_constructor_validates_arguments():
    contract = SharedState()
    with pytest.raises(ValueError):
        RangeAdapter(
            contract,
            lambda: None,
            obstacle_detected_cm=10,
            obstacle_critical_cm=10,
        )
    with pytest.raises(ValueError):
        RangeAdapter(
            contract,
            lambda: None,
            obstacle_detected_cm=50,
            obstacle_critical_cm=0,
        )
    with pytest.raises(ValueError):
        RangeAdapter(contract, lambda: None, poll_rate_hz=0)


# ── valid distance ─────────────────────────────────────────────────


def test_distance_above_detected_threshold_clears_obstacle():
    adapter, contract, _ = _make_adapter([100.0])
    state = adapter.update()
    assert state.distance_cm == 100.0
    assert state.confidence == 1.0
    assert state.obstacle_detected is False
    assert state.obstacle_critical is False
    assert state.obstacle_severity == 0.0
    assert contract.get_range().obstacle_detected is False


def test_distance_below_detected_threshold_marks_obstacle():
    adapter, _, _ = _make_adapter([30.0])
    state = adapter.update()
    assert state.obstacle_detected is True
    assert state.obstacle_critical is False
    assert 0.0 < state.obstacle_severity < 1.0


def test_distance_at_critical_threshold_is_critical():
    adapter, _, _ = _make_adapter([OBSTACLE_TOO_CLOSE_CM - 0.5])
    state = adapter.update()
    assert state.obstacle_detected is True
    assert state.obstacle_critical is True
    assert state.obstacle_severity == 1.0


def test_severity_ramps_linearly():
    detected = 50.0
    critical = 10.0
    adapter, _, _ = _make_adapter(
        [50.0, 30.0, 10.0],
        obstacle_detected_cm=detected,
        obstacle_critical_cm=critical,
    )
    s_top = adapter.update()
    s_mid = adapter.update()
    s_bot = adapter.update()
    assert s_top.obstacle_severity == 0.0
    # 30 cm halfway between 50 and 10 -> 0.5
    assert s_mid.obstacle_severity == pytest.approx(0.5)
    assert s_bot.obstacle_severity == 1.0


# ── invalid distance ──────────────────────────────────────────────


def test_none_distance_yields_zero_confidence_state():
    adapter, contract, _ = _make_adapter([None])
    state = adapter.update()
    assert state.distance_cm is None
    assert state.confidence == 0.0
    assert state.obstacle_detected is False
    assert state.obstacle_critical is False
    assert state.obstacle_severity == 0.0
    assert contract.get_range().confidence == 0.0


def test_read_exception_does_not_crash_update():
    contract = SharedState()

    def boom():
        raise RuntimeError("sensor exploded")

    adapter = RangeAdapter(contract, boom)
    state = adapter.update()
    assert state.confidence == 0.0
    assert state.distance_cm is None


# ── shared state authority ─────────────────────────────────────────


def test_publishes_with_authority_and_backend_metadata():
    adapter, contract, _ = _make_adapter(
        [42.0],
        backend=RangeBackend.LIDAR_C1,
    )
    state = adapter.update()
    assert state.authority == "RangeAdapter"
    assert state.backend == RangeBackend.LIDAR_C1
    assert contract.get_range().backend == RangeBackend.LIDAR_C1


# ── telemetry ──────────────────────────────────────────────────────


def test_update_emits_range_update_telemetry():
    captured = []
    logger = AsyncLogger(
        sink=CallableSink(captured.append),
        max_queue=32,
        flush_interval_s=0.05,
    )
    logger.start()
    adapter, _, _ = _make_adapter([42.0], logger=logger)
    try:
        adapter.update()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not captured:
            time.sleep(0.02)
    finally:
        logger.stop()

    assert any(
        e["event_type"] == TelemetryEventType.RANGE_UPDATE.value
        for e in captured
    )


# ── polling thread ─────────────────────────────────────────────────


def test_polling_thread_publishes_updates_and_stops_cleanly():
    adapter, contract, _ = _make_adapter(
        [42.0, 30.0, 10.0],
        poll_rate_hz=200.0,
    )
    adapter.start()
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if contract.get_range().distance_cm is not None:
                break
            time.sleep(0.01)
        assert contract.get_range().distance_cm is not None
    finally:
        adapter.stop()


# ── default thresholds ────────────────────────────────────────────


def test_default_thresholds_match_module_constants():
    adapter, _, _ = _make_adapter([5.0])
    state = adapter.update()
    assert state.obstacle_critical is True

    # Distance just over the default detected threshold (50 cm) clears the flag.
    adapter2, _, _ = _make_adapter([DEFAULT_OBSTACLE_DETECTED_CM + 1.0])
    state2 = adapter2.update()
    assert state2.obstacle_detected is False
