"""Tests for the event-driven ROCK 4D ultrasonic reader."""

from types import SimpleNamespace

import pytest

from cat_follow.perception.edge_ultrasonic import (
    EdgeTimedUltrasonic,
    release_legacy_ultrasonic,
)


class _FakeGpiod:
    class LineEvent:
        RISING_EDGE = 1
        FALLING_EDGE = 2


class _FakeEcho:
    def __init__(self, events):
        self._pending = []
        self._events = list(events)

    def activate(self):
        self._pending.extend(self._events)

    def event_wait(self, *, sec, nsec):  # noqa: ARG002
        return bool(self._pending)

    def event_read(self):
        return self._pending.pop(0)


class _FakeTrig:
    def __init__(self, echo):
        self._echo = echo
        self._was_high = False
        self.values = []

    def set_value(self, value):
        self.values.append(value)
        if value:
            self._was_high = True
        elif self._was_high:
            self._echo.activate()


def _event(event_type, timestamp_ns):
    return SimpleNamespace(
        type=event_type,
        sec=timestamp_ns // 1_000_000_000,
        nsec=timestamp_ns % 1_000_000_000,
    )


def test_measure_uses_kernel_edge_timestamps(monkeypatch):
    gpiod = _FakeGpiod()
    rising_ns = 10_000_000_000
    # 5,800 us corresponds to 100 cm using the HC-SR04 datasheet formula.
    falling_ns = rising_ns + 5_800_000
    echo = _FakeEcho(
        [
            _event(gpiod.LineEvent.RISING_EDGE, rising_ns),
            _event(gpiod.LineEvent.FALLING_EDGE, falling_ns),
        ]
    )
    trig = _FakeTrig(echo)
    reader = EdgeTimedUltrasonic(
        require_realtime=False,
        gpiod_module=gpiod,
    )
    monkeypatch.setattr(
        "cat_follow.perception.edge_ultrasonic.time.sleep",
        lambda _duration: None,
    )

    distance = reader._measure(trig, echo, gpiod)

    assert distance == pytest.approx(100.0)
    assert trig.values == [1, 0]


@pytest.mark.parametrize("pulse_ns", [0, -1, 1_000, 100_000_000])
def test_invalid_pulse_width_is_rejected(pulse_ns):
    reader = EdgeTimedUltrasonic(require_realtime=False)
    assert reader._distance_from_pulse_ns(pulse_ns) is None


def test_latest_distance_rejects_stale_sample(monkeypatch):
    now = 100.0
    monkeypatch.setattr(
        "cat_follow.perception.edge_ultrasonic.time.monotonic", lambda: now
    )
    reader = EdgeTimedUltrasonic(
        stale_after_s=0.25,
        require_realtime=False,
    )
    reader._publish(42.0)
    assert reader.latest_distance_cm() == 42.0

    now = 100.251
    assert reader.latest_distance_cm() is None


def test_release_legacy_ultrasonic_closes_gpio_owner():
    legacy = SimpleNamespace(closed=False)

    def close():
        legacy.closed = True

    legacy.close = close
    release_legacy_ultrasonic(SimpleNamespace(ultrasonic=legacy))
    assert legacy.closed is True


def test_constructor_rejects_unsafe_configuration():
    with pytest.raises(ValueError):
        EdgeTimedUltrasonic(cpu_core=-1)
    with pytest.raises(ValueError):
        EdgeTimedUltrasonic(rt_priority=100)
    with pytest.raises(ValueError):
        EdgeTimedUltrasonic(ping_interval_s=0)
