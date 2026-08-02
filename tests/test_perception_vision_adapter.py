"""Tests for VisionAdapter."""

import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.perception.vision_adapter import VisionAdapter  # noqa: E402
from cat_follow.runtime.shared_state import SharedState  # noqa: E402
from cat_follow.memory.pool import allocate_pool  # noqa: E402
from cat_follow.memory.shared_state import SharedState as ProtoSharedState  # noqa: E402
from cat_follow.telemetry.async_logger import AsyncLogger, CallableSink  # noqa: E402
from cat_follow.control.types import TelemetryEventType  # noqa: E402


class _FakePrototypeSS:
    """Stub for the prototype ``SharedState`` exposing get_bbox_tracker()."""

    def __init__(self, bbox=(0.0, 0.0, 0.0, 0.0, 0.0)):
        self._bbox = bbox
        self._lock = threading.Lock()

    def set_bbox(self, bbox):
        with self._lock:
            self._bbox = bbox

    def get_bbox_tracker(self):
        with self._lock:
            return self._bbox


def _make_adapter(image_width=640, image_height=480, **kwargs):
    proto = _FakePrototypeSS()
    contract = SharedState()
    adapter = VisionAdapter(
        prototype_shared_state=proto,
        contract_shared_state=contract,
        image_width=image_width,
        image_height=image_height,
        **kwargs,
    )
    return adapter, proto, contract


# ── construction ────────────────────────────────────────────────────


def test_constructor_validates_arguments():
    proto = _FakePrototypeSS()
    contract = SharedState()
    with pytest.raises(ValueError):
        VisionAdapter(proto, contract, image_width=0, image_height=480)
    with pytest.raises(ValueError):
        VisionAdapter(proto, contract, image_width=640, image_height=0)
    with pytest.raises(ValueError):
        VisionAdapter(
            proto, contract, image_width=640, image_height=480, stability_frames=0
        )
    with pytest.raises(ValueError):
        VisionAdapter(
            proto, contract, image_width=640, image_height=480, poll_rate_hz=0
        )
    with pytest.raises(ValueError):
        VisionAdapter(
            proto, contract, image_width=640, image_height=480, freshness_ttl_ms=0
        )


# ── invisible / visible ────────────────────────────────────────────


def test_no_bbox_publishes_invisible_state():
    adapter, proto, contract = _make_adapter()
    proto.set_bbox((0.0, 0.0, 0.0, 0.0, 0.0))  # valid=0
    state = adapter.update()
    assert state.cat_visible is False
    assert state.cat_visible_stable is False
    assert state.confidence == 0.0
    assert state.x_offset_norm == 0.0
    assert contract.get_vision().cat_visible is False


def test_valid_bbox_publishes_visible_state():
    adapter, proto, contract = _make_adapter()
    proto.set_bbox((100.0, 50.0, 60.0, 80.0, 1.0))  # valid=1
    state = adapter.update()
    assert state.cat_visible is True
    assert state.confidence == 1.0
    assert contract.get_vision().cat_visible is True


# ── x_offset_norm math ─────────────────────────────────────────────


def test_centered_bbox_yields_zero_offset():
    adapter, proto, _ = _make_adapter(image_width=640)
    # Bbox centered: top-left (310, 200), width=20 => center_x = 320 = image_center
    proto.set_bbox((310.0, 200.0, 20.0, 40.0, 1.0))
    state = adapter.update()
    assert state.x_offset_norm == pytest.approx(0.0, abs=1e-6)


def test_left_edge_bbox_yields_minus_one_offset():
    adapter, proto, _ = _make_adapter(image_width=640)
    # Bbox at left edge: center_x = 0
    proto.set_bbox((-10.0, 200.0, 20.0, 40.0, 1.0))
    state = adapter.update()
    assert state.x_offset_norm == pytest.approx(-1.0)


def test_right_edge_bbox_yields_plus_one_offset():
    adapter, proto, _ = _make_adapter(image_width=640)
    # Bbox at right edge: center_x = 640
    proto.set_bbox((630.0, 200.0, 20.0, 40.0, 1.0))
    state = adapter.update()
    assert state.x_offset_norm == pytest.approx(1.0)


def test_offset_is_clamped_outside_image_bounds():
    adapter, proto, _ = _make_adapter(image_width=640)
    # Cat bbox center way to the right of the image
    proto.set_bbox((1000.0, 200.0, 20.0, 40.0, 1.0))
    state = adapter.update()
    assert state.x_offset_norm == 1.0

    proto.set_bbox((-1000.0, 200.0, 20.0, 40.0, 1.0))
    state = adapter.update()
    assert state.x_offset_norm == -1.0


# ── stability ──────────────────────────────────────────────────────


def test_stable_lock_requires_n_consecutive_frames():
    adapter, proto, _ = _make_adapter(stability_frames=3)
    proto.set_bbox((300.0, 200.0, 40.0, 40.0, 1.0))

    s1 = adapter.update()
    s2 = adapter.update()
    s3 = adapter.update()
    assert s1.cat_visible_stable is False
    assert s2.cat_visible_stable is False
    assert s3.cat_visible_stable is True


def test_stability_resets_on_loss():
    adapter, proto, _ = _make_adapter(stability_frames=3)
    proto.set_bbox((300.0, 200.0, 40.0, 40.0, 1.0))
    for _ in range(3):
        adapter.update()
    assert adapter.update().cat_visible_stable is True

    # Lose the cat for one frame -> stability resets.
    proto.set_bbox((0.0, 0.0, 0.0, 0.0, 0.0))
    s = adapter.update()
    assert s.cat_visible is False
    assert s.cat_visible_stable is False

    # Re-acquire: requires 3 fresh frames before stable returns.
    proto.set_bbox((300.0, 200.0, 40.0, 40.0, 1.0))
    s1 = adapter.update()
    s2 = adapter.update()
    s3 = adapter.update()
    assert s1.cat_visible_stable is False
    assert s2.cat_visible_stable is False
    assert s3.cat_visible_stable is True


# ── generation-keyed stability / freshness ─────────────────────────


def test_stability_keyed_off_tracker_generation_not_poll_rate():
    """With a generation-aware prototype SS, repeatedly polling an unchanged
    tracker bbox must NOT accumulate stability; only new tracker frames do."""
    proto = ProtoSharedState(allocate_pool())
    contract = SharedState()
    adapter = VisionAdapter(
        proto, contract, image_width=640, image_height=480, stability_frames=3
    )

    proto.set_bbox_tracker(300.0, 200.0, 40.0, 40.0, 1.0)  # one new frame (gen)
    last = None
    for _ in range(5):
        last = adapter.update()
    # Five polls, but only ONE genuinely new tracker frame -> not stable.
    assert last.cat_visible_stable is False

    proto.set_bbox_tracker(300.0, 200.0, 40.0, 40.0, 1.0)  # 2nd new frame
    assert adapter.update().cat_visible_stable is False
    proto.set_bbox_tracker(300.0, 200.0, 40.0, 40.0, 1.0)  # 3rd new frame
    assert adapter.update().cat_visible_stable is True


def test_frozen_tracker_ages_out_received_ms():
    """A frozen tracker (no generation change) must not keep advancing
    received_ms, so the DecisionEngine can age vision out."""
    proto = ProtoSharedState(allocate_pool())
    contract = SharedState()
    adapter = VisionAdapter(proto, contract, image_width=640, image_height=480)

    proto.set_bbox_tracker(300.0, 200.0, 40.0, 40.0, 1.0)
    s1 = adapter.update()
    time.sleep(0.01)
    s2 = adapter.update()  # same generation -> stale, received_ms held
    assert s2.received_ms == s1.received_ms


def test_coasting_tracker_reports_not_fresh_once_past_the_ttl():
    """``fresh`` must track observation age, not just "the cat is visible"."""
    proto = ProtoSharedState(allocate_pool())
    contract = SharedState()
    adapter = VisionAdapter(
        proto,
        contract,
        image_width=640,
        image_height=480,
        freshness_ttl_ms=20,
    )

    proto.set_bbox_tracker(300.0, 200.0, 40.0, 40.0, 1.0)
    assert adapter.update().fresh is True

    time.sleep(0.05)
    stale = adapter.update()  # tracker coasting: still visible, no new frame
    assert stale.cat_visible is True
    assert stale.fresh is False


# ── last_seen_ms ───────────────────────────────────────────────────


def test_last_seen_ms_updates_only_on_visible_frames():
    adapter, proto, _ = _make_adapter()

    proto.set_bbox((300.0, 200.0, 40.0, 40.0, 1.0))
    s1 = adapter.update()
    assert s1.last_seen_ms > 0

    seen_ms = s1.last_seen_ms
    proto.set_bbox((0.0, 0.0, 0.0, 0.0, 0.0))
    s2 = adapter.update()
    # last_seen_ms is preserved when the cat is invisible.
    assert s2.last_seen_ms == seen_ms

    # Re-acquire bumps last_seen_ms forward.
    time.sleep(0.005)
    proto.set_bbox((300.0, 200.0, 40.0, 40.0, 1.0))
    s3 = adapter.update()
    assert s3.last_seen_ms >= seen_ms


# ── telemetry ──────────────────────────────────────────────────────


def test_update_emits_vision_update_telemetry():
    captured = []
    logger = AsyncLogger(
        sink=CallableSink(captured.append),
        max_queue=32,
        flush_interval_s=0.05,
    )
    logger.start()
    adapter, proto, _ = _make_adapter(logger=logger)
    proto.set_bbox((300.0, 200.0, 40.0, 40.0, 1.0))
    try:
        adapter.update()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not captured:
            time.sleep(0.02)
    finally:
        logger.stop()

    assert any(
        e["event_type"] == TelemetryEventType.VISION_UPDATE.value
        for e in captured
    )


# ── polling thread ─────────────────────────────────────────────────


def test_adapter_failure_retracts_the_vision_channel():
    """A broken adapter must not leave a stale "cat visible" for the FSM."""
    proto = ProtoSharedState(allocate_pool())
    contract = SharedState()
    adapter = VisionAdapter(proto, contract, image_width=640, image_height=480)

    proto.set_bbox_tracker(300.0, 200.0, 40.0, 40.0, 1.0)
    good = adapter.update()
    assert good.cat_visible is True
    assert good.fresh is True

    adapter._publish_fault_state()

    published = contract.get_vision()
    assert published.cat_visible is False
    assert published.cat_visible_stable is False
    assert published.fresh is False
    assert published.confidence == 0.0
    # The last genuine sighting stays visible for operators.
    assert published.last_seen_ms == good.last_seen_ms


def test_polling_thread_retracts_vision_after_a_failed_update():
    adapter, proto, contract = _make_adapter(poll_rate_hz=200.0)
    proto.set_bbox((300.0, 200.0, 40.0, 40.0, 1.0))
    adapter.update()
    assert contract.get_vision().cat_visible is True

    def _boom():
        raise RuntimeError("tracker read failed")

    proto.get_bbox_tracker = _boom
    adapter.start()
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if not contract.get_vision().cat_visible:
                break
            time.sleep(0.01)
        published = contract.get_vision()
        assert published.cat_visible is False
        assert published.fresh is False
    finally:
        adapter.stop()


def test_freshness_uses_monotonic_time_not_the_wall_clock():
    """An NTP step must not make an observation look fresh or stale."""
    proto = ProtoSharedState(allocate_pool())
    contract = SharedState()
    adapter = VisionAdapter(
        proto, contract, image_width=640, image_height=480, freshness_ttl_ms=50
    )
    proto.set_bbox_tracker(300.0, 200.0, 40.0, 40.0, 1.0)
    state = adapter.update()

    # received_ms comes from the monotonic clock, so it cannot be near the
    # wall-clock epoch milliseconds reported for display.
    assert state.received_ms < state.timestamp_ms
    assert state.fresh is True


def test_polling_thread_publishes_updates_and_stops_cleanly():
    adapter, proto, contract = _make_adapter(poll_rate_hz=200.0)
    proto.set_bbox((300.0, 200.0, 40.0, 40.0, 1.0))
    adapter.start()
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if contract.get_vision().cat_visible:
                break
            time.sleep(0.01)
        assert contract.get_vision().cat_visible is True
    finally:
        adapter.stop()
