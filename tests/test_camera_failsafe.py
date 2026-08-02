"""Focused camera fatal/startup/lifecycle regression tests."""

import threading
import time

import pytest

import cat_follow.threads.camera as camera_module
from cat_follow.camera_config import CameraConfig
from cat_follow.control.types import FsmState
from cat_follow.runtime.app import build_app
from cat_follow.threads.camera import CameraFatalHook, CameraHandshake
from cat_follow.vision.zerocopy_backend import ZerocopySession


def test_camera_fatal_hook_fires_exactly_once():
    received = []
    hook = CameraFatalHook()
    hook.set_handler(received.append)

    hook.fire("first")
    hook.fire("second")

    assert hook.fired is True
    assert received == ["first"]


def test_uncaught_camera_exception_stops_and_reports(monkeypatch):
    stop = threading.Event()
    hook = CameraFatalHook()
    received = []
    hook.set_handler(received.append)
    handshake = CameraHandshake()

    def fail(*_args, **_kwargs):
        raise ValueError("capture exploded")

    monkeypatch.setattr(camera_module, "_run_camera_loop_impl", fail)
    camera_module.run_camera_loop(
        object(), stop, handshake=handshake, on_fatal=hook
    )

    assert stop.is_set()
    assert received == ["capture exploded"]
    with pytest.raises(RuntimeError, match="capture exploded"):
        handshake.wait_ready(timeout=0.01)


class _CaptureIntent:
    """Minimal SharedState stand-in exposing only the lifecycle intent gate."""

    def __init__(self, inactive_polls: int) -> None:
        self._remaining = inactive_polls

    def capture_active(self) -> bool:
        if self._remaining > 0:
            self._remaining -= 1
            return False
        return True


def test_capture_pause_restarts_the_publish_deadline():
    """An idle HOME/FAILSAFE gap is not a stalled camera."""
    stop = threading.Event()
    stale = time.monotonic() - 3600.0

    running, last_publish_s = camera_module._wait_for_capture_active(
        _CaptureIntent(inactive_polls=2), stop, stale
    )

    assert running is True
    assert last_publish_s > stale
    # The first frame after resume must not look overdue.
    camera_module._check_publish_deadline(
        last_publish_s, CameraConfig(no_publish_timeout_sec=5.0)
    )


def test_uninterrupted_capture_keeps_its_publish_deadline():
    stop = threading.Event()
    baseline = time.monotonic() - 1.0

    running, last_publish_s = camera_module._wait_for_capture_active(
        _CaptureIntent(inactive_polls=0), stop, baseline
    )

    assert running is True
    assert last_publish_s == baseline


def test_publish_deadline_still_fires_on_a_real_stall():
    with pytest.raises(RuntimeError, match="published no frame"):
        camera_module._check_publish_deadline(
            time.monotonic() - 10.0,
            CameraConfig(no_publish_timeout_sec=5.0),
        )


class _ZcFrame:
    cam_fd = 41
    image_size = 640 * 480 * 3 // 2
    stride = 640

    def __init__(self, buffer_index):
        self.buffer_index = buffer_index


class _FakeZerocopySession:
    """Minimal stand-in for the native session used by the camera loop."""

    instance = None
    requeue_ok = True

    def __init__(self):
        self.last_error = "requeue rejected"
        self.requeued = []
        self.dequeued = 0
        self.closed = False

    @classmethod
    def open(cls, **_kwargs):
        cls.instance = cls()
        return cls.instance

    def self_test(self):
        return None

    def dequeue(self, *, timeout_ms):  # noqa: ARG002
        self.dequeued += 1
        return _ZcFrame(self.dequeued)

    def requeue(self, buffer_index):
        self.requeued.append(buffer_index)
        return type(self).requeue_ok

    def close(self):
        self.closed = True


def _zerocopy_shared():
    from cat_follow.memory.pool import allocate_pool
    from cat_follow.memory.shared_state import SharedState as ProtoSharedState

    shared = ProtoSharedState(allocate_pool())
    shared.set_perception_intent(
        capture_active=True,
        detector_required=True,
        detector_mission_override=False,
        stream_forced_off=False,
    )
    return shared


def _run_zerocopy_frames(monkeypatch, shared, *, frames, requeue_ok=True):
    import cat_follow.vision.zerocopy_backend as zc_module
    from cat_follow.camera_config import CameraConfig
    from cat_follow.perception_config import PerceptionConfig

    _FakeZerocopySession.requeue_ok = requeue_ok
    monkeypatch.setattr(zc_module, "ZerocopySession", _FakeZerocopySession)

    stop = threading.Event()
    published = {"count": 0}
    real_publish = shared.publish_dmabuf_from_write

    def _counting_publish(**kwargs):
        published["count"] += 1
        if published["count"] >= frames:
            stop.set()
        return real_publish(**kwargs)

    monkeypatch.setattr(shared, "publish_dmabuf_from_write", _counting_publish)

    camera_module._run_zerocopy_camera_loop(
        shared,
        stop,
        config=CameraConfig(lores_device=""),
        perception=PerceptionConfig(zerocopy="dmabuf"),
        target_fps=30.0,
        tick=0.03,
    )
    return _FakeZerocopySession.instance


class _RecordingStopEvent(threading.Event):
    """Records requested wait durations without actually sleeping."""

    def __init__(self) -> None:
        super().__init__()
        self.waits: list = []

    def wait(self, timeout=None):  # noqa: ANN001
        self.waits.append(timeout)
        return super().wait(0)


def test_zerocopy_loop_paces_to_the_configured_rate(monkeypatch):
    """Free-running capture would ignore target_fps whenever a slot is free."""
    import cat_follow.vision.zerocopy_backend as zc_module
    from cat_follow.perception_config import PerceptionConfig

    shared = _zerocopy_shared()
    _FakeZerocopySession.requeue_ok = True
    monkeypatch.setattr(zc_module, "ZerocopySession", _FakeZerocopySession)

    stop = _RecordingStopEvent()
    published = {"count": 0}
    real_publish = shared.publish_dmabuf_from_write

    def _counting_publish(**kwargs):
        published["count"] += 1
        if published["count"] >= 3:
            stop.set()
        return real_publish(**kwargs)

    monkeypatch.setattr(shared, "publish_dmabuf_from_write", _counting_publish)

    tick = 0.05
    camera_module._run_zerocopy_camera_loop(
        shared,
        stop,
        config=CameraConfig(lores_device=""),
        perception=PerceptionConfig(zerocopy="dmabuf"),
        target_fps=1.0 / tick,
        tick=tick,
    )

    # Capture is instantaneous here, so pacing must ask for nearly a full tick.
    assert any(w is not None and w > tick * 0.5 for w in stop.waits)


def test_zerocopy_loop_refuses_live_cat_inject(monkeypatch):
    """Inject only paints the NumPy ring, which dmabuf consumers never read."""
    shared = _zerocopy_shared()
    shared.set_cat_injection_enabled(True)

    _run_zerocopy_frames(monkeypatch, shared, frames=2)

    assert shared.cat_injection_enabled() is False
    assert shared.get_cat_injection_status()["bbox"] is None


def test_zerocopy_loop_does_not_double_requeue_a_published_buffer(monkeypatch):
    """A failed supersession QBUF must not make the camera requeue again."""
    shared = _zerocopy_shared()

    with pytest.raises(Exception):
        _run_zerocopy_frames(monkeypatch, shared, frames=8, requeue_ok=False)

    session = _FakeZerocopySession.instance
    assert session.closed is True
    # Buffer 1 was superseded by frame 2 and failed to requeue. Frame 2's own
    # buffer already belongs to the ring, so the loop must not requeue it.
    assert session.requeued == [1]


def test_zerocopy_self_test_requires_successful_requeue():
    session = ZerocopySession.__new__(ZerocopySession)
    session.last_error = "qbuf failed"
    session.consecutive_failures = 0
    frame = type("Frame", (), {"buffer_index": 7})()
    session.dequeue = lambda **_kwargs: frame
    session.infer = lambda *_args, **_kwargs: []
    session.requeue = lambda _index: False

    with pytest.raises(RuntimeError, match="qbuf failed"):
        session.self_test()


def test_zerocopy_self_test_requeues_after_infer_failure():
    session = ZerocopySession.__new__(ZerocopySession)
    session.last_error = None
    session.consecutive_failures = 0
    frame = type("Frame", (), {"buffer_index": 3})()
    requeued = []
    session.dequeue = lambda **_kwargs: frame

    def fail_infer(*_args, **_kwargs):
        session.consecutive_failures = 1
        session.last_error = "native warmup failed"
        return []

    session.infer = fail_infer
    session.requeue = lambda index: requeued.append(index) or True

    with pytest.raises(RuntimeError, match="native warmup failed"):
        session.self_test()
    assert requeued == [3]


def test_camera_fatal_latches_runtime_failsafe_and_reason(tmp_path):
    stop = threading.Event()
    perception_stop = threading.Event()
    hook = CameraFatalHook()
    app = build_app(
        log_path=tmp_path / "telemetry.jsonl",
        stop_event=stop,
        prototype_perception_stop_event=perception_stop,
        prototype_camera_fatal_hook=hook,
    )

    hook.fire("persistent dequeue")
    hook.fire("ignored duplicate")

    assert stop.is_set()
    assert perception_stop.is_set()
    assert app.fsm.state is FsmState.FAILSAFE
    assert (
        app.shared_state.get_runtime_fatal_reason()
        == "camera_fatal: persistent dequeue"
    )
