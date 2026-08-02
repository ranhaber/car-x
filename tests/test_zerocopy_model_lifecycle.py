import ctypes
from pathlib import Path
import threading
import time

from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState
from cat_follow.perception.status import get_perception_diagnostics
from cat_follow.perception_config import PerceptionConfig
import cat_follow.threads.detector as detector_module
import cat_follow.vision.zerocopy_backend as zerocopy_module


class _NativeLifecycleLib:
    def __init__(self):
        self.loaded = True
        self.load_calls = 0
        self.unload_calls = 0

    def cf_zc_model_loaded(self, _handle):
        return int(self.loaded)

    def cf_zc_model_load(self, _handle):
        self.load_calls += 1
        self.loaded = True
        return 0

    def cf_zc_model_unload(self, _handle):
        self.unload_calls += 1
        self.loaded = False
        return 0


def _wrapper_session():
    session = zerocopy_module.ZerocopySession.__new__(
        zerocopy_module.ZerocopySession
    )
    session._handle = ctypes.c_void_p(123)
    session._lifecycle = threading.Condition()
    session._model_lock = threading.RLock()
    session._owned_lock = threading.Lock()
    session._owned_buffers = set()
    session._closing = False
    session._active_calls = 0
    session._animal_mode = False
    session.last_error = None
    session.last_perf = {}
    session.consecutive_failures = 0
    return session


def test_ctypes_lifecycle_preserves_session_handle(monkeypatch):
    lib = _NativeLifecycleLib()
    monkeypatch.setattr(zerocopy_module, "_load_lib", lambda: lib)
    session = _wrapper_session()
    handle = session._handle

    assert session.model_loaded is True
    assert session.unload_model() is True
    assert session.model_loaded is False
    assert session._handle is handle
    assert session.load_model() is True
    assert session.model_loaded is True
    assert (lib.unload_calls, lib.load_calls) == (1, 1)


def test_dequeued_buffer_is_tracked_before_close_can_run(monkeypatch):
    """A buffer registered after the lifecycle guard could be released by a
    concurrent close() while still dequeued and untracked."""
    session = _wrapper_session()
    observed = {}

    class _Lib:
        def cf_zc_dequeue(self, _handle, out_ptr, _timeout_ms):
            out_ptr._obj.buffer_index = 7
            return 0

        def cf_zc_close(self, _handle):
            observed["owned_at_close"] = set(session._owned_buffers)

    monkeypatch.setattr(zerocopy_module, "_load_lib", lambda: _Lib())

    real_exit = session._exit_call

    def _exit_and_close():
        # Model a close() that wins the race the instant the guard is dropped.
        observed["owned_at_exit"] = set(session._owned_buffers)
        real_exit()

    session._exit_call = _exit_and_close
    frame = session.dequeue(timeout_ms=10)

    assert frame is not None
    assert observed["owned_at_exit"] == {7}


class _DetectorSession:
    offset_x = 160
    offset_y = 160
    consecutive_failures = 0
    last_error = None
    last_perf = {}

    def __init__(
        self, stop_event, *, loaded=True, load_ok=True, stop_on_unload=False
    ):
        self._loaded = loaded
        self._load_ok = load_ok
        self.stop_event = stop_event
        self.stop_on_unload = stop_on_unload
        self.unload_calls = 0
        self.load_calls = 0
        self.warmup_indices = []

    @property
    def model_loaded(self):
        return self._loaded

    def unload_model(self):
        self.unload_calls += 1
        self._loaded = False
        if self.stop_on_unload:
            self.stop_event.set()
        return True

    def load_model(self):
        self.load_calls += 1
        if not self._load_ok:
            self.last_error = "mock reload rejected"
            return False
        self._loaded = True
        return True

    def warmup(self, buffer_index, *, score_threshold):
        self.warmup_indices.append((buffer_index, score_threshold))
        self._loaded = True
        self.stop_event.set()

    def infer(self, _buffer_index, _score_threshold):
        return []


def _dmabuf_shared(session):
    shared = SharedState(allocate_pool())
    shared.attach_zerocopy_session(session, requeue_cb=lambda _index: True)
    write_buf = shared.try_get_write_buffer()
    assert write_buf is not None
    shared.publish_dmabuf_from_write(
        capture_started_ns=time.monotonic_ns(),
        dmabuf_fd=41,
        buffer_index=2,
        image_size=640 * 480 * 3 // 2,
        stride=640,
    )
    return shared


def test_dmabuf_warmup_uses_published_frame_lease(monkeypatch):
    stop = threading.Event()
    session = _DetectorSession(stop)
    shared = _dmabuf_shared(session)
    shared.request_detector_warmup()
    monkeypatch.setattr(detector_module, "runtime_available", lambda: True)

    detector_module.run_detector_loop(
        shared,
        stop,
        target_fps=100.0,
        config=PerceptionConfig(
            zerocopy="dmabuf",
            warmup_on_start=True,
            motion_gating=True,
            idle_unload_sec=10.0,
        ),
    )

    assert session.warmup_indices == [(2, 0.5)]


def test_dmabuf_reload_failure_escalates(monkeypatch):
    stop = threading.Event()
    session = _DetectorSession(stop, loaded=False, load_ok=False)
    shared = _dmabuf_shared(session)
    fatal = detector_module.DetectorFatalHook()
    reasons = []
    fatal.set_handler(reasons.append)
    monkeypatch.setattr(detector_module, "runtime_available", lambda: True)

    detector_module.run_detector_loop(
        shared,
        stop,
        target_fps=100.0,
        on_fatal=fatal,
        config=PerceptionConfig(
            zerocopy="dmabuf",
            warmup_on_start=True,
            motion_gating=False,
        ),
    )

    assert stop.is_set()
    assert session.load_calls == 1
    assert reasons and "reload failed" in reasons[0]
    assert get_perception_diagnostics().model_loaded is False


def test_dmabuf_model_unloads_after_idle_timeout(monkeypatch):
    stop = threading.Event()
    session = _DetectorSession(stop, stop_on_unload=True)
    shared = _dmabuf_shared(session)
    monkeypatch.setattr(detector_module, "runtime_available", lambda: True)
    clock = {"now": time.monotonic()}

    def _advancing_monotonic():
        clock["now"] += 1.0
        return clock["now"]

    monkeypatch.setattr(detector_module.time, "monotonic", _advancing_monotonic)

    detector_module.run_detector_loop(
        shared,
        stop,
        target_fps=100.0,
        config=PerceptionConfig(
            zerocopy="dmabuf",
            warmup_on_start=True,
            motion_gating=True,
            idle_unload_sec=0.5,
        ),
    )

    assert session.unload_calls == 1
    assert session.model_loaded is False


def test_dmabuf_force_off_releases_model_while_motion_continues(monkeypatch):
    stop = threading.Event()
    session = _DetectorSession(stop, stop_on_unload=True)
    shared = _dmabuf_shared(session)
    # HOME/FAILSAFE: detector demand is definitively off.
    shared.set_perception_intent(
        capture_active=True,
        detector_required=False,
        detector_mission_override=False,
        stream_forced_off=False,
        detector_force_off=True,
    )
    monkeypatch.setattr(detector_module, "runtime_available", lambda: True)

    # Bound the run: without the fix nothing unloads and the loop never exits.
    watchdog = threading.Timer(5.0, stop.set)
    watchdog.start()
    try:
        detector_module.run_detector_loop(
            shared,
            stop,
            target_fps=100.0,
            config=PerceptionConfig(
                zerocopy="dmabuf",
                warmup_on_start=True,
                # Continuous motion keeps the phase machine out of IDLE, which
                # used to pin the model resident for the whole force-off window.
                motion_gating=False,
                idle_unload_sec=10.0,
            ),
        )
    finally:
        watchdog.cancel()

    assert session.unload_calls == 1
    assert session.model_loaded is False


class _NativeQueueLib(_NativeLifecycleLib):
    """Tracks V4L2 QBUF calls so double-requeues are visible to the test."""

    def __init__(self):
        super().__init__()
        self.requeued: list[int] = []

    def cf_zc_dequeue(self, _handle, out_ptr, _timeout_ms):
        out_ptr._obj.buffer_index = 7
        return 0

    def cf_zc_requeue(self, _handle, index):
        self.requeued.append(int(index.value))
        return 0


def test_requeue_of_unowned_buffer_never_reaches_the_driver(monkeypatch):
    lib = _NativeQueueLib()
    monkeypatch.setattr(zerocopy_module, "_load_lib", lambda: lib)
    session = _wrapper_session()

    frame = session.dequeue(timeout_ms=10)
    assert frame is not None and frame.buffer_index == 7

    assert session.requeue(7) is True
    # Reported as success so a bookkeeping slip cannot kill the mission, but
    # QBUF must happen exactly once per dequeue.
    assert session.requeue(7) is True
    assert lib.requeued == [7]


def test_failed_requeue_keeps_the_buffer_claim_for_retry(monkeypatch):
    lib = _NativeQueueLib()
    monkeypatch.setattr(zerocopy_module, "_load_lib", lambda: lib)
    monkeypatch.setattr(zerocopy_module, "_last_native_error", lambda: "qbuf failed")
    session = _wrapper_session()
    session.dequeue(timeout_ms=10)

    lib.cf_zc_requeue = lambda _handle, _index: -1
    assert session.requeue(7) is False

    lib.cf_zc_requeue = _NativeQueueLib.cf_zc_requeue.__get__(lib)
    assert session.requeue(7) is True
    assert lib.requeued == [7]


def test_infer_serializes_against_model_unload(monkeypatch):
    """A concurrent idle-unload must not free RKNN state mid-invoke."""
    overlapped = []
    in_flight = threading.Event()

    class _SlowInferLib(_NativeLifecycleLib):
        def __init__(self):
            super().__init__()
            self.unloading = False

        def cf_zc_infer_detections(self, *_args):
            in_flight.set()
            time.sleep(0.2)
            overlapped.append(self.unloading)
            return 0

        def cf_zc_model_unload(self, handle):
            self.unloading = True
            try:
                return super().cf_zc_model_unload(handle)
            finally:
                self.unloading = False

    lib = _SlowInferLib()
    monkeypatch.setattr(zerocopy_module, "_load_lib", lambda: lib)
    session = _wrapper_session()

    worker = threading.Thread(target=session.infer, args=(7, 0.5))
    worker.start()
    assert in_flight.wait(1.0)
    session.unload_model()
    worker.join(2.0)

    assert overlapped == [False]


def test_open_closes_the_native_handle_when_construction_fails(monkeypatch):
    closed: list[int] = []

    class _BadOffsetLib:
        def cf_zc_open(self, *_args):
            return 555

        def cf_zc_close(self, handle):
            closed.append(handle)

        def cf_zc_crop_offset(self, *_args):
            return -1

    monkeypatch.setattr(zerocopy_module, "_load_lib", lambda: _BadOffsetLib())
    monkeypatch.setattr(zerocopy_module, "_last_native_error", lambda: "no offset")

    raised = False
    try:
        zerocopy_module.ZerocopySession.open(
            device="/dev/video0",
            model_path="model.rknn",
            src_w=1920,
            src_h=1080,
            crop_w=320,
            crop_h=320,
        )
    except RuntimeError:
        raised = True

    assert raised is True
    assert closed == [555]


def test_service_requires_native_library_only_for_dmabuf():
    service = (
        Path(__file__).resolve().parents[1]
        / "cat_follow"
        / "scripts"
        / "cat-follow.service"
    ).read_text(encoding="utf-8")

    assert '"$${CAT_FOLLOW_PERCEPTION_ZEROCOPY}" = "dmabuf"' in service
    assert (
        "CAT_FOLLOW_ZEROCOPY_LIB:-/opt/car-x/lib/libcat_follow_zerocopy.so"
        in service
    )
