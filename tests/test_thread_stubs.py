"""
Integration tests for cat_follow.threads (stub camera, tracker, detector).

Start all three threads with SharedState, run for a short burst, stop,
and verify that shared state was written by each thread.

Run:
    python -m pytest tests/test_thread_stubs.py -v
or:
    python tests/test_thread_stubs.py
"""

import sys
import os
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest
import cat_follow.threads.camera as camera_module
import cat_follow.threads.detector as detector_module
from cat_follow.memory.pool import allocate_pool, FRAME_SHAPE
from cat_follow.memory.shared_state import SharedState
from cat_follow.threads.camera import run_camera_loop
from cat_follow.threads.tracker import run_tracker_loop
from cat_follow.threads.detector import run_detector_loop


# ── helpers ──────────────────────────────────────────────────────────────

@pytest.fixture(autouse=True)
def _force_deterministic_stubs(monkeypatch):
    """Keep integration tests independent of host camera/model availability.

    The camera falls back to its no-cv2 stub, and the detector's backend
    factory is forced to report "no usable model" so the detector runs its
    deterministic bbox stub regardless of what is installed on the host.

    The deterministic stub is opt-in (production hard-fails without an NPU), so
    the dev/CI flag is enabled for these integration tests.
    """
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_ALLOW_STUB", "1")
    monkeypatch.setattr(camera_module, "_HAS_CV2", False)

    class _UnavailableBackend:
        loaded = False

        def runtime_available(self):
            # No RKNN runtime on the host -> detector takes the stub path
            # (not the hard-fail path).
            return False

        def available(self):
            return False

        def load(self):
            return False

        def unload(self):
            return None

        def self_test(self):
            return None

        def infer(self, frame_bgr, score_threshold):
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    monkeypatch.setattr(
        detector_module,
        "create_backend",
        lambda *args, **kwargs: _UnavailableBackend(),
    )


def _run_threads_for(seconds: float = 1.0):
    """Start all three stub threads, run for *seconds*, stop, return shared."""
    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()

    threads = [
        threading.Thread(
            target=run_camera_loop, args=(shared, stop),
            name="camera-stub", daemon=True,
        ),
        threading.Thread(
            target=run_tracker_loop, args=(shared, stop),
            name="tracker-stub", daemon=True,
        ),
        threading.Thread(
            target=run_detector_loop, args=(shared, stop),
            kwargs={"target_fps": 1.0},
            name="detector-stub", daemon=True,
        ),
    ]
    for t in threads:
        t.start()

    time.sleep(seconds)
    stop.set()

    for t in threads:
        t.join(timeout=3.0)

    return shared, threads


# ── tests ────────────────────────────────────────────────────────────────

def test_all_threads_start_and_stop():
    """Threads start, run briefly, and join without hanging."""
    shared, threads = _run_threads_for(0.5)
    for t in threads:
        assert not t.is_alive(), f"Thread {t.name} did not stop"


def test_camera_writes_frame_latest():
    """After running, frame_latest must not be all zeros."""
    shared, _ = _run_threads_for(0.5)
    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_latest(dst)
    assert not np.all(dst == 0), "Camera stub should have written non-zero frames"


def test_camera_frame_has_pattern():
    """Camera stub fills frames with (frame_index % 256), so at least
    some pixel values should be non-zero after many frames."""
    shared, _ = _run_threads_for(0.5)
    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_latest(dst)
    # All pixels in one frame should be the same value (stub fills uniformly)
    val = dst[0, 0, 0]
    assert np.all(dst == val), "Stub camera should fill the entire frame with one value"


def test_detector_writes_bbox():
    """Detector stub should write bbox_detector with valid=1."""
    shared, _ = _run_threads_for(0.5)

    # The detector needs frame_for_detector to be populated.
    # In a real app main would call copy_latest_to_detector_frame().
    # Our stub detector reads whatever is in the buffer (zeros is fine).
    # The real detector falls back to a stub that periodically publishes a bbox.
    # We need to wait for it.
    time.sleep(0.2)
    bbox = shared.get_bbox_detector()
    assert bbox[4] > 0, (
        f"Expected detector stub to publish a valid bbox, got {bbox}"
    )


def test_copy_latest_to_detector_during_run():
    """Simulate main copying frame_latest -> frame_for_detector while
    threads are running; verify detector still gets a valid frame."""
    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()

    threads = [
        threading.Thread(
            target=run_camera_loop, args=(shared, stop),
            name="camera-stub", daemon=True,
        ),
        threading.Thread(
            target=run_tracker_loop, args=(shared, stop),
            name="tracker-stub", daemon=True,
        ),
        threading.Thread(
            target=run_detector_loop, args=(shared, stop),
            kwargs={"target_fps": 1.0},
            name="detector-stub", daemon=True,
        ),
    ]
    for t in threads:
        t.start()

    # Simulate main loop copying every 0.1 s for 0.5 s
    for _ in range(5):
        time.sleep(0.1)
        shared.copy_latest_to_detector_frame()

    stop.set()
    for t in threads:
        t.join(timeout=3.0)

    # After copies, frame_for_detector should be non-zero
    dst = np.zeros(FRAME_SHAPE, dtype=np.uint8)
    shared.get_frame_for_detector(dst)
    assert not np.all(dst == 0), (
        "frame_for_detector should be non-zero after copies from camera"
    )


def test_no_exceptions_during_run():
    """Run threads; if any thread raised, it would have stopped early.
    We verify all threads were still alive right before stop."""
    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()
    alive_flags = {}

    threads = [
        threading.Thread(
            target=run_camera_loop, args=(shared, stop),
            name="camera-stub", daemon=True,
        ),
        threading.Thread(
            target=run_tracker_loop, args=(shared, stop),
            name="tracker-stub", daemon=True,
        ),
        threading.Thread(
            target=run_detector_loop, args=(shared, stop),
            kwargs={"target_fps": 1.0},
            name="detector-stub", daemon=True,
        ),
    ]
    for t in threads:
        t.start()

    time.sleep(0.5)

    # Snapshot alive status before stopping
    for t in threads:
        alive_flags[t.name] = t.is_alive()

    stop.set()
    for t in threads:
        t.join(timeout=3.0)

    for name, was_alive in alive_flags.items():
        assert was_alive, f"Thread {name} died before stop (likely exception)"


def test_detector_hard_fails_when_runtime_present_but_model_missing(monkeypatch):
    """If the RKNN runtime is present but the model is missing, the detector
    loop must raise RuntimeError rather than silently degrade."""

    class _RuntimePresentNoModel:
        loaded = False

        def runtime_available(self):
            return True

        def available(self):
            return False

        def load(self):
            return False

        def unload(self):
            return None

        def self_test(self):
            raise RuntimeError("model missing")

        def infer(self, frame_bgr, score_threshold):
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    monkeypatch.setattr(
        detector_module,
        "create_backend",
        lambda *args, **kwargs: _RuntimePresentNoModel(),
    )

    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()
    with pytest.raises(RuntimeError):
        run_detector_loop(shared, stop, target_fps=1.0)


def test_detector_hard_fails_when_model_fails_strict_validation(monkeypatch):
    """A model that loads but fails the strict inference/output-contract check
    must raise at preflight, not silently return empty detections."""

    class _RuntimePresentCorruptModel:
        loaded = False

        def runtime_available(self):
            return True

        def available(self):
            return True  # file exists...

        def load(self):
            return True  # ...loads, but inference/output contract is broken

        def unload(self):
            return None

        def self_test(self):
            # Strict validation catches the unusable model.
            raise ValueError("output contract mismatch")

        def infer(self, frame_bgr, score_threshold):
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    monkeypatch.setattr(
        detector_module,
        "create_backend",
        lambda *args, **kwargs: _RuntimePresentCorruptModel(),
    )

    with pytest.raises(RuntimeError):
        detector_module.preflight_perception()


def test_preflight_returns_false_without_runtime(monkeypatch):
    """When the RKNN runtime is absent (dev/CI), preflight reports stub mode
    (False) instead of raising."""

    class _NoRuntime:
        loaded = False

        def runtime_available(self):
            return False

        def available(self):
            return False

        def load(self):
            return False

        def unload(self):
            return None

        def self_test(self):
            return None

        def infer(self, frame_bgr, score_threshold):
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    monkeypatch.setattr(
        detector_module,
        "create_backend",
        lambda *args, **kwargs: _NoRuntime(),
    )
    assert detector_module.preflight_perception() is False


def test_detector_hard_fails_without_runtime_when_stub_disabled(monkeypatch):
    """In production (stub disabled) a missing RKNN runtime must NOT quietly
    enable the fake-detection stub -- it must hard-fail."""

    class _NoRuntime:
        loaded = False

        def runtime_available(self):
            return False

        def available(self):
            return False

        def load(self):
            return False

        def unload(self):
            return None

        def self_test(self):
            return None

        def infer(self, frame_bgr, score_threshold):
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_ALLOW_STUB", "0")
    monkeypatch.setattr(
        detector_module,
        "create_backend",
        lambda *args, **kwargs: _NoRuntime(),
    )
    with pytest.raises(RuntimeError):
        detector_module.preflight_perception()


def test_startup_handshake_reports_failure(monkeypatch):
    """A backend that fails validation must report the failure through the
    handshake so a supervisor waiting on it aborts startup."""

    class _RuntimePresentCorruptModel:
        loaded = False

        def runtime_available(self):
            return True

        def available(self):
            return True

        def load(self):
            return True

        def unload(self):
            return None

        def self_test(self):
            raise ValueError("output contract mismatch")

        def infer(self, frame_bgr, score_threshold):
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    monkeypatch.setattr(
        detector_module,
        "create_backend",
        lambda *args, **kwargs: _RuntimePresentCorruptModel(),
    )

    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()
    handshake = detector_module.DetectorHandshake()
    t = threading.Thread(
        target=run_detector_loop,
        args=(shared, stop),
        kwargs={"target_fps": 1.0, "handshake": handshake},
        daemon=True,
    )
    t.start()
    try:
        with pytest.raises(RuntimeError):
            handshake.wait_ready(timeout=3.0)
    finally:
        stop.set()
        t.join(timeout=3.0)
    assert stop.is_set()


def test_startup_handshake_reports_stub_ready(monkeypatch):
    """When the stub is allowed and no runtime is present, the handshake must
    report readiness in stub mode (not raise)."""
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_ALLOW_STUB", "1")

    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()
    handshake = detector_module.DetectorHandshake()
    t = threading.Thread(
        target=run_detector_loop,
        args=(shared, stop),
        kwargs={"target_fps": 5.0, "handshake": handshake},
        daemon=True,
    )
    t.start()
    try:
        npu_ready = handshake.wait_ready(timeout=3.0)
        assert npu_ready is False  # stub mode
    finally:
        stop.set()
        t.join(timeout=3.0)


def test_detector_escalates_on_repeated_inference_failure(monkeypatch):
    """Repeated inference failures must escalate: the detector sets stop_event
    (notifying the supervisor) rather than returning empty forever."""
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_ALLOW_STUB", "1")
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_MOTION_GATING", "0")
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_MAX_INFER_FAILURES", "3")

    class _FailingBackend:
        loaded = True
        consecutive_failures = 0
        last_error = "boom"

        def runtime_available(self):
            return True

        def available(self):
            return True

        def load(self):
            return True

        def unload(self):
            return None

        def self_test(self):
            return None

        def infer(self, frame_bgr, score_threshold):
            self.consecutive_failures += 1
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    monkeypatch.setattr(
        detector_module,
        "create_backend",
        lambda *args, **kwargs: _FailingBackend(),
    )

    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()
    handshake = detector_module.DetectorHandshake()
    t = threading.Thread(
        target=run_detector_loop,
        args=(shared, stop),
        kwargs={"target_fps": 20.0, "handshake": handshake},
        daemon=True,
    )
    t.start()
    # The worker should escalate within a few ticks and set stop_event itself.
    for _ in range(50):
        if stop.is_set():
            break
        time.sleep(0.05)
    t.join(timeout=3.0)
    assert stop.is_set(), "detector should escalate repeated failures via stop_event"


def test_fatal_hook_buffers_until_handler_set():
    """A fatal fired before the handler is wired is delivered once set."""
    hook = detector_module.DetectorFatalHook()
    received = []
    hook.fire("early failure")
    assert hook.fired is True
    assert received == []  # no handler yet
    hook.set_handler(received.append)
    assert received == ["early failure"]


def test_detector_startup_failure_fires_on_fatal(monkeypatch):
    """A backend that fails strict validation must notify the supervisor via
    on_fatal (so it can e-stop + FAILSAFE), not just die on the thread."""

    class _CorruptModel:
        loaded = False

        def runtime_available(self):
            return True

        def available(self):
            return True

        def load(self):
            return True

        def unload(self):
            return None

        def self_test(self):
            raise ValueError("contract mismatch")

        def infer(self, frame_bgr, score_threshold):
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    monkeypatch.setattr(
        detector_module,
        "create_backend",
        lambda *args, **kwargs: _CorruptModel(),
    )

    fatal = []
    hook = detector_module.DetectorFatalHook()
    hook.set_handler(fatal.append)

    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()
    handshake = detector_module.DetectorHandshake()
    t = threading.Thread(
        target=run_detector_loop,
        args=(shared, stop),
        kwargs={"target_fps": 1.0, "handshake": handshake, "on_fatal": hook},
        daemon=True,
    )
    t.start()
    with pytest.raises(RuntimeError):
        handshake.wait_ready(timeout=3.0)
    t.join(timeout=3.0)
    assert fatal, "on_fatal handler should have been invoked on startup failure"


def test_detector_runtime_escalation_fires_on_fatal(monkeypatch):
    """Repeated inference failures escalate through on_fatal and stop_event."""
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_ALLOW_STUB", "1")
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_MOTION_GATING", "0")
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_MAX_INFER_FAILURES", "2")

    class _FailingBackend:
        loaded = True
        consecutive_failures = 0
        last_error = "boom"

        def runtime_available(self):
            return True

        def available(self):
            return True

        def load(self):
            return True

        def unload(self):
            return None

        def self_test(self):
            return None

        def infer(self, frame_bgr, score_threshold):
            self.consecutive_failures += 1
            return (0.0, 0.0, 0.0, 0.0, 0.0)

    monkeypatch.setattr(
        detector_module,
        "create_backend",
        lambda *args, **kwargs: _FailingBackend(),
    )

    fatal = []
    hook = detector_module.DetectorFatalHook()
    hook.set_handler(fatal.append)

    pool = allocate_pool()
    shared = SharedState(pool)
    stop = threading.Event()
    t = threading.Thread(
        target=run_detector_loop,
        args=(shared, stop),
        kwargs={"target_fps": 20.0, "on_fatal": hook},
        daemon=True,
    )
    t.start()
    for _ in range(50):
        if stop.is_set():
            break
        time.sleep(0.05)
    t.join(timeout=3.0)
    assert stop.is_set()
    assert fatal, "on_fatal handler should have been invoked on runtime escalation"


# ── run as script ────────────────────────────────────────────────────────

if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"  PASS  {fn.__name__}")
    print(f"\nAll {len(tests)} tests passed.")
