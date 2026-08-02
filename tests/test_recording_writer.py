"""RecordingWriter host stub policy (REC-01..04) + stream independence."""

import os
import threading
import time
from types import SimpleNamespace

from cat_follow.control.types import (
    ConsumerState,
    FsmState,
    MissionState,
    PerceptionLifecycleState,
)
from cat_follow.perception.perception_lifecycle_manager import (
    LifecycleMissionContext,
    PerceptionLifecycleManager,
)
from cat_follow.perception.recording_store import RecordingStore, SegmentRecord
from cat_follow.perception.recording_writer import RecordingWriter, StubH264Encoder
from cat_follow.target_config import TargetRuntimeConfig


def _lifecycle_requesting(*, requested=True, now_ms=1000):
    return PerceptionLifecycleState(
        received_ms=now_ms,
        recording=ConsumerState(requested=requested, active=requested),
        capture_active=requested,
    )


def test_rec_01_writer_creates_segmented_stub_access_units(tmp_path):
    store = RecordingStore(str(tmp_path))
    writer = RecordingWriter(
        store, encoder=StubH264Encoder(au_size=32), segment_duration_ms=1000
    )
    feedback = writer.tick(_lifecycle_requesting(now_ms=1000), now_ms=1000)
    assert feedback.active is True
    assert feedback.segment_path is not None
    assert feedback.bytes_written > 0
    assert store.active_path() is not None
    assert store.active_path().endswith(".mkv.part")


def test_rec_01_rotate_finalizes_and_opens_next(tmp_path):
    store = RecordingStore(str(tmp_path))
    writer = RecordingWriter(
        store, encoder=StubH264Encoder(), segment_duration_ms=1000
    )
    writer.tick(_lifecycle_requesting(now_ms=1000), now_ms=1000)
    first = store.active_path()
    writer.tick(_lifecycle_requesting(now_ms=2100), now_ms=2100)
    assert writer.runtime_state().segments_finalized == 1
    assert store.active_path() != first
    assert store.list_finalized()


def test_rec_03_low_space_degrades_without_raising(tmp_path):
    store = RecordingStore(
        str(tmp_path),
        min_free_bytes=10_000,
        disk_usage=lambda _path: SimpleNamespace(free=1),
    )
    writer = RecordingWriter(store, encoder=StubH264Encoder())
    feedback = writer.tick(_lifecycle_requesting(), now_ms=1)
    assert feedback.active is False
    assert feedback.degraded_reason == "low_space"

    # Resume automatically once reserve recovers.
    store.min_free_bytes = None
    feedback = writer.tick(_lifecycle_requesting(), now_ms=2)
    assert feedback.active is True
    assert feedback.degraded_reason is None


def test_rec_03_merge_feedback_keeps_requested_when_inactive(tmp_path):
    mgr = PerceptionLifecycleManager(TargetRuntimeConfig())
    store = RecordingStore(
        str(tmp_path),
        min_free_bytes=10_000,
        disk_usage=lambda _path: SimpleNamespace(free=1),
    )
    writer = RecordingWriter(store, encoder=StubH264Encoder())
    life = mgr.tick(
        fsm_state=FsmState.CHASE,
        mission=MissionState(chase_recording_requested=True),
        context=LifecycleMissionContext(chase_recording_requested=True),
        now_ms=10,
    )
    assert life.recording.requested is True
    feedback = writer.tick(life, now_ms=10)
    merged = mgr.merge_recording_feedback(feedback)
    assert merged.recording.requested is True
    assert merged.recording.active is False
    assert merged.recording.degraded_reason == "low_space"


def test_rec_04_postroll_keeps_writer_until_deadline(tmp_path):
    mgr = PerceptionLifecycleManager(TargetRuntimeConfig())
    store = RecordingStore(str(tmp_path))
    writer = RecordingWriter(
        store, encoder=StubH264Encoder(), segment_duration_ms=60_000
    )

    life = mgr.tick(
        fsm_state=FsmState.IDLE,
        mission=MissionState(),
        context=LifecycleMissionContext(
            recording_postroll_deadline_ms=5000
        ),
        now_ms=1000,
    )
    assert life.recording.requested is True
    assert life.recording.reason == "postroll"
    feedback = writer.tick(life, now_ms=1000)
    assert feedback.active is True

    expired = mgr.tick(
        fsm_state=FsmState.IDLE,
        mission=MissionState(),
        context=LifecycleMissionContext(
            recording_postroll_deadline_ms=5000
        ),
        now_ms=6000,
    )
    feedback = writer.tick(expired, now_ms=6000)
    assert expired.recording.requested is False
    assert feedback.active is False


def test_str_02_recording_is_not_a_stream_client(tmp_path):
    mgr = PerceptionLifecycleManager(TargetRuntimeConfig())
    life = mgr.tick(
        fsm_state=FsmState.CHASE,
        mission=MissionState(chase_recording_requested=True),
        context=LifecycleMissionContext(chase_recording_requested=True),
        now_ms=1,
    )
    assert life.recording.requested is True
    assert life.stream_requested_clients == 0
    assert life.stream_active_clients == 0
    assert life.stream_encoder_ready is False
    assert life.capture_active is True

    store = RecordingStore(str(tmp_path))
    writer = RecordingWriter(store, encoder=StubH264Encoder())
    feedback = writer.tick(life, now_ms=1)
    merged = mgr.merge_recording_feedback(feedback)
    assert merged.stream_requested_clients == 0
    assert merged.recording.active is True


def test_rec_02_reserve_is_rechecked_while_a_segment_grows(tmp_path):
    free = [10_000_000]
    store = RecordingStore(
        str(tmp_path),
        min_free_bytes=10_000,
        disk_usage=lambda _path: SimpleNamespace(free=free[0]),
    )
    writer = RecordingWriter(
        store, encoder=StubH264Encoder(), segment_duration_ms=600_000
    )
    assert writer.tick(_lifecycle_requesting(), now_ms=1000).active is True

    # Disk fills up mid-segment; the writer must notice without waiting for
    # the next segment boundary.
    free[0] = 1
    feedback = writer.tick(_lifecycle_requesting(), now_ms=1100)
    assert feedback.active is False
    assert feedback.degraded_reason == "low_space"


def test_rec_02_active_part_counts_toward_quota(tmp_path):
    store = RecordingStore(str(tmp_path), quota_bytes=200)
    writer = RecordingWriter(
        store,
        encoder=StubH264Encoder(au_size=64),
        segment_duration_ms=1000,
        retention_interval_ms=0,
    )
    writer.tick(_lifecycle_requesting(), now_ms=1000)
    writer.tick(_lifecycle_requesting(), now_ms=2100)
    first = store.list_finalized()[0].path
    assert os.path.exists(first)

    for step in range(8):
        writer.tick(_lifecycle_requesting(), now_ms=2200 + step)

    # The growing .part pushed total usage over quota, so the finalized
    # segment had to be reclaimed while recording continued.
    assert not os.path.exists(first)
    assert os.path.exists(store.active_path())


def test_rec_02_active_segment_alone_cannot_outgrow_quota(tmp_path):
    store = RecordingStore(str(tmp_path), quota_bytes=256)
    writer = RecordingWriter(
        store,
        encoder=StubH264Encoder(au_size=64),
        # A far-away rotation deadline used to let one .part grow unbounded,
        # because retention can only reclaim finalized segments.
        segment_duration_ms=600_000,
        retention_interval_ms=0,
    )
    for step in range(12):
        writer.tick(_lifecycle_requesting(), now_ms=1000 + step)

    assert writer.runtime_state().segments_finalized >= 1
    assert store.active_bytes() <= 256
    assert store.total_finalized_bytes() + store.active_bytes() <= 256


def test_rec_02_quota_pressure_bypasses_the_retention_throttle(tmp_path):
    store = RecordingStore(str(tmp_path), quota_bytes=128)
    writer = RecordingWriter(
        store,
        encoder=StubH264Encoder(au_size=16),
        segment_duration_ms=600_000,
        retention_interval_ms=600_000,
    )
    writer.tick(_lifecycle_requesting(), now_ms=1000)

    # A crash-recovery backlog shows up right after that tick's prune, so the
    # throttle alone would hold the quota breach for ten more minutes.
    stale = tmp_path / "stale.mkv"
    stale.write_bytes(b"s" * 300)
    store._append_index(
        SegmentRecord(path=str(stale), bytes=300, finalized_at_ms=1)
    )

    writer.tick(_lifecycle_requesting(), now_ms=1100)

    # Disk pressure outranks the throttle, which only exists to keep index
    # rewrites off every steady-state tick.
    assert not stale.exists()


def test_rec_03_status_read_does_not_block_on_slow_disk(tmp_path):
    class _SlowStore(RecordingStore):
        def append_bytes(self, path, payload):
            time.sleep(0.3)
            return super().append_bytes(path, payload)

    store = _SlowStore(str(tmp_path))
    writer = RecordingWriter(store, encoder=StubH264Encoder())
    writer.tick(_lifecycle_requesting(), now_ms=1000)

    thread = threading.Thread(
        target=writer.tick,
        args=(_lifecycle_requesting(now_ms=1100),),
        kwargs={"now_ms": 1100},
    )
    thread.start()
    time.sleep(0.05)
    started = time.perf_counter()
    writer.runtime_state()
    elapsed = time.perf_counter() - started
    thread.join()

    assert elapsed < 0.1


def test_rec_03_degraded_writer_releases_the_camera(tmp_path):
    mgr = PerceptionLifecycleManager(TargetRuntimeConfig())
    store = RecordingStore(
        str(tmp_path),
        min_free_bytes=10_000,
        disk_usage=lambda _path: SimpleNamespace(free=1),
    )
    writer = RecordingWriter(store, encoder=StubH264Encoder())
    life = mgr.tick(
        fsm_state=FsmState.GETTING_CLOSE,
        mission=MissionState(chase_recording_requested=True),
        context=LifecycleMissionContext(chase_recording_requested=True),
        now_ms=10,
    )
    # Recording is the only consumer in GETTING_CLOSE.
    assert life.detector.requested is False
    assert life.capture_active is True

    merged = mgr.merge_recording_feedback(writer.tick(life, now_ms=10))
    assert merged.recording.degraded_reason == "low_space"
    assert merged.recording.consumer_refcount == 0
    assert merged.capture_active is False
    assert merged.camera_hardware_state.value == "ready_inactive"


def test_rec_03_finalize_failure_is_reported(tmp_path):
    class _BadFinalizeStore(RecordingStore):
        def finalize_segment(self, path):
            raise OSError("rename failed")

    store = _BadFinalizeStore(str(tmp_path))
    writer = RecordingWriter(store, encoder=StubH264Encoder())
    writer.tick(_lifecycle_requesting(), now_ms=1000)

    feedback = writer.tick(_lifecycle_requesting(requested=False), now_ms=2000)
    assert feedback.degraded_reason == "finalize_error"
    assert feedback.active is False


def test_rec_01_segment_names_use_wall_clock_not_monotonic(tmp_path):
    store = RecordingStore(str(tmp_path))
    writer = RecordingWriter(store, encoder=StubH264Encoder())
    # A monotonic uptime clock would name this segment in 1970.
    writer.tick(_lifecycle_requesting(now_ms=90_000), now_ms=90_000)
    name = os.path.basename(store.active_path())
    assert int(name[:4]) >= 2024


def test_rec_05_missing_encoder_fails_closed_and_releases_the_camera(tmp_path):
    """Fake bytes must never pass for a healthy chase recording."""
    mgr = PerceptionLifecycleManager(TargetRuntimeConfig())
    store = RecordingStore(str(tmp_path))
    writer = RecordingWriter(store)

    life = mgr.tick(
        fsm_state=FsmState.GETTING_CLOSE,
        mission=MissionState(chase_recording_requested=True),
        context=LifecycleMissionContext(chase_recording_requested=True),
        now_ms=10,
    )
    merged = mgr.merge_recording_feedback(writer.tick(life, now_ms=10))

    assert merged.recording.active is False
    assert merged.recording.degraded_reason == "encoder_unavailable"
    assert merged.capture_active is False
    assert store.active_path() is None
    assert not [p for p in os.listdir(tmp_path) if p.endswith(".part")]


def test_rec_05_worker_thread_keeps_disk_io_off_the_control_tick(tmp_path):
    """A slow SD card must not stall the thread that applies motor commands."""

    class _SlowStore(RecordingStore):
        def append_bytes(self, path, payload):
            time.sleep(0.5)
            return super().append_bytes(path, payload)

    store = _SlowStore(str(tmp_path))
    writer = RecordingWriter(store, encoder=StubH264Encoder())
    writer.start()
    try:
        started = time.perf_counter()
        writer.tick(_lifecycle_requesting(), now_ms=1000)
        elapsed = time.perf_counter() - started
        assert elapsed < 0.1

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if writer.runtime_state().bytes_written > 0:
                break
            time.sleep(0.02)
        assert writer.runtime_state().bytes_written > 0
    finally:
        writer.stop(timeout=5.0)


def test_rec_05_dead_worker_is_reported_instead_of_freezing_state(tmp_path):
    store = RecordingStore(str(tmp_path))
    writer = RecordingWriter(store, encoder=StubH264Encoder())
    writer.start()
    # Simulate the I/O thread dying without going through stop().
    writer._worker_stop.set()
    writer._worker.join(timeout=2.0)

    feedback = writer.tick(_lifecycle_requesting(), now_ms=1000)

    assert feedback.active is False
    assert feedback.degraded_reason == "writer_stopped"


def test_rec_05_stop_finalizes_the_active_segment(tmp_path):
    store = RecordingStore(str(tmp_path))
    writer = RecordingWriter(store, encoder=StubH264Encoder())
    writer.start()
    writer.tick(_lifecycle_requesting(), now_ms=1000)

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and store.active_path() is None:
        time.sleep(0.02)
    assert store.active_path() is not None

    writer.stop(timeout=5.0)

    assert store.active_path() is None
    assert store.list_finalized()


def test_str_01_stream_clients_independent_of_recording():
    mgr = PerceptionLifecycleManager(TargetRuntimeConfig())
    mgr.register_stream_client()
    life = mgr.tick(
        fsm_state=FsmState.IDLE,
        mission=MissionState(),
        context=LifecycleMissionContext(),
        now_ms=1,
    )
    assert life.recording.requested is False
    assert life.stream_active_clients == 1
    assert life.stream_encoder_ready is True
    assert life.capture_active is True

    forced = mgr.tick(
        fsm_state=FsmState.HOME,
        mission=MissionState(),
        context=LifecycleMissionContext(),
        now_ms=2,
    )
    assert forced.stream_forced_off is True
    assert forced.stream_active_clients == 0
    assert forced.stream_encoder_ready is False
