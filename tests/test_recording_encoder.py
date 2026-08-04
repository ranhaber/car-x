"""Host tests for the hardware chase-recording encoder (GStreamer mocked).

The real pipeline needs GStreamer and ``mpph264enc`` on the ROCK 4D, so these
tests exercise the ownership and segmentation contract with a fake encoder and
leave pixel-level verification to the board checklist.
"""

from __future__ import annotations

import time

import pytest

from cat_follow.control.types import ConsumerState, PerceptionLifecycleState
from cat_follow.perception.recording_encoder import (
    MppRecordingEncoder,
    create_recording_encoder,
)
from cat_follow.perception.recording_store import RecordingStore
from cat_follow.perception.recording_writer import RecordingWriter


class _FakeLease:
    def __init__(self, frame_seq: int, *, dmabuf: bool = True):
        self.frame_seq = frame_seq
        self.dmabuf = dmabuf
        self.dmabuf_fd = 7 if dmabuf else -1
        self.dmabuf_buffer_index = frame_seq
        self.dmabuf_stride = 640
        self.dmabuf_size = 640 * 480 * 3 // 2
        self.frame = None
        self.released = False

    def release(self) -> None:
        self.released = True


class _FakeFrameSource:
    def __init__(self, leases):
        self._leases = list(leases)
        self.handed_out = []

    def acquire_latest_frame(self, *, consumer=None):
        if not self._leases:
            return None
        lease = self._leases.pop(0)
        self.handed_out.append(lease)
        return lease


class _FakeMpp:
    """Stands in for MppH264Encoder without GStreamer."""

    instances = []

    def __init__(self):
        self.started = False
        self.stopped = False
        self.finished = False
        self.submitted = []
        self.polls = 0
        type(self).instances.append(self)

    def start(self) -> bool:
        self.started = True
        return True

    def stop(self) -> None:
        self.stopped = True

    def poll(self, *, wait_ns: int = 0):  # noqa: ARG002
        self.polls += 1
        return []

    def finish(self, *, budget_ms: int = 500):  # noqa: ARG002
        self.finished = True
        return [b"mkv-trailer"]

    def submit_dmabuf(self, lease):
        self.submitted.append(lease)
        lease.release()
        return [b"mkv-cluster"]


@pytest.fixture(autouse=True)
def _reset_fake_instances():
    _FakeMpp.instances = []
    yield
    _FakeMpp.instances = []


def _encoder(frame_source):
    return MppRecordingEncoder(
        frame_source,
        width=640,
        height=480,
        encoder_factory=_FakeMpp,
    )


def test_encode_tick_submits_dmabuf_leases_without_copying_frames():
    source = _FakeFrameSource([_FakeLease(1), _FakeLease(2)])
    encoder = _encoder(source)
    assert encoder.start() is True
    assert encoder.begin_segment() is True

    assert encoder.encode_tick(now_ms=0) == [b"mkv-cluster"]
    assert encoder.encode_tick(now_ms=100) == [b"mkv-cluster"]

    mpp = _FakeMpp.instances[-1]
    assert [lease.frame_seq for lease in mpp.submitted] == [1, 2]
    # Ownership transferred to the encoder, which released each buffer.
    assert all(lease.released for lease in source.handed_out)


def test_repeated_capture_is_not_encoded_twice():
    repeated = _FakeLease(5)
    again = _FakeLease(5)
    source = _FakeFrameSource([repeated, again])
    encoder = _encoder(source)
    encoder.start()
    encoder.begin_segment()

    assert encoder.encode_tick(now_ms=0) == [b"mkv-cluster"]
    assert encoder.encode_tick(now_ms=100) == []

    assert again.released is True
    assert len(_FakeMpp.instances[-1].submitted) == 1


def test_numpy_only_lease_is_released_and_skipped():
    lease = _FakeLease(1, dmabuf=False)
    encoder = _encoder(_FakeFrameSource([lease]))
    encoder.start()
    encoder.begin_segment()

    assert encoder.encode_tick(now_ms=0) == []
    assert lease.released is True
    assert _FakeMpp.instances[-1].submitted == []


def test_each_segment_gets_a_fresh_muxer():
    """A rotated Matroska segment needs its own headers to be playable."""
    encoder = _encoder(_FakeFrameSource([]))
    encoder.start()

    assert encoder.begin_segment() is True
    first = _FakeMpp.instances[-1]
    assert encoder.end_segment() == [b"mkv-trailer"]
    assert first.finished is True
    assert first.stopped is True

    assert encoder.begin_segment() is True
    second = _FakeMpp.instances[-1]
    assert second is not first


def test_encode_tick_is_inert_outside_a_segment():
    encoder = _encoder(_FakeFrameSource([_FakeLease(1)]))
    encoder.start()

    assert encoder.encode_tick(now_ms=0) == []
    assert _FakeMpp.instances == []


def test_writer_writes_the_container_trailer_before_finalizing(tmp_path):
    store = RecordingStore(str(tmp_path))
    source = _FakeFrameSource([_FakeLease(i) for i in range(1, 6)])
    writer = RecordingWriter(
        store, encoder=_encoder(source), segment_duration_ms=1000
    )
    lifecycle = PerceptionLifecycleState(
        received_ms=1000,
        recording=ConsumerState(requested=True, active=True),
        capture_active=True,
    )

    writer.tick(lifecycle, now_ms=1000)
    active = store.active_path()
    assert active is not None

    stopped = PerceptionLifecycleState(
        received_ms=2000,
        recording=ConsumerState(requested=False, active=False),
    )
    writer.tick(stopped, now_ms=2000)

    finalized = store.list_finalized()
    assert finalized
    with open(finalized[0].path, "rb") as handle:
        payload = handle.read()
    assert payload.startswith(b"mkv-cluster")
    assert payload.endswith(b"mkv-trailer")


def test_rotation_closes_and_reopens_the_container(tmp_path):
    store = RecordingStore(str(tmp_path))
    source = _FakeFrameSource([_FakeLease(i) for i in range(1, 20)])
    writer = RecordingWriter(
        store, encoder=_encoder(source), segment_duration_ms=1000
    )
    lifecycle = PerceptionLifecycleState(
        received_ms=1000,
        recording=ConsumerState(requested=True, active=True),
        capture_active=True,
    )

    writer.tick(lifecycle, now_ms=1000)
    writer.tick(lifecycle, now_ms=2100)

    assert writer.runtime_state().segments_finalized == 1
    finalized = store.list_finalized()
    with open(finalized[0].path, "rb") as handle:
        assert handle.read().endswith(b"mkv-trailer")
    # The new segment is muxed by a fresh instance, so it carries its own
    # Matroska headers.
    assert len(_FakeMpp.instances) == 2


def test_factory_refuses_to_fake_recordings_without_an_override(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_RECORDING_ALLOW_STUB", raising=False)
    monkeypatch.setattr(MppRecordingEncoder, "available", staticmethod(lambda: False))

    assert create_recording_encoder(object(), width=640, height=480) is None


def test_factory_allows_the_stub_only_when_explicitly_enabled(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_RECORDING_ALLOW_STUB", "1")
    monkeypatch.setattr(MppRecordingEncoder, "available", staticmethod(lambda: False))

    encoder = create_recording_encoder(object(), width=640, height=480)

    assert encoder is not None
    assert encoder.__class__.__name__ == "StubH264Encoder"


def test_factory_prefers_hardware_when_available(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_RECORDING_ALLOW_STUB", "1")
    monkeypatch.setattr(MppRecordingEncoder, "available", staticmethod(lambda: True))

    encoder = create_recording_encoder(
        _FakeFrameSource([]), width=640, height=480
    )

    assert isinstance(encoder, MppRecordingEncoder)


def test_factory_without_a_frame_source_has_no_hardware_path(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_RECORDING_ALLOW_STUB", raising=False)
    monkeypatch.setattr(MppRecordingEncoder, "available", staticmethod(lambda: True))

    assert create_recording_encoder(None, width=640, height=480) is None


def test_recording_survives_a_writer_restart(tmp_path):
    """Recording must not depend on stream clients or prior encoder state."""
    store = RecordingStore(str(tmp_path))
    source = _FakeFrameSource([_FakeLease(i) for i in range(1, 10)])
    encoder = _encoder(source)
    writer = RecordingWriter(store, encoder=encoder, segment_duration_ms=60_000)
    lifecycle = PerceptionLifecycleState(
        received_ms=1000,
        recording=ConsumerState(requested=True, active=True),
        capture_active=True,
    )

    writer.start()
    try:
        writer.tick(lifecycle, now_ms=1000)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and store.active_path() is None:
            time.sleep(0.02)
        assert store.active_path() is not None
    finally:
        writer.stop(timeout=5.0)

    assert store.list_finalized()
    assert encoder._encoder is None
