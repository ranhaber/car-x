"""RecordingStore quota, recovery, and active-segment safety (REC-01/02)."""

import os
from types import SimpleNamespace

from cat_follow.perception.recording_store import RecordingStore, SegmentRecord


def test_rec_01_recover_incomplete_part_files(tmp_path):
    store = RecordingStore(str(tmp_path))
    part = tmp_path / "crash.mkv.part"
    part.write_bytes(b"\x00\x00\x00\x01partial")
    empty = tmp_path / "empty.mkv.part"
    empty.write_bytes(b"")

    recovered = store.recover_incomplete()

    assert recovered == [str(tmp_path / "crash.mkv")]
    assert (tmp_path / "crash.mkv").is_file()
    assert not part.exists()
    assert not empty.exists()
    assert store.total_finalized_bytes() == len(b"\x00\x00\x00\x01partial")


def test_rec_02_quota_deletes_oldest_finalized_never_active(tmp_path):
    store = RecordingStore(str(tmp_path), quota_bytes=50)
    finalized = []
    for i, size in enumerate((30, 30, 30)):
        part = store.begin_segment(wall_clock_ms=1_000 * (i + 1))
        store.append_bytes(part, bytes([i + 1]) * size)
        finalized.append(store.finalize_segment(part))

    assert not os.path.exists(finalized[0])
    assert not os.path.exists(finalized[1])
    assert os.path.exists(finalized[2])

    active = store.begin_segment(wall_clock_ms=9_000)
    store.append_bytes(active, b"LIVE")
    # Plant an older oversized finalized segment so retention must run while
    # an active .part exists.
    older = tmp_path / "older.mkv"
    older.write_bytes(b"o" * 40)
    store._append_index(
        SegmentRecord(
            path=str(older),
            bytes=40,
            finalized_at_ms=1,
        )
    )
    deleted = store.enforce_retention()
    assert str(older) in deleted
    assert os.path.exists(active)
    assert open(active, "rb").read() == b"LIVE"
    assert store.active_path() == active


def test_rec_02_quota_pressure_includes_the_active_segment(tmp_path):
    store = RecordingStore(str(tmp_path), quota_bytes=100)
    part = store.begin_segment(wall_clock_ms=1_000)
    store.append_bytes(part, b"x" * 40)

    assert store.over_quota() is False
    assert store.active_segment_over_quota() is False

    store.append_bytes(part, b"x" * 80)

    # Retention cannot delete the in-progress segment, so the writer needs both
    # signals: total pressure and "this segment alone breaches the quota".
    assert store.over_quota() is True
    assert store.active_segment_over_quota() is True
    assert store.enforce_retention() == []


def test_rec_02_space_reserve_blocks_new_segment(tmp_path):
    store = RecordingStore(
        str(tmp_path),
        min_free_bytes=10_000,
        disk_usage=lambda _path: SimpleNamespace(free=100),
    )
    assert store.space_available_for_new_segment() is False
    raised = False
    try:
        store.begin_segment(wall_clock_ms=1)
    except OSError:
        raised = True
    assert raised is True
