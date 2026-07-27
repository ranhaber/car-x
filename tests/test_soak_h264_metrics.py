"""Tests for shared H.264 soak metric helpers."""

from scripts.soak_h264_metrics import (
    capture_stats,
    contiguous_generations,
    count_requeue_errors,
    parse_detect_perf,
)


def test_parse_detect_perf_and_contiguity():
    text = """
2026-07-26 19:07:11 [I] [DETECT-PERF] gen=10 capture=32.00ms queue=0.50ms
2026-07-26 19:07:11 [I] [DETECT-PERF] gen=11 capture=33.00ms queue=0.60ms
2026-07-26 19:07:11 [I] [DETECT-PERF] gen=12 capture=34.00ms queue=0.70ms
"""
    rows = parse_detect_perf(text)
    assert len(rows) == 3
    assert contiguous_generations(rows) is True
    stats = capture_stats(rows)
    assert stats["stalls_ge_50ms"] == 0
    assert stats["capture_max_ms"] == 34.0


def test_contiguous_generations_rejects_skips():
    rows = [(10, 32.0, 0.5), (12, 33.0, 0.6)]
    assert contiguous_generations(rows) is False


def test_count_requeue_errors():
    lines = [
        "camera ok",
        "dmabuf requeue failed for V4L2 buffer index 2",
    ]
    assert count_requeue_errors(lines) == 1
