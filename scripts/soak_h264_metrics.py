"""Shared metrics helpers for H.264 / DMA-BUF board soak scripts."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Iterable

DETECT_RE = re.compile(
    r"\[DETECT-PERF\] gen=(\d+) capture=([\d.]+)ms queue=([\d.]+)ms"
)
REQUEUE_RE = re.compile(
    r"(dmabuf requeue failed|requeue: buffer is not dequeued|VIDIOC_QBUF|QBUF failed)",
    re.IGNORECASE,
)


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = int(round((pct / 100.0) * (len(ordered) - 1)))
    return float(ordered[max(0, min(idx, len(ordered) - 1))])


def parse_detect_perf(text: str) -> list[tuple[int, float, float]]:
    rows: list[tuple[int, float, float]] = []
    for line in text.splitlines():
        match = DETECT_RE.search(line)
        if match:
            rows.append(
                (int(match.group(1)), float(match.group(2)), float(match.group(3)))
            )
    return rows


def parse_detect_perf_since(since_monotonic: float, log_dir: Path | None = None) -> list[tuple[int, float, float]]:
    root = log_dir or (Path.home() / "logs_car_x")
    if not root.is_dir():
        return []
    candidates = [
        path
        for path in root.glob("*.log")
        if path.stat().st_mtime >= since_monotonic - 1.0
    ]
    if not candidates:
        return []
    latest = max(candidates, key=lambda path: path.stat().st_mtime)
    return parse_detect_perf(latest.read_text(encoding="utf-8", errors="replace"))


def count_requeue_errors(lines: Iterable[str]) -> int:
    return sum(1 for line in lines if REQUEUE_RE.search(line))


def count_requeue_errors_since(since_monotonic: float, log_dir: Path | None = None) -> int:
    root = log_dir or (Path.home() / "logs_car_x")
    if not root.is_dir():
        return 0
    total = 0
    for path in root.glob("*.log"):
        if path.stat().st_mtime >= since_monotonic - 1.0:
            total += count_requeue_errors(
                path.read_text(encoding="utf-8", errors="replace").splitlines()
            )
    return total


def infer_span(rows: list[tuple[int, float, float]]) -> int:
    gens = [gen for gen, _, _ in rows]
    if not gens:
        return 0
    return max(gens) - min(gens) + 1


def contiguous_generations(rows: list[tuple[int, float, float]]) -> bool:
    gens = [gen for gen, _, _ in rows]
    if not gens:
        return False
    return len(set(gens)) == len(gens) and infer_span(rows) == len(gens)


def capture_stats(rows: list[tuple[int, float, float]]) -> dict[str, float | int]:
    captures = [capture for _, capture, _ in rows]
    return {
        "samples": len(captures),
        "capture_p95_ms": round(percentile(captures, 95.0), 2),
        "capture_max_ms": round(max(captures) if captures else 0.0, 2),
        "stalls_ge_50ms": sum(1 for value in captures if value >= 50.0),
    }
