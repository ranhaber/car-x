"""Shared monotonic timeline for perception performance logs."""

from __future__ import annotations

import time

_PERF_ORIGIN = time.perf_counter()


def perf_ms_from(value: float) -> float:
    """Convert a captured ``perf_counter`` value to milliseconds since import."""
    return (value - _PERF_ORIGIN) * 1000.0


def perf_now_ms() -> float:
    """Return milliseconds on the process-wide perception timeline."""
    return (time.perf_counter() - _PERF_ORIGIN) * 1000.0


__all__ = ["perf_ms_from", "perf_now_ms"]
