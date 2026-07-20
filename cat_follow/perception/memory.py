"""Memory reclamation helpers for the perception pipeline.

When a heavy interpreter (TFLite/XNNPACK or RKNN) is unloaded during idle
periods, Python's garbage collector reclaims the objects but glibc may keep
the freed arena mapped.  Calling ``malloc_trim(0)`` returns that arena to the
OS, which matters on a memory-constrained SBC running alongside ROS2/Nav2.

All calls are best-effort and safe to invoke on any platform (the
``malloc_trim`` binding simply no-ops where libc is unavailable).
"""

from __future__ import annotations

import ctypes
import ctypes.util
import gc

_libc = None
_malloc_trim = None


def _resolve_malloc_trim():
    global _libc, _malloc_trim
    if _malloc_trim is not None:
        return _malloc_trim
    try:
        name = ctypes.util.find_library("c") or "libc.so.6"
        _libc = ctypes.CDLL(name)
        _malloc_trim = _libc.malloc_trim
        _malloc_trim.argtypes = [ctypes.c_size_t]
        _malloc_trim.restype = ctypes.c_int
    except Exception:  # pragma: no cover - non-glibc platforms
        _malloc_trim = None
    return _malloc_trim


def reclaim_memory() -> None:
    """Run a full GC pass and, on glibc, trim the malloc arena."""
    gc.collect()
    trim = _resolve_malloc_trim()
    if trim is not None:
        try:
            trim(0)
        except Exception:  # pragma: no cover
            pass


__all__ = ["reclaim_memory"]
