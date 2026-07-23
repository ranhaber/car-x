"""Thread-safe web command sequence allocator.

Flask runs with ``threaded=True``; compound ``seq += 1`` on a shared dict or
module global is a race.  All web-originated contract command sequences go
through :func:`next_web_command_seq`.
"""

from __future__ import annotations

import threading
from typing import Any

_SEQ_LOCK = threading.Lock()


def next_web_command_seq(ctx: Any) -> int:
    """Atomically increment and return the next web command sequence id."""

    with _SEQ_LOCK:
        seq = getattr(ctx, "web_command_seq", None)
        if not isinstance(seq, dict):
            seq = {"value": 0}
            try:
                ctx.web_command_seq = seq
            except Exception:  # noqa: BLE001
                pass
        seq["value"] = int(seq.get("value", 0)) + 1
        return int(seq["value"])


__all__ = ["next_web_command_seq"]
