"""
Command interface: cat_location (x_cm, y_cm) in centimeters and stop_command.

Thread-safe (Option B): all access to the pending globals is protected
by a single lock so the Flask request-handler thread and the main loop
can safely write and read without torn values.
"""

import threading
from typing import Optional, Tuple, Callable

_lock = threading.Lock()
_pending_cat_location_cm: Optional[Tuple[float, float]] = None
_pending_stop: bool = False


def set_cat_location_cm(x_cm: float, y_cm: float) -> None:
    """Queue a cat location in centimeters (e.g. from Web UI or test)."""
    global _pending_cat_location_cm
    with _lock:
        _pending_cat_location_cm = (float(x_cm), float(y_cm))


def set_stop_command() -> None:
    """Queue stop (e.g. from Web UI or test)."""
    global _pending_stop
    with _lock:
        _pending_stop = True


def poll_commands(
    on_cat_location_cm: Optional[Callable[[float, float], None]] = None,
    on_stop: Optional[Callable[[], None]] = None,
) -> None:
    """Poll once; if pending cat_location or stop, call the callback and clear.

    Main loop calls this each tick.  The lock ensures we never see a
    partially-written cat_location or a missed stop flag.
    """
    global _pending_cat_location_cm, _pending_stop

    cat_loc_cm = None
    do_stop = False

    with _lock:
        if _pending_cat_location_cm is not None:
            cat_loc_cm = _pending_cat_location_cm
            _pending_cat_location_cm = None
        if _pending_stop:
            do_stop = True
            _pending_stop = False

    # Fire callbacks outside the lock to avoid holding it during
    # potentially slow state-machine dispatch or motion calls.
    if cat_loc_cm is not None and on_cat_location_cm:
        on_cat_location_cm(cat_loc_cm[0], cat_loc_cm[1])
    if do_stop and on_stop:
        on_stop()


def read_cat_location_cm_from_file(path: str) -> Optional[Tuple[float, float]]:
    """Optional: read 'x_cm y_cm' from file (one line). None if missing/invalid."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            line = f.readline().strip()
        parts = line.split()
        if len(parts) >= 2:
            return (float(parts[0]), float(parts[1]))
    except (FileNotFoundError, ValueError):
        pass
    return None
