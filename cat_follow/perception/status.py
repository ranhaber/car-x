"""Thread-safe perception diagnostics published for the web UI / monitoring.

The detector thread writes here every tick; ``/api/status`` reads a snapshot.
This keeps the UI from racing detector internals while remaining independent
of the control ``SharedState`` contract groups.
"""

from __future__ import annotations

import threading
from dataclasses import asdict, dataclass
from typing import Any, Dict


@dataclass(frozen=True)
class PerceptionDiagnostics:
    phase: str = "IDLE"
    backend: str = "tflite"
    model_loaded: bool = False
    lores_active: bool = False
    motion: bool = False
    motion_gating: bool = True


_lock = threading.Lock()
_current = PerceptionDiagnostics()


def update_perception_diagnostics(
    *,
    phase: str | None = None,
    backend: str | None = None,
    model_loaded: bool | None = None,
    lores_active: bool | None = None,
    motion: bool | None = None,
    motion_gating: bool | None = None,
) -> None:
    """Merge non-None fields into the published diagnostics snapshot."""
    global _current
    with _lock:
        cur = _current
        _current = PerceptionDiagnostics(
            phase=cur.phase if phase is None else str(phase),
            backend=cur.backend if backend is None else str(backend),
            model_loaded=(
                cur.model_loaded if model_loaded is None else bool(model_loaded)
            ),
            lores_active=(
                cur.lores_active if lores_active is None else bool(lores_active)
            ),
            motion=cur.motion if motion is None else bool(motion),
            motion_gating=(
                cur.motion_gating if motion_gating is None else bool(motion_gating)
            ),
        )


def get_perception_diagnostics() -> PerceptionDiagnostics:
    with _lock:
        return _current


def perception_diagnostics_dict() -> Dict[str, Any]:
    return asdict(get_perception_diagnostics())
