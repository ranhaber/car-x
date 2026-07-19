"""Runtime layer for the contract-driven control architecture.

This package keeps its ``__init__`` empty on purpose so importing the
``cat_follow.runtime`` package doesn't pull in the whole import graph.
That avoids circular imports between ``cat_follow.control.fsm`` and
``cat_follow.runtime.shared_state``.

Import the modules you need explicitly, e.g.::

    from cat_follow.runtime.shared_state import SharedState, now_monotonic_ms
    from cat_follow.runtime.control_loop import ControlLoop
    from cat_follow.runtime.app import build_app
"""
