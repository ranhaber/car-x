"""
Flask application: Web UI for cat-follow.

Route modules (Blueprints), same pattern as cat_ball_tracker:
  - routes_pages.py      — GET /, GET /calibration
  - routes_h264.py       — WebSocket /ws/h264 (hardware H.264 + overlay JSON)
  - routes_control.py   — POST /api/target, POST /api/stop
  - routes_status.py    — GET /api/status
  - routes_stream_config.py — GET /api/stream/capabilities
  - routes_detector.py   — GET/POST /api/detector_model
  - routes_calibration.py — GET/POST /api/calibration, POST /api/calibrate/run_speed, run_steer
  - routes_config.py     — GET /api/config (read-only camera + perception)
"""

from __future__ import annotations

import os
import threading
from typing import Any, Optional

from flask import Flask

from cat_follow.logger import get_logger
from cat_follow.memory.shared_state import SharedState

_log = get_logger("web_ui")

# ---------------------------------------------------------------------------
# Stream resolution
# ---------------------------------------------------------------------------
RESOLUTION_OPTIONS = {
    "640x480": (640, 480),
    "320x240": (320, 240),
    "160x120": (160, 120),
}
_stream_resolution: str = "640x480"
_stream_resolution_lock = threading.Lock()

# ---------------------------------------------------------------------------
# FPS counters
# ---------------------------------------------------------------------------
_stream_fps: float = 0.0
_tracker_fps: float = 0.0
_tracker_fps_lock = threading.Lock()
_stream_fps_lock = threading.Lock()

# ---------------------------------------------------------------------------
# Stream client counter — lets the processing/encoding paths skip expensive
# annotation + JPEG/H.264 work when nobody is watching (headless efficiency).
# ---------------------------------------------------------------------------
_stream_clients: int = 0
_stream_clients_lock = threading.Lock()


def inc_stream_clients() -> int:
    global _stream_clients
    with _stream_clients_lock:
        _stream_clients += 1
        return _stream_clients


def dec_stream_clients() -> int:
    global _stream_clients
    with _stream_clients_lock:
        _stream_clients = max(0, _stream_clients - 1)
        return _stream_clients


def get_stream_clients() -> int:
    with _stream_clients_lock:
        return _stream_clients


def set_tracker_fps(fps: float) -> None:
    """Called by the main loop or tracker thread to report current tracker FPS."""
    global _tracker_fps
    with _tracker_fps_lock:
        _tracker_fps = fps


def get_tracker_fps() -> float:
    with _tracker_fps_lock:
        return _tracker_fps


def _get_stream_fps() -> float:
    with _stream_fps_lock:
        return _stream_fps


def _set_stream_fps(fps: float) -> None:
    global _stream_fps
    with _stream_fps_lock:
        _stream_fps = fps


def _get_stream_resolution() -> str:
    with _stream_resolution_lock:
        return _stream_resolution


def _set_stream_resolution(res: str) -> None:
    global _stream_resolution
    with _stream_resolution_lock:
        _stream_resolution = res


# ---------------------------------------------------------------------------
# System metrics
# ---------------------------------------------------------------------------
_psutil_failed = False


def _get_cpu_percent() -> float:
    global _psutil_failed
    try:
        import psutil
        return psutil.cpu_percent(interval=0)
    except Exception as e:
        if not _psutil_failed:
            _log.warning("Could not get CPU/RAM stats. Is 'psutil' installed? Error: %s", e)
            _psutil_failed = True
        return -1.0


def _get_ram_percent() -> float:
    global _psutil_failed
    try:
        import psutil
        return psutil.virtual_memory().percent
    except Exception as e:
        if not _psutil_failed:
            _log.warning("Could not get CPU/RAM stats. Is 'psutil' installed? Error: %s", e)
            _psutil_failed = True
        return -1.0


def _get_cpu_temp() -> float:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return -1.0


_robot_hat_failed = False


def _get_battery_voltage() -> float:
    global _robot_hat_failed
    try:
        from robot_hat import utils
        return round(utils.get_battery_voltage(), 2)
    except Exception as e:
        if not _robot_hat_failed:
            _log.warning("Could not get battery voltage. Error: %s", e)
            _robot_hat_failed = True
        return -1.0


# ---------------------------------------------------------------------------
# App context and factory
# ---------------------------------------------------------------------------

class _AppContext:
    """Context passed to route inits (shared, state_machine, calibration, picarx + helpers)."""
    pass


def create_app(
    shared: SharedState,
    state_machine=None,
    calibration=None,
    picarx=None,
    *,
    runtime_shared: Any = None,
    comms_manager: Any = None,
    sequence_executor: Any = None,
    apply_safety_config: Any = None,
    target_runtime_config: Any = None,
    perception_lifecycle_manager: Any = None,
) -> Flask:
    """Create and configure the Flask application with Blueprint routes.

    Parameters
    ----------
    shared
        Prototype ``memory.SharedState`` used for frames / bbox / detector model.
    runtime_shared
        Optional contract ``runtime.SharedState`` (DecisionEngine / ROS / FSM).
    comms_manager
        Optional ``CommsManager`` for contract command routing from the UI.
    """
    ctx = _AppContext()
    ctx.shared = shared
    ctx.state_machine = state_machine
    ctx.calibration = calibration
    ctx.picarx = picarx
    ctx.runtime_shared = runtime_shared
    ctx.comms_manager = comms_manager
    ctx.apply_safety_config = apply_safety_config
    ctx.target_runtime_config = target_runtime_config
    ctx.perception_lifecycle_manager = perception_lifecycle_manager
    ctx.h264_available = False
    ctx.get_tracker_fps = get_tracker_fps
    ctx.get_stream_fps = _get_stream_fps
    ctx.set_stream_fps = _set_stream_fps
    ctx.get_stream_resolution = _get_stream_resolution
    ctx.set_stream_resolution = _set_stream_resolution
    ctx.resolution_lock = _stream_resolution_lock
    ctx.resolution_options = RESOLUTION_OPTIONS
    ctx.get_cpu_percent = _get_cpu_percent
    ctx.get_ram_percent = _get_ram_percent
    ctx.get_cpu_temp = _get_cpu_temp
    ctx.get_battery_voltage = _get_battery_voltage
    # The lifecycle manager owns the authoritative client count when present;
    # the module counter is the fallback. Mirrors are *assigned* that count
    # instead of counting independently, so they cannot drift apart.
    def _mirror_stream_clients(count: int) -> None:
        if hasattr(shared, "set_stream_clients"):
            shared.set_stream_clients(count)

    def _inc_both_stream_clients() -> int:
        plm = perception_lifecycle_manager
        if plm is not None:
            count = plm.register_stream_client()
        else:
            count = inc_stream_clients()
        _mirror_stream_clients(count)
        return count

    def _dec_both_stream_clients() -> int:
        plm = perception_lifecycle_manager
        if plm is not None:
            count = plm.unregister_stream_client()
        else:
            count = dec_stream_clients()
        _mirror_stream_clients(count)
        return count

    def _stream_forced_off() -> bool:
        plm = perception_lifecycle_manager
        if plm is not None:
            return bool(plm.last_state().stream_forced_off)
        if hasattr(shared, "stream_forced_off"):
            return bool(shared.stream_forced_off())
        return False

    def _get_both_stream_clients() -> int:
        plm = perception_lifecycle_manager
        if plm is not None:
            return plm.stream_clients
        if hasattr(shared, "get_stream_clients"):
            return shared.get_stream_clients()
        return get_stream_clients()

    ctx.inc_stream_clients = _inc_both_stream_clients
    ctx.dec_stream_clients = _dec_both_stream_clients
    ctx.get_stream_clients = _get_both_stream_clients
    ctx.stream_forced_off = _stream_forced_off
    # Serializes web-initiated direct hardware access (calibration motor tests)
    # so two routines cannot drive the shared Picarx concurrently.
    ctx.hardware_lock = threading.Lock()
    ctx.sequence_executor = None
    ctx.prototype_sequence_runner = None
    ctx.web_command_seq = {"value": 0}

    # Warn once if motion-causing endpoints are unauthenticated.
    from cat_follow.web_ui.auth import warn_if_unauthenticated

    warn_if_unauthenticated("0.0.0.0")

    template_dir = os.path.join(os.path.dirname(__file__), "templates")
    static_dir = os.path.join(os.path.dirname(__file__), "static")
    app = Flask(
        __name__,
        template_folder=template_dir,
        static_folder=static_dir,
    )

    from cat_follow.web_ui.routes_pages import pages_bp, init_pages_routes
    from cat_follow.web_ui.routes_control import control_bp, init_control_routes
    from cat_follow.web_ui.routes_status import status_bp, init_status_routes
    from cat_follow.web_ui.routes_stream_config import stream_config_bp, init_stream_config_routes
    from cat_follow.web_ui.routes_detector import detector_bp, init_detector_routes
    from cat_follow.web_ui.routes_calibration import calibration_bp, init_calibration_routes
    from cat_follow.web_ui.routes_config import config_bp, init_config_routes
    from cat_follow.web_ui.routes_map import map_bp, init_map_routes
    from cat_follow.web_ui.routes_movement import movement_bp, init_movement_routes
    from cat_follow.web_ui.routes_injection import injection_bp, init_injection_routes
    from cat_follow.motion.sequence_executor import MotionSequenceExecutor
    from cat_follow.motion.prototype_sequence_runner import PrototypeSequenceRunner

    ctx.sequence_executor = sequence_executor or MotionSequenceExecutor()
    if picarx is not None:
        ctx.prototype_sequence_runner = PrototypeSequenceRunner(
            picarx=picarx,
            hardware_lock=ctx.hardware_lock,
            executor=ctx.sequence_executor,
        )

    init_pages_routes()
    init_control_routes(ctx)
    init_status_routes(ctx)
    init_stream_config_routes(ctx)
    init_detector_routes(ctx)
    init_calibration_routes(ctx)
    init_config_routes(ctx)
    init_map_routes(ctx)
    init_movement_routes(ctx)
    init_injection_routes(ctx)

    # Hardware H.264 WebSocket stream. A production preference for H.264 must
    # not turn an optional monitoring dependency into a core-runtime startup
    # dependency; expose capabilities=false and keep the remaining UI alive.
    require_h264 = os.environ.get("CAT_FOLLOW_WEB_REQUIRE_H264", "1") == "1"
    try:
        from cat_follow.web_ui.routes_h264 import init_h264_routes

        ctx.h264_available = bool(init_h264_routes(ctx, app))
    except Exception as exc:  # noqa: BLE001
        _log.debug("H.264 route registration skipped: %s", exc)
        ctx.h264_available = False

    if require_h264 and not ctx.h264_available:
        _log.error(
            "Hardware H.264 stream is required (CAT_FOLLOW_WEB_REQUIRE_H264=1) "
            "but mpph264enc and/or flask-sock are unavailable on this host; "
            "continuing without monitoring video."
        )

    app.register_blueprint(pages_bp)
    app.register_blueprint(control_bp)
    app.register_blueprint(status_bp)
    app.register_blueprint(stream_config_bp)
    app.register_blueprint(detector_bp)
    app.register_blueprint(calibration_bp)
    app.register_blueprint(config_bp)
    app.register_blueprint(map_bp)
    app.register_blueprint(movement_bp)
    app.register_blueprint(injection_bp)

    return app
