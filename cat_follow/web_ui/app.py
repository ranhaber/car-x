"""
Flask application: Web UI for cat-follow.

Route modules (Blueprints), same pattern as cat_ball_tracker:
  - routes_pages.py      — GET /, GET /calibration
  - routes_streaming.py — GET /stream (MJPEG)
  - routes_control.py   — POST /api/target, POST /api/stop
  - routes_status.py    — GET /api/status
  - routes_stream_config.py — POST /api/stream/resolution, GET /api/stream/capabilities
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
    ctx.inc_stream_clients = inc_stream_clients
    ctx.dec_stream_clients = dec_stream_clients
    ctx.get_stream_clients = get_stream_clients

    template_dir = os.path.join(os.path.dirname(__file__), "templates")
    static_dir = os.path.join(os.path.dirname(__file__), "static")
    app = Flask(
        __name__,
        template_folder=template_dir,
        static_folder=static_dir,
    )

    from cat_follow.web_ui.routes_pages import pages_bp, init_pages_routes
    from cat_follow.web_ui.routes_streaming import streaming_bp, init_streaming_routes
    from cat_follow.web_ui.routes_control import control_bp, init_control_routes
    from cat_follow.web_ui.routes_status import status_bp, init_status_routes
    from cat_follow.web_ui.routes_stream_config import stream_config_bp, init_stream_config_routes
    from cat_follow.web_ui.routes_detector import detector_bp, init_detector_routes
    from cat_follow.web_ui.routes_calibration import calibration_bp, init_calibration_routes
    from cat_follow.web_ui.routes_config import config_bp, init_config_routes
    from cat_follow.web_ui.routes_map import map_bp, init_map_routes

    init_pages_routes()
    init_streaming_routes(ctx)
    init_control_routes(ctx)
    init_status_routes(ctx)
    init_stream_config_routes(ctx)
    init_detector_routes(ctx)
    init_calibration_routes(ctx)
    init_config_routes(ctx)
    init_map_routes(ctx)

    # Optional hardware H.264 WebSocket stream (guarded: no-op if flask-sock /
    # GStreamer mpph264enc are unavailable).
    try:
        from cat_follow.web_ui.routes_h264 import init_h264_routes

        ctx.h264_available = bool(init_h264_routes(ctx, app))
    except Exception as exc:  # noqa: BLE001
        _log.debug("H.264 route registration skipped: %s", exc)
        ctx.h264_available = False

    app.register_blueprint(pages_bp)
    app.register_blueprint(streaming_bp)
    app.register_blueprint(control_bp)
    app.register_blueprint(status_bp)
    app.register_blueprint(stream_config_bp)
    app.register_blueprint(detector_bp)
    app.register_blueprint(calibration_bp)
    app.register_blueprint(config_bp)
    app.register_blueprint(map_bp)

    return app
