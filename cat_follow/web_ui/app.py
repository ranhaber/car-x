"""
Flask application: Web UI for cat-follow.

Provides:
  - Main tab with MJPEG stream, Send target, Stop, status bar, resolution selector.
  - Calibration tab (stub for now).
  - API endpoints: /api/target, /api/stop, /api/status, /api/stream/resolution, /api/calibration.
"""

import os
import time
import threading
from typing import Optional

import numpy as np
from flask import Flask, Response, render_template, request, jsonify

from cat_follow import __version__
from cat_follow.logger import get_logger
from cat_follow.commands import set_cat_location, set_stop_command
from cat_follow import range_sensor
from cat_follow.motion.calibration_routines import run_speed_test, run_steer_test
from cat_follow.memory.shared_state import SharedState
from cat_follow.memory.pool import FRAME_SHAPE

_log = get_logger("web_ui")

# ---------------------------------------------------------------------------
# Stream resolution options
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


def set_tracker_fps(fps: float) -> None:
    """Called by the main loop or tracker thread to report current tracker FPS."""
    global _tracker_fps
    with _tracker_fps_lock:
        _tracker_fps = fps


def get_tracker_fps() -> float:
    with _tracker_fps_lock:
        return _tracker_fps


# ---------------------------------------------------------------------------
# System metrics helpers
# ---------------------------------------------------------------------------
_psutil_failed = False

def _get_cpu_percent() -> float:
    """Return CPU usage percent (simple /proc/stat or fallback)."""
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
    """Read CPU temperature (Linux /sys/thermal or fallback)."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return -1.0


_robot_hat_failed = False

def _get_battery_voltage() -> float:
    """Read battery voltage from robot_hat (ADC A4) if available."""
    global _robot_hat_failed
    try:
        from robot_hat import utils
        return round(utils.get_battery_voltage(), 2)
    except Exception as e:
        if not _robot_hat_failed:
            _log.warning("Could not get battery voltage. Is 'robot_hat' installed and working? Error: %s", e)
            _robot_hat_failed = True
        return -1.0


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
# Module-level refs set by create_app; used by routes and stream generator.
_shared: Optional[SharedState] = None
_state_machine = None
_calibration = None
_picarx = None


def create_app(
    shared: SharedState,
    state_machine=None,
    calibration=None,
    picarx=None,
) -> Flask:
    """Create and configure the Flask application.

    Parameters
    ----------
    shared : SharedState
        Thread-safe wrapper around the pre-allocated memory pool.
    state_machine : StateMachine, optional
        The state machine instance (for status reporting).
    calibration : Calibration, optional
        The calibration object instance.
    picarx : Picarx, optional
        The Picarx hardware instance.
    """
    global _shared, _state_machine, _calibration, _picarx
    _shared = shared
    _state_machine = state_machine
    _calibration = calibration
    _picarx = picarx

    template_dir = os.path.join(os.path.dirname(__file__), "templates")
    static_dir = os.path.join(os.path.dirname(__file__), "static")
    app = Flask(
        __name__,
        template_folder=template_dir,
        static_folder=static_dir,
    )

    # ------------------------------------------------------------------
    # Pages
    # ------------------------------------------------------------------
    @app.route("/")
    def index():
        return render_template("main.html", version=__version__)

    @app.route("/calibration")
    def calibration_page():
        """Serve the Calibration tab page."""
        return render_template("calibration.html", version=__version__)

    # ------------------------------------------------------------------
    # MJPEG stream
    # ------------------------------------------------------------------
    @app.route("/stream")
    def stream():
        return Response(
            _generate_mjpeg(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    # ------------------------------------------------------------------
    # API: target and stop
    # ------------------------------------------------------------------
    @app.route("/api/target", methods=["POST"])
    def api_target():
        data = request.get_json(silent=True) or {}
        try:
            x = float(data["x"])
            y = float(data["y"])
        except (KeyError, TypeError, ValueError):
            return jsonify({"error": "Need JSON with x and y (meters)"}), 400
        set_cat_location(x, y)
        _log.info("API target received: (%.2f, %.2f) meters", x, y)
        return jsonify({"status": "ok", "x": x, "y": y})

    @app.route("/api/stop", methods=["POST"])
    def api_stop():
        set_stop_command()
        _log.info("API stop received")
        return jsonify({"status": "ok"})

    # ------------------------------------------------------------------
    # API: status
    # ------------------------------------------------------------------
    @app.route("/api/status")
    def api_status():
        odom = _shared.get_odometry() if _shared else (0, 0, 0)
        bbox = _shared.get_bbox_tracker() if _shared else (0, 0, 0, 0, 0)
        state_name = "unknown"
        if _state_machine is not None:
            state_name = _state_machine.state.value

        ultrasonic_cm = range_sensor.get_last_distance_cm()
        return jsonify({
            "state": state_name,
            "odometry": {"x": odom[0], "y": odom[1], "heading_deg": odom[2]},
            "bbox_tracker": {
                "x": bbox[0], "y": bbox[1],
                "w": bbox[2], "h": bbox[3],
                "valid": bbox[4],
            },
            "ultrasonic_cm": round(ultrasonic_cm, 1) if ultrasonic_cm is not None else None,
            "tracker_fps": round(get_tracker_fps(), 1),
            "stream_fps": round(_stream_fps, 1),
            "app_version": __version__,
            "cpu_percent": round(_get_cpu_percent(), 1),
            "ram_percent": round(_get_ram_percent(), 1),
            "cpu_temp": round(_get_cpu_temp(), 1),
            "battery_v": _get_battery_voltage(),
        })

    # ------------------------------------------------------------------
    # API: stream resolution
    # ------------------------------------------------------------------
    @app.route("/api/stream/resolution", methods=["POST"])
    def api_stream_resolution():
        global _stream_resolution
        data = request.get_json(silent=True) or {}
        res = data.get("resolution", "")
        if res not in RESOLUTION_OPTIONS:
            return jsonify({
                "error": f"Invalid resolution. Choose from: {list(RESOLUTION_OPTIONS.keys())}"
            }), 400
        with _stream_resolution_lock:
            _stream_resolution = res
        _log.info("API stream resolution changed to %s", res)
        return jsonify({"status": "ok", "resolution": res})

    # ------------------------------------------------------------------
    # API: detector model selection
    # ------------------------------------------------------------------
    DETECTOR_OPTIONS = {
        "ssd_mobilenet_v2": "SSD MobileNet V2 (320x320, quantized)",
        "efficientdet_lite0": "EfficientDet-Lite0",
    }

    @app.route("/api/detector_model", methods=["GET"])
    def api_detector_model_get():
        current = _shared.get_detector_model() if _shared else None
        return jsonify({
            "current": current,
            "options": DETECTOR_OPTIONS,
        })

    @app.route("/api/detector_model", methods=["POST"])
    def api_detector_model_post():
        data = request.get_json(silent=True) or {}
        choice = data.get("model")
        if choice not in DETECTOR_OPTIONS:
            return jsonify({"error": f"Invalid model. Choose from: {list(DETECTOR_OPTIONS.keys())}"}), 400
        _shared.set_detector_model(choice)
        _log.info("Detector model changed to %s", choice)
        return jsonify({"status": "ok", "model": choice})

    # ------------------------------------------------------------------
    # API: calibration
    # ------------------------------------------------------------------
    @app.route("/api/calibration", methods=["GET"])
    def get_calibration():
        if not _calibration:
            return jsonify({"error": "Calibration not initialized"}), 500
        return jsonify(_calibration.get_all_calibration_data())

    @app.route("/api/calibration", methods=["POST"])
    def save_calibration():
        if not _calibration:
            return jsonify({"error": "Calibration not initialized"}), 500
        data = request.json
        if not data:
            return jsonify({"error": "Invalid JSON"}), 400
        _calibration.set_all_calibration_data(data)
        _calibration.save()
        return jsonify({"status": "ok", "message": "Calibration saved."})

    @app.route('/api/calibrate/run_speed', methods=['POST'])
    def api_run_speed_test():
        if not _picarx:
            return jsonify({"error": "Picarx not initialized"}), 500
        data = request.json or {}
        speed = int(data.get('speed', 30))
        duration = float(data.get('duration', 1.0))
        threading.Thread(target=run_speed_test, args=(_picarx, speed, duration)).start()
        return jsonify({"status": "ok", "message": f"Running speed test at speed {speed}."})

    @app.route('/api/calibrate/run_steer', methods=['POST'])
    def api_run_steer_test():
        if not _picarx:
            return jsonify({"error": "Picarx not initialized"}), 500
        data = request.json or {}
        angle = int(data.get('angle', 0))
        speed = int(data.get('speed', 30))
        duration = float(data.get('duration', 4.0))
        if not -40 < angle < 40:
             return jsonify({"error": "Angle must be between -40 and 40"}), 400
        threading.Thread(
            target=run_steer_test, args=(_picarx, angle, speed, duration)
        ).start()
        return jsonify({"status": "ok", "message": f"Running steer test with angle {angle}."})

    return app


# ---------------------------------------------------------------------------
# MJPEG generator
# ---------------------------------------------------------------------------
def _generate_mjpeg():
    """Yield MJPEG frames at ~10 FPS with bbox rectangle and state overlay."""
    global _stream_fps

    # Try to import cv2 for drawing and encoding
    try:
        import cv2
        _has_cv2 = True
    except ImportError:
        _has_cv2 = False

    # Pre-allocate one frame buffer for reading (no per-frame alloc)
    frame_buf = np.empty(FRAME_SHAPE, dtype=np.uint8)
    target_fps = 10.0
    tick = 1.0 / target_fps
    fps_counter = 0
    fps_timer = time.monotonic()

    while True:
        t0 = time.monotonic()

        if _shared is None:
            time.sleep(tick)
            continue

        # Read current frame and bbox
        _shared.get_frame_latest(frame_buf)
        bbox = _shared.get_bbox_tracker()
        state_name = "unknown"
        if _state_machine is not None:
            state_name = _state_machine.state.value

        # Get current resolution
        with _stream_resolution_lock:
            res_key = _stream_resolution
        target_w, target_h = RESOLUTION_OPTIONS[res_key]

        if _has_cv2:
            # Work on a copy so we don't modify the shared buffer
            display = frame_buf.copy()

            # Draw bbox rectangle if valid
            if bbox[4] > 0:
                x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
                cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
                label = f"cat ({w}x{h})"
                cv2.putText(display, label, (x, max(y - 8, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Draw state text overlay
            cv2.putText(display, f"State: {state_name}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Resize if needed
            src_h, src_w = display.shape[:2]
            if (target_w, target_h) != (src_w, src_h):
                display = cv2.resize(display, (target_w, target_h),
                                     interpolation=cv2.INTER_AREA)

            # Encode to JPEG
            _, jpeg = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_bytes = jpeg.tobytes()
        else:
            # Fallback: raw gray placeholder (no cv2)
            frame_bytes = b"\xff\xd8\xff\xe0" + b"\x00" * 100  # minimal

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )

        # FPS tracking
        fps_counter += 1
        now = time.monotonic()
        if now - fps_timer >= 1.0:
            _stream_fps = fps_counter / (now - fps_timer)
            fps_counter = 0
            fps_timer = now

        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, tick - elapsed))
