"""
Calibration API: get/save calibration data and run speed/steer tests.

Routes:
    GET  /api/calibration        — Get all calibration data
    POST /api/calibration        — Save calibration data
    POST /api/calibrate/run_speed  — Run speed test (distance calibration)
    POST /api/calibrate/run_steer  — Run steer test (radius calibration)
"""

import threading
from flask import Blueprint, request, jsonify

from cat_follow.logger import get_logger
from cat_follow.motion.calibration_routines import run_speed_test, run_steer_test
from cat_follow.web_ui.auth import require_control_token

calibration_bp = Blueprint("calibration", __name__)
_log = get_logger("web_ui.calibration")

_ctx = None


def init_calibration_routes(ctx):
    """Bind calibration context."""
    global _ctx
    _ctx = ctx


def _run_arbitrated(target, args):
    """Run a calibration routine while holding the hardware arbiter lock.

    Prevents a calibration routine from driving the shared Picarx concurrently
    with another calibration routine (and, when wired, the autonomous control
    loop).  Returns True if the routine was started, False if the hardware is
    already in use.
    """
    lock = getattr(_ctx, "hardware_lock", None)
    if lock is None:
        threading.Thread(target=target, args=args, daemon=True).start()
        return True
    if not lock.acquire(blocking=False):
        return False

    def _wrapped():
        try:
            target(*args)
        finally:
            lock.release()

    threading.Thread(target=_wrapped, daemon=True).start()
    return True


@calibration_bp.route("/api/calibration", methods=["GET"])
def get_calibration():
    if not _ctx or not _ctx.calibration:
        return jsonify({"error": "Calibration not initialized"}), 500
    return jsonify(_ctx.calibration.get_all_calibration_data())


@calibration_bp.route("/api/calibration", methods=["POST"])
@require_control_token
def save_calibration():
    if not _ctx or not _ctx.calibration:
        return jsonify({"error": "Calibration not initialized"}), 500
    data = request.json
    if not data:
        return jsonify({"error": "Invalid JSON"}), 400
    _ctx.calibration.set_all_calibration_data(data)
    _ctx.calibration.save()
    return jsonify({"status": "ok", "message": "Calibration saved."})


@calibration_bp.route("/api/calibrate/run_speed", methods=["POST"])
@require_control_token
def api_run_speed_test():
    if not _ctx or not _ctx.picarx:
        return jsonify({"error": "Picarx not initialized"}), 500
    data = request.json or {}
    speed = int(data.get("speed", 30))
    duration = float(data.get("duration", 1.0))
    if not _run_arbitrated(run_speed_test, (_ctx.picarx, speed, duration)):
        return jsonify({"error": "hardware busy (another motion routine is running)"}), 409
    return jsonify({"status": "ok", "message": f"Running speed test at speed {speed}."})


@calibration_bp.route("/api/calibrate/run_steer", methods=["POST"])
@require_control_token
def api_run_steer_test():
    if not _ctx or not _ctx.picarx:
        return jsonify({"error": "Picarx not initialized"}), 500
    data = request.json or {}
    angle = int(data.get("angle", 0))
    speed = int(data.get("speed", 30))
    duration = float(data.get("duration", 4.0))
    if not -40 < angle < 40:
        return jsonify({"error": "Angle must be between -40 and 40"}), 400
    if not _run_arbitrated(run_steer_test, (_ctx.picarx, angle, speed, duration)):
        return jsonify({"error": "hardware busy (another motion routine is running)"}), 409
    return jsonify({"status": "ok", "message": f"Running steer test with angle {angle}."})
