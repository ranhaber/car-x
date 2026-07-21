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

calibration_bp = Blueprint("calibration", __name__)
_log = get_logger("web_ui.calibration")

_ctx = None


def init_calibration_routes(ctx):
    """Bind calibration context."""
    global _ctx
    _ctx = ctx


@calibration_bp.route("/api/calibration", methods=["GET"])
def get_calibration():
    if not _ctx or not _ctx.calibration:
        return jsonify({"error": "Calibration not initialized"}), 500
    return jsonify(_ctx.calibration.get_all_calibration_data())


@calibration_bp.route("/api/calibration", methods=["POST"])
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
def api_run_speed_test():
    if not _ctx or not _ctx.picarx:
        return jsonify({"error": "Picarx not initialized"}), 500
    data = request.json or {}
    speed = int(data.get("speed", 30))
    duration = float(data.get("duration", 1.0))
    threading.Thread(target=run_speed_test, args=(_ctx.picarx, speed, duration)).start()
    return jsonify({"status": "ok", "message": f"Running speed test at speed {speed}."})


@calibration_bp.route("/api/calibrate/run_steer", methods=["POST"])
def api_run_steer_test():
    if not _ctx or not _ctx.picarx:
        return jsonify({"error": "Picarx not initialized"}), 500
    data = request.json or {}
    angle = int(data.get("angle", 0))
    speed = int(data.get("speed", 30))
    duration = float(data.get("duration", 4.0))
    if not -40 < angle < 40:
        return jsonify({"error": "Angle must be between -40 and 40"}), 400
    threading.Thread(
        target=run_steer_test, args=(_ctx.picarx, angle, speed, duration)
    ).start()
    return jsonify({"status": "ok", "message": f"Running steer test with angle {angle}."})
