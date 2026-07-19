"""
Control API: send target and stop.

Routes:
    POST /api/target — Set cat target (x, y) in meters
    POST /api/stop   — Stop command
"""

from flask import Blueprint, request, jsonify

from cat_follow.logger import get_logger
from cat_follow.commands import set_cat_location, set_stop_command

control_bp = Blueprint("control", __name__)
_log = get_logger("web_ui.control")

_ctx = None


def init_control_routes(ctx):
    """Register control routes. ctx not used (commands are module-level)."""
    global _ctx
    _ctx = ctx

    @control_bp.route("/api/target", methods=["POST"])
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

    @control_bp.route("/api/stop", methods=["POST"])
    def api_stop():
        set_stop_command()
        _log.info("API stop received")
        return jsonify({"status": "ok"})
