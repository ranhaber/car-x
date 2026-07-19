"""
Status API: system and odometry status.

Routes:
    GET /api/status — State, odometry, bbox, ultrasonic, FPS, CPU, battery, etc.
"""

from flask import Blueprint, jsonify

from cat_follow import __version__
from cat_follow import range_sensor

status_bp = Blueprint("status", __name__)

_ctx = None


def init_status_routes(ctx):
    """Register status routes. ctx: shared, state_machine, get_tracker_fps, get_stream_fps, get_cpu_percent, get_ram_percent, get_cpu_temp, get_battery_voltage."""
    global _ctx
    _ctx = ctx

    @status_bp.route("/api/status")
    def api_status():
        odom = _ctx.shared.get_odometry() if _ctx.shared else (0, 0, 0)
        bbox = _ctx.shared.get_bbox_tracker() if _ctx.shared else (0, 0, 0, 0, 0)
        state_name = "unknown"
        if _ctx.state_machine is not None:
            state_name = _ctx.state_machine.state.value

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
            "tracker_fps": round(_ctx.get_tracker_fps(), 1),
            "stream_fps": round(_ctx.get_stream_fps(), 1),
            "app_version": __version__,
            "cpu_percent": round(_ctx.get_cpu_percent(), 1),
            "ram_percent": round(_ctx.get_ram_percent(), 1),
            "cpu_temp": round(_ctx.get_cpu_temp(), 1),
            "battery_v": _ctx.get_battery_voltage(),
        })
