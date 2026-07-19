"""
Stream configuration API: resolution selection.

Routes:
    POST /api/stream/resolution — Set stream resolution (640x480, 320x240, 160x120)
"""

from flask import Blueprint, request, jsonify

from cat_follow.logger import get_logger

stream_config_bp = Blueprint("stream_config", __name__)
_log = get_logger("web_ui.stream_config")

_ctx = None


def init_stream_config_routes(ctx):
    """Register stream config routes. ctx: get_stream_resolution, set_stream_resolution, resolution_lock, resolution_options."""
    global _ctx
    _ctx = ctx

    @stream_config_bp.route("/api/stream/resolution", methods=["POST"])
    def api_stream_resolution():
        data = request.get_json(silent=True) or {}
        res = data.get("resolution", "")
        if res not in _ctx.resolution_options:
            return jsonify({
                "error": f"Invalid resolution. Choose from: {list(_ctx.resolution_options.keys())}"
            }), 400
        _ctx.set_stream_resolution(res)
        _log.info("API stream resolution changed to %s", res)
        return jsonify({"status": "ok", "resolution": res})
