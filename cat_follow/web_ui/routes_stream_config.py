"""
Stream configuration API: H.264 capabilities.

Routes:
    GET  /api/stream/capabilities — H.264 availability (required for web UI)
"""

from flask import Blueprint, jsonify

from cat_follow.logger import get_logger

stream_config_bp = Blueprint("stream_config", __name__)
_log = get_logger("web_ui.stream_config")

_ctx = None


def init_stream_config_routes(ctx):
    """Bind stream-config context."""
    global _ctx
    _ctx = ctx


@stream_config_bp.route("/api/stream/capabilities", methods=["GET"])
def api_stream_capabilities():
    return jsonify({
        "h264": bool(getattr(_ctx, "h264_available", False)),
        "resolution": "640x480",
        "stream_clients": _ctx.get_stream_clients(),
    })
