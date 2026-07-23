"""
Stream configuration API: resolution selection and capabilities.

Routes:
    POST /api/stream/resolution — Set stream resolution
    GET  /api/stream/capabilities — MJPEG / H.264 availability
"""

from flask import Blueprint, request, jsonify

from cat_follow.logger import get_logger
from cat_follow.web_ui.auth import require_control_token

stream_config_bp = Blueprint("stream_config", __name__)
_log = get_logger("web_ui.stream_config")

_ctx = None


def init_stream_config_routes(ctx):
    """Bind stream-config context."""
    global _ctx
    _ctx = ctx


@stream_config_bp.route("/api/stream/resolution", methods=["POST"])
@require_control_token
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


@stream_config_bp.route("/api/stream/capabilities", methods=["GET"])
def api_stream_capabilities():
    return jsonify({
        "mjpeg": True,
        "h264": bool(getattr(_ctx, "h264_available", False)),
        "resolution": _ctx.get_stream_resolution(),
        "resolutions": list(_ctx.resolution_options.keys()),
        "stream_clients": _ctx.get_stream_clients(),
    })
