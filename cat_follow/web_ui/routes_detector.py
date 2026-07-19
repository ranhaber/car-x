"""
Detector model selection API.

Routes:
    GET  /api/detector_model — Current model and options
    POST /api/detector_model — Set detector model
"""

from flask import Blueprint, request, jsonify

from cat_follow.logger import get_logger

detector_bp = Blueprint("detector", __name__)
_log = get_logger("web_ui.detector")

DETECTOR_OPTIONS = {
    "ssd_mobilenet_v2": "SSD MobileNet V2 (320x320, quantized)",
    "efficientdet_lite0": "EfficientDet-Lite0",
}

_ctx = None


def init_detector_routes(ctx):
    """Register detector routes. ctx: shared."""
    global _ctx
    _ctx = ctx

    @detector_bp.route("/api/detector_model", methods=["GET"])
    def api_detector_model_get():
        current = _ctx.shared.get_detector_model() if _ctx.shared else None
        return jsonify({
            "current": current,
            "options": DETECTOR_OPTIONS,
        })

    @detector_bp.route("/api/detector_model", methods=["POST"])
    def api_detector_model_post():
        data = request.get_json(silent=True) or {}
        choice = data.get("model")
        if choice not in DETECTOR_OPTIONS:
            return jsonify({"error": f"Invalid model. Choose from: {list(DETECTOR_OPTIONS.keys())}"}), 400
        _ctx.shared.set_detector_model(choice)
        _log.info("Detector model changed to %s", choice)
        return jsonify({"status": "ok", "model": choice})
