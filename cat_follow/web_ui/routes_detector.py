"""
Detector model selection API.

Routes:
    GET  /api/detector_model — Current model and options
    POST /api/detector_model — Set detector model
"""

from flask import Blueprint, request, jsonify

from cat_follow.logger import get_logger
from cat_follow.perception_config import load_perception_config

detector_bp = Blueprint("detector", __name__)
_log = get_logger("web_ui.detector")

TFLITE_OPTIONS = {
    "ssd_mobilenet_v2": "SSD MobileNet V2 (320x320, quantized)",
    "efficientdet_lite0": "EfficientDet-Lite0",
}

RKNN_OPTIONS = {
    "rknn": "RKNN NPU (configured model path)",
}

_ctx = None


def _options_for_backend():
    try:
        cfg = load_perception_config()
    except Exception:  # noqa: BLE001
        return TFLITE_OPTIONS, "tflite"
    if cfg.uses_rknn:
        return RKNN_OPTIONS, "rknn"
    return TFLITE_OPTIONS, "tflite"


def init_detector_routes(ctx):
    """Bind detector context."""
    global _ctx
    _ctx = ctx


@detector_bp.route("/api/detector_model", methods=["GET"])
def api_detector_model_get():
    options, backend = _options_for_backend()
    current = _ctx.shared.get_detector_model() if _ctx and _ctx.shared else None
    if backend == "rknn":
        current = "rknn"
    return jsonify({
        "current": current,
        "options": options,
        "backend": backend,
    })


@detector_bp.route("/api/detector_model", methods=["POST"])
def api_detector_model_post():
    options, backend = _options_for_backend()
    data = request.get_json(silent=True) or {}
    choice = data.get("model")
    if choice not in options:
        return jsonify({
            "error": f"Invalid model. Choose from: {list(options.keys())}"
        }), 400
    if backend == "rknn":
        _log.info("Detector backend is RKNN; model selection is env-driven")
        return jsonify({"status": "ok", "model": choice, "backend": backend})
    _ctx.shared.set_detector_model(choice)
    _log.info("Detector model changed to %s", choice)
    return jsonify({"status": "ok", "model": choice, "backend": backend})
