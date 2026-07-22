"""
Detector model selection API (RKNN NPU only).

The detection model is fixed to the RK3576 NPU (RKNN) and its path is
env-driven (``CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH``).  There is no runtime
model swapping, so the POST endpoint is a no-op that simply acknowledges the
env-driven selection.

Routes:
    GET  /api/detector_model — Current model and options
    POST /api/detector_model — Acknowledge (env-driven; no runtime change)
"""

from flask import Blueprint, request, jsonify

from cat_follow.logger import get_logger
from cat_follow.perception_config import load_perception_config

detector_bp = Blueprint("detector", __name__)
_log = get_logger("web_ui.detector")

RKNN_OPTIONS = {
    "rknn": "RKNN NPU (configured model path)",
}

_ctx = None


def init_detector_routes(ctx):
    """Bind detector context."""
    global _ctx
    _ctx = ctx


def _model_path() -> str:
    try:
        return load_perception_config().rknn_model_path
    except Exception:  # noqa: BLE001
        return ""


@detector_bp.route("/api/detector_model", methods=["GET"])
def api_detector_model_get():
    return jsonify({
        "current": "rknn",
        "options": RKNN_OPTIONS,
        "backend": "rknn",
        "model_path": _model_path(),
    })


@detector_bp.route("/api/detector_model", methods=["POST"])
def api_detector_model_post():
    data = request.get_json(silent=True) or {}
    choice = data.get("model")
    if choice not in RKNN_OPTIONS:
        return jsonify({
            "error": f"Invalid model. Choose from: {list(RKNN_OPTIONS.keys())}"
        }), 400
    _log.info("Detector backend is RKNN; model selection is env-driven")
    return jsonify({"status": "ok", "model": "rknn", "backend": "rknn"})
