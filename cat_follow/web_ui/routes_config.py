"""
Read-only effective configuration API.

Routes:
    GET /api/config — CameraConfig + PerceptionConfig as loaded from env
"""

from __future__ import annotations

from dataclasses import asdict

from flask import Blueprint, jsonify

from cat_follow.logger import get_logger

config_bp = Blueprint("config", __name__)
_log = get_logger("web_ui.config")

_ctx = None


def init_config_routes(ctx):
    """Bind config context (currently unused; env-sourced)."""
    global _ctx
    _ctx = ctx


@config_bp.route("/api/config", methods=["GET"])
def api_config():
    from cat_follow.camera_config import load_camera_config
    from cat_follow.perception_config import load_perception_config

    try:
        camera = asdict(load_camera_config())
    except Exception as exc:  # noqa: BLE001
        camera = {"error": str(exc)}
    try:
        perception = asdict(load_perception_config())
    except Exception as exc:  # noqa: BLE001
        perception = {"error": str(exc)}

    return jsonify({
        "camera": camera,
        "perception": perception,
        "note": (
            "Read-only snapshot of env-driven settings. "
            "Edit /etc/car-x/car-x.env and restart to change."
        ),
    })
