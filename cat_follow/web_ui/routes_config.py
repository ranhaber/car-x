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
    from cat_follow.safety_config import load_safety_config_from_env

    try:
        camera = asdict(load_camera_config())
    except Exception as exc:  # noqa: BLE001
        camera = {"error": str(exc)}
    try:
        perception = asdict(load_perception_config())
    except Exception as exc:  # noqa: BLE001
        perception = {"error": str(exc)}
    try:
        env_safety = asdict(load_safety_config_from_env())
        env_safety["safety_degraded"] = False
        env_safety["safety_error"] = None
    except Exception as exc:  # noqa: BLE001
        env_safety = {
            "error": str(exc),
            "safety_degraded": True,
            "safety_error": str(exc),
            "obstacle_too_close_cm": None,
            "obstacle_detected_cm": None,
        }
    effective = env_safety
    calib = getattr(_ctx, "calibration", None) if _ctx is not None else None
    if calib is not None:
        from cat_follow.safety_config import safe_resolve_safety_config

        cfg, err = safe_resolve_safety_config(calib)
        if cfg is None:
            effective = {
                "error": err,
                "safety_degraded": True,
                "safety_error": err,
                "obstacle_too_close_cm": None,
                "obstacle_detected_cm": None,
            }
        else:
            effective = asdict(cfg)
            effective["safety_degraded"] = False
            effective["safety_error"] = None

    return jsonify({
        "camera": camera,
        "perception": perception,
        "safety_env": env_safety,
        "safety_effective": effective,
        "note": (
            "Read-only snapshot of env-driven settings. "
            "Safety failsafe thresholds can also be overridden from Calibration "
            "(steering_limits.json). Restart is not required for calibration overrides."
        ),
    })
