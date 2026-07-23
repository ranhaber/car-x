"""
Calibration API: get/save calibration data and queue-friendly motion plans.

Routes:
    GET  /api/calibration        — Get all calibration data
    POST /api/calibration        — Save calibration data (+ apply safety thresholds)
    POST /api/calibrate/run_speed  — Build validated speed-test plan (queue only)
    POST /api/calibrate/run_steer  — Build validated steer-test plan (queue only)
"""

from flask import Blueprint, jsonify, request

from cat_follow.calibration.loader import validate_calibration_candidate
from cat_follow.logger import get_logger
from cat_follow.motion.action_plan import plan_to_public_dict
from cat_follow.motion.calibration_plans import speed_test_plan, steer_test_plan
from cat_follow.safety_config import safe_resolve_safety_config
from cat_follow.web_ui.auth import require_control_token

calibration_bp = Blueprint("calibration", __name__)
_log = get_logger("web_ui.calibration")

_ctx = None


def init_calibration_routes(ctx):
    """Bind calibration context."""
    global _ctx
    _ctx = ctx


def _safety_payload(calib) -> dict:
    cfg, err = safe_resolve_safety_config(calib)
    if cfg is None:
        return {
            "safety_degraded": True,
            "safety_error": err or "invalid safety configuration",
            "obstacle_too_close_cm": None,
            "obstacle_detected_cm": None,
        }
    return {
        "safety_degraded": False,
        "safety_error": None,
        "obstacle_too_close_cm": cfg.obstacle_too_close_cm,
        "obstacle_detected_cm": cfg.obstacle_detected_cm,
    }


def _apply_saved_safety() -> dict:
    calib = getattr(_ctx, "calibration", None)
    apply_fn = getattr(_ctx, "apply_safety_config", None)
    cfg, err = safe_resolve_safety_config(calib)
    if cfg is None:
        raise ValueError(err or "invalid safety configuration")
    if apply_fn is not None:
        apply_fn(calib)
    return {
        "obstacle_too_close_cm": cfg.obstacle_too_close_cm,
        "obstacle_detected_cm": cfg.obstacle_detected_cm,
    }


@calibration_bp.route("/api/calibration", methods=["GET"])
def get_calibration():
    if not _ctx or not _ctx.calibration:
        return jsonify({"error": "Calibration not initialized"}), 500
    body = _ctx.calibration.get_all_calibration_data()
    body["safety_effective"] = _safety_payload(_ctx.calibration)
    return jsonify(body)


@calibration_bp.route("/api/calibration", methods=["POST"])
@require_control_token
def save_calibration():
    if not _ctx or not _ctx.calibration:
        return jsonify({"error": "Calibration not initialized"}), 500
    data = request.get_json(silent=True)
    if not isinstance(data, dict) or not data:
        return jsonify({"error": "Invalid JSON"}), 400
    try:
        candidate = _ctx.calibration.preview_merged_calibration(data)
    except TypeError as exc:
        return jsonify({"error": str(exc)}), 400
    errors = validate_calibration_candidate(candidate)
    if errors:
        return jsonify({"error": "; ".join(errors), "errors": errors}), 400
    # Validate safety application path before mutating memory/disk.
    try:
        from cat_follow.safety_config import resolve_safety_config

        class _Candidate:
            def get_all_calibration_data(self_inner):
                return candidate

        resolve_safety_config(_Candidate())
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400

    _ctx.calibration.apply_validated_calibration(candidate)
    _ctx.calibration.save()
    try:
        safety = _apply_saved_safety()
    except ValueError as exc:
        # Memory/disk already match the validated candidate; report apply failure.
        _log.warning("calibration saved but safety apply failed: %s", exc)
        return jsonify({"error": str(exc)}), 400
    return jsonify(
        {
            "status": "ok",
            "message": "Calibration saved.",
            "safety_effective": safety,
        }
    )


@calibration_bp.route("/api/calibrate/run_speed", methods=["POST"])
@require_control_token
def api_run_speed_test():
    data = request.get_json(silent=True) or {}
    speed = int(data.get("speed", 30))
    duration = float(data.get("duration", 1.0))
    plan, errors = speed_test_plan(speed=speed, duration_s=duration)
    if errors:
        return jsonify({"ok": False, "errors": errors}), 400
    return jsonify(
        {
            "ok": True,
            "actions": plan_to_public_dict(plan),
            "message": "Add these actions to the queue and run from Movement.",
        }
    )


@calibration_bp.route("/api/calibrate/run_steer", methods=["POST"])
@require_control_token
def api_run_steer_test():
    data = request.get_json(silent=True) or {}
    angle = float(data.get("angle", 0))
    speed = int(data.get("speed", 30))
    duration = float(data.get("duration", 4.0))
    max_steer = None
    if _ctx is not None and getattr(_ctx, "calibration", None) is not None:
        try:
            max_steer = float(_ctx.calibration.get_max_steer_angle_deg())
        except Exception:  # noqa: BLE001
            max_steer = None
    plan, errors = steer_test_plan(
        angle=angle,
        speed=speed,
        duration_s=duration,
        max_steer_deg=max_steer,
    )
    if errors:
        return jsonify({"ok": False, "errors": errors}), 400
    return jsonify(
        {
            "ok": True,
            "actions": plan_to_public_dict(plan),
            "message": "Add these actions to the queue and run from Movement.",
        }
    )
