"""Movement tab API: validated action-sequence builder and runner."""

from __future__ import annotations

from flask import Blueprint, jsonify, request

from cat_follow.logger import get_logger
from cat_follow.motion.action_plan import plan_to_public_dict
from cat_follow.motion.sequence_service import (
    abort_sequence,
    start_sequence,
    validate_actions,
)
from cat_follow.runtime.shared_state import now_monotonic_ms
from cat_follow.safety_config import safe_resolve_safety_config
from cat_follow.web_ui.auth import require_control_token

movement_bp = Blueprint("movement", __name__)
_log = get_logger("web_ui.movement")

_ctx = None


def init_movement_routes(ctx) -> None:
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


@movement_bp.route("/api/movement/safety", methods=["GET"])
def movement_safety():
    calib = getattr(_ctx, "calibration", None) if _ctx is not None else None
    return jsonify(_safety_payload(calib))


@movement_bp.route("/api/movement/sequence/validate", methods=["POST"])
@require_control_token
def validate_sequence():
    data = request.get_json(silent=True) or {}
    actions = data.get("actions")
    executor = _ctx.sequence_executor if _ctx is not None else None
    if executor is None:
        return jsonify({"error": "sequence executor not initialized"}), 500
    plan, errors = validate_actions(actions, ctx=_ctx)
    if errors:
        return jsonify({"ok": False, "errors": errors}), 400
    return jsonify({"ok": True, "actions": plan_to_public_dict(plan)})


@movement_bp.route("/api/movement/sequence/status", methods=["GET"])
def sequence_status():
    executor = _ctx.sequence_executor if _ctx is not None else None
    if executor is None:
        return jsonify({"error": "sequence executor not initialized"}), 500
    body = executor.status()
    body["mode"] = "contract" if getattr(_ctx, "comms_manager", None) else "prototype"
    runtime_shared = getattr(_ctx, "runtime_shared", None)
    body["fsm_state"] = (
        runtime_shared.get_fsm().state.value if runtime_shared is not None else None
    )
    calib = getattr(_ctx, "calibration", None)
    body.update(_safety_payload(calib))
    return jsonify(body)


@movement_bp.route("/api/movement/sequence/heartbeat", methods=["POST"])
@require_control_token
def sequence_heartbeat():
    executor = _ctx.sequence_executor if _ctx is not None else None
    if executor is None:
        return jsonify({"error": "sequence executor not initialized"}), 500
    if not executor.is_running:
        return jsonify({"error": "no sequence running"}), 409
    executor.heartbeat(now_ms=now_monotonic_ms())
    return jsonify({"status": "ok"})


@movement_bp.route("/api/movement/sequence/run", methods=["POST"])
@require_control_token
def run_sequence():
    data = request.get_json(silent=True) or {}
    actions_raw = data.get("actions")
    plan, errors = validate_actions(actions_raw, ctx=_ctx)
    if errors:
        return jsonify({"ok": False, "errors": errors}), 400
    body, code = start_sequence(_ctx, plan, source="movement")
    if code == 200:
        _log.info("Movement sequence started (%s actions)", len(plan))
    return jsonify(body), code


@movement_bp.route("/api/movement/sequence/stop", methods=["POST"])
def stop_sequence():
    abort_sequence(_ctx, "operator_stop")
    from cat_follow.motion.sequence_service import _submit_stop_chase

    stop_ack = _submit_stop_chase(_ctx)
    body = {"status": "ok"}
    if stop_ack is not None:
        body["stop_chase_ack"] = stop_ack
    return jsonify(body)


def abort_sequence_on_operator_stop(reason: str = "operator_stop") -> None:
    """Called by global STOP / E-STOP routes to halt an active sequence."""

    abort_sequence(_ctx, reason)
