"""Developer API for live camera-frame cat injection."""

from __future__ import annotations

from flask import Blueprint, jsonify, request

from cat_follow.web_ui.auth import require_control_token

injection_bp = Blueprint("injection", __name__)
_ctx = None


def init_injection_routes(ctx) -> None:
    global _ctx
    _ctx = ctx


@injection_bp.route("/api/dev/inject_cat", methods=["GET"])
def api_inject_cat_get():
    return jsonify(_ctx.shared.get_cat_injection_status())


@injection_bp.route("/api/dev/inject_cat", methods=["POST"])
@require_control_token
def api_inject_cat_post():
    data = request.get_json(silent=True) or {}
    action = str(data.get("action", "toggle")).strip().lower()
    current = _ctx.shared.cat_injection_enabled()
    if action == "start":
        enabled = True
    elif action == "stop":
        enabled = False
    elif action == "toggle":
        enabled = not current
    else:
        return jsonify({"error": "action must be start, stop, or toggle"}), 400

    _ctx.shared.set_cat_injection_enabled(enabled)
    status = _ctx.shared.get_cat_injection_status()
    status["status"] = "ok"
    return jsonify(status)


__all__ = ["injection_bp", "init_injection_routes"]
