"""
Control API: send target and stop.

Routes:
    POST /api/target — Set cat target (x_cm, y_cm) in centimeters
    POST /api/stop   — Stop command
    POST /api/command/start_chase — Contract START_CHASE (when CommsManager present)
    POST /api/command/emergency_stop — Contract EMERGENCY_STOP
"""

from __future__ import annotations

import uuid
from math import isfinite

from flask import Blueprint, request, jsonify

from cat_follow.logger import get_logger
from cat_follow.commands import set_cat_location_cm, set_stop_command
from cat_follow.web_ui.auth import require_control_token

control_bp = Blueprint("control", __name__)
_log = get_logger("web_ui.control")

_ctx = None


def _next_command_id(prefix: str) -> str:
    return f"web-{prefix}-{uuid.uuid4().hex[:12]}"


def _submit_contract_command(command, params=None):
    """Submit a CommandMessage via CommsManager when available.

    Returns (ack_dict_or_none, used_contract: bool).
    """
    comms = getattr(_ctx, "comms_manager", None) if _ctx is not None else None
    if comms is None:
        return None, False

    from cat_follow.comms.messages import CommandMessage
    from cat_follow.control.types import AckStatus
    from cat_follow.runtime.shared_state import now_monotonic_ms
    from cat_follow.web_ui.command_seq import next_web_command_seq

    seq = next_web_command_seq(_ctx)
    msg = CommandMessage(
        sequence=seq,
        timestamp_ms=now_monotonic_ms(),
        command_id=_next_command_id(command.value),
        command=command,
        params=dict(params or {}),
    )
    ack = comms.submit_command(msg)
    return {
        "status": ack.status.value,
        "state": ack.state.value,
        "reason": ack.reason.value,
        "cause": ack.cause.value if ack.cause is not None else None,
        "command_id": ack.command_id,
        "applied": ack.status == AckStatus.ACCEPTED,
        "applied_control_seq": ack.applied_control_sequence,
    }, True


def init_control_routes(ctx):
    """Bind the control context used by command routes."""
    global _ctx
    _ctx = ctx


@control_bp.route("/api/target", methods=["POST"])
@require_control_token
def api_target():
    data = request.get_json(silent=True) or {}
    try:
        x_cm = float(data["x_cm"])
        y_cm = float(data["y_cm"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "Need JSON with x_cm and y_cm (centimeters)"}), 400
    if not isfinite(x_cm) or not isfinite(y_cm):
        return jsonify({"error": "x_cm and y_cm must be finite"}), 400

    from cat_follow.control.types import CommandName

    ack, used = _submit_contract_command(
        CommandName.GO_TO,
        params={"target": {"x_cm": x_cm, "y_cm": y_cm, "frame_id": "yard"}},
    )
    set_cat_location_cm(x_cm, y_cm)
    _log.info(
        "API target received: (%.1f, %.1f) cm (contract=%s)",
        x_cm,
        y_cm,
        used,
    )
    body = {
        "status": "ok",
        "x_cm": x_cm,
        "y_cm": y_cm,
        "mode": "contract" if used else "prototype",
    }
    if ack is not None:
        body["ack"] = ack
    return jsonify(body)


@control_bp.route("/api/stop", methods=["POST"])
def api_stop():
    from cat_follow.control.types import CommandName
    from cat_follow.web_ui.routes_movement import abort_sequence_on_operator_stop

    abort_sequence_on_operator_stop("operator_stop")

    params = {}
    data = request.get_json(silent=True) or {}
    target_id = data.get("target_id")
    if target_id:
        params["target_id"] = str(target_id)
    elif _ctx is not None and getattr(_ctx, "runtime_shared", None):
        active = _ctx.runtime_shared.get_mission().active_target_id
        if active:
            params["target_id"] = active

    ack, used = _submit_contract_command(CommandName.STOP_CHASE, params=params or None)
    set_stop_command()
    _log.info("API stop received (contract=%s)", used)
    body = {"status": "ok", "mode": "contract" if used else "prototype"}
    if ack is not None:
        body["ack"] = ack
    return jsonify(body)


@control_bp.route("/api/command/start_chase", methods=["POST"])
@require_control_token
def api_start_chase():
    from cat_follow.control.types import CommandName

    data = request.get_json(silent=True) or {}
    target_id = data.get("target_id")
    if not target_id and _ctx is not None and getattr(_ctx, "runtime_shared", None):
        target_id = _ctx.runtime_shared.get_overhead().selected_target_id
    if not target_id:
        return jsonify(
            {
                "error": "target_id required in JSON body or overhead selected_target_id",
            }
        ), 400

    ack, used = _submit_contract_command(
        CommandName.START_CHASE,
        params={"target_id": str(target_id)},
    )
    if not used:
        return jsonify({"error": "CommsManager not available (prototype mode)"}), 400
    return jsonify({"status": "ok", "mode": "contract", "ack": ack})


@control_bp.route("/api/command/emergency_stop", methods=["POST"])
def api_emergency_stop():
    from cat_follow.control.types import CommandName
    from cat_follow.web_ui.routes_movement import abort_sequence_on_operator_stop

    abort_sequence_on_operator_stop("emergency_stop")

    ack, used = _submit_contract_command(CommandName.EMERGENCY_STOP)
    set_stop_command()
    body = {"status": "ok", "mode": "contract" if used else "prototype"}
    if ack is not None:
        body["ack"] = ack
    return jsonify(body)
