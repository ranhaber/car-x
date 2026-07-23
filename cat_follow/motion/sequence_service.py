"""Shared helpers for starting validated motion sequences from any web route."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

from cat_follow.control.types import FsmState
from cat_follow.motion.action_plan import ValidatedAction, validate_plan
from cat_follow.runtime.shared_state import now_monotonic_ms


def _max_steer_deg_from_ctx(ctx: Any) -> Optional[float]:
    calib = getattr(ctx, "calibration", None) if ctx is not None else None
    if calib is None:
        return None
    try:
        return float(calib.get_max_steer_angle_deg())
    except Exception:  # noqa: BLE001
        return None


def validate_actions(
    raw_actions: Any,
    *,
    max_steer_deg: Optional[float] = None,
    ctx: Any = None,
) -> Tuple[List[ValidatedAction], List[str]]:
    limit = max_steer_deg
    if limit is None and ctx is not None:
        limit = _max_steer_deg_from_ctx(ctx)
    return validate_plan(raw_actions, max_steer_deg=limit)


def start_sequence(
    ctx: Any,
    actions: Sequence[ValidatedAction],
    *,
    source: str = "web",
) -> Tuple[Dict[str, Any], int]:
    """Start a validated plan using the contract or prototype execution path."""

    executor = getattr(ctx, "sequence_executor", None)
    if executor is None:
        return {"error": "sequence executor not initialized"}, 500

    fsm_state = _runtime_fsm_state(ctx)
    if fsm_state == FsmState.FAILSAFE.value:
        return {"error": "FAILSAFE latched; clear failsafe before moving"}, 409

    now_ms = now_monotonic_ms()
    stop_ack = _submit_stop_chase(ctx)

    comms = getattr(ctx, "comms_manager", None)
    if comms is not None:
        ok, message = executor.start(list(actions), now_ms=now_ms)
        if not ok:
            return {"error": message}, 409
        mode = "contract"
    else:
        runner = getattr(ctx, "prototype_sequence_runner", None)
        if runner is None:
            return {"error": "prototype runner not available"}, 500
        ok, message = runner.start(list(actions), now_ms=now_ms)
        if not ok:
            return {"error": message}, 409
        mode = "prototype"

    body: Dict[str, Any] = {
        "status": "ok",
        "mode": mode,
        "source": source,
        "action_count": len(actions),
        "sequence": executor.status(),
    }
    if stop_ack is not None:
        body["stop_chase_ack"] = stop_ack
    return body, 200


def abort_sequence(ctx: Any, reason: str = "operator_stop") -> None:
    executor = getattr(ctx, "sequence_executor", None)
    if executor is not None:
        executor.stop(reason)
    runner = getattr(ctx, "prototype_sequence_runner", None)
    if runner is not None:
        runner.stop(reason)


def _runtime_fsm_state(ctx: Any) -> Optional[str]:
    runtime_shared = getattr(ctx, "runtime_shared", None)
    if runtime_shared is None:
        return None
    return runtime_shared.get_fsm().state.value


def _submit_stop_chase(ctx: Any):
    comms = getattr(ctx, "comms_manager", None)
    if comms is None:
        return None

    import uuid

    from cat_follow.comms.messages import CommandMessage
    from cat_follow.control.types import CommandName
    from cat_follow.web_ui.command_seq import next_web_command_seq

    msg = CommandMessage(
        sequence=next_web_command_seq(ctx),
        timestamp_ms=now_monotonic_ms(),
        command_id=f"web-stop-{uuid.uuid4().hex[:12]}",
        command=CommandName.STOP_CHASE,
        params={},
    )
    ack = comms.submit_command(msg)
    return {
        "status": ack.status.value,
        "state": ack.state.value,
        "reason": ack.reason.value,
        "cause": ack.cause.value if ack.cause is not None else None,
        "command_id": ack.command_id,
    }
