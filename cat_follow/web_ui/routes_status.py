"""
Status API: system, odometry, and contract SharedSnapshot.

Routes:
    GET /api/status — Prototype + optional contract monitoring payload
"""

from __future__ import annotations

from flask import Blueprint, jsonify

from cat_follow import __version__
from cat_follow import range_sensor
from cat_follow.perception.status import perception_diagnostics_dict

status_bp = Blueprint("status", __name__)

_ctx = None


def _enum_val(value):
    return value.value if hasattr(value, "value") else value


def _range_dict(range_state) -> dict:
    return {
        "distance_cm": range_state.distance_cm,
        "fresh": bool(range_state.fresh),
        "backend": _enum_val(range_state.backend),
        "obstacle_detected": bool(range_state.obstacle_detected),
        "obstacle_critical": bool(range_state.obstacle_critical),
        "obstacle_severity": float(range_state.obstacle_severity),
        "zone": range_state.zone,
    }


def _navigation_dict(nav) -> dict:
    return {
        "heading": float(nav.heading),
        "heading_valid": bool(nav.heading_valid),
        "path_correction": float(nav.path_correction),
        "speed_limit": float(nav.speed_limit),
        "no_progress": bool(nav.no_progress),
        "dead_end": bool(nav.dead_end),
        "fresh": bool(nav.fresh),
    }


def _vision_dict(vision) -> dict:
    return {
        "cat_visible": bool(vision.cat_visible),
        "cat_visible_stable": bool(vision.cat_visible_stable),
        "x_offset_norm": float(vision.x_offset_norm),
        "confidence": float(vision.confidence),
        "fresh": bool(vision.fresh),
    }


def _tracked_targets_dict(targets) -> dict:
    result = {}
    for role, target in targets.items():
        result[role] = {
            "track_id": target[0],
            "x": target[1],
            "y": target[2],
            "w": target[3],
            "h": target[4],
            "confidence": target[5],
            "frames_since_update": target[6],
            "valid": target[7],
        }
    return result


def _decision_dict(decision) -> dict:
    return {
        "speed": float(decision.speed),
        "steering": float(decision.steering),
        "brake": bool(decision.brake),
        "reason": _enum_val(decision.reason),
        "active_constraints": list(decision.active_constraints),
        "requested_state": _enum_val(decision.requested_state),
        "fresh": bool(decision.fresh),
    }


def _fsm_dict(fsm) -> dict:
    return {
        "state": _enum_val(fsm.state),
        "previous_state": _enum_val(fsm.previous_state) if fsm.previous_state else None,
        "last_transition_reason": _enum_val(fsm.last_transition_reason),
        "fresh": bool(fsm.fresh),
    }


def init_status_routes(ctx):
    """Bind the status context used by /api/status."""
    global _ctx
    _ctx = ctx


@status_bp.route("/api/status")
def api_status():
    odom = _ctx.shared.get_odometry() if _ctx and _ctx.shared else (0, 0, 0)
    bbox = _ctx.shared.get_bbox_tracker() if _ctx and _ctx.shared else (0, 0, 0, 0, 0)
    tracked_targets = (
        _ctx.shared.get_tracked_targets() if _ctx and _ctx.shared else {}
    )
    legacy_state = "unknown"
    if _ctx is not None and _ctx.state_machine is not None:
        legacy_state = _ctx.state_machine.state.value

    ultrasonic_cm = range_sensor.get_last_distance_cm()
    perception = perception_diagnostics_dict()
    stream_clients = (
        _ctx.get_stream_clients()
        if _ctx is not None and hasattr(_ctx, "get_stream_clients")
        else 0
    )

    from cat_follow.safety_config import safe_resolve_safety_config

    calib = getattr(_ctx, "calibration", None) if _ctx is not None else None
    safety_cfg, safety_err = safe_resolve_safety_config(calib)
    safety_status = {
        "safety_degraded": safety_cfg is None,
        "safety_error": safety_err,
        "obstacle_too_close_cm": (
            None if safety_cfg is None else safety_cfg.obstacle_too_close_cm
        ),
        "obstacle_detected_cm": (
            None if safety_cfg is None else safety_cfg.obstacle_detected_cm
        ),
    }

    legacy = {
        "state": legacy_state,
        "odometry": {"x": odom[0], "y": odom[1], "heading_deg": odom[2]},
        "bbox_tracker": {
            "x": bbox[0],
            "y": bbox[1],
            "w": bbox[2],
            "h": bbox[3],
            "valid": bbox[4],
        },
        "tracked_targets": _tracked_targets_dict(tracked_targets),
        # Alias for role-aware clients; same payload as tracked_targets.
        "cats": _tracked_targets_dict(tracked_targets),
        "ultrasonic_cm": (
            round(ultrasonic_cm, 1) if ultrasonic_cm is not None else None
        ),
        "tracker_fps": round(_ctx.get_tracker_fps(), 1) if _ctx else 0.0,
        "stream_fps": round(_ctx.get_stream_fps(), 1) if _ctx else 0.0,
        "app_version": __version__,
        "cpu_percent": round(_ctx.get_cpu_percent(), 1) if _ctx else -1.0,
        "ram_percent": round(_ctx.get_ram_percent(), 1) if _ctx else -1.0,
        "cpu_temp": round(_ctx.get_cpu_temp(), 1) if _ctx else -1.0,
        "battery_v": _ctx.get_battery_voltage() if _ctx else -1.0,
        **safety_status,
    }

    payload = {
        "mode": "prototype",
        "perception": perception,
        "stream_clients": stream_clients,
        "legacy": legacy,
        **legacy,
    }

    runtime_shared = getattr(_ctx, "runtime_shared", None) if _ctx else None
    if runtime_shared is not None:
        snap = runtime_shared.get_snapshot()
        payload["mode"] = "contract"
        payload["fsm"] = _fsm_dict(snap.fsm)
        payload["decision"] = _decision_dict(snap.decision)
        payload["range"] = _range_dict(snap.range)
        payload["lidar"] = _range_dict(snap.lidar)
        payload["navigation"] = _navigation_dict(snap.navigation)
        payload["vision"] = _vision_dict(snap.vision)
        payload["state"] = payload["fsm"]["state"]

    return jsonify(payload)
