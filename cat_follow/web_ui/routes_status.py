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
    intent = nav.goal_intent
    result = nav.last_result
    return {
        "heading": float(nav.heading),
        "heading_valid": bool(nav.heading_valid),
        "path_correction": float(nav.path_correction),
        "speed_limit": float(nav.speed_limit),
        "no_progress": bool(nav.no_progress),
        "dead_end": bool(nav.dead_end),
        "fresh": bool(nav.fresh),
        "healthy": bool(nav.healthy),
        "path_viable": bool(nav.path_viable),
        "safe_steering_min": float(nav.safe_steering_min),
        "safe_steering_max": float(nav.safe_steering_max),
        "speed_cap_mps": float(nav.speed_cap_mps),
        "completion_qualified": bool(nav.completion_qualified),
        "failures_exhausted": bool(nav.failures_exhausted),
        "goal_intent": (
            None
            if intent is None
            else {
                "goal_intent_id": intent.goal_intent_id,
                "objective_type": intent.objective_type.value,
                "target_id": intent.target_id,
                "frame_id": intent.frame_id,
                "x_m": intent.x_m,
                "y_m": intent.y_m,
                "yaw_rad": intent.yaw_rad,
                "moving_goal": intent.moving_goal,
                "action_goal_id": intent.action_goal_id,
                "refresh_count": intent.refresh_count,
                "last_refresh_ms": intent.last_refresh_ms,
            }
        ),
        "last_result": (
            None
            if result is None
            else {
                "goal_intent_id": result.goal_intent_id,
                "action_goal_id": result.action_goal_id,
                "status": result.status.value,
                "failure_class": (
                    result.failure_class.value
                    if result.failure_class is not None
                    else None
                ),
                "pose_qualified": result.pose_qualified,
                "dwell_qualified": result.dwell_qualified,
            }
        ),
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
    cat_injection = (
        _ctx.shared.get_cat_injection_status()
        if _ctx
        and _ctx.shared
        and hasattr(_ctx.shared, "get_cat_injection_status")
        else {"enabled": False, "bbox": None, "detection_fallback": False}
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
        "cat_injection": cat_injection,
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
    target_cfg = getattr(_ctx, "target_runtime_config", None) if _ctx else None
    if target_cfg is not None:
        payload["effective_target_config"] = target_cfg.telemetry_dict()

    if safety_cfg is not None:
        from cat_follow.active_config import active_runtime_config_dict

        payload["effective_active_config"] = active_runtime_config_dict(
            safety_cfg, target_cfg
        )

    runtime_shared = getattr(_ctx, "runtime_shared", None) if _ctx else None
    if runtime_shared is not None:
        snap = runtime_shared.get_snapshot()
        payload["mode"] = "contract"
        payload["fsm"] = _fsm_dict(snap.fsm)
        payload["decision"] = _decision_dict(snap.decision)
        payload["range"] = _range_dict(snap.range)
        payload["lidar"] = _range_dict(snap.lidar)
        dual = (
            runtime_shared.get_dual_sensor_health()
            if hasattr(runtime_shared, "get_dual_sensor_health")
            else None
        )
        if dual is not None:
            payload["dual_sensor_health"] = dual
        payload["navigation"] = _navigation_dict(snap.navigation)
        payload["vision"] = _vision_dict(snap.vision)
        payload["overhead"] = {
            "sequence": int(snap.overhead.sequence),
            "selected_target_id": snap.overhead.selected_target_id,
            "cat_target_id": snap.overhead.cat.target_id,
            "perimeter_id": snap.overhead.perimeter_id,
        }
        mission = runtime_shared.get_mission()
        payload["mission"] = {
            "active_target_id": mission.active_target_id,
            "last_event_observation_seq": mission.last_event_observation_seq,
            "blocked_target_id": mission.blocked_target_id,
            "blocked_through_observation_seq": (
                mission.blocked_through_observation_seq
            ),
            "handoff_deadline_ms": mission.handoff_deadline_ms,
            "overhead_invalid_started_ms": mission.overhead_invalid_started_ms,
            "search_stage": mission.search_stage,
            "search_lock_observations": mission.search_lock_observations,
            "home_version_frozen": mission.home_version_frozen,
        }
        home = snap.home
        payload["home"] = {
            "set": bool(home.set),
            "valid": bool(home.valid),
            "home_version": int(home.home_version),
            "map_id": home.map_id,
            "frame_id": home.frame_id,
            "frozen_for_mission": bool(home.frozen_for_mission),
            "checksum": home.checksum,
        }
        geofence = snap.geofence
        payload["geofence"] = {
            "configured": bool(geofence.configured),
            "car_geofence_id": geofence.car_geofence_id,
            "car_inside": bool(geofence.car_inside),
            "car_distance_to_boundary_cm": geofence.car_distance_to_boundary_cm,
            "localization_valid_for_containment": bool(
                geofence.localization_valid_for_containment
            ),
            "breach_confirmed": bool(geofence.breach_confirmed),
        }
        payload["startup"] = {
            "ready": bool(snap.system.startup_ready),
            "seed_applied": bool(snap.system.startup_seed_applied),
            "degraded_reason": snap.system.startup_degraded_reason,
        }
        life = snap.perception_lifecycle
        payload["perception_lifecycle"] = {
            "detector": {
                "requested": bool(life.detector.requested),
                "active": bool(life.detector.active),
                "consumer_refcount": int(life.detector.consumer_refcount),
                "reason": life.detector.reason,
            },
            "recording": {
                "requested": bool(life.recording.requested),
                "active": bool(life.recording.active),
                "consumer_refcount": int(life.recording.consumer_refcount),
                "postroll_deadline_ms": life.recording.postroll_deadline_ms,
                "degraded_reason": life.recording.degraded_reason,
                "segment_path": life.recording.segment_path,
            },
            "stream": {
                "requested_clients": int(life.stream_requested_clients),
                "active_clients": int(life.stream_active_clients),
                "encoder_ready": bool(life.stream_encoder_ready),
                "forced_off": bool(life.stream_forced_off),
                "degraded_reason": life.stream_degraded_reason,
            },
            "camera": {
                "hardware_state": life.camera_hardware_state.value,
                "streamoff_capable": bool(life.camera_streamoff_capable),
                "last_revalidation_ms": int(life.camera_last_revalidation_ms),
                "fatal_fault": bool(life.camera_fatal_fault),
            },
        }
        payload["state"] = payload["fsm"]["state"]
        payload["fatal_reason"] = (
            runtime_shared.get_runtime_fatal_reason()
            if hasattr(runtime_shared, "get_runtime_fatal_reason")
            else None
        )

    return jsonify(payload)
