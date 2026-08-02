"""Dual lidar/ultrasonic health snapshot for status and soak telemetry."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from cat_follow.control.types import RangeState
from cat_follow.navigation.ultrasonic_range import ULTRASONIC_FRAME_ID


@dataclass(frozen=True)
class DualSensorChannelHealth:
    fresh: bool
    valid: bool
    faulted: bool
    distance_cm: Optional[float]
    stale_ms: int
    backend: str
    frame_id: Optional[str] = None
    costmap_layer_enabled: Optional[bool] = None


@dataclass(frozen=True)
class DualSensorHealthState:
    """Contract-shaped dual-sensor health (Interface §11)."""

    lidar: DualSensorChannelHealth
    ultrasonic: DualSensorChannelHealth
    required_for_motion: bool = True
    hold_active: bool = False
    hold_started_ms: Optional[int] = None
    hold_reason: Optional[str] = None
    recovery_deadline_ms: Optional[int] = None

    def to_dict(self) -> dict:
        def _channel(ch: DualSensorChannelHealth) -> dict:
            payload = {
                "fresh": bool(ch.fresh),
                "valid": bool(ch.valid),
                "faulted": bool(ch.faulted),
                "distance_cm": ch.distance_cm,
                "stale_ms": int(ch.stale_ms),
                "backend": ch.backend,
            }
            if ch.frame_id is not None:
                payload["frame_id"] = ch.frame_id
            if ch.costmap_layer_enabled is not None:
                payload["costmap_layer_enabled"] = bool(ch.costmap_layer_enabled)
            return payload

        return {
            "lidar": _channel(self.lidar),
            "ultrasonic": _channel(self.ultrasonic),
            "required_for_motion": bool(self.required_for_motion),
            "hold_active": bool(self.hold_active),
            "hold_started_ms": self.hold_started_ms,
            "hold_reason": self.hold_reason,
            "recovery_deadline_ms": self.recovery_deadline_ms,
        }


def _channel_from_range(
    state: RangeState,
    *,
    now_ms: int,
    stale_ttl_ms: int,
    backend_name: str,
    frame_id: Optional[str] = None,
    costmap_layer_enabled: Optional[bool] = None,
) -> DualSensorChannelHealth:
    received = int(state.received_ms)
    if received <= 0:
        stale_ms = stale_ttl_ms + 1
        fresh = False
    else:
        stale_ms = max(0, int(now_ms) - received)
        fresh = stale_ms <= int(stale_ttl_ms)
    valid = (
        fresh
        and state.distance_cm is not None
        and float(state.confidence) > 0.0
    )
    faulted = (not fresh) or (state.distance_cm is None) or (
        float(state.confidence) <= 0.0 and received > 0
    )
    return DualSensorChannelHealth(
        fresh=fresh,
        valid=valid,
        faulted=faulted,
        distance_cm=state.distance_cm,
        stale_ms=stale_ms,
        backend=backend_name,
        frame_id=frame_id,
        costmap_layer_enabled=costmap_layer_enabled,
    )


def build_dual_sensor_health(
    *,
    ultrasonic: RangeState,
    lidar: RangeState,
    now_ms: int,
    ultrasonic_stale_ms: int,
    lidar_stale_ms: int,
    hold_active: bool = False,
    hold_started_ms: Optional[int] = None,
    hold_reason: Optional[str] = None,
    recovery_deadline_ms: Optional[int] = None,
    costmap_layer_enabled: bool = True,
) -> DualSensorHealthState:
    """Compose the dual-sensor status payload from independent channels."""

    ultra = _channel_from_range(
        ultrasonic,
        now_ms=now_ms,
        stale_ttl_ms=ultrasonic_stale_ms,
        backend_name=str(
            getattr(ultrasonic.backend, "value", ultrasonic.backend) or "ultrasonic"
        ),
        frame_id=ULTRASONIC_FRAME_ID,
        costmap_layer_enabled=costmap_layer_enabled,
    )
    lidar_ch = _channel_from_range(
        lidar,
        now_ms=now_ms,
        stale_ttl_ms=lidar_stale_ms,
        backend_name=str(
            getattr(lidar.backend, "value", lidar.backend) or "rplidar_c1"
        ),
    )
    return DualSensorHealthState(
        lidar=lidar_ch,
        ultrasonic=ultra,
        required_for_motion=True,
        hold_active=bool(hold_active),
        hold_started_ms=hold_started_ms,
        hold_reason=hold_reason,
        recovery_deadline_ms=recovery_deadline_ms,
    )
