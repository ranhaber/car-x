"""Validated ``sensor_msgs/Range`` payload for HC-SR04 costmap integration.

Host CI validates the message contract without ROS. On the ROCK 4D,
``ros_bridge`` publishes the equivalent ``sensor_msgs/msg/Range`` when
``CAT_FOLLOW_NAV_ULTRASONIC_COSTMAP=1``. Direct DecisionEngine ultrasonic
safety remains independent of the costmap layer enable flag.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping, Optional


# Canonical topic/frame used by Nav2 RangeSensorLayer and the URDF link.
ULTRASONIC_RANGE_TOPIC = "/ultrasonic_range"
ULTRASONIC_FRAME_ID = "ultrasonic_link"

# HC-SR04 typical optical properties (radians / meters).
ULTRASONIC_FIELD_OF_VIEW_RAD = math.radians(15.0)
ULTRASONIC_MIN_RANGE_M = 0.02
ULTRASONIC_MAX_RANGE_M = 4.0

# Costmap republish TTL. Matches DecisionEngine RANGE_STALE_MS and the
# Nav2 RangeSensorLayer ``no_readings_timeout: 0.5``. Once SharedState age
# exceeds this, the bridge must stop publishing so the layer sees a real gap
# instead of a freshly stamped ghost of the last good reading.
ULTRASONIC_COSTMAP_STALE_MS = 500

# sensor_msgs/Range radiation_type enum.
RADIATION_ULTRASOUND = 0
RADIATION_INFRARED = 1


@dataclass(frozen=True)
class UltrasonicRangeMessage:
    """Host-side validated Range message (ROS-independent)."""

    topic: str
    frame_id: str
    radiation_type: int
    field_of_view: float
    min_range: float
    max_range: float
    range: float
    stamp_sec: float
    costmap_layer_enabled: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class UltrasonicRangeValidationError(ValueError):
    """Raised when a Range payload violates the NAV-15 contract."""


def build_ultrasonic_range_message(
    distance_cm: float,
    *,
    stamp_sec: float,
    costmap_layer_enabled: bool = True,
    topic: str = ULTRASONIC_RANGE_TOPIC,
    frame_id: str = ULTRASONIC_FRAME_ID,
    field_of_view: float = ULTRASONIC_FIELD_OF_VIEW_RAD,
    min_range_m: float = ULTRASONIC_MIN_RANGE_M,
    max_range_m: float = ULTRASONIC_MAX_RANGE_M,
) -> UltrasonicRangeMessage:
    """Build and validate a Range payload from a centimeter distance."""

    if not math.isfinite(stamp_sec) or stamp_sec < 0.0:
        raise UltrasonicRangeValidationError("stamp_sec must be finite and >= 0")
    if not math.isfinite(distance_cm):
        raise UltrasonicRangeValidationError("distance_cm must be finite")
    if distance_cm <= 0.0:
        raise UltrasonicRangeValidationError("distance_cm must be > 0")
    if not math.isfinite(field_of_view) or field_of_view <= 0.0:
        raise UltrasonicRangeValidationError("field_of_view must be > 0")
    if min_range_m <= 0.0 or max_range_m <= min_range_m:
        raise UltrasonicRangeValidationError("min/max range envelope is invalid")
    if not topic.startswith("/"):
        raise UltrasonicRangeValidationError("topic must be absolute")
    if not frame_id:
        raise UltrasonicRangeValidationError("frame_id is required")

    range_m = float(distance_cm) / 100.0
    if range_m < min_range_m or range_m > max_range_m:
        raise UltrasonicRangeValidationError(
            "range must be within [min_range, max_range]"
        )

    return UltrasonicRangeMessage(
        topic=topic,
        frame_id=frame_id,
        radiation_type=RADIATION_ULTRASOUND,
        field_of_view=float(field_of_view),
        min_range=float(min_range_m),
        max_range=float(max_range_m),
        range=range_m,
        stamp_sec=float(stamp_sec),
        costmap_layer_enabled=bool(costmap_layer_enabled),
    )


def validate_ultrasonic_range_dict(payload: Mapping[str, Any]) -> UltrasonicRangeMessage:
    """Re-validate a serialized Range dict (status/telemetry/replay)."""

    return build_ultrasonic_range_message(
        float(payload["range"]) * 100.0,
        stamp_sec=float(payload["stamp_sec"]),
        costmap_layer_enabled=bool(payload.get("costmap_layer_enabled", True)),
        topic=str(payload.get("topic", ULTRASONIC_RANGE_TOPIC)),
        frame_id=str(payload["frame_id"]),
        field_of_view=float(payload["field_of_view"]),
        min_range_m=float(payload["min_range"]),
        max_range_m=float(payload["max_range"]),
    )


def ultrasonic_reading_is_fresh(
    received_ms: int,
    now_ms: int,
    *,
    stale_ttl_ms: int = ULTRASONIC_COSTMAP_STALE_MS,
) -> bool:
    """Return True when a SharedState range sample is still costmap-worthy."""

    if int(received_ms) <= 0:
        return False
    return (int(now_ms) - int(received_ms)) <= int(stale_ttl_ms)


def maybe_build_from_range_state(
    distance_cm: Optional[float],
    *,
    stamp_sec: float,
    costmap_layer_enabled: bool,
    received_ms: Optional[int] = None,
    now_ms: Optional[int] = None,
    stale_ttl_ms: int = ULTRASONIC_COSTMAP_STALE_MS,
) -> Optional[UltrasonicRangeMessage]:
    """Return a validated Range message or None when the reading is unusable.

    When ``received_ms`` and ``now_ms`` are both provided, a stale SharedState
    sample is rejected so callers cannot keep stamping a dead sensor as live
    for Nav2's RangeSensorLayer.
    """

    if distance_cm is None:
        return None
    if received_ms is not None and now_ms is not None:
        if not ultrasonic_reading_is_fresh(
            received_ms, now_ms, stale_ttl_ms=stale_ttl_ms
        ):
            return None
    try:
        return build_ultrasonic_range_message(
            float(distance_cm),
            stamp_sec=stamp_sec,
            costmap_layer_enabled=costmap_layer_enabled,
        )
    except UltrasonicRangeValidationError:
        return None
