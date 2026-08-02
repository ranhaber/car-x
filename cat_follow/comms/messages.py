"""Wire-form dataclasses for tracking, command, and ACK messages.

Each message has a ``to_dict()`` and ``from_dict()`` helper that mirrors the
exact JSON envelope from the Interface and Data Contract Specification
sections 4-7.  Producers and consumers exchange these dataclasses
in-process; the JSON round-trip is preserved for future UDP transport.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from cat_follow.control.types import (
    AckStatus,
    AckType,
    CommandName,
    FsmState,
    MessageType,
    MissionEventName,
    ReasonCode,
    RejectionCause,
)


SCHEMA_VERSION = 1


class SchemaVersionError(ValueError):
    """Raised when an incoming message has an unsupported schema_version."""


# ── Tracking ────────────────────────────────────────────────────────


@dataclass(frozen=True)
class TrackingCar:
    x: float
    y: float
    heading: float = 0.0
    heading_valid: bool = False
    confidence: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "x_cm": self.x,
            "y_cm": self.y,
            "heading": self.heading,
            "yaw_rad": self.heading,
            "heading_valid": self.heading_valid,
            "confidence": self.confidence,
        }

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "TrackingCar":
        return cls(
            x=float(payload["x"] if "x" in payload else payload["x_cm"]),
            y=float(payload["y"] if "y" in payload else payload["y_cm"]),
            heading=float(payload.get("heading", payload.get("yaw_rad", 0.0))),
            heading_valid=bool(payload.get("heading_valid", False)),
            confidence=float(payload.get("confidence", 0.0)),
        )


@dataclass(frozen=True)
class TrackingCat:
    x: float
    y: float
    confidence: float = 0.0
    target_id: Optional[str] = None
    inside_perimeter: bool = True

    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "x_cm": self.x,
            "y_cm": self.y,
            "confidence": self.confidence,
            "target_id": self.target_id,
            "inside_perimeter": self.inside_perimeter,
        }

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "TrackingCat":
        return cls(
            x=float(payload["x"] if "x" in payload else payload["x_cm"]),
            y=float(payload["y"] if "y" in payload else payload["y_cm"]),
            confidence=float(payload.get("confidence", 0.0)),
            target_id=(
                str(payload["target_id"])
                if payload.get("target_id") is not None
                else None
            ),
            inside_perimeter=bool(payload.get("inside_perimeter", True)),
        )


@dataclass(frozen=True)
class TrackingMessage:
    sequence: int
    timestamp_ms: int
    car: TrackingCar
    cat: TrackingCat
    frame_id: str = "yard"
    perimeter_id: str = ""
    calibration_version: int = 0
    selected_target_id: Optional[str] = None
    schema_version: int = SCHEMA_VERSION

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": MessageType.OVERHEAD_OBSERVATION.value,
            "protocol_version": self.schema_version,
            "schema_version": self.schema_version,
            "observation_seq": self.sequence,
            "observed_at_ms": self.timestamp_ms,
            "sequence": self.sequence,
            "timestamp_ms": self.timestamp_ms,
            "frame_id": self.frame_id,
            "car": self.car.to_dict(),
            "cat": self.cat.to_dict(),
            "cats": [self.cat.to_dict()],
            "selected_target_id": self.selected_target_id,
            "perimeter_id": self.perimeter_id,
            "calibration_version": self.calibration_version,
        }

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "TrackingMessage":
        if payload.get("type") not in {
            MessageType.TRACKING.value,
            MessageType.OVERHEAD_OBSERVATION.value,
        }:
            _require_type(payload, MessageType.TRACKING)
        _require_schema_version(payload)
        return cls(
            sequence=int(
                payload["sequence"]
                if "sequence" in payload
                else payload["observation_seq"]
            ),
            timestamp_ms=int(
                payload["timestamp_ms"]
                if "timestamp_ms" in payload
                else payload["observed_at_ms"]
            ),
            car=TrackingCar.from_dict(payload["car"]),
            cat=TrackingCat.from_dict(
                payload["cat"]
                if "cat" in payload
                else _selected_cat_payload(payload)
            ),
            frame_id=str(payload.get("frame_id", "yard")),
            perimeter_id=str(payload.get("perimeter_id", "")),
            calibration_version=int(payload.get("calibration_version", 0)),
            selected_target_id=(
                str(payload["selected_target_id"])
                if payload.get("selected_target_id") is not None
                else None
            ),
            schema_version=int(payload.get("schema_version", SCHEMA_VERSION)),
        )


# ── Command ─────────────────────────────────────────────────────────


@dataclass(frozen=True)
class CommandMessage:
    sequence: int
    timestamp_ms: int
    command_id: str
    command: CommandName
    params: Dict[str, Any] = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION

    def to_dict(self) -> Dict[str, Any]:
        target_id = self.params.get("target_id")
        return {
            "type": MessageType.COMMAND.value,
            "protocol_version": self.schema_version,
            "schema_version": self.schema_version,
            "sequence": self.sequence,
            "timestamp_ms": self.timestamp_ms,
            "command_id": self.command_id,
            "command": self.command.value,
            "params": dict(self.params),
            "name": self.command.name,
            "args": dict(self.params),
            "issued_at_ms": self.timestamp_ms,
            "target_id": target_id,
        }

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "CommandMessage":
        _require_type(payload, MessageType.COMMAND)
        _require_schema_version(payload)
        return cls(
            sequence=int(payload.get("sequence", 0)),
            timestamp_ms=int(
                payload.get("timestamp_ms", payload.get("issued_at_ms", 0))
            ),
            command_id=str(payload["command_id"]),
            command=_command_name(payload),
            params=dict(payload.get("params", payload.get("args", {})) or {}),
            schema_version=int(payload.get("schema_version", SCHEMA_VERSION)),
        )

    @property
    def target_id(self) -> Optional[str]:
        value = self.params.get("target_id")
        return str(value) if value is not None else None


@dataclass(frozen=True)
class MissionEventMessage:
    event_id: str
    mission_id: str
    timestamp_ms: int
    name: MissionEventName
    target_id: str
    perimeter_id: str
    observation_sequence: int
    schema_version: int = SCHEMA_VERSION

    def to_dict(self) -> Dict[str, Any]:
        return {
            "protocol_version": self.schema_version,
            "schema_version": self.schema_version,
            "type": MessageType.MISSION_EVENT.value,
            "event_id": self.event_id,
            "mission_id": self.mission_id,
            "issued_at_ms": self.timestamp_ms,
            "timestamp_ms": self.timestamp_ms,
            "name": self.name.value,
            "target_id": self.target_id,
            "perimeter_id": self.perimeter_id,
            "observation_seq": self.observation_sequence,
        }

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "MissionEventMessage":
        _require_type(payload, MessageType.MISSION_EVENT)
        _require_schema_version(payload)
        return cls(
            event_id=str(payload["event_id"]),
            mission_id=str(payload["mission_id"]),
            timestamp_ms=int(
                payload["timestamp_ms"]
                if "timestamp_ms" in payload
                else payload["issued_at_ms"]
            ),
            name=MissionEventName(str(payload["name"]).upper()),
            target_id=str(payload["target_id"]),
            perimeter_id=str(payload["perimeter_id"]),
            observation_sequence=int(payload["observation_seq"]),
            schema_version=int(
                payload.get(
                    "schema_version",
                    payload.get("protocol_version", SCHEMA_VERSION),
                )
            ),
        )


# ── ACK ─────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class AckMessage:
    sequence: int
    timestamp_ms: int
    ack_sequence: int
    ack_type: AckType
    command_id: Optional[str]
    status: AckStatus
    state: FsmState
    reason: ReasonCode
    cause: Optional[RejectionCause] = None
    applied_control_sequence: Optional[int] = None
    schema_version: int = SCHEMA_VERSION

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": MessageType.ACK.value,
            "protocol_version": self.schema_version,
            "schema_version": self.schema_version,
            "sequence": self.sequence,
            "timestamp_ms": self.timestamp_ms,
            "ack_sequence": self.ack_sequence,
            "ack_type": self.ack_type.value,
            "command_id": self.command_id,
            "status": self.status.value,
            "state": self.state.value,
            "reason": self.reason.value,
            "cause": self.cause.value if self.cause is not None else None,
            "message_id": self.command_id,
            "message_type": self.ack_type.value,
            "applied": self.status == AckStatus.ACCEPTED,
            "resulting_state": self.state.value,
            "applied_control_seq": self.applied_control_sequence,
        }

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "AckMessage":
        _require_type(payload, MessageType.ACK)
        _require_schema_version(payload)
        cause_value = payload.get("cause")
        return cls(
            sequence=int(payload["sequence"]),
            timestamp_ms=int(payload["timestamp_ms"]),
            ack_sequence=int(payload["ack_sequence"]),
            ack_type=AckType(payload["ack_type"]),
            command_id=(
                str(payload["command_id"])
                if payload.get("command_id") is not None
                else None
            ),
            status=AckStatus(payload["status"]),
            state=FsmState(payload["state"]),
            reason=ReasonCode(payload["reason"]),
            cause=RejectionCause(cause_value) if cause_value else None,
            applied_control_sequence=(
                int(payload["applied_control_seq"])
                if payload.get("applied_control_seq") is not None
                else None
            ),
            schema_version=int(payload.get("schema_version", SCHEMA_VERSION)),
        )


# ── helpers ─────────────────────────────────────────────────────────


def _require_type(payload: Dict[str, Any], expected: MessageType) -> None:
    actual = payload.get("type")
    if actual != expected.value:
        raise ValueError(
            f"expected message type {expected.value!r}, got {actual!r}"
        )


def _require_schema_version(payload: Dict[str, Any]) -> None:
    version = int(
        payload.get(
            "schema_version", payload.get("protocol_version", SCHEMA_VERSION)
        )
    )
    if version != SCHEMA_VERSION:
        raise SchemaVersionError(
            f"unsupported schema_version {version}; this build supports {SCHEMA_VERSION}"
        )


def _command_name(payload: Dict[str, Any]) -> CommandName:
    raw = payload.get("command", payload.get("name"))
    if raw is None:
        raise KeyError("command")
    normalized = str(raw).lower()
    return CommandName(normalized)


def _selected_cat_payload(payload: Dict[str, Any]) -> Dict[str, Any]:
    cats = payload.get("cats")
    if not isinstance(cats, list) or not cats:
        raise ValueError("overhead observation requires at least one cat")
    selected = payload.get("selected_target_id")
    if selected is None:
        if len(cats) != 1:
            raise ValueError("selected_target_id is required with multiple cats")
        return cats[0]
    for cat in cats:
        if str(cat.get("target_id")) == str(selected):
            return cat
    raise ValueError("selected_target_id does not identify a cat")


@dataclass(frozen=True)
class PendingTransaction:
    command: Optional[CommandMessage] = None
    mission_event: Optional[MissionEventMessage] = None

    def __post_init__(self) -> None:
        if (self.command is None) == (self.mission_event is None):
            raise ValueError(
                "pending transaction requires exactly one of command or mission_event"
            )


__all__ = [
    "AckMessage",
    "CommandMessage",
    "MissionEventMessage",
    "PendingTransaction",
    "SCHEMA_VERSION",
    "SchemaVersionError",
    "TrackingCar",
    "TrackingCat",
    "TrackingMessage",
]
