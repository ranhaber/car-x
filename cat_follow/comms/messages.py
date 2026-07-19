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
            "heading": self.heading,
            "heading_valid": self.heading_valid,
            "confidence": self.confidence,
        }

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "TrackingCar":
        return cls(
            x=float(payload["x"]),
            y=float(payload["y"]),
            heading=float(payload.get("heading", 0.0)),
            heading_valid=bool(payload.get("heading_valid", False)),
            confidence=float(payload.get("confidence", 0.0)),
        )


@dataclass(frozen=True)
class TrackingCat:
    x: float
    y: float
    confidence: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {"x": self.x, "y": self.y, "confidence": self.confidence}

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "TrackingCat":
        return cls(
            x=float(payload["x"]),
            y=float(payload["y"]),
            confidence=float(payload.get("confidence", 0.0)),
        )


@dataclass(frozen=True)
class TrackingMessage:
    sequence: int
    timestamp_ms: int
    car: TrackingCar
    cat: TrackingCat
    frame_id: str = "yard"
    schema_version: int = SCHEMA_VERSION

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": MessageType.TRACKING.value,
            "schema_version": self.schema_version,
            "sequence": self.sequence,
            "timestamp_ms": self.timestamp_ms,
            "frame_id": self.frame_id,
            "car": self.car.to_dict(),
            "cat": self.cat.to_dict(),
        }

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "TrackingMessage":
        _require_type(payload, MessageType.TRACKING)
        _require_schema_version(payload)
        return cls(
            sequence=int(payload["sequence"]),
            timestamp_ms=int(payload["timestamp_ms"]),
            car=TrackingCar.from_dict(payload["car"]),
            cat=TrackingCat.from_dict(payload["cat"]),
            frame_id=str(payload.get("frame_id", "yard")),
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
        return {
            "type": MessageType.COMMAND.value,
            "schema_version": self.schema_version,
            "sequence": self.sequence,
            "timestamp_ms": self.timestamp_ms,
            "command_id": self.command_id,
            "command": self.command.value,
            "params": dict(self.params),
        }

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "CommandMessage":
        _require_type(payload, MessageType.COMMAND)
        _require_schema_version(payload)
        return cls(
            sequence=int(payload["sequence"]),
            timestamp_ms=int(payload["timestamp_ms"]),
            command_id=str(payload["command_id"]),
            command=CommandName(payload["command"]),
            params=dict(payload.get("params", {}) or {}),
            schema_version=int(payload.get("schema_version", SCHEMA_VERSION)),
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
    schema_version: int = SCHEMA_VERSION

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": MessageType.ACK.value,
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
    version = int(payload.get("schema_version", SCHEMA_VERSION))
    if version != SCHEMA_VERSION:
        raise SchemaVersionError(
            f"unsupported schema_version {version}; this build supports {SCHEMA_VERSION}"
        )


__all__ = [
    "AckMessage",
    "CommandMessage",
    "SCHEMA_VERSION",
    "SchemaVersionError",
    "TrackingCar",
    "TrackingCat",
    "TrackingMessage",
]
