"""UDP sender that ships outbound :class:`AckMessage` packets.

Designed to be plugged into ``CommsManager(ack_sink=sender)``: the sender
is callable and takes an :class:`AckMessage`, serializes it as JSON, and
sends it via UDP to a configured ``(host, port)`` target.

Send failures are caught, logged at warning severity, and never raised so a
flaky link cannot cascade into the control loop.
"""

from __future__ import annotations

import json
import socket
import threading
from typing import Optional, Tuple

from cat_follow.comms.messages import AckMessage
from cat_follow.control.types import (
    TelemetryEventType,
    TelemetrySeverity,
)
from cat_follow.telemetry.async_logger import AsyncLogger


class UdpSender:
    """Callable that ships ``AckMessage`` instances over UDP."""

    def __init__(
        self,
        target_host: str,
        target_port: int,
        logger: Optional[AsyncLogger] = None,
        source: str = "UdpSender",
    ) -> None:
        self._target: Tuple[str, int] = (target_host, target_port)
        self._logger = logger
        self._source = source
        self._lock = threading.Lock()
        self._sock: socket.socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )

    def __call__(self, ack: AckMessage) -> None:
        self.send(ack)

    def send(self, ack: AckMessage) -> None:
        try:
            payload = json.dumps(ack.to_dict(), separators=(",", ":"))
            data = payload.encode("utf-8")
        except (TypeError, ValueError) as exc:
            self._log_send_error("serialization_error", str(exc))
            return

        try:
            with self._lock:
                self._sock.sendto(data, self._target)
        except OSError as exc:
            self._log_send_error("socket_error", str(exc))

    def close(self) -> None:
        with self._lock:
            try:
                self._sock.close()
            except OSError:
                pass

    @property
    def target(self) -> Tuple[str, int]:
        return self._target

    # ── internals ───────────────────────────────────────────────────

    def _log_send_error(self, cause: str, detail: str) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.THREAD_HEALTH,
            severity=TelemetrySeverity.WARNING,
            source=self._source,
            state=None,
            data={
                "event": "udp_send_failed",
                "target_host": self._target[0],
                "target_port": self._target[1],
                "cause": cause,
                "detail": detail,
            },
        )


__all__ = ["UdpSender"]
