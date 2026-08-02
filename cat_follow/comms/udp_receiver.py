"""UDP receiver that adapts incoming bytes to ``CommsManager``.

Listens on a configurable bind address/port, parses each datagram as JSON,
dispatches the typed message to ``CommsManager.submit_tracking`` or
``CommsManager.submit_command``, and logs malformed packets via telemetry
without ever killing the receiver thread.

Tracking is published inline because it is a cheap state update.  Commands and
mission events block until the control loop commits them, so they are handed to
a single worker thread: overhead tracking must keep flowing while a command is
in flight, and one worker preserves command ordering.
"""

from __future__ import annotations

import hmac
import json
import queue
import socket
import threading
from typing import Optional, Tuple, Union

from cat_follow.comms.comms_manager import CommsManager
from cat_follow.comms.messages import (
    CommandMessage,
    MissionEventMessage,
    SchemaVersionError,
    TrackingMessage,
)
from cat_follow.control.types import (
    MessageType,
    TelemetryEventType,
    TelemetrySeverity,
)
from cat_follow.telemetry.async_logger import AsyncLogger
from cat_follow.web_ui.control_policy import (
    ALLOW_UNAUTHENTICATED_ENV,
    COMMS_TOKEN_ENV,
    load_control_auth_policy,
)


# Conservative buffer size for incoming JSON datagrams.  Tracking packets
# and command packets are well under 1 KB; 64 KB matches the UDP MTU
# theoretical max and prevents truncation of any sane payload.
DEFAULT_RECV_BUFSIZE = 65535

# Socket timeout makes the recvfrom loop responsive to stop_event without
# requiring asynchronous I/O.
DEFAULT_RECV_TIMEOUT_S = 0.1

# Bound on commands awaiting commit.  Small on purpose: a control loop that is
# too slow to drain this should shed load visibly rather than queue minutes of
# stale operator intent.
DEFAULT_COMMAND_QUEUE_SIZE = 32

# Environment variable holding the shared secret required on command
# datagrams.  When set, a COMMAND packet must carry a matching top-level
# ``token`` field or it is dropped.  When unset, command auth is disabled
# (backwards-compatible), and a warning is emitted at startup.  UDP is an
# unauthenticated, spoofable transport and commands can move the car, so
# operators binding beyond localhost should always configure this.
COMMAND_TOKEN_ENV = COMMS_TOKEN_ENV

_ContractMessage = Union[CommandMessage, MissionEventMessage]
_QueuedMessage = Tuple[_ContractMessage, Tuple[str, int]]


class UdpReceiver:
    """Receives JSON-encoded contract messages via UDP."""

    def __init__(
        self,
        comms_manager: CommsManager,
        bind_host: str = "0.0.0.0",
        bind_port: int = 0,
        logger: Optional[AsyncLogger] = None,
        recv_bufsize: int = DEFAULT_RECV_BUFSIZE,
        recv_timeout_s: float = DEFAULT_RECV_TIMEOUT_S,
        thread_name: str = "CatFollow-Comms-RX",
        source: str = "UdpReceiver",
        command_token: Optional[str] = None,
        command_queue_size: int = DEFAULT_COMMAND_QUEUE_SIZE,
    ) -> None:
        self._comms = comms_manager
        self._bind_host = bind_host
        self._bind_port = bind_port
        self._logger = logger
        self._recv_bufsize = recv_bufsize
        self._recv_timeout_s = recv_timeout_s
        self._thread_name = thread_name
        self._source = source
        # Explicit argument overrides the environment. Missing command auth is
        # allowed only when the operator explicitly enables bench mode.
        policy = load_control_auth_policy()
        if command_token is None:
            command_token = policy.comms_token or ""
        command_token = command_token.strip()
        self._command_token: Optional[str] = command_token or None
        if self._command_token is None and not policy.allow_unauthenticated:
            raise RuntimeError(
                f"{COMMAND_TOKEN_ENV} is required for UDP commands; set it or "
                f"set {ALLOW_UNAUTHENTICATED_ENV}=1 for explicit bench mode"
            )

        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._command_thread: Optional[threading.Thread] = None
        self._command_queue: "queue.Queue[_QueuedMessage]" = queue.Queue(
            maxsize=command_queue_size
        )
        self._stop = threading.Event()

    # ── lifecycle ───────────────────────────────────────────────────

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._bind_host, self._bind_port))
        self._sock.settimeout(self._recv_timeout_s)
        if self._command_token is None:
            self._log_auth_disabled()
        self._command_thread = threading.Thread(
            target=self._run_commands,
            name=f"{self._thread_name}-Cmd",
            daemon=True,
        )
        self._command_thread.start()
        self._thread = threading.Thread(
            target=self._run,
            name=self._thread_name,
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=timeout)
        command_thread = self._command_thread
        self._command_thread = None
        if command_thread is not None:
            command_thread.join(timeout=timeout)
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    @property
    def bound_address(self) -> Optional[Tuple[str, int]]:
        """Return the actual ``(host, port)`` bound, or None before start."""

        if self._sock is None:
            return None
        try:
            return self._sock.getsockname()
        except OSError:
            return None

    # ── internals ───────────────────────────────────────────────────

    def _run(self) -> None:
        sock = self._sock
        if sock is None:
            return
        while not self._stop.is_set():
            try:
                data, addr = sock.recvfrom(self._recv_bufsize)
            except socket.timeout:
                continue
            except OSError:
                # Socket closed or other transport error.  Bail out of the
                # loop; ``stop`` releases resources.
                return
            try:
                self._handle_packet(data, addr)
            except Exception as exc:  # noqa: BLE001
                # Ingress must outlive any single packet: losing this thread
                # would silently stop tracking and command reception until the
                # process restarts.
                self._log_packet_error(addr, "handler_error", repr(exc))

    def _run_commands(self) -> None:
        while True:
            try:
                item = self._command_queue.get(timeout=self._recv_timeout_s)
            except queue.Empty:
                if self._stop.is_set():
                    return
                continue
            if self._stop.is_set():
                self._discard_backlog(item)
                return
            msg, addr = item
            try:
                if isinstance(msg, CommandMessage):
                    self._comms.submit_command(msg)
                else:
                    self._comms.submit_mission_event(msg)
            except TimeoutError as exc:
                # The control loop did not commit within the ACK deadline. The
                # sender retries with the same command_id, which the
                # idempotency cache resolves, so keep serving the queue.
                self._log_packet_error(addr, "commit_timeout", str(exc))
            except Exception as exc:  # noqa: BLE001
                self._log_packet_error(addr, "handler_error", repr(exc))

    def _discard_backlog(self, item: "_QueuedMessage") -> None:
        """Drop queued commands once shutdown has been requested.

        Committing them would be wrong twice over: the operator intent is
        stale, and a control loop that is itself stopping makes every submit
        block for the full commit timeout, which would keep this thread alive
        for ``command_queue_size * commit_timeout`` seconds past ``stop``.
        """

        while True:
            self._log_packet_error(
                item[1],
                "receiver_stopping",
                "command dropped without commit; receiver is shutting down",
            )
            try:
                item = self._command_queue.get_nowait()
            except queue.Empty:
                return

    def _enqueue_command(
        self, msg: "_ContractMessage", addr: Tuple[str, int]
    ) -> None:
        try:
            self._command_queue.put_nowait((msg, addr))
        except queue.Full:
            self._log_packet_error(
                addr,
                "command_queue_full",
                "control loop is not draining queued commands",
            )

    def _handle_packet(self, data: bytes, addr: Tuple[str, int]) -> None:
        try:
            payload = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            self._log_packet_error(addr, "json_decode_error", str(exc))
            return

        msg_type = payload.get("type") if isinstance(payload, dict) else None
        try:
            if msg_type in {
                MessageType.TRACKING.value,
                MessageType.OVERHEAD_OBSERVATION.value,
            }:
                self._comms.submit_tracking(TrackingMessage.from_dict(payload))
                return
            if msg_type == MessageType.COMMAND.value:
                # Commands can move the car; reject unauthenticated datagrams
                # when a shared secret is configured.
                if not self._command_authorized(payload):
                    self._log_packet_error(
                        addr,
                        "unauthorized_command",
                        "missing or invalid token",
                    )
                    return
                self._enqueue_command(CommandMessage.from_dict(payload), addr)
                return
            if msg_type == MessageType.MISSION_EVENT.value:
                if not self._command_authorized(payload):
                    self._log_packet_error(
                        addr,
                        "unauthorized_mission_event",
                        "missing or invalid token",
                    )
                    return
                self._enqueue_command(
                    MissionEventMessage.from_dict(payload), addr
                )
                return
        except SchemaVersionError as exc:
            self._log_packet_error(addr, "schema_version_error", str(exc))
            return
        except (KeyError, ValueError, TypeError) as exc:
            self._log_packet_error(addr, "invalid_payload", str(exc))
            return

        # Unknown / unsupported type.
        self._log_packet_error(
            addr,
            "unsupported_message_type",
            f"type={msg_type!r}",
        )

    def _command_authorized(self, payload: dict) -> bool:
        """Return True if a command datagram is allowed to be dispatched.

        Auth is only enforced when a token is configured.  The comparison is
        constant-time to avoid leaking the secret via timing.
        """
        if self._command_token is None:
            return True
        provided = payload.get("token")
        if not isinstance(provided, str):
            return False
        return hmac.compare_digest(provided, self._command_token)

    def _log_auth_disabled(self) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.THREAD_HEALTH,
            severity=TelemetrySeverity.WARNING,
            source=self._source,
            state=None,
            data={
                "event": "udp_command_auth_disabled",
                "detail": (
                    f"{COMMAND_TOKEN_ENV} not set; UDP commands are "
                    f"unauthenticated on {self._bind_host}:{self._bind_port}"
                ),
            },
        )

    def _log_packet_error(
        self, addr: Tuple[str, int], cause: str, detail: str
    ) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.THREAD_HEALTH,
            severity=TelemetrySeverity.WARNING,
            source=self._source,
            state=None,
            data={
                "event": "udp_packet_dropped",
                "remote_host": addr[0],
                "remote_port": addr[1],
                "cause": cause,
                "detail": detail,
            },
        )


__all__ = [
    "UdpReceiver",
    "DEFAULT_RECV_BUFSIZE",
    "DEFAULT_RECV_TIMEOUT_S",
    "DEFAULT_COMMAND_QUEUE_SIZE",
]
