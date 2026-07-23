"""Lightweight token auth for motion-causing web endpoints.

The web UI binds on ``0.0.0.0`` so anyone on the LAN can reach it.  Endpoints
that can *move* the car (or impose meaningful load) are guarded by a shared
secret from ``CAT_FOLLOW_WEB_CONTROL_TOKEN``:

- Production: both ``CAT_FOLLOW_WEB_CONTROL_TOKEN`` and
  ``CAT_FOLLOW_COMMS_TOKEN`` must be non-empty unless
  ``CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL=1`` is set for bench use.
- When a web token is configured, motion endpoints require it via the
  ``X-Control-Token`` header (preferred) or a ``token`` field in the
  JSON/form body.  Query-string tokens are rejected to avoid log leakage.
- When no web token is set and the bench override is present, the guard is
  disabled and a one-time warning is logged.
- When no web token is set and the bench override is absent, mutating
  control endpoints fail closed with ``503``.

Endpoints that only *stop* the car (stop / emergency_stop) are intentionally
left unauthenticated so anyone can always halt the vehicle.
"""

from __future__ import annotations

import functools
import hmac
from typing import Optional

from flask import jsonify, request

from cat_follow.logger import get_logger
from cat_follow.web_ui.control_policy import (
    WEB_CONTROL_TOKEN_ENV,
    load_control_auth_policy,
)

log = get_logger("web_ui.auth")

_warned = False


def control_token() -> Optional[str]:
    """Return the configured web control token, or ``None`` if unset."""
    return load_control_auth_policy().web_token


def auth_enabled() -> bool:
    return control_token() is not None


def warn_if_unauthenticated(bind_host: str) -> None:
    """Log a one-time warning when motor endpoints are open (bench override)."""
    global _warned
    if _warned:
        return
    policy = load_control_auth_policy()
    if policy.web_token is not None:
        return
    _warned = True
    if policy.allow_unauthenticated:
        log.warning(
            "Motor-control web endpoints are UNAUTHENTICATED (bind host %s) "
            "because %s=1. Set %s for production.",
            bind_host,
            "CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL",
            WEB_CONTROL_TOKEN_ENV,
        )
    else:
        log.warning(
            "Motor-control web endpoints refuse unauthenticated access "
            "(bind host %s): set %s (and CAT_FOLLOW_COMMS_TOKEN), or set "
            "CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL=1 for bench use.",
            bind_host,
            WEB_CONTROL_TOKEN_ENV,
        )


def _provided_token() -> Optional[str]:
    """Extract a control token from header or body only (never query args)."""

    tok = request.headers.get("X-Control-Token")
    if tok:
        return tok
    # Intentionally do not read request.args — query tokens leak into logs.
    data = request.get_json(silent=True)
    if isinstance(data, dict) and data.get("token"):
        return str(data["token"])
    if request.form.get("token"):
        return request.form.get("token")
    return None


def require_control_token(fn):
    """Decorator: require a valid control token, or explicit bench override."""

    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        policy = load_control_auth_policy()
        if not policy.is_production_ready:
            return (
                jsonify(
                    {
                        "error": (
                            "control authentication misconfigured: set "
                            f"{WEB_CONTROL_TOKEN_ENV} and CAT_FOLLOW_COMMS_TOKEN "
                            "or CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL=1"
                        )
                    }
                ),
                503,
            )
        expected = policy.web_token
        if expected is None:
            # Explicit bench override with no web token.
            return fn(*args, **kwargs)
        provided = _provided_token()
        if not provided or not hmac.compare_digest(str(provided), str(expected)):
            return (
                jsonify({"error": "unauthorized: missing or invalid control token"}),
                401,
            )
        return fn(*args, **kwargs)

    return wrapper


__all__ = [
    "auth_enabled",
    "control_token",
    "require_control_token",
    "warn_if_unauthenticated",
]
