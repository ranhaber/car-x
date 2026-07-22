"""Lightweight token auth for motion-causing web endpoints.

The web UI binds on ``0.0.0.0`` so anyone on the LAN can reach it.  Endpoints
that can *move* the car are therefore guarded by a shared secret read from the
``CAT_FOLLOW_WEB_CONTROL_TOKEN`` environment variable:

- If the variable is set, motion endpoints require the token, supplied either as
  the ``X-Control-Token`` header, a ``token`` query parameter, or a ``token``
  field in the JSON/form body.  A missing/invalid token yields ``401``.
- If the variable is unset the guard is disabled (backwards-compatible local-dev
  behavior) and a one-time warning is logged so operators know the motor paths
  are open.

Endpoints that only *stop* the car (stop / emergency_stop) are intentionally
left unauthenticated so anyone can always halt the vehicle.
"""

from __future__ import annotations

import functools
import hmac
import os
from typing import Optional

from flask import jsonify, request

from cat_follow.logger import get_logger

log = get_logger("web_ui.auth")

_ENV_VAR = "CAT_FOLLOW_WEB_CONTROL_TOKEN"
_warned = False


def control_token() -> Optional[str]:
    """Return the configured control token, or ``None`` if auth is disabled."""
    tok = os.environ.get(_ENV_VAR, "").strip()
    return tok or None


def auth_enabled() -> bool:
    return control_token() is not None


def warn_if_unauthenticated(bind_host: str) -> None:
    """Log a one-time warning when motor endpoints are open (no token set)."""
    global _warned
    if not auth_enabled() and not _warned:
        _warned = True
        log.warning(
            "Motor-control web endpoints are UNAUTHENTICATED (bind host %s). "
            "Set %s to require a token for motion commands.",
            bind_host,
            _ENV_VAR,
        )


def _provided_token() -> Optional[str]:
    tok = request.headers.get("X-Control-Token")
    if tok:
        return tok
    tok = request.args.get("token")
    if tok:
        return tok
    data = request.get_json(silent=True)
    if isinstance(data, dict) and data.get("token"):
        return str(data["token"])
    if request.form.get("token"):
        return request.form.get("token")
    return None


def require_control_token(fn):
    """Decorator: require a valid control token when auth is enabled."""

    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        expected = control_token()
        if expected is None:
            return fn(*args, **kwargs)  # auth disabled
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
