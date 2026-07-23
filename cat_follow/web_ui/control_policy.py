"""Production vs bench control-token policy.

Production requires both web and UDP control tokens unless the operator
explicitly opts into unauthenticated bench mode via
``CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL=1``.

This module is intentionally free of Flask / UDP dependencies so both the
web UI and runtime/comms startup can share the same rules.  Runtime and
UDP callers outside the Web UI subset must still invoke these helpers at
startup (see remaining integration notes).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional

WEB_CONTROL_TOKEN_ENV = "CAT_FOLLOW_WEB_CONTROL_TOKEN"
COMMS_TOKEN_ENV = "CAT_FOLLOW_COMMS_TOKEN"
ALLOW_UNAUTHENTICATED_ENV = "CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL"


def _env_flag(name: str) -> bool:
    raw = os.environ.get(name, "").strip().lower()
    return raw in {"1", "true", "yes", "on"}


def _env_token(name: str) -> Optional[str]:
    raw = os.environ.get(name, "").strip()
    return raw or None


@dataclass(frozen=True)
class ControlAuthPolicy:
    """Resolved control authentication posture."""

    allow_unauthenticated: bool
    web_token: Optional[str]
    comms_token: Optional[str]

    @property
    def web_auth_required(self) -> bool:
        return self.web_token is not None

    @property
    def comms_auth_required(self) -> bool:
        return self.comms_token is not None

    @property
    def is_production_ready(self) -> bool:
        """True when both tokens are set, or bench override is explicit."""

        if self.allow_unauthenticated:
            return True
        return self.web_token is not None and self.comms_token is not None

    @property
    def mode(self) -> str:
        if self.allow_unauthenticated:
            return "bench_override"
        if self.web_token is not None and self.comms_token is not None:
            return "required"
        return "misconfigured"


def load_control_auth_policy() -> ControlAuthPolicy:
    """Read control auth posture from the environment."""

    return ControlAuthPolicy(
        allow_unauthenticated=_env_flag(ALLOW_UNAUTHENTICATED_ENV),
        web_token=_env_token(WEB_CONTROL_TOKEN_ENV),
        comms_token=_env_token(COMMS_TOKEN_ENV),
    )


def require_production_control_tokens(
    policy: Optional[ControlAuthPolicy] = None,
) -> ControlAuthPolicy:
    """Return the policy or raise if production tokens are missing.

    Callers that own process startup (``runtime/app.py``, UDP receiver
    construction) should invoke this before enabling motion or listening
    for commands.  The web UI uses it to fail closed when a motion
    endpoint is hit without a configured token and without the bench
    override.
    """

    resolved = policy or load_control_auth_policy()
    if resolved.is_production_ready:
        return resolved
    missing = []
    if resolved.web_token is None:
        missing.append(WEB_CONTROL_TOKEN_ENV)
    if resolved.comms_token is None:
        missing.append(COMMS_TOKEN_ENV)
    raise RuntimeError(
        "Control authentication is misconfigured for production: missing "
        + ", ".join(missing)
        + f". Set both tokens, or set {ALLOW_UNAUTHENTICATED_ENV}=1 for "
        "explicit unauthenticated bench operation."
    )


__all__ = [
    "ALLOW_UNAUTHENTICATED_ENV",
    "COMMS_TOKEN_ENV",
    "ControlAuthPolicy",
    "WEB_CONTROL_TOKEN_ENV",
    "load_control_auth_policy",
    "require_production_control_tokens",
]
