"""Resolve which local odometry source owns ``/odom`` and ``odom -> base_link``.

Only one publisher may be active at a time:

- ``lidar`` — RF2O (or another ROS scan-matching node) started by bringup.
  This is the production/default source.
- ``bicycle`` — cat_follow's commanded-motion :class:`OdomPublisher`.
  **Disabled.** The contract runtime (``runtime/app.py`` + ``DecisionEngine``)
  never integrates commanded motion into :mod:`cat_follow.odometry`, so the
  bicycle publisher would emit a *frozen* ``/odom`` (and a static
  ``odom -> base_link``). Feeding a stationary pose to slam_toolbox / Nav2 is
  unsafe, so ``bicycle`` is rejected rather than silently publishing frozen
  odometry. Fail closed to ``lidar`` in the tolerant resolver, and raise a
  clear error on any explicit activation path.
"""

from __future__ import annotations

import os
import sys
from typing import Literal, Mapping, Optional, get_args

OdomSource = Literal["lidar", "bicycle"]
_DEFAULT_SOURCE: OdomSource = "lidar"

# Syntactically recognized names (for "unknown value" diagnostics)...
_KNOWN_SOURCES = frozenset(get_args(OdomSource))
# ...versus the subset that may actually be activated.
_SUPPORTED_SOURCES = frozenset({"lidar"})

# Shared, user-facing explanation used everywhere bicycle mode is refused so the
# journal / stderr message is identical regardless of which layer catches it.
BICYCLE_ODOM_DISABLED_MSG = (
    "bicycle odometry source is disabled: the contract runtime does not "
    "integrate commanded motion into cat_follow.odometry, so the bicycle "
    "OdomPublisher would publish a frozen /odom and a static odom->base_link "
    "(unsafe for slam_toolbox/Nav2). Use CAT_FOLLOW_ODOM_SOURCE=lidar so RF2O "
    "scan-matching owns /odom instead."
)


class BicycleOdomDisabledError(ValueError):
    """Raised when the (disabled) bicycle odometry source is requested.

    Subclasses :class:`ValueError` so existing tolerant callers that catch
    ``ValueError`` continue to fall back to the default source.
    """


def bicycle_odom_supported() -> bool:
    """Return whether the bicycle odometry source may be activated (always False)."""
    return "bicycle" in _SUPPORTED_SOURCES


def resolve_odom_source(
    env: Optional[Mapping[str, str]] = None,
    *,
    override: Optional[str] = None,
) -> OdomSource:
    """Return the configured odometry source.

    Precedence: explicit ``override`` > ``CAT_FOLLOW_ODOM_SOURCE`` env >
    default (``lidar``).

    Raises
    ------
    BicycleOdomDisabledError
        When ``bicycle`` is requested (it is a known but disabled source).
    ValueError
        When the resolved value is not a recognized source at all.
    """
    raw = override
    if raw is None:
        source_env = env if env is not None else os.environ
        raw = source_env.get("CAT_FOLLOW_ODOM_SOURCE", _DEFAULT_SOURCE)
    value = str(raw).strip().lower()
    if value not in _KNOWN_SOURCES:
        allowed = ", ".join(sorted(_KNOWN_SOURCES))
        raise ValueError(
            f"invalid odometry source {raw!r}; expected one of: {allowed}"
        )
    if value not in _SUPPORTED_SOURCES:
        raise BicycleOdomDisabledError(BICYCLE_ODOM_DISABLED_MSG)
    return value  # type: ignore[return-value]


def resolve_odom_source_or_default(
    env: Optional[Mapping[str, str]] = None,
    *,
    override: Optional[str] = None,
) -> OdomSource:
    """Like :func:`resolve_odom_source`, but warn and fall back on bad/disabled input.

    A disabled (``bicycle``) or unrecognized source degrades to the production
    default (``lidar``) with a clear stderr warning, so the runtime keeps a
    valid, non-frozen odometry source instead of aborting.
    """
    try:
        return resolve_odom_source(env, override=override)
    except ValueError as exc:
        sys.stderr.write(
            f"warning: {exc}; falling back to odometry source "
            f"{_DEFAULT_SOURCE!r}\n"
        )
        return _DEFAULT_SOURCE


def uses_bicycle_odom_source(source: OdomSource) -> bool:
    """True when the resolved source names bicycle (a disabled source).

    NOTE: because :func:`resolve_odom_source_or_default` never returns
    ``bicycle``, this predicate returns True only for a raw/unchecked value.
    Activation paths must additionally honor :func:`bicycle_odom_supported`.
    """
    return source == "bicycle"


def lidar_odom_launch_enabled(
    env: Optional[Mapping[str, str]] = None,
    *,
    override: Optional[str] = None,
) -> bool:
    """True when ROS bringup should start RF2O lidar odometry.

    With bicycle disabled this is effectively always True (any invalid/disabled
    value falls back to lidar), guaranteeing exactly one non-frozen ``/odom``
    owner.
    """
    return not uses_bicycle_odom_source(
        resolve_odom_source_or_default(env, override=override)
    )
