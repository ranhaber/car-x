"""Resolve and validate the saved map used for slam_toolbox localization / Nav2.

First-time *mapping* is map-free (lidar RF2O + slam_toolbox builds a new map),
so this module is only consulted on the localization / navigation path.  Normal
localization must fail *clearly* when no saved map is configured rather than
silently starting slam_toolbox with an empty ``map_file_name`` (which would
localize against nothing).

``CAT_FOLLOW_MAP_FILE`` (or the ``map_file`` launch argument) names the saved
map's *basename without extension*.  slam_toolbox serializes a map as two
sidecar files:

- ``<basename>.posegraph``
- ``<basename>.data``

These pure helpers have no ROS dependency so they can be unit-tested in CI and
reused from the launch file.
"""

from __future__ import annotations

import os
from typing import Callable, Mapping, Optional

MAP_FILE_ENV = "CAT_FOLLOW_MAP_FILE"

POSEGRAPH_SUFFIX = ".posegraph"
DATA_SUFFIX = ".data"


def resolve_map_file(
    env: Optional[Mapping[str, str]] = None,
    *,
    override: Optional[str] = None,
) -> str:
    """Return the configured saved-map basename (may be empty).

    Precedence: explicit ``override`` > ``CAT_FOLLOW_MAP_FILE`` env > "".
    Any surrounding whitespace and a stray ``.posegraph``/``.data`` extension
    are stripped so callers may pass either the basename or a sidecar path.
    """
    raw = override
    if raw is None:
        source = env if env is not None else os.environ
        raw = source.get(MAP_FILE_ENV, "")
    value = str(raw).strip()
    for suffix in (POSEGRAPH_SUFFIX, DATA_SUFFIX):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
            break
    return value


def posegraph_path(map_file: str) -> str:
    return map_file + POSEGRAPH_SUFFIX


def data_path(map_file: str) -> str:
    return map_file + DATA_SUFFIX


def validate_localization_map(
    map_file: str,
    *,
    exists: Callable[[str], bool] = os.path.exists,
) -> str:
    """Validate that ``map_file`` is configured and its serialized map exists.

    Returns the validated basename.

    Raises
    ------
    ValueError
        When ``map_file`` is empty/unset (localization has no map to use).
    FileNotFoundError
        When the expected ``<basename>.posegraph`` (or ``.data``) is missing.
    """
    if not map_file:
        raise ValueError(
            f"no saved map configured: set {MAP_FILE_ENV} (or the 'map_file' "
            "launch argument) to the basename of a map saved during a mapping "
            "session. Run mapping.launch.py first; localization/Nav2 cannot "
            "start without a saved map."
        )
    missing = [
        path
        for path in (posegraph_path(map_file), data_path(map_file))
        if not exists(path)
    ]
    if missing:
        raise FileNotFoundError(
            "configured map is incomplete; expected serialized slam_toolbox "
            f"files are missing: {', '.join(missing)}. Re-run the mapping "
            "session or fix CAT_FOLLOW_MAP_FILE."
        )
    return map_file


def resolve_and_validate_localization_map(
    env: Optional[Mapping[str, str]] = None,
    *,
    override: Optional[str] = None,
    exists: Callable[[str], bool] = os.path.exists,
) -> str:
    """Resolve ``CAT_FOLLOW_MAP_FILE`` and validate the serialized map exists."""
    return validate_localization_map(
        resolve_map_file(env, override=override), exists=exists
    )
