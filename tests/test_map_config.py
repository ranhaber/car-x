"""Tests for saved-map resolution/validation used by localization/Nav2.

Pure helpers with an injectable ``exists`` predicate so they run without ROS or
a real filesystem map.
"""

import pytest

from cat_follow.navigation.map_config import (
    MAP_FILE_ENV,
    data_path,
    posegraph_path,
    resolve_and_validate_localization_map,
    resolve_map_file,
    validate_localization_map,
)


def test_resolve_map_file_precedence_and_default():
    assert resolve_map_file({}) == ""
    assert resolve_map_file({MAP_FILE_ENV: "/maps/yard"}) == "/maps/yard"
    assert (
        resolve_map_file({MAP_FILE_ENV: "/maps/env"}, override="/maps/cli")
        == "/maps/cli"
    )


def test_resolve_map_file_strips_whitespace_and_suffix():
    assert resolve_map_file({MAP_FILE_ENV: "  /maps/yard  "}) == "/maps/yard"
    assert resolve_map_file({MAP_FILE_ENV: "/maps/yard.posegraph"}) == "/maps/yard"
    assert resolve_map_file({MAP_FILE_ENV: "/maps/yard.data"}) == "/maps/yard"


def test_validate_empty_map_raises_value_error():
    with pytest.raises(ValueError):
        validate_localization_map("")


def test_validate_missing_sidecars_raises_file_not_found():
    with pytest.raises(FileNotFoundError):
        validate_localization_map("/maps/yard", exists=lambda _p: False)


def test_validate_missing_one_sidecar_raises():
    present = {posegraph_path("/maps/yard")}
    with pytest.raises(FileNotFoundError):
        validate_localization_map(
            "/maps/yard", exists=lambda p: p in present
        )


def test_validate_complete_map_ok():
    present = {posegraph_path("/maps/yard"), data_path("/maps/yard")}
    assert (
        validate_localization_map("/maps/yard", exists=lambda p: p in present)
        == "/maps/yard"
    )


def test_resolve_and_validate_end_to_end():
    present = {posegraph_path("/maps/yard"), data_path("/maps/yard")}
    out = resolve_and_validate_localization_map(
        {MAP_FILE_ENV: "/maps/yard.posegraph"},
        exists=lambda p: p in present,
    )
    assert out == "/maps/yard"


def test_resolve_and_validate_unset_raises():
    with pytest.raises(ValueError):
        resolve_and_validate_localization_map({})
