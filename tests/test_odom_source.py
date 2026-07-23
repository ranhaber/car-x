"""Tests for odometry source selection.

Bicycle odometry is disabled (the contract runtime never integrates commanded
motion, so it would publish a frozen /odom). Lidar RF2O is the production
default and the only supported source.
"""

import pytest

from cat_follow.navigation.odom_source import (
    BICYCLE_ODOM_DISABLED_MSG,
    BicycleOdomDisabledError,
    bicycle_odom_supported,
    lidar_odom_launch_enabled,
    resolve_odom_source,
    resolve_odom_source_or_default,
    uses_bicycle_odom_source,
)
from cat_follow.runtime.app import build_app


def test_resolve_odom_source_defaults_to_lidar():
    assert resolve_odom_source({}) == "lidar"


def test_resolve_odom_source_rejects_bicycle_as_disabled():
    with pytest.raises(BicycleOdomDisabledError):
        resolve_odom_source({"CAT_FOLLOW_ODOM_SOURCE": "bicycle"})


def test_resolve_odom_source_override_wins():
    assert (
        resolve_odom_source(
            {"CAT_FOLLOW_ODOM_SOURCE": "bicycle"},
            override="lidar",
        )
        == "lidar"
    )


def test_resolve_odom_source_rejects_unknown():
    with pytest.raises(ValueError):
        resolve_odom_source({"CAT_FOLLOW_ODOM_SOURCE": "wheel"})


def test_resolve_odom_source_or_default_warns_and_falls_back(capsys):
    source = resolve_odom_source_or_default({"CAT_FOLLOW_ODOM_SOURCE": "wheel"})
    assert source == "lidar"
    assert "warning:" in capsys.readouterr().err


def test_resolve_odom_source_or_default_falls_back_on_bicycle(capsys):
    source = resolve_odom_source_or_default(
        {"CAT_FOLLOW_ODOM_SOURCE": "bicycle"}
    )
    assert source == "lidar"
    err = capsys.readouterr().err
    assert "warning:" in err
    assert "disabled" in err


def test_bicycle_odom_not_supported():
    assert bicycle_odom_supported() is False
    assert "frozen" in BICYCLE_ODOM_DISABLED_MSG


def test_uses_bicycle_odom_source_predicate():
    assert uses_bicycle_odom_source("bicycle") is True
    assert uses_bicycle_odom_source("lidar") is False


def test_lidar_odom_launch_always_enabled_now():
    # Bicycle is disabled, so RF2O lidar odometry is always the enabled default.
    assert lidar_odom_launch_enabled({"CAT_FOLLOW_ODOM_SOURCE": "lidar"}) is True
    assert lidar_odom_launch_enabled({"CAT_FOLLOW_ODOM_SOURCE": "bicycle"}) is True


def test_build_app_threads_start_bicycle_odom_flag():
    app = build_app(start_bicycle_odom=True)
    assert app.start_bicycle_odom is True


def test_main_bicycle_request_falls_back_to_lidar(monkeypatch):
    import threading

    from cat_follow.runtime.app import main

    captured = {}

    class _FakeApp:
        def start(self) -> None:
            pass

        def stop(self, timeout: float = 2.0) -> None:
            pass

    def _fake_build_app(**kwargs):
        captured.update(kwargs)
        return _FakeApp()

    monkeypatch.setattr("cat_follow.runtime.app.build_app", _fake_build_app)
    monkeypatch.setattr("cat_follow.runtime.app._install_signal_handlers", lambda _e: None)
    monkeypatch.setattr(threading.Event, "wait", lambda self, timeout=None: True)

    # Requesting the disabled bicycle source degrades to lidar (no frozen odom).
    assert main(["--ros-nav", "--odom-source", "bicycle"]) == 0
    assert captured["start_bicycle_odom"] is False
