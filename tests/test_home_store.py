"""Durable home store persistence tests."""

import json
import os

import pytest

from cat_follow.home.store import HomePersistError, HomeStore


def test_commit_persists_and_reloads(tmp_path):
    path = tmp_path / "home.json"
    store = HomeStore(str(path), map_id="yard-map", calibration_version=3)
    home = store.commit(x=120.0, y=40.0, frame_id="yard", yaw_rad=0.1)
    assert home.home_version == 1
    assert home.valid
    assert path.exists()

    reloaded = HomeStore(
        str(path), map_id="yard-map", calibration_version=3
    ).load()
    assert reloaded is not None
    assert reloaded.home_version == 1
    assert reloaded.x == 120.0
    assert reloaded.y == 40.0
    assert reloaded.checksum.startswith("sha256:")


def test_corrupt_checksum_fails_closed(tmp_path):
    path = tmp_path / "home.json"
    store = HomeStore(str(path))
    store.commit(x=1.0, y=2.0)
    raw = json.loads(path.read_text(encoding="utf-8"))
    raw["checksum"] = "sha256:deadbeef"
    path.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(HomePersistError, match="checksum"):
        HomeStore(str(path)).load()


def test_version_bumps_on_each_commit(tmp_path):
    store = HomeStore(str(tmp_path / "home.json"))
    first = store.commit(x=0.0, y=0.0)
    second = store.commit(x=10.0, y=10.0)
    assert first.home_version == 1
    assert second.home_version == 2
