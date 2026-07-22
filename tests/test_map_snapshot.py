"""Unit tests for occupancy map snapshot helpers and /api/map."""

from __future__ import annotations

import base64

from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState as PrototypeSharedState
from cat_follow.navigation.map_snapshot import (
    downsample_occupancy,
    downsample_scan,
    get_map_snapshot,
    map_snapshot_dict,
    occupancy_to_uint8,
    publish_map_grid,
    publish_robot_pose,
    publish_scan_overlay,
    reset_map_snapshot_for_tests,
)
from cat_follow.web_ui.app import create_app


def setup_function():
    reset_map_snapshot_for_tests()


def test_occupancy_to_uint8():
    assert occupancy_to_uint8(-1) == 128
    assert occupancy_to_uint8(0) == 0
    assert occupancy_to_uint8(100) == 255
    assert occupancy_to_uint8(50) == 127


def test_downsample_occupancy_prefers_occupied():
    # 4x4 grid: one occupied cell in a free neighborhood.
    data = [0] * 16
    data[5] = 100
    cells, w, h, scale = downsample_occupancy(data, 4, 4, max_dim=2)
    assert w == 2 and h == 2
    assert scale == 2
    assert 255 in cells


def test_downsample_scan_keeps_finite():
    ranges = [float("nan"), 1.0, 2.0, 0.0, 50.0]
    pts = downsample_scan(ranges, 0.0, 0.1, range_max=10.0, max_points=10)
    assert len(pts) == 2
    assert pts[0].range_m == 1.0
    assert pts[1].range_m == 2.0


def test_publish_and_api_map():
    # 10x10 free map with origin at (-1, -2).
    data = [0] * 100
    data[0] = 100
    publish_map_grid(
        data=data,
        width=10,
        height=10,
        resolution_m=0.05,
        origin_x=-1.0,
        origin_y=-2.0,
        origin_yaw=0.0,
    )
    publish_robot_pose(x=0.5, y=-1.0, yaw=1.2, frame="map")
    publish_scan_overlay([1.0, 2.0, 3.0], -1.0, 0.5, 10.0)

    snap = get_map_snapshot()
    assert snap.available is True
    assert snap.width == 10
    assert snap.pose.x == 0.5
    assert snap.pose.fresh is True
    assert len(snap.scan) >= 1
    raw = base64.b64decode(snap.cells_b64)
    assert len(raw) == snap.width * snap.height
    assert raw[0] == 255

    proto = PrototypeSharedState(allocate_pool())
    app = create_app(shared=proto)
    client = app.test_client()
    res = client.get("/api/map")
    assert res.status_code == 200
    data = res.get_json()
    assert data["available"] is True
    assert data["pose"]["frame"] == "map"
    assert data["origin_x"] == -1.0


def test_map_snapshot_dict_empty_note():
    d = map_snapshot_dict()
    assert d["available"] is False
    assert "No map" in d["note"]


def test_freshness_recomputed_at_read_time(monkeypatch):
    import cat_follow.navigation.map_snapshot as ms

    clock = {"t": 1000}
    monkeypatch.setattr(ms, "_now_ms", lambda: clock["t"])

    publish_map_grid(
        data=[0] * 100,
        width=10,
        height=10,
        resolution_m=0.05,
        origin_x=0.0,
        origin_y=0.0,
    )
    publish_robot_pose(x=0.0, y=0.0, yaw=0.0, frame="map")
    publish_scan_overlay([1.0, 2.0], 0.0, 0.5, 10.0)

    d = map_snapshot_dict()
    assert d["pose_fresh"] is True
    assert d["scan_fresh"] is True
    assert d["map_fresh"] is True
    assert d["pose_on_map"] is True
    assert d["pose"]["fresh"] is True

    # Advance the clock past the pose/scan TTL but within the map TTL.
    clock["t"] = 1000 + ms.POSE_STALE_MS + 1
    d = map_snapshot_dict()
    assert d["pose_fresh"] is False
    assert d["scan_fresh"] is False
    assert d["pose_on_map"] is False
    assert d["pose"]["fresh"] is False
    assert d["map_fresh"] is True

    # Advance past the map TTL: map ages out too.
    clock["t"] = 1000 + ms.MAP_STALE_MS + 1
    assert map_snapshot_dict()["map_fresh"] is False


def test_odom_frame_pose_not_flagged_for_map_overlay(monkeypatch):
    import cat_follow.navigation.map_snapshot as ms

    monkeypatch.setattr(ms, "_now_ms", lambda: 5000)

    publish_map_grid(
        data=[0] * 100,
        width=10,
        height=10,
        resolution_m=0.05,
        origin_x=0.0,
        origin_y=0.0,
    )
    # TF failed -> bridge falls back to the odom frame.
    publish_robot_pose(x=1.0, y=2.0, yaw=0.3, frame="odom")

    d = map_snapshot_dict()
    assert d["pose_fresh"] is True
    assert d["pose_on_map"] is False  # odom pose must not overlay the map grid
