"""Tests for costmap steering-envelope sweep."""

from cat_follow.navigation.steering_envelope import (
    CostmapSweepEnvelopeProvider,
    OccupancyGridSnapshot,
    PointEnvelopeProvider,
)
from cat_follow.target_config import TargetRuntimeConfig


def _empty_grid(size=40, resolution=0.05):
    """Free (0) grid centered so pose (0,0) is in the middle."""
    data = [0] * (size * size)
    origin = -0.5 * size * resolution
    return OccupancyGridSnapshot(
        width=size,
        height=size,
        resolution=resolution,
        origin_x=origin,
        origin_y=origin,
        origin_yaw=0.0,
        data=data,
        received_ms=1000,
    )


def _wall_left_grid(size=40, resolution=0.05):
    grid = _empty_grid(size=size, resolution=resolution)
    data = list(grid.data)
    # Occupy left half columns.
    for row in range(size):
        for col in range(size // 2):
            data[row * size + col] = 100
    return OccupancyGridSnapshot(
        width=size,
        height=size,
        resolution=resolution,
        origin_x=grid.origin_x,
        origin_y=grid.origin_y,
        origin_yaw=0.0,
        data=data,
        received_ms=1000,
    )


def test_point_envelope_is_degenerate():
    p = PointEnvelopeProvider()
    r = p.compute(
        path_correction=0.25,
        pose_x_m=0.0,
        pose_y_m=0.0,
        pose_yaw_rad=0.0,
        now_ms=1000,
        costmap=None,
    )
    assert r.envelope_source == "point"
    assert r.safe_steering_min == r.safe_steering_max == 0.25
    assert r.path_viable is True


def test_missing_costmap_fails_closed():
    p = CostmapSweepEnvelopeProvider(TargetRuntimeConfig())
    r = p.compute(
        path_correction=0.0,
        pose_x_m=0.0,
        pose_y_m=0.0,
        pose_yaw_rad=0.0,
        now_ms=1000,
        costmap=None,
    )
    assert r.path_viable is False
    assert r.envelope_source == "none"


def test_stale_costmap_fails_closed():
    cfg = TargetRuntimeConfig(envelope_stale_ttl_ms=100)
    p = CostmapSweepEnvelopeProvider(cfg)
    grid = _empty_grid()
    r = p.compute(
        path_correction=0.0,
        pose_x_m=0.0,
        pose_y_m=0.0,
        pose_yaw_rad=0.0,
        now_ms=grid.received_ms + 500,
        costmap=grid,
    )
    assert r.path_viable is False
    assert r.reason == "costmap_stale"


def test_open_corridor_contains_path_correction():
    cfg = TargetRuntimeConfig(
        envelope_lookahead_m=0.3,
        envelope_sample_count=11,
        envelope_max_half_width=0.8,
        envelope_stale_ttl_ms=1000,
    )
    p = CostmapSweepEnvelopeProvider(cfg)
    grid = _empty_grid()
    r = p.compute(
        path_correction=0.1,
        pose_x_m=0.0,
        pose_y_m=0.0,
        pose_yaw_rad=0.0,
        now_ms=1000,
        costmap=grid,
    )
    assert r.path_viable is True
    assert r.envelope_source == "costmap_sweep"
    assert r.safe_steering_min <= 0.1 <= r.safe_steering_max


def test_wall_left_asymmetric_envelope():
    cfg = TargetRuntimeConfig(
        envelope_lookahead_m=0.35,
        envelope_sample_count=15,
        envelope_max_half_width=0.9,
        envelope_stale_ttl_ms=1000,
        envelope_footprint_width_m=0.12,
        envelope_footprint_length_m=0.2,
    )
    p = CostmapSweepEnvelopeProvider(cfg)
    grid = _wall_left_grid()
    r = p.compute(
        path_correction=0.0,
        pose_x_m=0.0,
        pose_y_m=0.0,
        pose_yaw_rad=0.0,
        now_ms=1000,
        costmap=grid,
    )
    # Left turns more likely blocked; band should not extend fully left if
    # viable, or path_viable false in a tight wall case.
    if r.path_viable:
        assert r.safe_steering_min > -0.9
        assert r.safe_steering_max >= r.safe_steering_min
        # Asymmetry: left (negative) side more restricted than right.
        assert abs(r.safe_steering_min) <= abs(r.safe_steering_max) + 1e-6


def test_path_outside_free_band_fails_closed():
    """When path_correction sits outside every free run, do not publish nearest."""
    cfg = TargetRuntimeConfig(
        envelope_lookahead_m=0.35,
        envelope_sample_count=15,
        envelope_max_half_width=0.9,
        envelope_stale_ttl_ms=1000,
        envelope_footprint_width_m=0.12,
        envelope_footprint_length_m=0.2,
    )
    p = CostmapSweepEnvelopeProvider(cfg)
    grid = _wall_left_grid()
    r = p.compute(
        path_correction=-0.95,
        pose_x_m=0.0,
        pose_y_m=0.0,
        pose_yaw_rad=0.0,
        now_ms=1000,
        costmap=grid,
    )
    assert r.path_viable is False
    assert r.reason == "no_free_band"
