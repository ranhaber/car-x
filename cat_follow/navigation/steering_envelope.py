"""Safe steering envelope providers for NavigationManager.

Production ROS navigation uses :class:`CostmapSweepEnvelopeProvider`.  The
point provider exists for tests and explicit fallback only — it must never
silently widen to ``[-1, 1]``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Protocol, Sequence

from cat_follow.target_config import TargetRuntimeConfig


@dataclass(frozen=True)
class OccupancyGridSnapshot:
    """Local costmap snapshot in its native frame (usually `odom` or `map`)."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Sequence[int]  # row-major, -1 unknown, 0 free, 1..100 occupied
    received_ms: int


@dataclass(frozen=True)
class EnvelopeResult:
    path_viable: bool
    safe_steering_min: float
    safe_steering_max: float
    envelope_source: str
    costmap_age_ms: Optional[int] = None
    reason: str = ""


class SteeringEnvelopeProvider(Protocol):
    def compute(
        self,
        *,
        path_correction: float,
        pose_x_m: float,
        pose_y_m: float,
        pose_yaw_rad: float,
        now_ms: int,
        costmap: Optional[OccupancyGridSnapshot],
    ) -> EnvelopeResult:
        ...


class PointEnvelopeProvider:
    """Conservative point envelope: min = max = path_correction."""

    def compute(
        self,
        *,
        path_correction: float,
        pose_x_m: float,
        pose_y_m: float,
        pose_yaw_rad: float,
        now_ms: int,
        costmap: Optional[OccupancyGridSnapshot],
    ) -> EnvelopeResult:
        steer = max(-1.0, min(1.0, float(path_correction)))
        return EnvelopeResult(
            path_viable=True,
            safe_steering_min=steer,
            safe_steering_max=steer,
            envelope_source="point",
            costmap_age_ms=None,
            reason="point_envelope",
        )


class CostmapSweepEnvelopeProvider:
    """Short-horizon bicycle-model sweep against a local occupancy grid."""

    def __init__(self, config: Optional[TargetRuntimeConfig] = None) -> None:
        self._config = config or TargetRuntimeConfig()

    def compute(
        self,
        *,
        path_correction: float,
        pose_x_m: float,
        pose_y_m: float,
        pose_yaw_rad: float,
        now_ms: int,
        costmap: Optional[OccupancyGridSnapshot],
    ) -> EnvelopeResult:
        cfg = self._config
        if costmap is None:
            return EnvelopeResult(
                path_viable=False,
                safe_steering_min=0.0,
                safe_steering_max=0.0,
                envelope_source="none",
                costmap_age_ms=None,
                reason="costmap_missing",
            )
        age = max(0, int(now_ms) - int(costmap.received_ms))
        if age > cfg.envelope_stale_ttl_ms:
            return EnvelopeResult(
                path_viable=False,
                safe_steering_min=0.0,
                safe_steering_max=0.0,
                envelope_source="none",
                costmap_age_ms=age,
                reason="costmap_stale",
            )

        samples = cfg.envelope_sample_count
        # Normalized steer samples in [-max_half, +max_half]
        half = cfg.envelope_max_half_width
        steers = [
            -half + (2.0 * half * i / (samples - 1)) for i in range(samples)
        ]
        free = [
            self._arc_free(
                steer,
                pose_x_m=pose_x_m,
                pose_y_m=pose_y_m,
                pose_yaw_rad=pose_yaw_rad,
                costmap=costmap,
            )
            for steer in steers
        ]

        path = max(-1.0, min(1.0, float(path_correction)))
        # Viable only when a free band contains path_correction (fail closed).
        best = self._band_containing(steers, free, path)
        if best is None:
            return EnvelopeResult(
                path_viable=False,
                safe_steering_min=0.0,
                safe_steering_max=0.0,
                envelope_source="costmap_sweep",
                costmap_age_ms=age,
                reason="no_free_band",
            )
        lo, hi = best
        return EnvelopeResult(
            path_viable=True,
            safe_steering_min=lo,
            safe_steering_max=hi,
            envelope_source="costmap_sweep",
            costmap_age_ms=age,
            reason="sweep_ok",
        )

    def _arc_free(
        self,
        steer_norm: float,
        *,
        pose_x_m: float,
        pose_y_m: float,
        pose_yaw_rad: float,
        costmap: OccupancyGridSnapshot,
    ) -> bool:
        cfg = self._config
        # Map normalized steer to curvature via max steer angle ~30 deg.
        max_steer_rad = math.radians(30.0)
        delta = float(steer_norm) * max_steer_rad
        wheelbase = cfg.envelope_wheelbase_m
        steps = 8
        ds = cfg.envelope_lookahead_m / steps
        x, y, yaw = pose_x_m, pose_y_m, pose_yaw_rad
        for _ in range(steps):
            if abs(delta) < 1e-6:
                x += ds * math.cos(yaw)
                y += ds * math.sin(yaw)
            else:
                radius = wheelbase / math.tan(delta)
                dtheta = ds / radius
                yaw += dtheta
                x += radius * (math.sin(yaw) - math.sin(yaw - dtheta))
                y += -radius * (math.cos(yaw) - math.cos(yaw - dtheta))
            if not self._footprint_free(x, y, yaw, costmap):
                return False
        return True

    def _footprint_free(
        self,
        x: float,
        y: float,
        yaw: float,
        costmap: OccupancyGridSnapshot,
    ) -> bool:
        cfg = self._config
        half_l = 0.5 * cfg.envelope_footprint_length_m
        half_w = 0.5 * cfg.envelope_footprint_width_m
        corners = [
            (half_l, half_w),
            (half_l, -half_w),
            (-half_l, half_w),
            (-half_l, -half_w),
            (0.0, 0.0),
        ]
        c, s = math.cos(yaw), math.sin(yaw)
        for lx, ly in corners:
            wx = x + c * lx - s * ly
            wy = y + s * lx + c * ly
            if not self._cell_free(wx, wy, costmap):
                return False
        return True

    def _cell_free(
        self, x: float, y: float, costmap: OccupancyGridSnapshot
    ) -> bool:
        # Transform world -> grid assuming origin yaw ~ 0 for local costmap.
        dx = x - costmap.origin_x
        dy = y - costmap.origin_y
        if abs(costmap.origin_yaw) > 1e-6:
            c = math.cos(-costmap.origin_yaw)
            s = math.sin(-costmap.origin_yaw)
            dx, dy = c * dx - s * dy, s * dx + c * dy
        if costmap.resolution <= 0:
            return False
        col = int(dx / costmap.resolution)
        row = int(dy / costmap.resolution)
        if col < 0 or row < 0 or col >= costmap.width or row >= costmap.height:
            return False
        idx = row * costmap.width + col
        if idx < 0 or idx >= len(costmap.data):
            return False
        val = int(costmap.data[idx])
        if val < 0:
            return False  # unknown treated as blocked for fail-closed chase
        return val < self._config.envelope_lethal_cost

    @staticmethod
    def _band_containing(
        steers: Sequence[float], free: Sequence[bool], path: float
    ) -> Optional[tuple[float, float]]:
        n = len(steers)
        # Contiguous free runs
        runs: list[tuple[int, int]] = []
        i = 0
        while i < n:
            if not free[i]:
                i += 1
                continue
            j = i
            while j + 1 < n and free[j + 1]:
                j += 1
            runs.append((i, j))
            i = j + 1
        if not runs:
            return None
        containing = [
            (i, j)
            for i, j in runs
            if steers[i] - 1e-9 <= path <= steers[j] + 1e-9
        ]
        if containing:
            i, j = containing[0]
            return steers[i], steers[j]
        # Spec: viable envelope MUST contain path_correction.  Do not fail
        # open onto a nearest free band that excludes the planned steer.
        return None


def make_envelope_provider(
    config: Optional[TargetRuntimeConfig] = None,
) -> SteeringEnvelopeProvider:
    cfg = config or TargetRuntimeConfig()
    if cfg.envelope_provider == "point":
        return PointEnvelopeProvider()
    return CostmapSweepEnvelopeProvider(cfg)


__all__ = [
    "CostmapSweepEnvelopeProvider",
    "EnvelopeResult",
    "OccupancyGridSnapshot",
    "PointEnvelopeProvider",
    "SteeringEnvelopeProvider",
    "make_envelope_provider",
]
