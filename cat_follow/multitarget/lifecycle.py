"""Inactive future policy for role-aware crop tracking.

This module is intentionally not exported or wired into the current full-frame
tracker. Its crop/overlap evidence inputs do not yet exist in the live pipeline;
callers must not assume it governs current track freshness or loss behavior.
"""

HELD = "HELD"
COASTING = "COASTING"
LOST = "LOST"


class RoleLifecycle:
    """Track loss evidence without treating an unsearched region as empty."""

    def __init__(
        self,
        now: float = 0.0,
        empty_k: int = 2,
        normal_timeout: float = 2.5,
        occlusion_timeout: float = 8.0,
    ) -> None:
        self.empty_k = int(empty_k)
        self.normal_timeout = float(normal_timeout)
        self.occlusion_timeout = float(occlusion_timeout)
        self.state = HELD
        self.coast_ref_t = now
        self.empty_count = 0
        self._occluded_prev = False

    def observe(
        self,
        seen: bool,
        region_cropped: bool = False,
        overlap: bool = False,
        interior: bool = False,
        in_perimeter: bool = True,
        now: float = 0.0,
    ) -> str:
        if self.state == LOST:
            return LOST
        if not in_perimeter:
            self.state = LOST
            return LOST
        if seen:
            self.state = HELD
            self.coast_ref_t = now
            self.empty_count = 0
            self._occluded_prev = False
            return HELD

        occluded = bool(overlap or interior)
        if self._occluded_prev and not occluded:
            self.coast_ref_t = now
            self.empty_count = 0
        self._occluded_prev = occluded
        if occluded:
            self.empty_count = 0
            if now - self.coast_ref_t >= self.occlusion_timeout:
                self.state = LOST
                return LOST
            self.state = COASTING
            return COASTING
        if region_cropped:
            self.empty_count += 1
            if self.empty_count >= self.empty_k:
                self.state = LOST
                return LOST
        if now - self.coast_ref_t >= self.normal_timeout:
            self.state = LOST
            return LOST
        self.state = COASTING
        return COASTING

    def is_lost(self) -> bool:
        return self.state == LOST
