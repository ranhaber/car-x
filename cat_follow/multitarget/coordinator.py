"""Coordinate predictive tracks and sticky cat roles."""

from __future__ import annotations

from cat_follow.multitarget.predictive_tracker import PredictiveTracker, TrackState
from cat_follow.multitarget.roles import PRIMARY_CAT, RoleManager


class MultiTargetCoordinator:
    """Small full-frame coordinator used before crop scheduling is introduced."""

    def __init__(
        self,
        *,
        max_distance: float = 300.0,
        max_disappeared: int = 25,
        high_conf: float = 0.30,
        low_conf: float = 0.10,
        velocity_alpha: float = 0.5,
    ) -> None:
        self.tracker = PredictiveTracker(
            max_distance=max_distance,
            max_disappeared=max_disappeared,
            high_conf=high_conf,
            low_conf=low_conf,
            velocity_alpha=velocity_alpha,
        )
        self.roles = RoleManager(max_cats=2)
        self.tracks: dict[int, TrackState] = {}
        self.role_map: dict[str, int] = {}

    def update(self, detections) -> dict[int, str]:
        self.tracks = self.tracker.update(detections)
        roles_by_track = self.roles.update(self.tracks)
        self.role_map = {role: track_id for track_id, role in roles_by_track.items()}
        return roles_by_track

    def state_for_role(self, role: str) -> TrackState | None:
        track_id = self.role_map.get(role)
        return self.tracks.get(track_id) if track_id is not None else None

    def primary(self) -> TrackState | None:
        return self.state_for_role(PRIMARY_CAT)

    def reset(self) -> None:
        self.tracker.reset()
        self.roles.reset()
        self.tracks = {}
        self.role_map = {}
