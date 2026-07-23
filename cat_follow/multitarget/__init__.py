"""Role-aware predictive multi-target tracking."""

from cat_follow.multitarget.coordinator import MultiTargetCoordinator
from cat_follow.multitarget.predictive_tracker import PredictiveTracker, TrackState
from cat_follow.multitarget.roles import PRIMARY_CAT, SECONDARY_CAT, RoleManager

__all__ = [
    "MultiTargetCoordinator",
    "PredictiveTracker",
    "TrackState",
    "RoleManager",
    "PRIMARY_CAT",
    "SECONDARY_CAT",
]
