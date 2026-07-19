"""Perception adapters bridging prototype sensors into ``SharedState``."""

from cat_follow.perception.range_adapter import RangeAdapter
from cat_follow.perception.vision_adapter import VisionAdapter

__all__ = ["RangeAdapter", "VisionAdapter"]
