"""cat_follow.memory — pre-allocated buffer pool and thread-safe shared state."""

from cat_follow.memory.pool import (
    allocate_pool,
    MemoryPool,
    FRAME_H,
    FRAME_W,
    FRAME_C,
    FRAME_SHAPE,
    FRAME_NBYTES,
    BBOX_LEN,
    ODOM_LEN,
)
from cat_follow.memory.shared_state import SharedState

__all__ = [
    "allocate_pool",
    "MemoryPool",
    "SharedState",
    "FRAME_H",
    "FRAME_W",
    "FRAME_C",
    "FRAME_SHAPE",
    "FRAME_NBYTES",
    "BBOX_LEN",
    "ODOM_LEN",
]
