"""
Memory pool: pre-allocate all large buffers at startup.

Call allocate_pool() once at application start.  The returned MemoryPool
holds every buffer that the camera, tracker, detector, and main loop will
use.  No per-frame allocation should happen anywhere else in the hot path.
"""

from dataclasses import dataclass
import numpy as np

# ---------------------------------------------------------------------------
# Frame constants (change here if you switch resolution)
# ---------------------------------------------------------------------------
FRAME_H: int = 480
FRAME_W: int = 640
FRAME_C: int = 3
FRAME_SHAPE: tuple = (FRAME_H, FRAME_W, FRAME_C)
FRAME_NBYTES: int = FRAME_H * FRAME_W * FRAME_C  # 921 600

# Number of rotating frame buffers the camera will write into. Keep small
# (3 is usually enough to avoid reader/writer contention).
FRAME_RING_N: int = 3

# ---------------------------------------------------------------------------
# Bbox layout: 5 floats  [x, y, w, h, valid]
#   indices 0-3 : bounding-box (x, y, width, height) in pixels
#   index   4   : valid flag (1.0 = bbox is current, 0.0 = no detection)
# ---------------------------------------------------------------------------
BBOX_LEN: int = 5

# ---------------------------------------------------------------------------
# Odometry layout: 3 floats  [x, y, heading_deg]
# ---------------------------------------------------------------------------
ODOM_LEN: int = 3


@dataclass
class MemoryPool:
    """Container for every pre-allocated buffer.

    Attributes are *references* to the underlying NumPy arrays.
    Callers write into these arrays in-place; they must never reassign
    the attributes (e.g. ``pool.frame_latest = new_array`` is forbidden).
    """

    # Rotating ring of full-frame buffers (uint8, N x H x W x 3)
    # Camera writes into one slot, readers read the latest published index.
    frame_ring: np.ndarray
    frame_for_detector: np.ndarray

    # Two bbox arrays (float64, length 5 each)
    bbox_tracker: np.ndarray
    bbox_detector: np.ndarray

    # Odometry (float64, length 3)
    odometry_xyh: np.ndarray


def allocate_pool() -> MemoryPool:
    """Allocate every shared buffer once and return a MemoryPool.

    This function must be called exactly once, at application startup,
    before any thread is started.
    """
    # Allocate a small ring of full-frame buffers so the camera can write
    # into a rotating slot and readers can atomically publish the latest
    # index without copying the whole frame twice.
    frame_ring_shape = (FRAME_RING_N, FRAME_H, FRAME_W, FRAME_C)
    return MemoryPool(
        frame_ring=np.zeros(frame_ring_shape, dtype=np.uint8),
        frame_for_detector=np.zeros(FRAME_SHAPE, dtype=np.uint8),
        bbox_tracker=np.zeros(BBOX_LEN, dtype=np.float64),
        bbox_detector=np.zeros(BBOX_LEN, dtype=np.float64),
        odometry_xyh=np.zeros(ODOM_LEN, dtype=np.float64),
    )
