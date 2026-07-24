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
FRAME_BGR_SHAPE: tuple = (FRAME_H, FRAME_W, FRAME_C)
FRAME_NV12_SHAPE: tuple = (FRAME_H * 3 // 2, FRAME_W)
# FRAME_SHAPE is the shared ring-slot shape. The ring stores native NV12.
FRAME_SHAPE: tuple = FRAME_NV12_SHAPE
FRAME_NBYTES: int = FRAME_H * FRAME_W * 3 // 2  # 460 800

# Detector + MJPEG + H.264 may pin independent slots concurrently. Four slots
# leave the single camera writer one reclaimable slot without blocking.
FRAME_RING_N: int = 4

# ---------------------------------------------------------------------------
# Bbox layout: 5 floats  [x, y, w, h, confidence/valid]
#   indices 0-3 : bounding-box (x, y, width, height) in pixels
#   index   4   : confidence; any positive value means the bbox is valid
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
    the attributes (e.g. ``pool.frame_ring = new_array`` is forbidden).
    """

    # Rotating ring of packed NV12 buffers (uint8, N x (H*3/2) x W)
    # Camera writes into one slot, readers read the latest published index.
    frame_ring: np.ndarray

    # Two bbox arrays (float64, length 5 each)
    bbox_tracker: np.ndarray
    bbox_detector: np.ndarray

    # Odometry (float64, length 3)
    odometry: np.ndarray


def allocate_pool() -> MemoryPool:
    """Allocate every shared buffer once and return a MemoryPool.

    This function must be called exactly once, at application startup,
    before any thread is started.
    """
    # Allocate a small ring of full-frame buffers so the camera can write
    # into a rotating slot and readers can atomically publish the latest
    # index without copying the whole frame twice.
    frame_ring_shape = (FRAME_RING_N, *FRAME_NV12_SHAPE)
    return MemoryPool(
        frame_ring=np.zeros(frame_ring_shape, dtype=np.uint8),
        bbox_tracker=np.zeros(BBOX_LEN, dtype=np.float64),
        bbox_detector=np.zeros(BBOX_LEN, dtype=np.float64),
        odometry=np.zeros(ODOM_LEN, dtype=np.float64),
    )
