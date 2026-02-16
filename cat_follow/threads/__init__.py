"""cat_follow.threads — worker thread entry points (camera, tracker, detector)."""

from cat_follow.threads.camera import run_camera_loop
from cat_follow.threads.tracker import run_tracker_loop
from cat_follow.threads.detector import run_detector_loop

__all__ = [
    "run_camera_loop",
    "run_tracker_loop",
    "run_detector_loop",
]
