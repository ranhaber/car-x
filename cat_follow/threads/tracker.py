"""Tracker thread.

Improved tracker with robust re-init logic:
- uses OpenCV single-object tracker (KCF/CSRT/MOSSE) when available
- maintains a short detector-history buffer for temporal confirmation
- computes IoU between tracker and detector to decide merge vs re-init
- enforces cooldown and smoothing to avoid thrash

All frame I/O uses the pre-allocated buffers from SharedState so no
per-frame allocations occur.
"""

import threading
import time
import math
from typing import Optional, Tuple, List

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.memory.shared_state import SharedState
from cat_follow.memory.pool import FRAME_SHAPE

log = get_logger("thread.tracker")


def _create_tracker():
    try:
        import cv2
    except Exception:
        return None

    # Try common creators (new API or legacy)
    creators = ["TrackerKCF_create", "TrackerCSRT_create", "TrackerMOSSE_create"]
    for name in creators:
        try:
            creator = getattr(cv2, name)
            return creator()
        except Exception:
            pass
    try:
        legacy = getattr(cv2, "legacy")
        for name in creators:
            try:
                creator = getattr(legacy, name)
                return creator()
            except Exception:
                pass
    except Exception:
        pass
    return None


def _bbox_iou(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ax2 = ax + aw
    ay2 = ay + ah
    bx2 = bx + bw
    by2 = by + bh
    ix1 = max(ax, bx)
    iy1 = max(ay, by)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = aw * ah
    area_b = bw * bh
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def run_tracker_loop(shared: SharedState, stop_event: threading.Event, *, target_fps: float = 30.0) -> None:
    tick = 1.0 / target_fps
    frame_buf = np.empty(FRAME_SHAPE, dtype=np.uint8)

    # Tracker instance
    tracker = None

    # Re-init / confirmation parameters
    last_reinit = 0.0
    REINIT_COOLDOWN = 0.5
    DET_HISTORY_WINDOW = 1.0
    DET_CONFIRM_N = 2
    DET_CONFIRM_IOU = 0.5
    IOU_REINIT_HIGH = 0.6
    IOU_REINIT_LOW = 0.2

    det_history: List[Tuple[Tuple[float, float, float, float], float]] = []

    tracker_creator = _create_tracker()
    if tracker_creator is None:
        log.info("OpenCV trackers unavailable; tracker will publish detector bboxes only.")

    log.info("Tracker loop started (target %.0f FPS).", target_fps)

    while not stop_event.is_set():
        t0 = time.monotonic()

        # Read latest frame
        shared.get_frame_latest(frame_buf)
        now = time.monotonic()

        # Read detector bbox and maintain short history for confirmation
        det = shared.get_bbox_detector()
        if det[4] > 0:
            det_bbox = (float(det[0]), float(det[1]), float(det[2]), float(det[3]))
            det_history.append((det_bbox, now))
        # prune history
        det_history = [(b, ts) for (b, ts) in det_history if now - ts <= DET_HISTORY_WINDOW]

        # Current tracker snapshot
        tr = shared.get_bbox_tracker()
        tr_valid = tr[4] > 0
        tr_bbox = (float(tr[0]), float(tr[1]), float(tr[2]), float(tr[3])) if tr_valid else None

        # If we have no active tracker, attempt confirmed re-init from detector
        if tracker is None:
            confirmed = False
            if len(det_history) >= DET_CONFIRM_N:
                matches = 0
                for i in range(len(det_history) - 1):
                    if _bbox_iou(det_history[i][0], det_history[i + 1][0]) >= DET_CONFIRM_IOU:
                        matches += 1
                if matches >= (DET_CONFIRM_N - 1):
                    confirmed = True
            if det[4] > 0 and confirmed and (now - last_reinit) >= REINIT_COOLDOWN:
                x, y, w, h = det_history[-1][0]
                bbox = (int(x), int(y), int(w), int(h))
                if tracker_creator is not None:
                    try:
                        tracker = _create_tracker()
                        ok = tracker.init(frame_buf, bbox)
                        if ok:
                            shared.set_bbox_tracker(float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]), 1.0)
                            last_reinit = now
                            log.info("Tracker initialized from confirmed detector bbox: %s", str(bbox))
                        else:
                            tracker = None
                    except Exception as e:
                        log.warning("Tracker init failed: %s", e)
                        tracker = None
                else:
                    # no OpenCV available; publish detector bbox directly
                    shared.set_bbox_tracker(float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]), 1.0)
                    last_reinit = now

        # If we have a tracker, try updating it
        if tracker is not None:
            try:
                ok, newbox = tracker.update(frame_buf)
            except Exception:
                ok = False
                newbox = None

            if ok and newbox is not None:
                nx, ny, nw, nh = newbox
                shared.set_bbox_tracker(float(nx), float(ny), float(nw), float(nh), 1.0)
            else:
                # tracking failed -> mark invalid and drop tracker to allow re-init
                shared.set_bbox_tracker(0.0, 0.0, 0.0, 0.0, 0.0)
                tracker = None

        # If tracker exists and detector is present, decide whether to fuse or re-init
        if tracker is not None and det[4] > 0 and tr_bbox is not None:
            det_bbox = det_history[-1][0] if det_history else (float(det[0]), float(det[1]), float(det[2]), float(det[3]))
            iou = _bbox_iou(tr_bbox, det_bbox)
            if iou >= IOU_REINIT_HIGH:
                # strong agreement -> smooth towards detector bbox
                alpha = 0.4
                fused = (
                    tr_bbox[0] * (1 - alpha) + det_bbox[0] * alpha,
                    tr_bbox[1] * (1 - alpha) + det_bbox[1] * alpha,
                    tr_bbox[2] * (1 - alpha) + det_bbox[2] * alpha,
                    tr_bbox[3] * (1 - alpha) + det_bbox[3] * alpha,
                )
                shared.set_bbox_tracker(float(fused[0]), float(fused[1]), float(fused[2]), float(fused[3]), 1.0)
            elif iou < IOU_REINIT_LOW and (now - last_reinit) >= REINIT_COOLDOWN:
                # strong disagreement -> re-init only if detector is confirmed
                confirmed = False
                if len(det_history) >= DET_CONFIRM_N:
                    matches = 0
                    for i in range(len(det_history) - 1):
                        if _bbox_iou(det_history[i][0], det_history[i + 1][0]) >= DET_CONFIRM_IOU:
                            matches += 1
                    if matches >= (DET_CONFIRM_N - 1):
                        confirmed = True
                if confirmed:
                    x, y, w, h = det_history[-1][0]
                    bbox = (int(x), int(y), int(w), int(h))
                    try:
                        tracker = _create_tracker()
                        ok = tracker.init(frame_buf, bbox)
                        if ok:
                            shared.set_bbox_tracker(float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]), 1.0)
                            last_reinit = now
                            log.info("Tracker re-initialized after disagreement: %s", str(bbox))
                        else:
                            tracker = None
                    except Exception as e:
                        log.warning("Tracker re-init failed: %s", e)
                        tracker = None

        # If no tracker and no detector, ensure bbox invalid
        if tracker is None and det[4] == 0:
            shared.set_bbox_tracker(0.0, 0.0, 0.0, 0.0, 0.0)

        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, tick - elapsed))
