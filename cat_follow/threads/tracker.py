"""Predictive multi-target tracker thread.

Association runs when the detector publishes a new generation. Between detector
ticks the thread extrapolates the current ``PRIMARY_CAT`` with constant velocity
and republishes at the tracker poll rate so chase/control sees smooth motion.
Only the primary role is written to the legacy ``bbox_tracker`` contract.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from cat_follow.logger import get_logger
from cat_follow.memory.shared_state import SharedState
from cat_follow.multitarget import MultiTargetCoordinator
from cat_follow.multitarget.roles import PRIMARY_CAT, SECONDARY_CAT
from cat_follow.perception.tuning import apply_affinity
from cat_follow.perception_config import load_perception_config

log = get_logger("thread.tracker")


def _xywh_for_state(state, *, extrapolate_fraction: float = 0.0):
    """Translate the last box to the track centroid, optionally coasting forward."""
    if state is None or state.bbox is None:
        return None
    x1, y1, x2, y2 = state.bbox
    width = max(0.0, x2 - x1)
    height = max(0.0, y2 - y1)
    center_x = state.centroid[0] + state.velocity[0] * extrapolate_fraction
    center_y = state.centroid[1] + state.velocity[1] * extrapolate_fraction
    return (
        center_x - width / 2.0,
        center_y - height / 2.0,
        width,
        height,
    )


def _target_snapshot(state, bbox, *, detector_backed: bool):
    if state is None or bbox is None:
        return None
    return (
        state.track_id,
        bbox[0],
        bbox[1],
        bbox[2],
        bbox[3],
        state.confidence,
        (
            state.frames_since_update
            if detector_backed
            else max(1, state.frames_since_update)
        ),
        1.0,
        1.0 if detector_backed else 0.0,
    )


def run_tracker_loop(
    shared: SharedState,
    stop_event: threading.Event,
    *,
    target_fps: float = 30.0,
    on_fps: Optional[Callable[[float], None]] = None,
) -> None:
    """Associate detector results and publish the current primary cat."""
    # Match detector affinity: production pins both to A53 detector cores.
    perception = load_perception_config()
    if perception.affinity_enabled:
        apply_affinity(perception.detector_cores)

    coordinator = MultiTargetCoordinator()
    tick = 1.0 / max(1.0, target_fps)
    last_generation = None
    last_observation_at = None
    observation_interval = None
    fps_counter = 0
    fps_timer = time.monotonic()
    log.info(
        "PredictiveTracker loop started (detection-event + %.0f FPS coast deadline).",
        target_fps,
    )

    while not stop_event.is_set():
        started = time.monotonic()
        fps_counter += 1
        detections, generation = shared.get_detector_detections_with_gen()

        if generation >= 0:
            observation_key = ("multi", generation)
            observation = detections
        else:
            # Backward-compatible path for tests or producers that publish only
            # the old single detector bbox.
            legacy = shared.get_bbox_detector_with_gen()
            observation_key = ("legacy", legacy)
            observation = (
                (
                    legacy[0],
                    legacy[1],
                    legacy[0] + legacy[2],
                    legacy[1] + legacy[3],
                    1.0,
                    17,
                ),
            ) if legacy[4] > 0 else ()

        detector_backed_update = observation_key != last_generation
        if detector_backed_update:
            last_generation = observation_key
            coordinator.update(observation)
            if last_observation_at is not None:
                observation_interval = max(tick, started - last_observation_at)
            last_observation_at = started

        # Tracker velocity is measured in pixels per detector observation, not
        # pixels per 30 Hz poll. Scale elapsed wall time by the measured detector
        # interval and cap prediction to one observation ahead.
        extrapolate_fraction = 0.0
        if last_observation_at is not None and observation_interval is not None:
            extrapolate_fraction = min(
                1.0,
                max(0.0, (started - last_observation_at) / observation_interval),
            )

        role_targets = {}
        role_bboxes = {}
        for role in (PRIMARY_CAT, SECONDARY_CAT):
            state = coordinator.state_for_role(role)
            bbox = _xywh_for_state(
                state,
                extrapolate_fraction=extrapolate_fraction,
            )
            target = _target_snapshot(
                state,
                bbox,
                detector_backed=(
                    detector_backed_update
                    and state is not None
                    and state.frames_since_update == 0
                ),
            )
            if target is not None:
                role_targets[role] = target
                role_bboxes[role] = bbox
        primary = role_bboxes.get(PRIMARY_CAT)
        if primary is None:
            primary_bbox = (0.0, 0.0, 0.0, 0.0, 0.0)
            primary_detector_backed = detector_backed_update
        else:
            primary_state = coordinator.state_for_role(PRIMARY_CAT)
            confidence = max(0.0, min(1.0, float(primary_state.confidence)))
            primary_bbox = (*primary, confidence)
            primary_detector_backed = (
                detector_backed_update
                and primary_state.frames_since_update == 0
            )
        shared.publish_tracking_snapshot(
            role_targets,
            primary_bbox,
            detector_backed=primary_detector_backed,
        )

        now = time.monotonic()
        if now - fps_timer >= 1.0:
            if on_fps is not None:
                try:
                    on_fps(fps_counter / (now - fps_timer))
                except Exception:  # noqa: BLE001
                    log.exception("Tracker FPS callback failed.")
            fps_counter = 0
            fps_timer = now

        elapsed = now - started
        shared.wait_for_detector_update(
            generation,
            stop_event,
            timeout_s=max(0.0, tick - elapsed),
        )

    log.info("PredictiveTracker loop stopped.")
