#!/usr/bin/env python3
"""Run camera + real detector only, without web, tracker, ROS, or motors."""

from __future__ import annotations

import argparse
from dataclasses import replace
import os
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cat_follow.memory.pool import allocate_pool  # noqa: E402
from cat_follow.memory.shared_state import SharedState  # noqa: E402
from cat_follow.perception_config import load_perception_config  # noqa: E402
from cat_follow.threads.camera import run_camera_loop  # noqa: E402
from cat_follow.threads.detector import (  # noqa: E402
    DetectorHandshake,
    run_detector_loop,
)


def _load_env_file(path: str) -> None:
    for raw_line in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        name, value = line.split("=", 1)
        os.environ[name.strip()] = value.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--env-file", default="/opt/car-x/cat_follow/scripts/car-x.env"
    )
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument(
        "--detect-fps",
        type=float,
        default=None,
        help="Override CAT_FOLLOW_PERCEPTION_DETECT_FPS for this run",
    )
    args = parser.parse_args()
    _load_env_file(args.env_file)

    # Force inference every detector tick so a static yard does not make this
    # validation depend on motion. All other production RKNN settings remain.
    base_config = load_perception_config()
    config = replace(
        base_config,
        motion_gating=False,
        idle_unload_sec=0.0,
        detect_fps=(
            args.detect_fps
            if args.detect_fps is not None
            else base_config.detect_fps
        ),
    )
    shared = SharedState(allocate_pool())
    stop_event = threading.Event()
    handshake = DetectorHandshake()
    camera = threading.Thread(
        target=run_camera_loop,
        args=(shared, stop_event),
        name="NV12-Validation-Camera",
        daemon=True,
    )
    detector = threading.Thread(
        target=run_detector_loop,
        args=(shared, stop_event),
        kwargs={"config": config, "handshake": handshake},
        name="NV12-Validation-Detector",
        daemon=True,
    )

    camera.start()
    detector.start()
    try:
        real_backend = handshake.wait_ready(timeout=30.0)
        if not real_backend:
            raise RuntimeError("detector started in stub mode, not RKNN")
        deadline = time.monotonic() + max(1.0, args.seconds)
        detector_gen = -1
        detections = ()
        while time.monotonic() < deadline:
            detections, detector_gen = shared.get_detector_detections_with_gen()
            time.sleep(0.05)

        frame_gen = shared.latest_frame_generation()
        print(f"camera_frame_gen={frame_gen}")
        print(f"detector_frame_gen={detector_gen}")
        print(f"detections={len(detections)}")
        print(f"camera_alive={camera.is_alive()}")
        print(f"detector_alive={detector.is_alive()}")
        if frame_gen < 2:
            raise RuntimeError("camera did not publish two NV12 frames")
        if detector_gen < 1:
            raise RuntimeError("real RKNN detector did not publish a result")
        if not camera.is_alive() or not detector.is_alive():
            raise RuntimeError("perception worker exited during validation")
    finally:
        stop_event.set()
        camera.join(timeout=3.0)
        detector.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
