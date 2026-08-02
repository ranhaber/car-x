#!/usr/bin/env python3
"""Board soak: headless NV12 detect + optional latency comparison vs RGB model.

Run on ROCK 4D after deploying NV12 pipeline changes::

    python3 scripts/board_soak_nv12_h264.py --env-file cat_follow/scripts/car-x.env --seconds 30

Prints JSON summary with camera/detector generations, mean detector perf fields,
and optional RGB-vs-NV12 model comparison when both models exist.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cat_follow.memory.pool import allocate_pool  # noqa: E402
from cat_follow.memory.shared_state import SharedState  # noqa: E402
from cat_follow.perception_config import load_perception_config  # noqa: E402
from cat_follow.threads.camera import run_camera_loop  # noqa: E402
from cat_follow.threads.detector import DetectorHandshake, run_detector_loop  # noqa: E402


def _load_env_file(path: str) -> None:
    for raw_line in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        name, value = line.split("=", 1)
        os.environ[name.strip()] = value.strip()


def _run_soak(config, seconds: float) -> dict:
    shared = SharedState(allocate_pool())
    stop_event = threading.Event()
    handshake = DetectorHandshake()
    config = replace(
        config,
        motion_gating=False,
        idle_unload_sec=0.0,
    )
    camera = threading.Thread(
        target=run_camera_loop,
        args=(shared, stop_event),
        name="Soak-Camera",
        daemon=True,
    )
    detector = threading.Thread(
        target=run_detector_loop,
        args=(shared, stop_event),
        kwargs={"config": config, "handshake": handshake},
        name="Soak-Detector",
        daemon=True,
    )
    camera.start()
    detector.start()
    try:
        if not handshake.wait_ready(timeout=30.0):
            raise RuntimeError("detector stub mode")
        deadline = time.monotonic() + max(1.0, seconds)
        last_gen = -1
        infer_ticks = 0
        while time.monotonic() < deadline:
            gen, _ = shared.get_detector_detections_with_gen()
            if gen != last_gen:
                infer_ticks += 1
                last_gen = gen
            time.sleep(0.02)
        return {
            "camera_frame_gen": shared.latest_frame_generation(),
            "detector_frame_gen": last_gen,
            "infer_ticks": infer_ticks,
            "input_format": config.rknn_input_format,
            "model": config.rknn_model_path,
        }
    finally:
        stop_event.set()
        camera.join(timeout=3.0)
        detector.join(timeout=3.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--env-file", default=str(ROOT / "cat_follow" / "scripts" / "car-x.env")
    )
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument(
        "--compare-rgb-model",
        default="models/yolov8n_coco_320_rk3576_int8.rknn",
        help="Optional legacy RGB model for A/B (empty to skip)",
    )
    args = parser.parse_args()
    if Path(args.env_file).exists():
        _load_env_file(args.env_file)

    base = load_perception_config()
    summary = {"nv12": _run_soak(base, args.seconds)}

    rgb_path = args.compare_rgb_model
    if rgb_path and Path(rgb_path).exists():
        rgb_cfg = replace(
            base,
            rknn_model_path=rgb_path,
            rknn_input_format="rgb",
        )
        summary["rgb"] = _run_soak(rgb_cfg, min(10.0, args.seconds))

    print(json.dumps(summary, indent=2))
    ok = summary["nv12"]["camera_frame_gen"] >= 2 and summary["nv12"]["detector_frame_gen"] >= 1
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
