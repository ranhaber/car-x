#!/usr/bin/env python3
"""Headless zerocopy soak gate: fd stability + sustained infer under load.

Run on ROCK 4D with the camera free (service stopped).  Promotion criteria from
the migration plan: ``fd_delta≈0`` over >=300 frames at production cadence.

Usage::

    python scripts/zerocopy_soak.py --frames 300 --device /dev/video11
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np

from cat_follow.perception_config import load_perception_config
from cat_follow.vision.zerocopy_backend import ZerocopySession, runtime_available


def _open_fd_count() -> int:
    try:
        return len(os.listdir("/proc/self/fd")) - 1
    except OSError:
        return -1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video11")
    parser.add_argument(
        "--model", default="models/yolov8n_coco_320_rk3576_int8.rknn"
    )
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--score-threshold", type=float, default=0.5)
    args = parser.parse_args()

    if not runtime_available():
        print(json.dumps({"status": "skip", "reason": "zerocopy unavailable"}))
        return 0

    model_path = Path(args.model)
    if not model_path.is_absolute():
        model_path = ROOT / model_path

    fds_before_open = _open_fd_count()
    session = ZerocopySession.open(
        device=args.device,
        model_path=str(model_path),
        src_w=640,
        src_h=480,
        crop_w=320,
        crop_h=320,
        animal_mode=load_perception_config().animal_mode,
    )
    if session is None:
        print(json.dumps({"status": "fail", "error": "session open failed"}))
        return 1

    fds_before = _open_fd_count()
    tick = 1.0 / max(1.0, args.fps)
    ok_frames = 0
    failures = 0
    rga_ms: list[float] = []
    npu_ms: list[float] = []
    t_start = time.monotonic()

    fds_after = fds_before
    try:
        for _ in range(args.frames):
            t0 = time.monotonic()
            frame = session.dequeue(timeout_ms=3000)
            if frame is None:
                failures += 1
                continue
            session.infer(frame.buffer_index, args.score_threshold)
            session.requeue(frame.buffer_index)
            ok_frames += 1
            rga_ms.append(session.last_perf.get("pre", 0.0))
            npu_ms.append(session.last_perf.get("invoke", 0.0))
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, tick - elapsed))
    finally:
        fds_after = _open_fd_count()
        session.close()

    fds_after_close = _open_fd_count()
    elapsed_s = time.monotonic() - t_start
    report = {
        "status": "pass" if failures == 0 and ok_frames >= args.frames else "fail",
        "frames_ok": ok_frames,
        "frames_failed": failures,
        "duration_s": round(elapsed_s, 2),
        "effective_fps": round(ok_frames / elapsed_s, 2) if elapsed_s > 0 else 0,
        "rga_ms_p50": float(np.median(rga_ms)) if rga_ms else None,
        "npu_ms_p50": float(np.median(npu_ms)) if npu_ms else None,
        "fd_delta": fds_after - fds_before,
        "lifecycle_fd_delta": fds_after_close - fds_before_open,
    }
    print(json.dumps(report))
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
