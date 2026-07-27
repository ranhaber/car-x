#!/usr/bin/env python3
"""Concurrent camera + detector + H.264 soak with DMA-BUF ownership gates.

Run on ROCK 4D after deploying H.264 ownership fixes::

    python3 scripts/board_soak_concurrent_h264.py \\
        --env-file cat_follow/scripts/car-x.env --seconds 90

Pass gates:
  - capture p95 <= 34 ms, max < 40 ms, zero stalls >= 50 ms (detector log)
  - detect FPS >= 29.5 from detector log rows, contiguous generations
  - stream chunks FPS >= 28 using production submit/poll path
  - fd_delta == 0 after thread join, zero requeue errors in soak logs
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
from cat_follow.perception.h264_encoder import MppH264Encoder  # noqa: E402
from cat_follow.perception_config import load_perception_config  # noqa: E402
from cat_follow.threads.camera import run_camera_loop  # noqa: E402
from cat_follow.threads.detector import DetectorHandshake, run_detector_loop  # noqa: E402
from scripts.soak_h264_metrics import (  # noqa: E402
    capture_stats,
    contiguous_generations,
    count_requeue_errors_since,
    parse_detect_perf_since,
)


def _load_env_file(path: str) -> None:
    for raw_line in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        name, value = line.split("=", 1)
        os.environ[name.strip()] = value.strip()


def _open_fd_count() -> int:
    try:
        return len(os.listdir("/proc/self/fd")) - 1
    except OSError:
        return -1


def _run_h264_consumer(
    shared: SharedState,
    stop_event: threading.Event,
    stats: dict,
) -> None:
    encoder = MppH264Encoder(640, 480, fps=30, pixel_format="NV12")
    if not encoder.start():
        stats["encoder_error"] = "start failed"
        return
    last_frame_gen = 0
    try:
        while not stop_event.is_set():
            for chunk in encoder.poll():
                stats["stream_chunks"] += 1
                stats["stream_bytes"] += len(chunk)

            frame_gen = shared.wait_for_new_frame(
                last_frame_gen,
                stop_event,
                timeout_s=0.1,
            )
            if frame_gen <= last_frame_gen:
                continue
            frame_lease = shared.acquire_latest_frame()
            if frame_lease is None:
                continue
            if frame_lease.frame_seq <= last_frame_gen:
                frame_lease.release()
                continue
            last_frame_gen = frame_lease.frame_seq
            if frame_lease.dmabuf:
                chunks = encoder.submit_dmabuf(frame_lease)
            else:
                with frame_lease:
                    chunks = (
                        encoder.submit(frame_lease.frame)
                        if frame_lease.frame is not None
                        else []
                    )
            for chunk in chunks:
                stats["stream_chunks"] += 1
                stats["stream_bytes"] += len(chunk)
    finally:
        encoder.stop()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--env-file", default=str(ROOT / "cat_follow" / "scripts" / "car-x.env")
    )
    parser.add_argument("--seconds", type=float, default=90.0)
    parser.add_argument("--warmup-s", type=float, default=5.0)
    args = parser.parse_args()
    if Path(args.env_file).exists():
        _load_env_file(args.env_file)

    if not MppH264Encoder.available():
        print(json.dumps({"status": "fail", "error": "MPP H.264 unavailable"}))
        return 1

    config = replace(
        load_perception_config(),
        motion_gating=False,
        idle_unload_sec=0.0,
    )
    shared = SharedState(allocate_pool())
    stop_event = threading.Event()
    handshake = DetectorHandshake()
    stats: dict = {
        "stream_chunks": 0,
        "stream_bytes": 0,
    }

    camera = threading.Thread(
        target=run_camera_loop,
        args=(shared, stop_event),
        name="Soak-Camera",
        daemon=False,
    )
    detector = threading.Thread(
        target=run_detector_loop,
        args=(shared, stop_event),
        kwargs={"config": config, "handshake": handshake},
        name="Soak-Detector",
        daemon=False,
    )
    h264 = threading.Thread(
        target=_run_h264_consumer,
        args=(shared, stop_event, stats),
        name="Soak-H264",
        daemon=False,
    )

    log_since = time.time()
    fds_before = _open_fd_count()
    camera.start()
    detector.start()
    h264.start()

    if not handshake.wait_ready(timeout=30.0):
        stop_event.set()
        print(json.dumps({"status": "fail", "error": "detector stub mode"}))
        return 1

    t0 = time.monotonic()
    deadline = t0 + max(5.0, args.seconds)
    camera_gen_start = shared.latest_frame_generation()
    while time.monotonic() < deadline:
        time.sleep(0.005)

    elapsed = time.monotonic() - t0
    steady_elapsed = max(1e-6, elapsed - max(0.0, args.warmup_s))
    stop_event.set()
    camera.join(timeout=5.0)
    detector.join(timeout=5.0)
    h264.join(timeout=5.0)
    fds_after = _open_fd_count()

    threads_alive = any(thread.is_alive() for thread in (camera, detector, h264))
    detect_rows = parse_detect_perf_since(log_since)
    if args.warmup_s > 0 and detect_rows:
        first_gen = detect_rows[0][0]
        skip = int(args.warmup_s * 30)
        detect_rows = [row for row in detect_rows if row[0] >= first_gen + skip]
    cap = capture_stats(detect_rows)
    detect_fps = len(detect_rows) / steady_elapsed if detect_rows else 0.0
    stream_fps = stats["stream_chunks"] / steady_elapsed if steady_elapsed > 0 else 0.0
    camera_gen = shared.latest_frame_generation()
    camera_samples = max(0, camera_gen - camera_gen_start)
    requeue_errors = count_requeue_errors_since(log_since)

    gates = {
        "capture_p95_le_34": cap["capture_p95_ms"] <= 34.0 if detect_rows else False,
        "capture_max_lt_40": cap["capture_max_ms"] < 40.0 if detect_rows else False,
        "zero_stalls_ge_50": cap["stalls_ge_50ms"] == 0 if detect_rows else False,
        "detect_fps_ge_29_5": detect_fps >= 29.5 if detect_rows else False,
        "contiguous_detector_gens": contiguous_generations(detect_rows),
        "detect_matches_camera": abs(len(detect_rows) - camera_samples) <= 2,
        "stream_fps_ge_28": stream_fps >= 28.0,
        "fd_delta_zero": fds_after == fds_before,
        "zero_requeue_errors": requeue_errors == 0,
        "threads_joined": not threads_alive,
    }
    passed = all(gates.values()) and stats.get("encoder_error") is None

    report = {
        "status": "pass" if passed else "fail",
        "duration_s": round(elapsed, 2),
        "steady_duration_s": round(steady_elapsed, 2),
        "camera_frame_gen": camera_gen,
        "camera_samples": camera_samples,
        "detect_samples": len(detect_rows),
        "detect_fps": round(detect_fps, 2),
        "stream_chunks": stats["stream_chunks"],
        "stream_fps": round(stream_fps, 2),
        "capture_p95_ms": cap["capture_p95_ms"],
        "capture_max_ms": cap["capture_max_ms"],
        "stalls_ge_50ms": cap["stalls_ge_50ms"],
        "requeue_errors": requeue_errors,
        "fd_before": fds_before,
        "fd_after": fds_after,
        "threads_alive": threads_alive,
        "gates": gates,
    }
    if stats.get("encoder_error"):
        report["encoder_error"] = stats["encoder_error"]
    print(json.dumps(report, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
