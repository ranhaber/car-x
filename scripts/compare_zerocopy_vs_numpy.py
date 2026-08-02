#!/usr/bin/env python3
"""Compare Option A (NumPy/RKNNLite) vs Option B (native zerocopy) detections.

Board gate script: run while ``cat-follow.service`` is stopped so ``/dev/video11``
is exclusive.  Reports detection-count deltas, top-box IoU, latency, and fd growth.

Usage (on ROCK 4D)::

    python scripts/compare_zerocopy_vs_numpy.py \\
        --device /dev/video11 \\
        --model models/yolov8n_coco_320_rk3576_int8.rknn \\
        --frames 60
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np

from cat_follow.perception_config import load_perception_config
from cat_follow.vision.backends import create_backend
from cat_follow.vision.rga_crop import crop_center_bottom_nv12
from cat_follow.vision.nv12_utils import nv12_to_bgr
from cat_follow.vision.zerocopy_backend import ZerocopySession, runtime_available


def _iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - inter
    return float(inter / union) if union > 0 else 0.0


def _open_fd_count() -> int:
    try:
        return len(os.listdir("/proc/self/fd")) - 1
    except OSError:
        return -1


def _parity_failures(
    *,
    frame_count: int,
    count_delta: int,
    top_box_iou_p50: float | None,
    max_count_delta: int,
    min_top_box_iou: float,
) -> list[str]:
    failures = []
    if count_delta > max_count_delta:
        failures.append(
            f"detection_count_delta {count_delta} exceeds {max_count_delta}"
        )
    if top_box_iou_p50 is not None and top_box_iou_p50 < min_top_box_iou:
        failures.append(
            f"top_box_iou_p50 {top_box_iou_p50:.6f} is below "
            f"{min_top_box_iou:.6f}"
        )
    if frame_count == 0:
        failures.append("no frames were compared")
    return failures


def _numpy_path_detect(
    session_frames: list[np.ndarray],
    *,
    model: str,
    score_threshold: float,
) -> list[list[tuple]]:
    config = load_perception_config()
    backend = create_backend(
        model,
        input_size=config.rknn_input_size,
        animal_mode=config.animal_mode,
        input_format=config.rknn_input_format,
    )
    if not backend.load():
        raise RuntimeError(f"failed to load RKNN model {model}")
    crop_buf = np.empty((320 * 3 // 2, 320), dtype=np.uint8)
    crop_bgr = np.empty((320, 320, 3), dtype=np.uint8)
    results = []
    for frame in session_frames:
        crop, ox, oy = crop_center_bottom_nv12(
            frame, 640, 480, 320, 320, dst=crop_buf
        )
        if hasattr(backend, "infer_all_nv12") and backend.input_format == "nv12":
            dets = backend.infer_all_nv12(crop, score_threshold)
        else:
            nv12_to_bgr(crop, 320, 320, dst=crop_bgr)
            dets = backend.infer_all(crop_bgr, score_threshold)
        results.append(
            [
                (d[0] + ox, d[1] + oy, d[2] + ox, d[3] + oy, d[4], d[5])
                for d in dets
            ]
        )
    backend.unload()
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video11")
    parser.add_argument(
        "--model", default="models/yolov8n_coco_320_rk3576_int8.rknn"
    )
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--score-threshold", type=float, default=0.5)
    parser.add_argument(
        "--max-count-delta",
        type=int,
        default=0,
        help="maximum aggregate detection-count difference (default: 0)",
    )
    parser.add_argument(
        "--min-top-box-iou",
        type=float,
        default=0.90,
        help="minimum median top-box IoU when comparable boxes exist (default: 0.90)",
    )
    parser.add_argument("--numpy-worker", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be at least 1")
    if args.max_count_delta < 0:
        parser.error("--max-count-delta must be non-negative")
    if not 0.0 <= args.min_top_box_iou <= 1.0:
        parser.error("--min-top-box-iou must be between 0 and 1")

    model_path = Path(args.model)
    if not model_path.is_absolute():
        model_path = ROOT / model_path

    if args.numpy_worker:
        frames = list(np.load(args.numpy_worker, allow_pickle=False))
        results = _numpy_path_detect(
            frames,
            model=str(model_path),
            score_threshold=args.score_threshold,
        )
        print("PARITY_JSON=" + json.dumps(results))
        return 0

    if not runtime_available():
        print(
            json.dumps(
                {
                    "status": "skip",
                    "reason": "zerocopy runtime unavailable on this host",
                }
            )
        )
        return 0

    if not model_path.is_file():
        print(json.dumps({"status": "fail", "error": f"model missing: {model_path}"}))
        return 1

    animal_mode = load_perception_config().animal_mode

    def _open_session():
        return ZerocopySession.open(
            device=args.device,
            model_path=str(model_path),
            src_w=640,
            src_h=480,
            crop_w=320,
            crop_h=320,
            animal_mode=animal_mode,
        )

    fds_before_open = _open_fd_count()
    session = _open_session()
    if session is None:
        print(json.dumps({"status": "fail", "error": "cf_zc_open failed"}))
        return 1

    fds_before = _open_fd_count()
    frames_nv12: list[np.ndarray] = []
    zc_results: list[list[tuple]] = []
    zc_lat_rga: list[float] = []
    zc_lat_npu: list[float] = []

    fds_after = fds_before
    try:
        for _ in range(args.frames):
            zc_frame = session.dequeue(timeout_ms=5000)
            if zc_frame is None:
                break
            try:
                nv12 = np.empty((480 * 3 // 2, 640), dtype=np.uint8)
                if not session.copy_camera_nv12(zc_frame.buffer_index, nv12):
                    raise RuntimeError(
                        f"camera NV12 copy failed: {session.last_error}"
                    )
                frames_nv12.append(nv12)
                dets = session.infer(zc_frame.buffer_index, args.score_threshold)
                ox, oy = session.offset_x, session.offset_y
                zc_results.append(
                    [
                        (
                            d[0] + ox,
                            d[1] + oy,
                            d[2] + ox,
                            d[3] + oy,
                            d[4],
                            d[5],
                        )
                        for d in dets
                    ]
                )
                zc_lat_rga.append(session.last_perf.get("pre", 0.0))
                zc_lat_npu.append(session.last_perf.get("invoke", 0.0))
            finally:
                if not session.requeue(zc_frame.buffer_index):
                    raise RuntimeError(
                        f"camera buffer requeue failed: {session.last_error}"
                    )
    finally:
        fds_after = _open_fd_count()
        session.close()

    fds_after_close = _open_fd_count()
    # RKNN/RGA may retain a bounded process-global driver descriptor after the
    # first initialization. A second open/close cycle distinguishes that
    # one-time initialization from a per-session leak, which must remain zero.
    repeat_fds_before = _open_fd_count()
    repeat_session = _open_session()
    if repeat_session is None:
        raise RuntimeError("repeat cf_zc_open failed during lifecycle leak gate")
    repeat_session.close()
    repeat_lifecycle_fd_delta = _open_fd_count() - repeat_fds_before
    # RKNNLite cannot always initialize after a native rknn_destroy in the same
    # process. Run Option A in a clean child process over the exact copied
    # frames, preserving same-frame parity without opening the camera twice.
    with tempfile.NamedTemporaryFile(suffix=".npy", delete=False) as tmp:
        frames_path = tmp.name
    try:
        np.save(frames_path, np.stack(frames_nv12))
        worker = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                "--model",
                str(model_path),
                "--score-threshold",
                str(args.score_threshold),
                "--numpy-worker",
                frames_path,
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if worker.returncode != 0:
            raise RuntimeError(
                "Option A worker failed:\nstdout:\n"
                + worker.stdout.strip()
                + "\nstderr:\n"
                + worker.stderr.strip()
            )
        result_lines = [
            line.removeprefix("PARITY_JSON=")
            for line in worker.stdout.splitlines()
            if line.startswith("PARITY_JSON=")
        ]
        if not result_lines:
            raise RuntimeError(
                "Option A worker emitted no result:\n" + worker.stdout.strip()
            )
        np_results = json.loads(result_lines[-1])
    finally:
        os.unlink(frames_path)

    count_delta = 0
    ious: list[float] = []
    pairs = min(len(zc_results), len(np_results))
    for index in range(pairs):
        count_delta += abs(len(zc_results[index]) - len(np_results[index]))
        if zc_results[index] and np_results[index]:
            a = zc_results[index][0]
            b = np_results[index][0]
            ious.append(
                _iou((a[0], a[1], a[2], a[3]), (b[0], b[1], b[2], b[3]))
            )

    top_box_iou_p50 = float(np.median(ious)) if ious else None
    failures = _parity_failures(
        frame_count=pairs,
        count_delta=count_delta,
        top_box_iou_p50=top_box_iou_p50,
        max_count_delta=args.max_count_delta,
        min_top_box_iou=args.min_top_box_iou,
    )
    steady_fd_delta = fds_after - fds_before
    if steady_fd_delta != 0:
        failures.append(f"steady fd delta {steady_fd_delta} != 0")
    if repeat_lifecycle_fd_delta != 0:
        failures.append(
            f"repeat lifecycle fd delta {repeat_lifecycle_fd_delta} != 0"
        )

    report = {
        "status": "fail" if failures else "pass",
        "frames": pairs,
        "detection_count_delta": count_delta,
        "top_box_iou_p50": top_box_iou_p50,
        "top_box_iou_min": float(min(ious)) if ious else None,
        "thresholds": {
            "max_count_delta": args.max_count_delta,
            "min_top_box_iou": args.min_top_box_iou,
        },
        "failures": failures,
        "zc_rga_ms_p50": float(np.median(zc_lat_rga)) if zc_lat_rga else None,
        "zc_npu_ms_p50": float(np.median(zc_lat_npu)) if zc_lat_npu else None,
        "fd_delta": steady_fd_delta,
        "lifecycle_fd_delta": fds_after_close - fds_before_open,
        "repeat_lifecycle_fd_delta": repeat_lifecycle_fd_delta,
    }
    print(json.dumps(report))
    return 1 if failures else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(json.dumps({"status": "fail", "error": str(error)}))
        raise SystemExit(1) from None
