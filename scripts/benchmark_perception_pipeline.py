#!/usr/bin/env python3
"""Benchmark detection plus predictive tracking on the ROCK 4D.

Measures the app path:

  BGR frame -> letterbox/RGB (if needed) -> RKNN invoke -> YOLO DFL/NMS
  -> coordinator

Affinity matches production A53 map from ``car-x.env``:

  - optional camera capture / frame prep -> camera cores ``0,1``
  - pre / invoke / post / tracker        -> detector core ``2``

When ``--image`` points at an already-cropped NPU input (e.g. 320x320), the
file is fed as-is. Do not resize those crops to the camera frame size.

Stop ``cat-follow.service`` while this script owns the NPU.
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from collections import defaultdict
from typing import Iterable, Optional, Sequence

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.camera_config import load_camera_config
from cat_follow.memory.pool import FRAME_H, FRAME_W
from cat_follow.multitarget import MultiTargetCoordinator
from cat_follow.perception.tuning import apply_affinity
from cat_follow.threads.camera import _open_capture, _prepare_frame
from cat_follow.vision.rknn_backend import RknnBackend
from cat_follow.vision.nv12_utils import (
    align_nv12_crop,
    extract_nv12_crop,
    nv12_to_bgr,
)


def _percentile(values: Sequence[float], pct: float) -> float:
    return float(np.percentile(np.asarray(values), pct))


def _parse_cores(raw: str) -> tuple[int, ...]:
    cores = tuple(int(token.strip()) for token in raw.split(",") if token.strip())
    if not cores:
        raise SystemExit(f"empty core list: {raw!r}")
    return cores


def _current_affinity() -> Optional[list[int]]:
    getter = getattr(os, "sched_getaffinity", None)
    if getter is None:
        return None
    return sorted(getter(0))


def _pin(cores: Iterable[int], label: str) -> None:
    cores = tuple(cores)
    ok = apply_affinity(cores)
    affinity = _current_affinity()
    print(f"[affinity] {label}: request={list(cores)} ok={ok} now={affinity}")


def _load_image(image_path: str) -> np.ndarray:
    """Load an image without forcing camera-frame resize.

    Already-cropped NPU inputs (e.g. 320x320) must stay native so letterbox
    does not re-scale them into a fake 640x480 frame.
    """
    import cv2

    frame = cv2.imread(image_path)
    if frame is None:
        raise RuntimeError(f"image did not load: {image_path}")
    return np.ascontiguousarray(frame)


def _capture_camera_frame(input_size: tuple[int, int]) -> np.ndarray:
    config = load_camera_config()
    cap = _open_capture(config)
    try:
        if not cap.isOpened():
            raise RuntimeError(f"camera did not open: {config.device}")
        frame = None
        for _ in range(5):
            ok, candidate = cap.read()
            if ok and candidate is not None:
                frame = candidate
        if frame is None:
            raise RuntimeError(f"camera returned no frame: {config.device}")
        prepared = _prepare_frame(frame, config)
        crop_w, crop_h = input_size
        region = align_nv12_crop(
            (FRAME_W - crop_w) // 2,
            FRAME_H - crop_h,
            crop_w,
            crop_h,
            FRAME_W,
            FRAME_H,
        )
        crop = extract_nv12_crop(prepared, FRAME_W, FRAME_H, region)
        return np.ascontiguousarray(nv12_to_bgr(crop, crop_w, crop_h))
    finally:
        cap.release()


def _print_results(
    samples: dict[str, list[float]],
    detections: list[int],
    *,
    camera_cores: Sequence[int],
    detector_cores: Sequence[int],
    best_detection,
) -> None:
    print()
    print("Full perception timing (camera acquisition excluded)")
    print(
        f"Affinity: camera/frame={list(camera_cores)}  "
        f"detect+track={list(detector_cores)}"
    )
    print("-" * 72)
    print(f"{'Stage':<18} {'Mean':>10} {'Median':>10} {'P95':>10} {'FPS':>9}")
    print("-" * 72)
    for stage in ("pre", "invoke", "post", "tracker", "end_to_end"):
        values = samples[stage]
        mean = statistics.fmean(values)
        print(
            f"{stage:<18} {mean:>9.2f}ms "
            f"{statistics.median(values):>9.2f}ms "
            f"{_percentile(values, 95):>9.2f}ms "
            f"{1000.0 / mean:>9.1f}"
        )
    print("-" * 72)
    print(
        f"Detections/run: min={min(detections)} "
        f"median={statistics.median(detections):g} max={max(detections)}"
    )
    if best_detection is None:
        print("Best cat: none")
    else:
        x1, y1, x2, y2, conf, class_id = best_detection
        print(
            "Best cat: "
            f"class={class_id} conf={conf:.3f} "
            f"xyxy=({x1},{y1},{x2},{y2})"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        default="models/yolov8n_coco_320_rk3576_int8.rknn",
    )
    parser.add_argument("--input", default="320,320", help="RKNN input W,H")
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--score-threshold", type=float, default=0.5)
    parser.add_argument(
        "--image",
        default=None,
        help="Already-cropped or full-frame image; native size is preserved",
    )
    parser.add_argument(
        "--camera-cores",
        default="0,1",
        help="A53 cores for camera capture / optional frame prep",
    )
    parser.add_argument(
        "--detector-cores",
        default="2",
        help="A53 cores for pre/invoke/post/tracker",
    )
    parser.add_argument(
        "--animal-mode",
        action="store_true",
        help="Collapse dog/horse/sheep/cow/bear to cat (matches ANIMAL_MODE=1)",
    )
    args = parser.parse_args()
    if args.runs < 1 or args.warmup < 0:
        raise SystemExit("--runs must be >= 1 and --warmup >= 0")

    input_size = tuple(int(value) for value in args.input.split(","))
    if len(input_size) != 2:
        raise SystemExit("--input must be W,H")

    camera_cores = _parse_cores(args.camera_cores)
    detector_cores = _parse_cores(args.detector_cores)

    if args.image:
        # Cropped NPU images are not camera frames; skip camera affinity.
        frame = _load_image(args.image)
        source = args.image
    else:
        _pin(camera_cores, "camera capture")
        frame = _capture_camera_frame(input_size)
        source = "app camera"

    print(
        f"Input frame: {frame.shape[1]}x{frame.shape[0]} BGR ({source}); "
        f"model input {input_size[0]}x{input_size[1]}"
    )
    if (frame.shape[1], frame.shape[0]) == input_size:
        print("Native NPU crop: no letterbox resize")
    else:
        print("Will letterbox into model input")

    _pin(detector_cores, "detect+track")
    backend = RknnBackend(
        args.model, input_size=input_size, animal_mode=args.animal_mode
    )
    coordinator = MultiTargetCoordinator()
    samples: dict[str, list[float]] = defaultdict(list)
    detection_counts: list[int] = []
    best_detection = None
    try:
        backend.self_test()
        for _ in range(args.warmup):
            detections = backend.infer_all(frame, args.score_threshold)
            coordinator.update(detections)

        coordinator.reset()
        for _ in range(args.runs):
            started = time.perf_counter()
            detections = backend.infer_all(frame, args.score_threshold)
            tracker_started = time.perf_counter()
            coordinator.update(detections)
            coordinator.primary()
            finished = time.perf_counter()

            samples["pre"].append(backend.last_perf["pre"])
            samples["invoke"].append(backend.last_perf["invoke"])
            samples["post"].append(backend.last_perf["post"])
            samples["tracker"].append((finished - tracker_started) * 1000.0)
            samples["end_to_end"].append((finished - started) * 1000.0)
            detection_counts.append(len(detections))
            if detections:
                candidate = max(detections, key=lambda item: item[4])
                if best_detection is None or candidate[4] > best_detection[4]:
                    best_detection = candidate
    finally:
        backend.unload()

    _print_results(
        samples,
        detection_counts,
        camera_cores=camera_cores,
        detector_cores=detector_cores,
        best_detection=best_detection,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
