"""Benchmark the RKNN NPU detector on the ROCK 4D.

Measures average inference time for N runs on either a captured camera frame
or a synthetic image, using the same :class:`RknnBackend` the runtime uses.
Requires ``rknnlite`` (present on Rockchip vendor images) and a converted
``.rknn`` model (see ``scripts/convert_to_rknn.py``).

Usage:
  python scripts/benchmark_detector.py --model models/ssd_mobilenet_v2.rknn --runs 50
"""

import argparse
import os
import sys
import time

import numpy as np

# Ensure the repo root (not scripts/) is importable from a fresh checkout.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.vision.rknn_backend import RknnBackend
from cat_follow.vision.ssd_postprocess import validate_ssd_output_contract


def run_bench(
    model_path: str, runs: int = 50, input_size=(320, 320), strict: bool = True
) -> None:
    backend = RknnBackend(model_path, input_size=input_size)
    if not backend.runtime_available():
        raise RuntimeError(
            "RKNN runtime (rknnlite) not available. Run this on the ROCK 4D."
        )
    if not backend.available():
        raise RuntimeError(f"RKNN model not found: {model_path}")

    # Strict validation up front: load + one real inference + output-contract
    # check.  Without this a broken/incompatible model would silently return
    # empty detections and still yield plausible-looking timings.
    if strict:
        backend.self_test()  # raises on load / inference / contract failure
    elif not backend.load():
        raise RuntimeError(f"Failed to load RKNN model: {model_path}")

    # Capture one frame from the default camera if available, otherwise random.
    frame = None
    try:
        import cv2

        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        if not ret:
            frame = None
    except Exception:
        frame = None
    if frame is None:
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # Warmup, and in strict mode confirm the real frame also yields a valid
    # output contract (not just the dummy self-test frame).
    for _ in range(3):
        outputs = backend._raw_infer(frame)
        if strict:
            validate_ssd_output_contract(outputs)

    times = []
    for _ in range(runs):
        t0 = time.time()
        backend._raw_infer(frame)
        times.append(time.time() - t0)
    backend.unload()

    times = np.array(times)
    print(f"Model: {model_path}")
    print(f"Runs: {runs} ({'strict' if strict else 'non-strict'})")
    print(
        f"Mean: {times.mean()*1000:.2f} ms, "
        f"Median: {np.median(times)*1000:.2f} ms, "
        f"95th: {np.percentile(times, 95)*1000:.2f} ms"
    )


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--model", required=True)
    p.add_argument("--runs", type=int, default=50)
    p.add_argument(
        "--input",
        default="320,320",
        help="Model input size 'W,H' (default 320,320)",
    )
    p.add_argument(
        "--no-strict",
        dest="strict",
        action="store_false",
        help="Skip output-contract validation (measure timing only)",
    )
    args = p.parse_args()
    if not os.path.exists(args.model):
        print("Model not found:", args.model)
        return
    w, h = (int(x) for x in args.input.split(","))
    run_bench(args.model, args.runs, input_size=(w, h), strict=args.strict)


if __name__ == "__main__":
    main()
