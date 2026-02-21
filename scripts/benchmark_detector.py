"""Benchmark a TFLite detector model on the local machine.

Measures average inference time for N runs on either a captured camera frame
or a synthetic image. Requires `tflite-runtime` or TensorFlow installed.

Usage:
  python scripts/benchmark_detector.py --model models/ssd_mobilenet_v2_320x320.tflite --runs 50
"""

import argparse
import time
import os
import numpy as np

try:
    from tflite_runtime.interpreter import Interpreter
except Exception:
    try:
        from tensorflow.lite import Interpreter
    except Exception:
        Interpreter = None

import cv2


def load_interpreter(model_path: str):
    if Interpreter is None:
        raise RuntimeError("No TFLite interpreter available. Install tflite-runtime or tensorflow.")
    interp = Interpreter(model_path)
    interp.allocate_tensors()
    return interp


def prepare_input(interp, frame):
    idet = interp.get_input_details()[0]
    idx = idet["index"]
    shape = idet["shape"]
    h = int(shape[1])
    w = int(shape[2])
    resized = cv2.resize(frame, (w, h))
    # Try to set uint8 directly; otherwise normalize to float32
    try:
        interp.set_tensor(idx, np.expand_dims(resized, axis=0))
        return idx
    except Exception:
        arr = np.expand_dims(resized.astype(np.float32) / 255.0, axis=0)
        interp.set_tensor(idx, arr)
        return idx


def run_bench(model_path: str, runs: int = 50):
    interp = load_interpreter(model_path)
    # Capture one frame from default camera if available, otherwise random
    cap = None
    frame = None
    try:
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
    except Exception:
        frame = None
    if frame is None:
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    # Warmup
    for _ in range(3):
        prepare_input(interp, frame)
        interp.invoke()

    times = []
    for i in range(runs):
        t0 = time.time()
        prepare_input(interp, frame)
        interp.invoke()
        times.append(time.time() - t0)

    print(f"Model: {model_path}")
    print(f"Runs: {runs}")
    times = np.array(times)
    print(f"Mean: {times.mean()*1000:.2f} ms, Median: {np.median(times)*1000:.2f} ms, 95th: {np.percentile(times,95)*1000:.2f} ms")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", required=True)
    p.add_argument("--runs", type=int, default=50)
    args = p.parse_args()
    if not os.path.exists(args.model):
        print("Model not found:", args.model)
        return
    run_bench(args.model, args.runs)


if __name__ == "__main__":
    main()
