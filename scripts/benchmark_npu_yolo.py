#!/usr/bin/env python3
"""Standalone NPU-only timing benchmark for YOLOv8 RKNN models.

Measures **only** ``RKNNLite.inference()`` time on the ROCK 4D. Host
preprocess (letterbox / color convert) and YOLO postprocess are excluded so
the numbers reflect NPU invoke cost alone.

Compares four combinations when the matching ``.rknn`` files are present:

  - YOLOv8n 320x320
  - YOLOv8n 640x640
  - YOLOv8s 320x320
  - YOLOv8s 640x640

Default filenames under ``--models-dir`` (override with explicit flags)::

    yolov8n_coco_320_rk3576.rknn
    yolov8n_coco_640_rk3576.rknn
    yolov8s_coco_320_rk3576.rknn
    yolov8s_coco_640_rk3576.rknn

Build missing models on x86 / WSL with::

    python scripts/convert_yolo_to_rknn.py \\
      --onnx yolov8n_320.onnx \\
      --output models/yolov8n_coco_320_rk3576.rknn \\
      --platform rk3576 --no-quant

Run on the ROCK 4D (needs ``rknnlite``)::

    /opt/car-x/venv/bin/python scripts/benchmark_npu_yolo.py --runs 100
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class ModelSpec:
    label: str
    family: str  # n | s
    size: int  # 320 | 640
    path: str


@dataclass(frozen=True)
class BenchResult:
    spec: ModelSpec
    runs: int
    warmup: int
    mean_ms: float
    median_ms: float
    p95_ms: float
    min_ms: float
    max_ms: float
    fps: float


def _percentile(sorted_vals: Sequence[float], pct: float) -> float:
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    rank = (pct / 100.0) * (len(sorted_vals) - 1)
    low = int(rank)
    high = min(low + 1, len(sorted_vals) - 1)
    weight = rank - low
    return sorted_vals[low] * (1.0 - weight) + sorted_vals[high] * weight


def _require_rknnlite():
    try:
        from rknnlite.api import RKNNLite  # type: ignore
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(
            "rknnlite is not importable. Run this script on the ROCK 4D "
            f"inside /opt/car-x/venv. Import error: {exc}"
        ) from exc
    return RKNNLite


def _load_runtime(model_path: str):
    RKNNLite = _require_rknnlite()
    rknn = RKNNLite()
    if rknn.load_rknn(model_path) != 0:
        rknn.release()
        raise RuntimeError(f"load_rknn failed: {model_path}")
    core_mask = getattr(RKNNLite, "NPU_CORE_AUTO", 0)
    if rknn.init_runtime(core_mask=core_mask) != 0:
        rknn.release()
        raise RuntimeError(f"init_runtime failed: {model_path}")
    return rknn


def _make_input(size: int, seed: int = 0) -> np.ndarray:
    """Pre-built NHWC RGB uint8 tensor matching the convert mean=0/std=255 path."""
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, size=(1, size, size, 3), dtype=np.uint8)


def benchmark_model(
    spec: ModelSpec,
    *,
    runs: int,
    warmup: int,
) -> BenchResult:
    if not os.path.isfile(spec.path):
        raise FileNotFoundError(spec.path)

    rknn = _load_runtime(spec.path)
    inp = _make_input(spec.size)
    try:
        for _ in range(max(0, warmup)):
            outputs = rknn.inference(inputs=[inp])
            if outputs is None:
                raise RuntimeError(f"warmup inference returned None: {spec.path}")

        samples_ms: List[float] = []
        for _ in range(runs):
            t0 = time.perf_counter()
            outputs = rknn.inference(inputs=[inp])
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            if outputs is None:
                raise RuntimeError(f"timed inference returned None: {spec.path}")
            samples_ms.append(elapsed_ms)
    finally:
        try:
            rknn.release()
        except Exception:  # noqa: BLE001
            pass

    ordered = sorted(samples_ms)
    mean_ms = statistics.fmean(samples_ms)
    return BenchResult(
        spec=spec,
        runs=runs,
        warmup=warmup,
        mean_ms=mean_ms,
        median_ms=statistics.median(samples_ms),
        p95_ms=_percentile(ordered, 95.0),
        min_ms=min(samples_ms),
        max_ms=max(samples_ms),
        fps=(1000.0 / mean_ms) if mean_ms > 0 else float("nan"),
    )


def _default_specs(models_dir: str) -> List[ModelSpec]:
    naming = (
        ("YOLOv8n 320", "n", 320, "yolov8n_coco_320_rk3576.rknn"),
        ("YOLOv8n 640", "n", 640, "yolov8n_coco_640_rk3576.rknn"),
        ("YOLOv8s 320", "s", 320, "yolov8s_coco_320_rk3576.rknn"),
        ("YOLOv8s 640", "s", 640, "yolov8s_coco_640_rk3576.rknn"),
    )
    return [
        ModelSpec(
            label=label,
            family=family,
            size=size,
            path=os.path.join(models_dir, filename),
        )
        for label, family, size, filename in naming
    ]


def _resolve_specs(args: argparse.Namespace) -> List[ModelSpec]:
    defaults = { (s.family, s.size): s for s in _default_specs(args.models_dir) }
    overrides = {
        ("n", 320): args.n320,
        ("n", 640): args.n640,
        ("s", 320): args.s320,
        ("s", 640): args.s640,
    }
    specs: List[ModelSpec] = []
    for key, default in defaults.items():
        override = overrides[key]
        path = override if override else default.path
        specs.append(
            ModelSpec(
                label=default.label,
                family=default.family,
                size=default.size,
                path=path,
            )
        )
    return specs


def _print_table(results: Iterable[BenchResult], missing: Sequence[ModelSpec]) -> None:
    rows = list(results)
    print()
    print("NPU-only timing (RKNNLite.inference, excludes preprocess/postprocess)")
    print("-" * 88)
    header = (
        f"{'Model':<14} {'Size':>7} {'Runs':>5} "
        f"{'Mean':>8} {'Median':>8} {'P95':>8} {'Min':>8} {'Max':>8} {'FPS':>7}"
    )
    print(header)
    print("-" * 88)
    for result in rows:
        print(
            f"{result.spec.family.upper():<14} "
            f"{result.spec.size:>7} "
            f"{result.runs:>5} "
            f"{result.mean_ms:>7.2f}ms "
            f"{result.median_ms:>7.2f}ms "
            f"{result.p95_ms:>7.2f}ms "
            f"{result.min_ms:>7.2f}ms "
            f"{result.max_ms:>7.2f}ms "
            f"{result.fps:>7.1f}"
        )
    print("-" * 88)

    if missing:
        print("Missing models (skipped):")
        for spec in missing:
            print(f"  - {spec.label}: {spec.path}")
        print(
            "Build them with scripts/convert_yolo_to_rknn.py "
            "--platform rk3576 --no-quant"
        )

    if len(rows) >= 2:
        print()
        print("Relative mean latency (lower is faster):")
        baseline = min(rows, key=lambda item: item.mean_ms)
        for result in rows:
            ratio = result.mean_ms / baseline.mean_ms
            marker = "  (baseline)" if result is baseline else ""
            print(
                f"  {result.spec.label:<14} "
                f"{result.mean_ms:7.2f} ms  "
                f"x{ratio:5.2f}{marker}"
            )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--models-dir",
        default="models",
        help="Directory with default yolov8{n,s}_coco_{320,640}_rk3576.rknn names",
    )
    parser.add_argument("--n320", default=None, help="Override path for YOLOv8n 320")
    parser.add_argument("--n640", default=None, help="Override path for YOLOv8n 640")
    parser.add_argument("--s320", default=None, help="Override path for YOLOv8s 320")
    parser.add_argument("--s640", default=None, help="Override path for YOLOv8s 640")
    parser.add_argument("--runs", type=int, default=100, help="Timed runs per model")
    parser.add_argument("--warmup", type=int, default=10, help="Warmup runs per model")
    args = parser.parse_args(argv)

    if args.runs < 1:
        raise SystemExit("--runs must be >= 1")
    if args.warmup < 0:
        raise SystemExit("--warmup must be >= 0")

    # Fail fast if the lite runtime is missing (laptop / wrong venv).
    _require_rknnlite()

    specs = _resolve_specs(args)
    present = [spec for spec in specs if os.path.isfile(spec.path)]
    missing = [spec for spec in specs if not os.path.isfile(spec.path)]
    if not present:
        print("No RKNN models found. Looked for:", file=sys.stderr)
        for spec in specs:
            print(f"  {spec.path}", file=sys.stderr)
        return 1

    print(
        f"Benchmarking {len(present)}/{len(specs)} models "
        f"(runs={args.runs}, warmup={args.warmup})"
    )
    results: List[BenchResult] = []
    for spec in present:
        print(f"[bench] {spec.label} <- {spec.path}")
        result = benchmark_model(spec, runs=args.runs, warmup=args.warmup)
        results.append(result)
        print(
            f"        mean={result.mean_ms:.2f} ms  "
            f"median={result.median_ms:.2f} ms  "
            f"p95={result.p95_ms:.2f} ms  "
            f"fps={result.fps:.1f}"
        )

    _print_table(results, missing)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
