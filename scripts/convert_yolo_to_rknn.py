#!/usr/bin/env python3
"""Convert a YOLOv8 ONNX (airockchip 9-tensor model-zoo head) to ``.rknn``.

Run on **x86 Linux / WSL** with ``rknn-toolkit2`` — not on the ROCK 4D
(the board only has ``rknnlite``).

The ONNX MUST be the airockchip ``rknn_model_zoo`` head:
``3 scales x (box[64], score[80], score_sum[1]) = 9 outputs``.
A plain Ultralytics ``yolo export`` emits one ``(1, 84, N)`` tensor and is
not accepted by the production YOLO postprocess.

This project does not ship a calibration image set. Prefer ``--no-quant``
(FP model). Pass ``--dataset`` only if you later add INT8 calibration images.

Example (rk3576, no quantization)::

    python scripts/convert_yolo_to_rknn.py \\
        --onnx yolov8n_320.onnx \\
        --output models/yolov8n_coco_320_rk3576.rknn \\
        --platform rk3576 \\
        --no-quant

Mean/std ``0 / 255``: feed RGB uint8; the NPU normalizes to 0..1.
"""

from __future__ import annotations

import argparse
import os


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--onnx", required=True, help="Input ONNX (9-tensor head)")
    parser.add_argument("--output", required=True, help="Output .rknn path")
    parser.add_argument(
        "--platform",
        default="rk3576",
        help="target_platform (default rk3576 for ROCK 4D)",
    )
    parser.add_argument(
        "--dataset",
        default=None,
        help="Optional INT8 calibration list (one image path per line)",
    )
    parser.add_argument(
        "--no-quant",
        action="store_true",
        help="Build FP (no INT8). Required when no --dataset is available",
    )
    parser.add_argument("--mean", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    parser.add_argument("--std", type=float, nargs=3, default=[255.0, 255.0, 255.0])
    args = parser.parse_args()

    if not os.path.exists(args.onnx):
        raise SystemExit(f"ONNX not found: {args.onnx}")
    if not args.no_quant and not args.dataset:
        raise SystemExit("INT8 build needs --dataset (or pass --no-quant)")
    if args.dataset and not os.path.exists(args.dataset):
        raise SystemExit(f"dataset list not found: {args.dataset}")

    try:
        from rknn.api import RKNN  # type: ignore
    except ImportError:
        raise SystemExit(
            "rknn-toolkit2 not importable. Run on x86 Linux/WSL with "
            "rknn-toolkit2 installed (the board only has the lite runtime)."
        )

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)

    rknn = RKNN(verbose=True)
    print(f"[convert] config: platform={args.platform} mean={args.mean} std={args.std}")
    rknn.config(
        mean_values=[args.mean],
        std_values=[args.std],
        target_platform=args.platform,
    )

    print(f"[convert] load_onnx: {args.onnx}")
    if rknn.load_onnx(model=args.onnx) != 0:
        raise SystemExit("load_onnx failed")

    do_quant = not args.no_quant
    print(f"[convert] build: do_quantization={do_quant} dataset={args.dataset}")
    if rknn.build(do_quantization=do_quant, dataset=args.dataset) != 0:
        raise SystemExit("build failed")

    print(f"[convert] export_rknn: {args.output}")
    if rknn.export_rknn(args.output) != 0:
        raise SystemExit("export_rknn failed")

    rknn.release()
    size_mb = os.path.getsize(args.output) / (1024.0 * 1024.0)
    print(
        f"[convert] OK -> {args.output} ({size_mb:.1f} MB, "
        f"{'INT8' if do_quant else 'FP'})"
    )


if __name__ == "__main__":
    main()
