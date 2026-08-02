#!/usr/bin/env python3
"""Convert a YOLOv8 ONNX (airockchip 9-tensor model-zoo head) to ``.rknn``.

Run on **x86 Linux / WSL** with ``rknn-toolkit2`` — not on the ROCK 4D
(the board only has ``rknnlite``).

The ONNX MUST be the airockchip ``rknn_model_zoo`` head:
``3 scales x (box[64], score[80], score_sum[1]) = 9 outputs``.
A plain Ultralytics ``yolo export`` emits one ``(1, 84, N)`` tensor and is
not accepted by the production YOLO postprocess.

The supported input format is RGB uint8 ``(1, H, W, 3)`` with mean=0/std=255.
RKNN Toolkit2 cannot turn a three-channel YOLO ONNX tensor into a packed NV12
tensor by overriding ``input_size_list``. Use RGA NV12→RGB conversion before
RKNN for a zero-copy camera path.

Example (rk3576 RGB INT8)::

    python scripts/convert_yolo_to_rknn.py \\
        --onnx yolov8n_320.onnx \\
        --output models/yolov8n_coco_320_rk3576_int8.rknn \\
        --platform rk3576 \\
        --dataset calib_rgb.txt
"""

from __future__ import annotations

import argparse
import os


def _parse_input_format(value: str) -> str:
    fmt = value.strip().lower()
    if fmt not in ("rgb", "nv12"):
        raise argparse.ArgumentTypeError("input-format must be rgb or nv12")
    return fmt


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
        "--input-format",
        type=_parse_input_format,
        default="rgb",
        choices=("rgb", "nv12"),
        help="Runtime input layout (default rgb)",
    )
    parser.add_argument(
        "--input-width",
        type=int,
        default=320,
        help="Model input width (default 320)",
    )
    parser.add_argument(
        "--input-height",
        type=int,
        default=320,
        help="Model input height (default 320)",
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

    if args.input_format == "nv12":
        raise SystemExit(
            "packed NV12 model conversion is unsupported: the YOLO ONNX input "
            "is RGB [N,3,H,W]. Build an RGB RKNN model and use RGA NV12->RGB."
        )
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

    in_w = int(args.input_width)
    in_h = int(args.input_height)
    input_size_list = [[1, in_h, in_w, 3]]

    rknn = RKNN(verbose=True)
    print(
        f"[convert] config: platform={args.platform} "
        f"input={args.input_format} size={input_size_list[0]} "
        f"mean={args.mean} std={args.std}"
    )
    rknn.config(
        mean_values=[args.mean],
        std_values=[args.std],
        target_platform=args.platform,
    )

    print(f"[convert] load_onnx: {args.onnx}")
    if (
        rknn.load_onnx(
            model=args.onnx,
            input_size_list=input_size_list,
        )
        != 0
    ):
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
        f"{'INT8' if do_quant else 'FP'}, input={args.input_format})"
    )


if __name__ == "__main__":
    main()
