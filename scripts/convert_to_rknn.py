"""Convert a finalized SSD-MobileNet **TFLite** model to ``.rknn`` for the NPU.

Run this ONCE on an x86 workstation that has ``rknn-toolkit2`` installed
(NOT on the ROCK 4D and NOT in the runtime venv -- the board uses the
lightweight ``rknnlite`` runtime instead).

    pip install rknn-toolkit2   # x86 workstation only

Usage:
    python scripts/convert_to_rknn.py \
        --src models/ssd_mobilenet_v2_320x320.tflite \
        --dst models/ssd_mobilenet_v2.rknn \
        --dataset dataset.txt

``dataset.txt`` is a text file with one image path per line, used for
INT8 quantization calibration.  Omit ``--dataset`` to build without
quantization (larger/slower, but useful for a quick smoke test).

Verified input contract
------------------------
Only finalized SSD-MobileNet TFLite graphs are supported -- i.e. models that
already contain the ``TFLite_Detection_PostProcess`` op and expose the four
detection outputs (boxes / classes / scores / count).  This matches what the
runtime feeds (NHWC RGB uint8) and what ``cat_follow.vision.ssd_postprocess``
expects.  Raw ONNX SSD exports (undecoded heads, NCHW, anchor decoding, NMS)
are intentionally **not** accepted because they would need model-specific
layout and output decoders that the runtime does not implement.

Preprocessing (important)
-------------------------
The documented model (``ssd_mobilenet_v2_320x320_coco_quant``) is **quantized**
and expects raw ``uint8`` pixels in ``[0, 255]``.  The runtime therefore feeds
raw ``uint8`` and applies NO normalization.  To avoid double normalization the
RKNN input transform must be a pass-through (``mean=0, std=1``) -- these are the
defaults below.  Only override ``--mean``/``--std`` if your source model
documents a float input contract (e.g. a non-quantized model expecting
``(x-127.5)/127.5`` -> ``--mean 127.5 --std 127.5``).

The resulting ``.rknn`` file is what the runtime loads via
``CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH``.
"""

import argparse
import os
import sys


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--src", required=True, help="Source finalized SSD-MobileNet .tflite model"
    )
    p.add_argument("--dst", required=True, help="Output .rknn path")
    p.add_argument("--dataset", default=None, help="Quantization calibration list")
    p.add_argument("--target", default="rk3576", help="Target platform")
    p.add_argument(
        "--mean",
        type=float,
        default=0.0,
        help="Per-channel input mean (default 0 = raw uint8 pass-through)",
    )
    p.add_argument(
        "--std",
        type=float,
        default=1.0,
        help="Per-channel input std (default 1 = raw uint8 pass-through)",
    )
    args = p.parse_args()

    ext = os.path.splitext(args.src)[1].lower()
    if ext != ".tflite":
        print(
            f"Unsupported source format {ext!r}. Only finalized SSD-MobileNet "
            "TFLite models (with TFLite_Detection_PostProcess) are supported; "
            "raw ONNX SSD exports need model-specific decoders the runtime does "
            "not implement.",
            file=sys.stderr,
        )
        return 1

    try:
        from rknn.api import RKNN  # type: ignore
    except Exception as exc:  # noqa: BLE001
        print(
            "rknn-toolkit2 is not installed. Install it on an x86 workstation:\n"
            "    pip install rknn-toolkit2\n"
            f"(import error: {exc})",
            file=sys.stderr,
        )
        return 2

    if not os.path.exists(args.src):
        print(f"Source model not found: {args.src}", file=sys.stderr)
        return 1

    rknn = RKNN(verbose=True)
    # Pass-through by default: the quantized model expects raw uint8 [0,255], so
    # mean=0/std=1 avoids double normalization. Override --mean/--std only for a
    # documented float input contract.
    rknn.config(
        target_platform=args.target,
        mean_values=[[args.mean, args.mean, args.mean]],
        std_values=[[args.std, args.std, args.std]],
    )

    if rknn.load_tflite(model=args.src) != 0:
        print("Failed to load source model", file=sys.stderr)
        return 1

    do_quant = args.dataset is not None
    if rknn.build(do_quantization=do_quant, dataset=args.dataset) != 0:
        print("RKNN build failed", file=sys.stderr)
        return 1

    os.makedirs(os.path.dirname(os.path.abspath(args.dst)), exist_ok=True)
    if rknn.export_rknn(args.dst) != 0:
        print("RKNN export failed", file=sys.stderr)
        return 1

    print(f"Wrote {args.dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
