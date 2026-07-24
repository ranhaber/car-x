#!/usr/bin/env python3
"""Validate raw OpenCV NV12 capture and crop conversion on the ROCK board."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cat_follow.vision.nv12_utils import (  # noqa: E402
    center_bottom_nv12_region,
    extract_nv12_crop,
    nv12_shape,
    nv12_to_bgr,
    validate_nv12,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video11")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument(
        "--backend", choices=("opencv", "gst_nv12"), default="gst_nv12"
    )
    parser.add_argument(
        "--io-mode",
        type=int,
        default=2,
        help="GStreamer v4l2src memory mode; only 2 (mmap CPU pack) is supported",
    )
    parser.add_argument("--output", default="/tmp/car-x-nv12-validation.jpg")
    args = parser.parse_args()

    if args.backend == "gst_nv12":
        from cat_follow.vision.gst_nv12_capture import GstV4l2Nv12Capture

        cap = GstV4l2Nv12Capture(
            args.device, args.width, args.height, 30, io_mode=args.io_mode
        )
        if not cap.start():
            raise RuntimeError(f"GStreamer camera did not open: {args.device}")
    else:
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"NV12"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        if not cap.isOpened():
            raise RuntimeError(f"OpenCV camera did not open: {args.device}")

    raw = None
    actual_width = 0
    actual_height = 0
    try:
        for _ in range(max(1, args.frames)):
            if args.backend == "gst_nv12":
                candidate = np.empty(
                    nv12_shape(args.width, args.height), dtype=np.uint8
                )
                ok, candidate = cap.read(dst=candidate)
            else:
                ok, candidate = cap.read()
            if ok and candidate is not None:
                raw = candidate
        if args.backend == "gst_nv12":
            actual_width, actual_height = args.width, args.height
        else:
            actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    finally:
        cap.release()
    if raw is None:
        raise RuntimeError("camera returned no frame")

    packed = validate_nv12(raw, args.width, args.height)
    bgr = nv12_to_bgr(packed, args.width, args.height)
    crop_size = min(320, args.width, args.height) & ~1
    region = center_bottom_nv12_region(
        args.width, args.height, crop_size, crop_size
    )
    crop_nv12 = extract_nv12_crop(
        packed, args.width, args.height, region
    )
    crop_bgr = nv12_to_bgr(crop_nv12, region[2], region[3])
    x, y, width, height = region
    reference = bgr[y : y + height, x : x + width]
    max_crop_delta = int(
        np.max(np.abs(crop_bgr.astype(np.int16) - reference.astype(np.int16)))
    )

    if args.output and not cv2.imwrite(args.output, bgr):
        raise RuntimeError(f"could not write validation image: {args.output}")

    print(f"device={args.device}")
    print(f"backend={args.backend}")
    print(f"requested={args.width}x{args.height} NV12")
    print(f"raw_shape={raw.shape} dtype={raw.dtype} strides={raw.strides}")
    print(f"contiguous={raw.flags.c_contiguous} bytes={raw.nbytes}")
    print(f"capture_properties={actual_width}x{actual_height}")
    print(
        f"y_range={int(packed[:args.height].min())}:"
        f"{int(packed[:args.height].max())}"
    )
    print(
        f"uv_range={int(packed[args.height:].min())}:"
        f"{int(packed[args.height:].max())}"
    )
    print(f"crop_region={region} max_bgr_delta={max_crop_delta}")
    print(f"output={args.output}")
    if max_crop_delta != 0:
        raise RuntimeError(
            f"crop conversion differs from full-frame reference by {max_crop_delta}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
