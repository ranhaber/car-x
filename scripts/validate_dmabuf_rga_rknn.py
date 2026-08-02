#!/usr/bin/env python3
"""Standalone Option B validation: dmabuf capture → RGA crop → RKNN NV12.

Board-only script — not linked to main_loop or Flask. Probes whether the ROCK
4D can run a fd-based path without the production NumPy capture ring.

Usage (on ROCK 4D over SSH)::

    python3 scripts/validate_dmabuf_rga_rknn.py \\
        --device /dev/video11 \\
        --frames 30 \\
        --model models/yolov8n_coco_320_rk3576_int8.rknn

Exit codes: 0 pass/partial, 1 fail.

JSON summary is printed to stdout (last line).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np

from cat_follow.vision.nv12_utils import center_bottom_nv12_region, nv12_shape
from cat_follow.vision.rga_crop import crop_center_bottom_nv12, rga_available
from cat_follow.vision.rknn_backend import RknnBackend, infer_input_format_from_model_path


def _probe_gst_dmabuf() -> tuple[bool, str]:
    try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        if Gst.ElementFactory.find("v4l2src") is None:
            return False, "v4l2src missing"
        return True, "ok"
    except Exception as exc:  # noqa: BLE001
        return False, str(exc)


def _capture_one_dmabuf(
    device: str, width: int, height: int
) -> tuple[np.ndarray | None, str]:
    """Try GStreamer v4l2src io-mode=4 (dmabuf). Fall back to mmap pack."""
    try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
    except Exception as exc:  # noqa: BLE001
        return None, f"gstreamer unavailable: {exc}"

    for io_mode, label in ((4, "dmabuf"), (2, "mmap")):
        pipeline_desc = (
            f"v4l2src device={device} io-mode={io_mode} num-buffers=1 ! "
            f"video/x-raw,format=NV12,width={width},height={height},framerate=30/1 ! "
            "appsink name=sink sync=false max-buffers=1 drop=true"
        )
        try:
            pipeline = Gst.parse_launch(pipeline_desc)
            appsink = pipeline.get_by_name("sink")
            if appsink is None:
                continue
            pipeline.set_state(Gst.State.PLAYING)
            sample = appsink.emit("try-pull-sample", int(5 * Gst.SECOND))
            pipeline.set_state(Gst.State.NULL)
            if sample is None:
                continue
            buf = sample.get_buffer()
            ok, map_info = buf.map(Gst.MapFlags.READ)
            if not ok:
                continue
            try:
                expected = nv12_shape(width, height)
                arr = np.frombuffer(map_info.data, dtype=np.uint8, count=expected[0] * expected[1])
                frame = arr.reshape(expected).copy()
            finally:
                buf.unmap(map_info)
            return frame, label
        except Exception as exc:  # noqa: BLE001
            if io_mode == 2:
                return None, f"capture failed: {exc}"
    return None, "capture failed"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video11")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--crop", type=int, default=320)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument(
        "--model",
        default="models/yolov8n_coco_320_rk3576_int8.rknn",
    )
    parser.add_argument("--score-threshold", type=float, default=0.25)
    args = parser.parse_args()

    summary: dict = {
        "status": "fail",
        "capture_mode": None,
        "rga_available": rga_available(),
        "rga_ms_p50": None,
        "npu_ms_p50": None,
        "detections_total": 0,
        "frames_ok": 0,
        "error": None,
    }

    if not os.path.exists(args.model):
        summary["error"] = f"model not found: {args.model}"
        print(json.dumps(summary))
        return 1

    gst_ok, gst_msg = _probe_gst_dmabuf()
    if not gst_ok:
        summary["error"] = gst_msg
        print(json.dumps(summary))
        return 1

    try:
        from rknnlite.api import RKNNLite  # noqa: F401
    except Exception:
        summary["error"] = "rknnlite not importable"
        print(json.dumps(summary))
        return 1

    fmt = infer_input_format_from_model_path(args.model)
    backend = RknnBackend(
        args.model,
        input_size=(args.crop, args.crop),
        input_format=fmt,
    )
    if not backend.load():
        summary["error"] = "RKNN load failed"
        print(json.dumps(summary))
        return 1

    crop_w = crop_h = args.crop
    crop_dst = np.empty(nv12_shape(crop_w, crop_h), dtype=np.uint8)
    region = center_bottom_nv12_region(args.width, args.height, crop_w, crop_h)
    rga_times: list[float] = []
    npu_times: list[float] = []
    detections_total = 0
    frames_ok = 0
    capture_mode = None
    used_mmap_fallback = False

    for _ in range(max(1, args.frames)):
        frame, mode = _capture_one_dmabuf(args.device, args.width, args.height)
        if frame is None:
            summary["error"] = mode
            break
        if capture_mode is None:
            capture_mode = mode
        if mode == "mmap":
            used_mmap_fallback = True

        t0 = time.perf_counter()
        try:
            crop_center_bottom_nv12(
                frame,
                args.width,
                args.height,
                crop_w,
                crop_h,
                dst=crop_dst,
                region=region,
            )
        except Exception as exc:  # noqa: BLE001
            summary["error"] = f"crop failed: {exc}"
            break
        rga_ms = (time.perf_counter() - t0) * 1000.0
        rga_times.append(rga_ms)

        t1 = time.perf_counter()
        dets = backend.infer_all_nv12(crop_dst, args.score_threshold)
        npu_ms = (time.perf_counter() - t1) * 1000.0
        npu_times.append(npu_ms)
        detections_total += len(dets)
        frames_ok += 1

    backend.unload()

    summary["capture_mode"] = capture_mode
    summary["frames_ok"] = frames_ok
    summary["detections_total"] = detections_total
    if rga_times:
        summary["rga_ms_p50"] = float(np.percentile(rga_times, 50))
    if npu_times:
        summary["npu_ms_p50"] = float(np.percentile(npu_times, 50))

    if frames_ok >= args.frames:
        if used_mmap_fallback:
            summary["status"] = "partial"
            summary["error"] = (
                "dmabuf io-mode=4 unavailable; mmap fallback used (RGA+NPU OK)"
            )
        else:
            summary["status"] = "pass"
    elif frames_ok > 0:
        summary["status"] = "partial"
        if summary["error"] is None:
            summary["error"] = f"only {frames_ok}/{args.frames} frames completed"
    else:
        summary["status"] = "fail"

    print(json.dumps(summary))
    return 0 if summary["status"] in ("pass", "partial") else 1


if __name__ == "__main__":
    raise SystemExit(main())
