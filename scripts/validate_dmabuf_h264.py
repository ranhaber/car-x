#!/usr/bin/env python3
"""Board gate for camera DMA-BUF -> Rockchip MPP H.264 without raw-frame copy."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from cat_follow.perception.h264_encoder import MppH264Encoder
from cat_follow.vision.zerocopy_backend import ZerocopySession


class _Lease:
    def __init__(self, session: ZerocopySession, frame) -> None:  # noqa: ANN001
        self._session = session
        self._index = int(frame.buffer_index)
        self.dmabuf_fd = int(frame.cam_fd)
        self.dmabuf_size = int(frame.image_size)
        self.dmabuf_stride = int(frame.stride)
        self.dmabuf = True
        self._released = False

    def release(self) -> None:
        if not self._released:
            if not self._session.requeue(self._index):
                raise RuntimeError(
                    self._session.last_error
                    or f"camera buffer requeue failed: {self._index}"
                )
            self._released = True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video11")
    parser.add_argument(
        "--model", default="models/yolov8n_coco_320_rk3576_int8.rknn"
    )
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--fps", type=int, default=15)
    args = parser.parse_args()

    model = Path(args.model)
    if not model.is_absolute():
        model = ROOT / model
    session = ZerocopySession.open(
        device=args.device,
        model_path=str(model),
        src_w=640,
        src_h=480,
        crop_w=320,
        crop_h=320,
    )
    encoder = MppH264Encoder(640, 480, fps=args.fps, pixel_format="NV12")
    if session is None or not encoder.start():
        print(json.dumps({"status": "fail", "error": "session/encoder open failed"}))
        return 1

    chunks = 0
    encoded_bytes = 0
    stride = 0
    started = time.monotonic()
    try:
        for _ in range(args.frames):
            frame = session.dequeue(timeout_ms=3000)
            if frame is None:
                raise RuntimeError(session.last_error or "camera dequeue failed")
            stride = int(frame.stride)
            ready = encoder.submit_dmabuf(_Lease(session, frame))
            for chunk in ready:
                chunks += 1
                encoded_bytes += len(chunk)
            time.sleep(1.0 / args.fps)
        # MPP output is asynchronous. Keep polling after the final submit so
        # the promotion gate counts delayed access units rather than dropping
        # the last frame merely because it missed the submit-side wait window.
        drain_deadline = time.monotonic() + 1.0
        while time.monotonic() < drain_deadline:
            ready = encoder.poll(wait_ns=10_000_000)
            for chunk in ready:
                chunks += 1
                encoded_bytes += len(chunk)
            if not ready:
                time.sleep(0.01)
    finally:
        encoder.stop()
        session.close()

    elapsed = time.monotonic() - started
    result = {
        "status": "pass" if chunks >= max(1, args.frames - 1) else "fail",
        "input": "camera-dmabuf",
        "raw_frame_cpu_copy": False,
        "frames_pushed": args.frames,
        "chunks_encoded": chunks,
        "encoded_bytes": encoded_bytes,
        "stride": stride,
        "elapsed_s": round(elapsed, 2),
    }
    print(json.dumps(result))
    return 0 if result["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
