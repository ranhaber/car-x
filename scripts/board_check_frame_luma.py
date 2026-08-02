#!/usr/bin/env python3
"""Sample latest camera frame luminance + one H.264 AU size."""
import time
import numpy as np
from cat_follow.memory.pool import allocate_pool, FRAME_H, FRAME_W
from cat_follow.memory.shared_state import SharedState
from cat_follow.perception.h264_encoder import MppH264Encoder

# Attach to running process? Can't. Instead encode a synthetic bright frame
# and also print what /dev video looks like via a quick gst/opencv open.

import cv2
cap = cv2.VideoCapture("/dev/video11", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ok, frame = cap.read()
cap.release()
print("opencv_read", ok, None if frame is None else getattr(frame, "shape", None), getattr(frame, "dtype", None))
if ok and frame is not None:
    if frame.ndim == 2:
        print("luma_mean", float(frame[:480].mean()) if frame.shape[0] >= 480 else float(frame.mean()))
    else:
        print("bgr_mean", [float(x) for x in frame.mean(axis=(0, 1))])

enc = MppH264Encoder(640, 480, fps=15, pixel_format="NV12")
assert enc.start()
nv12 = np.empty((FRAME_H * 3 // 2, FRAME_W), dtype=np.uint8)
nv12[:FRAME_H] = 180
nv12[FRAME_H:] = 128
# left half brighter
nv12[:FRAME_H, :320] = 220
chunks = []
for i in range(30):
    c = enc.encode(nv12)
    if c:
        chunks.append(c)
enc.stop()
print("encoded_chunks", len(chunks), "first_len", len(chunks[0]) if chunks else None)
if chunks:
    open("/tmp/bright.au", "wb").write(chunks[0])
    print("wrote /tmp/bright.au")
