"""
Streaming route: MJPEG video stream.

Routes:
    GET /stream — MJPEG stream with bbox and state overlay.
"""

import time
from flask import Blueprint, Response

import numpy as np

from cat_follow.memory.pool import FRAME_SHAPE

streaming_bp = Blueprint("streaming", __name__)

# Set by init_streaming_routes
_ctx = None

# Optional libjpeg-turbo encoder (NEON-accelerated on ARM). Falls back to cv2.
try:
    import simplejpeg as _simplejpeg

    _HAS_SIMPLEJPEG = True
except Exception:  # pragma: no cover - optional dependency
    _simplejpeg = None
    _HAS_SIMPLEJPEG = False


def init_streaming_routes(ctx):
    """Bind streaming context. Route is registered at import time."""
    global _ctx
    _ctx = ctx


@streaming_bp.route("/stream")
def stream():
    return Response(
        _generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


def _encode_jpeg(cv2, display) -> bytes:
    """Encode a BGR frame to JPEG, preferring simplejpeg (libjpeg-turbo)."""
    if _HAS_SIMPLEJPEG:
        # simplejpeg wants a contiguous array; colorspace BGR matches OpenCV.
        return _simplejpeg.encode_jpeg(
            np.ascontiguousarray(display), quality=80, colorspace="BGR"
        )
    _, jpeg = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return jpeg.tobytes()


def _generate_mjpeg():
    """Yield MJPEG frames at ~10 FPS with bbox rectangle and state overlay.

    Annotation and JPEG encoding only run while this generator is alive, i.e.
    while a browser is connected.  The generator registers itself in the
    shared stream-client counter so the rest of the system can cheaply tell
    whether anyone is watching (detection never depends on this).
    """
    try:
        import cv2
        _has_cv2 = True
    except ImportError:
        _has_cv2 = False

    frame_buf = np.empty(FRAME_SHAPE, dtype=np.uint8)
    display = np.empty(FRAME_SHAPE, dtype=np.uint8)
    target_fps = 10.0
    tick = 1.0 / target_fps
    fps_counter = 0
    fps_timer = time.monotonic()

    if _ctx is not None and getattr(_ctx, "inc_stream_clients", None):
        _ctx.inc_stream_clients()

    try:
        while True:
            t0 = time.monotonic()
            if _ctx is None or _ctx.shared is None:
                time.sleep(tick)
                continue

            _ctx.shared.get_frame_latest(frame_buf)
            bbox = _ctx.shared.get_bbox_tracker()
            state_name = "unknown"
            if _ctx.state_machine is not None:
                state_name = _ctx.state_machine.state.value

            res_key = _ctx.get_stream_resolution()
            target_w, target_h = _ctx.resolution_options[res_key]

            if _has_cv2:
                np.copyto(display, frame_buf)
                if bbox[4] > 0:
                    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
                    cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    label = f"cat ({w}x{h})"
                    cv2.putText(display, label, (x, max(y - 8, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(display, f"State: {state_name}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                out = display
                src_h, src_w = display.shape[:2]
                if (target_w, target_h) != (src_w, src_h):
                    out = cv2.resize(display, (target_w, target_h), interpolation=cv2.INTER_AREA)
                frame_bytes = _encode_jpeg(cv2, out)
            else:
                frame_bytes = b"\xff\xd8\xff\xe0" + b"\x00" * 100

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
            )

            fps_counter += 1
            now = time.monotonic()
            if now - fps_timer >= 1.0:
                if _ctx.set_stream_fps:
                    _ctx.set_stream_fps(fps_counter / (now - fps_timer))
                fps_counter = 0
                fps_timer = now

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, tick - elapsed))
    finally:
        if _ctx is not None and getattr(_ctx, "dec_stream_clients", None):
            _ctx.dec_stream_clients()
