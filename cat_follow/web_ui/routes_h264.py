"""Optional hardware H.264 monitoring stream over WebSocket.

Uses ``flask-sock`` for the WebSocket transport and the Rockchip MPP encoder
(:class:`cat_follow.perception.h264_encoder.MppH264Encoder`).  The encoder is
created lazily on the first connected client and torn down when the last
client disconnects, so it costs nothing while nobody is watching.

Registration is fully guarded: if ``flask-sock`` or GStreamer/``mpph264enc``
are unavailable, :func:`init_h264_routes` logs and returns without touching the
Flask app, leaving MJPEG as the only stream.  Bounding-box/state overlays are
sent as JSON text frames and drawn client-side (the encoded video stays clean).
"""

from __future__ import annotations

import json
import threading
import time

import numpy as np

from cat_follow.logger import get_logger
from cat_follow.memory.pool import FRAME_SHAPE

log = get_logger("web_ui.h264")

_ctx = None
_encoder = None
_encoder_lock = threading.Lock()
_clients = 0


def init_h264_routes(ctx, app) -> bool:
    """Register the /ws/h264 WebSocket route if dependencies are available."""
    global _ctx
    _ctx = ctx

    try:
        from flask_sock import Sock
    except Exception:  # noqa: BLE001
        log.info("flask-sock not installed; H.264 WebSocket stream disabled.")
        return False

    from cat_follow.perception.h264_encoder import MppH264Encoder

    if not MppH264Encoder.available():
        log.info("mpph264enc/GStreamer unavailable; H.264 stream disabled.")
        return False

    sock = Sock(app)

    @sock.route("/ws/h264")
    def h264_ws(ws):  # noqa: ANN001
        _serve_h264(ws)

    log.info("Hardware H.264 WebSocket stream enabled at /ws/h264")
    return True


def _get_encoder():
    global _encoder
    from cat_follow.perception.h264_encoder import MppH264Encoder

    with _encoder_lock:
        if _encoder is None:
            enc = MppH264Encoder(FRAME_SHAPE[1], FRAME_SHAPE[0], fps=15)
            if enc.start():
                _encoder = enc
        return _encoder


def _release_encoder():
    global _encoder
    with _encoder_lock:
        if _encoder is not None:
            _encoder.stop()
            _encoder = None


def _serve_h264(ws) -> None:  # noqa: ANN001
    global _clients
    frame_buf = np.empty(FRAME_SHAPE, dtype=np.uint8)
    target_fps = 15.0
    tick = 1.0 / target_fps

    with _encoder_lock:
        _clients += 1
    if _ctx is not None and getattr(_ctx, "inc_stream_clients", None):
        _ctx.inc_stream_clients()

    encoder = _get_encoder()
    if encoder is None:
        ws.close()
        return

    try:
        while True:
            t0 = time.monotonic()
            if _ctx is None or _ctx.shared is None:
                time.sleep(tick)
                continue

            _ctx.shared.get_frame_latest(frame_buf)
            chunk = encoder.encode(frame_buf)
            if chunk:
                ws.send(chunk)

            # Overlay metadata as a JSON text frame (drawn client-side).
            bbox = _ctx.shared.get_bbox_tracker()
            state_name = "unknown"
            if _ctx.state_machine is not None:
                state_name = _ctx.state_machine.state.value
            ws.send(
                json.dumps(
                    {
                        "type": "overlay",
                        "state": state_name,
                        "bbox": [bbox[0], bbox[1], bbox[2], bbox[3], bbox[4]],
                    }
                )
            )

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, tick - elapsed))
    except Exception:  # noqa: BLE001 - client disconnected
        pass
    finally:
        with _encoder_lock:
            _clients = max(0, _clients - 1)
            last = _clients == 0
        if _ctx is not None and getattr(_ctx, "dec_stream_clients", None):
            _ctx.dec_stream_clients()
        if last:
            _release_encoder()
