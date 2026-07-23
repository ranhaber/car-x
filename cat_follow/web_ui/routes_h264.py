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


def _serve_h264(ws) -> None:  # noqa: ANN001
    global _clients
    frame_buf = np.empty(FRAME_SHAPE, dtype=np.uint8)
    target_fps = 15.0
    tick = 1.0 / target_fps

    with _encoder_lock:
        _clients += 1
    if _ctx is not None and getattr(_ctx, "inc_stream_clients", None):
        _ctx.inc_stream_clients()

    try:
        # Acquire the encoder inside the try so an encoder-start failure still
        # runs the finally below.  Otherwise an early return here would leak the
        # incremented _clients / stream-client counters, leaving the system
        # convinced a viewer is attached (encode/processing never quiesces).
        encoder = _get_encoder()
        if encoder is None:
            ws.close()
            return
        while True:
            t0 = time.monotonic()
            if _ctx is None or _ctx.shared is None:
                time.sleep(tick)
                continue

            _ctx.shared.get_frame_latest(frame_buf)
            # Encode without holding _encoder_lock — MPP encode can be slow.
            chunk = encoder.encode(frame_buf)
            if chunk:
                ws.send(chunk)

            # Overlay metadata as a JSON text frame (drawn client-side).
            bbox = _ctx.shared.get_bbox_tracker()
            tracked_targets = _ctx.shared.get_tracked_targets()
            state_name = "unknown"
            if _ctx.state_machine is not None:
                state_name = _ctx.state_machine.state.value
            ws.send(
                json.dumps(
                    {
                        "type": "overlay",
                        "state": state_name,
                        "bbox": [bbox[0], bbox[1], bbox[2], bbox[3], bbox[4]],
                        "targets": {
                            role: {
                                "track_id": target[0],
                                "x": target[1],
                                "y": target[2],
                                "w": target[3],
                                "h": target[4],
                                "confidence": target[5],
                                "frames_since_update": target[6],
                                "valid": target[7],
                            }
                            for role, target in tracked_targets.items()
                        },
                    }
                )
            )

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, tick - elapsed))
    except Exception:  # noqa: BLE001 - client disconnected
        pass
    finally:
        # Atomically drop the client count and detach the encoder reference so a
        # concurrent new client never receives an encoder that is about to stop.
        # Stop the old encoder outside the lock (stop can block).
        global _encoder
        enc_to_stop = None
        with _encoder_lock:
            _clients = max(0, _clients - 1)
            if _clients == 0 and _encoder is not None:
                enc_to_stop = _encoder
                _encoder = None
        if enc_to_stop is not None:
            try:
                enc_to_stop.stop()
            except Exception:  # noqa: BLE001
                log.warning("H.264 encoder stop failed during last-client teardown")
        if _ctx is not None and getattr(_ctx, "dec_stream_clients", None):
            _ctx.dec_stream_clients()
