from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from cat_follow.web_ui import routes_h264


class _Ws:
    def __init__(self, fail_after: int | None = None) -> None:
        self.sent = []
        self.closed = 0
        self._fail_after = fail_after

    def send(self, value) -> None:  # noqa: ANN001
        self.sent.append(value)
        if self._fail_after is not None and len(self.sent) >= self._fail_after:
            raise RuntimeError("disconnect")

    def close(self) -> None:
        self.closed += 1


class _Encoder:
    def __init__(self, polls=None, submitted=None) -> None:
        self._polls = iter(polls or [])
        self._submitted = submitted or []
        self.stops = 0

    def poll(self):
        return next(self._polls, [])

    def submit_dmabuf(self, lease):  # noqa: ANN001
        return list(self._submitted)

    def stop(self) -> None:
        self.stops += 1


@pytest.fixture(autouse=True)
def _reset_route_globals():
    routes_h264._ctx = None
    routes_h264._encoder = None
    routes_h264._clients = 0
    yield
    routes_h264._ctx = None
    routes_h264._encoder = None
    routes_h264._clients = 0


def _ctx(shared=None):
    return SimpleNamespace(
        shared=shared,
        state_machine=None,
        inc_stream_clients=MagicMock(),
        dec_stream_clients=MagicMock(),
    )


def test_delayed_au_is_polled_without_a_new_camera_frame(monkeypatch):
    encoder = _Encoder(polls=[[b"delayed"]])
    ws = _Ws(fail_after=1)
    ctx = _ctx()
    routes_h264._ctx = ctx
    routes_h264._encoder = encoder
    monkeypatch.setattr(routes_h264, "_get_encoder", lambda: encoder)

    routes_h264._serve_h264(ws)

    assert ws.sent == [b"delayed"]
    assert encoder.stops == 1
    ctx.inc_stream_clients.assert_called_once_with()
    ctx.dec_stream_clients.assert_called_once_with()
    assert routes_h264._clients == 0
    assert routes_h264._encoder is None


def test_every_submit_au_is_sent_in_fifo_order(monkeypatch):
    lease = SimpleNamespace(dmabuf=True, frame_seq=1)
    shared = SimpleNamespace(
        wait_for_new_frame=lambda *args, **kwargs: 1,
        acquire_latest_frame=lambda: lease,
    )
    encoder = _Encoder(polls=[[]], submitted=[b"old", b"new"])
    ws = _Ws(fail_after=2)
    routes_h264._ctx = _ctx(shared)
    routes_h264._encoder = encoder
    monkeypatch.setattr(routes_h264, "_get_encoder", lambda: encoder)

    routes_h264._serve_h264(ws)

    assert ws.sent == [b"old", b"new"]
    assert encoder.stops == 1


def test_second_viewer_is_rejected_before_counting(monkeypatch):
    ctx = _ctx()
    ws = _Ws()
    routes_h264._ctx = ctx
    routes_h264._clients = 1
    monkeypatch.setattr(
        routes_h264,
        "_get_encoder",
        MagicMock(side_effect=AssertionError("must not create encoder")),
    )

    routes_h264._serve_h264(ws)

    assert ws.closed == 1
    assert routes_h264._clients == 1
    ctx.inc_stream_clients.assert_not_called()
    ctx.dec_stream_clients.assert_not_called()


def test_encoder_start_failure_balances_client_accounting(monkeypatch):
    ctx = _ctx()
    ws = _Ws()
    routes_h264._ctx = ctx
    monkeypatch.setattr(routes_h264, "_get_encoder", lambda: None)

    routes_h264._serve_h264(ws)

    assert ws.closed == 1
    assert routes_h264._clients == 0
    ctx.inc_stream_clients.assert_called_once_with()
    ctx.dec_stream_clients.assert_called_once_with()


def test_browser_guards_return_before_websocket_creation():
    template = (
        Path(__file__).parents[1]
        / "cat_follow"
        / "web_ui"
        / "templates"
        / "main.html"
    ).read_text(encoding="utf-8")
    start = template.index("    startH264() {")
    websocket = template.index("new WebSocket", start)
    prefix = template[start:websocket]

    insecure = prefix.index("if (!window.isSecureContext)")
    webcodecs = prefix.index("if (!('VideoDecoder' in window))")
    assert "return;" in prefix[insecure:webcodecs]
    assert "return;" in prefix[webcodecs:]
    assert "window.addEventListener('pagehide'" in template
