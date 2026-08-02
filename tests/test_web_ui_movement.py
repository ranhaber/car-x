"""Tests for Movement tab web routes."""

from __future__ import annotations

from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState as PrototypeSharedState
from cat_follow.motion.sequence_executor import MotionSequenceExecutor
from cat_follow.web_ui.app import create_app


def _make_app(*, token: str | None = None, comms_manager=None, calibration=None):
    if token is not None:
        import os

        os.environ["CAT_FOLLOW_WEB_CONTROL_TOKEN"] = token
    proto = PrototypeSharedState(allocate_pool())
    executor = MotionSequenceExecutor()
    app = create_app(
        shared=proto,
        comms_manager=comms_manager,
        sequence_executor=executor,
        calibration=calibration,
    )
    return app, executor


def test_validate_requires_token_when_set(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", "s3cret")
    monkeypatch.setenv("CAT_FOLLOW_COMMS_TOKEN", "comms")
    app, _ = _make_app(token="s3cret")
    client = app.test_client()

    res = client.post("/api/movement/sequence/validate", json={"actions": []})
    assert res.status_code == 401

    res = client.post(
        "/api/movement/sequence/validate",
        json={"actions": [{"type": "wait", "duration_s": 0.5}]},
        headers={"X-Control-Token": "s3cret"},
    )
    assert res.status_code == 200
    assert res.get_json()["ok"] is True


def test_sequence_stop_never_requires_token(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", "s3cret")
    monkeypatch.setenv("CAT_FOLLOW_COMMS_TOKEN", "comms")
    app, _ = _make_app(token="s3cret")
    client = app.test_client()
    assert client.post("/api/movement/sequence/stop").status_code == 200


def test_run_rejects_invalid_plan(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", "1")
    app, _ = _make_app()
    client = app.test_client()
    res = client.post(
        "/api/movement/sequence/run",
        json={"actions": [{"type": "drive", "speed_pct": 999, "duration_s": 1.0}]},
    )
    assert res.status_code == 400


def test_query_param_token_rejected(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", "s3cret")
    monkeypatch.setenv("CAT_FOLLOW_COMMS_TOKEN", "comms")
    app, _ = _make_app(token="s3cret")
    client = app.test_client()
    res = client.post(
        "/api/movement/sequence/validate?token=s3cret",
        json={"actions": [{"type": "wait", "duration_s": 0.5}]},
    )
    assert res.status_code == 401



def test_misconfigured_auth_fails_closed(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    app, _ = _make_app()
    client = app.test_client()
    res = client.post(
        "/api/movement/sequence/validate",
        json={"actions": [{"type": "wait", "duration_s": 0.5}]},
    )
    assert res.status_code == 503


def test_web_token_alone_still_misconfigured(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", "s3cret")
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    app, _ = _make_app(token="s3cret")
    client = app.test_client()
    res = client.post(
        "/api/movement/sequence/validate",
        json={"actions": [{"type": "wait", "duration_s": 0.5}]},
        headers={"X-Control-Token": "s3cret"},
    )
    assert res.status_code == 503
