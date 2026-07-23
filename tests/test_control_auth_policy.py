"""Tests for control auth policy and thread-safe web command sequences."""

from __future__ import annotations

import threading
from types import SimpleNamespace

import pytest

from cat_follow.web_ui.command_seq import next_web_command_seq
from cat_follow.web_ui.control_policy import (
    load_control_auth_policy,
    require_production_control_tokens,
)


def test_policy_misconfigured_without_tokens(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    policy = load_control_auth_policy()
    assert policy.mode == "misconfigured"
    assert policy.is_production_ready is False
    with pytest.raises(RuntimeError):
        require_production_control_tokens(policy)


def test_policy_bench_override(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", "1")
    policy = load_control_auth_policy()
    assert policy.mode == "bench_override"
    assert require_production_control_tokens(policy).allow_unauthenticated is True


def test_policy_required_when_both_tokens_set(monkeypatch):
    monkeypatch.delenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", raising=False)
    monkeypatch.setenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", "web")
    monkeypatch.setenv("CAT_FOLLOW_COMMS_TOKEN", "udp")
    policy = load_control_auth_policy()
    assert policy.mode == "required"
    assert policy.web_token == "web"
    assert policy.comms_token == "udp"


def test_next_web_command_seq_is_unique_under_contention():
    ctx = SimpleNamespace(web_command_seq={"value": 0})
    values: list[int] = []
    barrier = threading.Barrier(8)

    def worker() -> None:
        barrier.wait()
        for _ in range(50):
            values.append(next_web_command_seq(ctx))

    threads = [threading.Thread(target=worker) for _ in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert len(values) == 400
    assert len(set(values)) == 400
    assert max(values) == 400
