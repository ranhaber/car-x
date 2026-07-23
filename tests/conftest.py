"""Shared pytest fixtures for the cat_follow test suite."""

from __future__ import annotations

import pytest

BENCH_AUTH_ENV = "CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL"


@pytest.fixture(autouse=True)
def _enable_bench_control_auth_by_default(monkeypatch):
    """Most tests exercise motion/UDP/Web paths without production tokens.

    Individual tests that assert production misconfiguration delete this env
    var (or set explicit tokens) in their own bodies/fixtures.
    """

    monkeypatch.setenv(BENCH_AUTH_ENV, "1")
