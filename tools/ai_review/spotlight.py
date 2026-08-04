"""Rank changed symbols so caps keep the hunks that deserve deep scrutiny.

Deterministic scoring from path, risk tags, and ownership/safety tokens — no LLM.
"""

from __future__ import annotations

import re

from .classify import RiskAssessment, is_meta_tooling
from .diff import FileDiff
from .symbols import SymbolSpan

# High-signal tokens that deserve spotlight over boilerplate helpers.
_SPOTLIGHT_TOKENS: list[tuple[re.Pattern[str], int]] = [
    (re.compile(r"admit|admission|FrameConsumer|refcount|lease", re.I), 40),
    (re.compile(r"\bLock\b|\bRLock\b|\.acquire\(|\.release\("), 35),
    (re.compile(r"mission_override|force_off|perception_intent|recording_required", re.I), 30),
    (re.compile(r"emergency.?stop|fail.?closed|set_speed", re.I), 40),
    (re.compile(r"create_subscription|create_publisher|QoS"), 25),
    (re.compile(r"\brknn\b|RKNN|infer\(|preprocess", re.I), 30),
]

_PATH_TAG_BONUS: list[tuple[str, str, int]] = [
    ("shared_state", "shared_state", 50),
    ("memory/pool", "frame_ring", 45),
    ("frame_ring", "frame_ring", 45),
    ("threads/camera", "camera", 40),
    ("threads/detector", "perception", 40),
    ("control/fsm", "fsm", 45),
    ("decision_engine", "fsm", 40),
    ("web_ui/routes_control", "http_mutation", 35),
    ("ros_bridge", "ros", 35),
    ("ros_ws/", "ros", 30),
]

# When the active risk tag is a sibling of the path tag, still apply full bonus.
_TAG_ALIASES: dict[str, set[str]] = {
    "shared_state": {"shared_state", "frame_ring", "concurrency"},
    "frame_ring": {"frame_ring", "shared_state", "perception", "camera"},
    "perception": {"perception", "frame_ring", "camera", "rknn"},
    "camera": {"camera", "perception", "frame_ring"},
    "rknn": {"rknn", "perception"},
    "fsm": {"fsm", "motor"},
    "motor": {"motor", "fsm", "hardware"},
    "ros": {"ros"},
    "http_mutation": {"http_mutation"},
}


def rank_changed_symbols(
    spans: list[SymbolSpan],
    files: list[FileDiff],
    risk: RiskAssessment,
) -> list[SymbolSpan]:
    """Return *spans* ordered by descending review priority (stable on ties)."""
    tags = set(risk.tags)
    scored: list[tuple[int, int, SymbolSpan]] = []
    for index, span in enumerate(spans):
        scored.append((_score(span, tags), index, span))
    scored.sort(key=lambda item: (-item[0], item[1]))
    return [span for _, _, span in scored]


def spotlight_names(spans: list[SymbolSpan], *, limit: int = 8) -> list[str]:
    """Short citations for the pack's Spotlight section."""
    out: list[str] = []
    for span in spans[:limit]:
        out.append(f"{span.file}:{span.qualname}")
    return out


def _score(span: SymbolSpan, tags: set[str]) -> int:
    path = span.file.replace("\\", "/")
    name = span.qualname.split(".")[-1]
    score = 0

    if path.startswith("cat_follow/"):
        score += 20
    elif path.startswith("ros_ws/"):
        score += 15
    elif path.startswith("tests/") or path.endswith("smoke_test.py"):
        score -= 15
    if is_meta_tooling(path):
        score -= 25

    for needle, tag, bonus in _PATH_TAG_BONUS:
        if needle not in path:
            continue
        aliases = _TAG_ALIASES.get(tag, {tag})
        if not tags or tags & aliases:
            score += bonus
        else:
            score += max(bonus // 2, 15)
        break

    blob = f"{span.qualname}\n{span.source}"
    for pattern, bonus in _SPOTLIGHT_TOKENS:
        if pattern.search(blob):
            score += bonus

    if _is_private_leaf(name):
        score -= 20
    elif span.kind in {"function", "async_function", "class"}:
        score += 10

    if span.kind == "module":
        score -= 5

    return score


def _is_private_leaf(name: str) -> bool:
    if name.startswith("__") and name.endswith("__"):
        return False
    return name.startswith("_")
