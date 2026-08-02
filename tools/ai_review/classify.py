"""Semantic shallow/deep classification from diff hunks and paths."""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from .diff import FileDiff

# Path-based deep tags: (substring_in_path, tag, reason)
_PATH_RULES: list[tuple[str, str, str]] = [
    ("shared_state", "shared_state", "touches shared_state module"),
    ("memory/pool", "frame_ring", "touches memory pool / frame buffers"),
    ("frame_ring", "frame_ring", "touches frame ring"),
    ("rknn", "rknn", "touches RKNN / NPU path"),
    ("zerocopy", "rknn", "touches zero-copy vision path"),
    ("vision/", "perception", "touches vision pipeline"),
    ("perception/", "perception", "touches perception pipeline"),
    ("threads/camera", "camera", "touches camera thread"),
    ("threads/detector", "perception", "touches detector thread"),
    ("threads/tracker", "perception", "touches tracker thread"),
    ("control/fsm", "fsm", "touches FSM"),
    ("decision_engine", "fsm", "touches decision engine"),
    ("navigation/", "ros", "touches navigation"),
    ("ros_bridge", "ros", "touches ROS bridge"),
    ("ros_ws/", "ros", "touches ROS workspace"),
    ("web_ui/routes_control", "http_mutation", "touches HTTP control mutations"),
    ("comms/", "udp", "touches UDP/comms"),
    ("ultrasonic", "hardware", "touches ultrasonic / ranging"),
    ("motor", "motor", "touches motor path"),
    ("picarx", "hardware", "touches Picarx / hardware"),
]

_TOKEN_RULES: list[tuple[re.Pattern[str], str, str]] = [
    (re.compile(r"\bLock\b|\bRLock\b|\bacquire\(|\brelease\(|threading\.Lock"), "concurrency", "lock usage changed"),
    (re.compile(r"\bThread\b|\bthreading\.|Thread\("), "concurrency", "thread creation/usage changed"),
    (re.compile(r"\bQueue\b|\bqueue\.|deque\("), "concurrency", "queue usage changed"),
    (re.compile(r"\bEvent\b|\bCondition\b|\bBarrier\b"), "concurrency", "sync primitive changed"),
    (re.compile(r"frame.?ring|FrameRing|lease|generation|refcount|NV12|zero.?copy", re.I), "frame_ring", "frame/buffer ownership tokens changed"),
    (re.compile(r"\brknn\b|RKNN|NPU|infer\(|preprocess", re.I), "rknn", "NPU / inference tokens changed"),
    (re.compile(r"create_subscription|create_publisher|create_timer|create_service|QoS|Node\("), "ros", "ROS API tokens changed"),
    (re.compile(r"emergency.?stop|estop|fail.?closed|set_speed|motor|servo", re.I), "motor", "motor / safety tokens changed"),
    (re.compile(r"\bFSM\b|StateMachine|dispatch\(|safety_precedence", re.I), "fsm", "FSM / safety tokens changed"),
    (re.compile(r"@.*route|request\.json|require_auth|mutate|POST|PUT|DELETE", re.I), "http_mutation", "HTTP mutation tokens changed"),
    (re.compile(r"shared_state|SharedState|generation"), "shared_state", "shared-state tokens changed"),
]

_COMMENT_LINE = re.compile(r"^\s*#")
_DOCSTRING_ONLY = re.compile(r'^\s*("""|\'\'\')')
_IMPORT_LINE = re.compile(r"^\s*(from\s+\S+\s+import|import\s+)")
_TYPE_HINT_LINE = re.compile(
    r"^\s*(?:async\s+)?def\s+\w+\([^)]*\)\s*->\s*[^:]+:\s*$"
    r"|^\s*\w+\s*:\s*[A-Za-z_][\w\[\],\s\|\.]*\s*(?:=.*)?$"
    r"|^\s*from\s+typing\s+import\b"
)
_LOG_LINE = re.compile(
    r"^\s*(logging\.|logger\.|log\.|_log\.|print\()"
    r"|^\s*(debug|info|warning|error|exception|critical)\("
)


@dataclass
class RiskAssessment:
    level: str  # shallow | deep
    tags: list[str] = field(default_factory=list)
    reasons: list[str] = field(default_factory=list)


def classify_diffs(files: list[FileDiff]) -> RiskAssessment:
    tags: set[str] = set()
    reasons: list[str] = []
    path_tags_all: set[str] = set()
    path_reasons: list[str] = []

    if not files:
        return RiskAssessment(level="shallow", tags=[], reasons=["empty diff"])

    any_behavior = False
    for fd in files:
        for tag, reason in _tags_from_path(fd.path):
            path_tags_all.add(tag)
            path_reasons.append(reason)

        for hunk in fd.hunk_texts:
            hunk_tags, hunk_reasons, behavior = _classify_hunk(hunk)
            tags.update(hunk_tags)
            reasons.extend(hunk_reasons)
            any_behavior = any_behavior or behavior

        if fd.status in {"added", "deleted"} and fd.path.endswith(
            (".py", ".yaml", ".yml", ".launch.py")
        ):
            if not fd.path.endswith((".md", ".txt", ".rst")):
                any_behavior = True
                if "code_structure" not in tags:
                    tags.add("code_structure")
                    reasons.append(f"{fd.status} file {fd.path}")

    # Path tags are advisory: attach only when the diff has behavioral edits.
    if any_behavior or not _all_hunks_shallow(files):
        tags.update(path_tags_all - {"docs"})
        reasons.extend(path_reasons)
    else:
        tags.update(path_tags_all & {"docs"})

    seen: set[str] = set()
    uniq_reasons: list[str] = []
    for r in reasons:
        if r not in seen:
            seen.add(r)
            uniq_reasons.append(r)

    if _all_hunks_shallow(files) and not any_behavior:
        return RiskAssessment(
            level="shallow",
            tags=sorted(tags) or ["docs"],
            reasons=uniq_reasons or ["comment/format/type/log-only changes"],
        )

    deep_tags = sorted(tags - {"docs", "format", "rename", "types", "logging"})
    return RiskAssessment(
        level="deep",
        tags=deep_tags or ["code_structure"],
        reasons=uniq_reasons or ["behavioral code changes"],
    )


def _tags_from_path(path: str) -> list[tuple[str, str]]:
    out: list[tuple[str, str]] = []
    lower = path.replace("\\", "/")
    if lower.endswith((".md", ".txt", ".rst")):
        out.append(("docs", f"documentation path {path}"))
        return out
    for needle, tag, reason in _PATH_RULES:
        if needle in lower:
            out.append((tag, reason))
    return out


def _classify_hunk(hunk: str) -> tuple[set[str], list[str], bool]:
    tags: set[str] = set()
    reasons: list[str] = []
    changed_lines = [
        line[1:]
        for line in hunk.splitlines()
        if (line.startswith("+") and not line.startswith("+++"))
        or (line.startswith("-") and not line.startswith("---"))
    ]
    if not changed_lines:
        return tags, reasons, False

    non_empty = [ln for ln in changed_lines if ln.strip()]
    if not non_empty:
        return {"format"}, ["whitespace-only hunk"], False

    behavior = False
    for ln in non_empty:
        stripped = ln.strip()
        if _COMMENT_LINE.match(ln) or stripped in {'"""', "'''"} or _DOCSTRING_ONLY.match(ln):
            tags.add("docs")
            continue
        if _IMPORT_LINE.match(ln):
            # Import reorder alone is shallow; import of new symbol may be deep via tokens below.
            tags.add("format")
        if _looks_type_hint_only(ln):
            tags.add("types")
            continue
        if _LOG_LINE.match(ln) and not _has_deep_token(ln):
            tags.add("logging")
            continue

        matched_deep = False
        for pattern, tag, reason in _TOKEN_RULES:
            if pattern.search(ln):
                tags.add(tag)
                reasons.append(reason)
                matched_deep = True
                behavior = True
        if matched_deep:
            continue

        # Generic code line.
        if not (_COMMENT_LINE.match(ln) or _looks_type_hint_only(ln)):
            behavior = True
            tags.add("code_structure")

    return tags, reasons, behavior


def _looks_type_hint_only(line: str) -> bool:
    s = line.strip()
    if s.startswith("from typing import") or s.startswith("from __future__ import annotations"):
        return True
    # Annotation-only assignment without obvious runtime call.
    if re.match(r"^[A-Za-z_][\w]*\s*:\s*[A-Za-z_][\w\[\],\s\|\.]*\s*$", s):
        return True
    if "->" in s and s.startswith("def ") and s.endswith(":"):
        return True
    return False


def _has_deep_token(line: str) -> bool:
    return any(p.search(line) for p, _, _ in _TOKEN_RULES)


def _all_hunks_shallow(files: list[FileDiff]) -> bool:
    for fd in files:
        if fd.path.endswith((".md", ".txt", ".rst")):
            continue
        if fd.status == "deleted":
            return False
        for hunk in fd.hunk_texts:
            _, _, behavior = _classify_hunk(hunk)
            if behavior:
                # Still shallow if only docs/types/logging/format tags and no deep tokens.
                tags, _, _ = _classify_hunk(hunk)
                deep = tags - {"docs", "format", "rename", "types", "logging"}
                if deep or (behavior and "code_structure" in tags and deep):
                    # Re-check lines: if any non-shallow line exists, not all shallow.
                    if not _hunk_is_shallow(hunk):
                        return False
                elif behavior and not _hunk_is_shallow(hunk):
                    return False
    return True


def _hunk_is_shallow(hunk: str) -> bool:
    changed = [
        line[1:]
        for line in hunk.splitlines()
        if (line.startswith("+") and not line.startswith("+++"))
        or (line.startswith("-") and not line.startswith("---"))
    ]
    non_empty = [ln for ln in changed if ln.strip()]
    if not non_empty:
        return True
    for ln in non_empty:
        if _COMMENT_LINE.match(ln) or _DOCSTRING_ONLY.match(ln) or ln.strip() in {'"""', "'''"}:
            continue
        if _IMPORT_LINE.match(ln):
            # Treat pure import lines as shallow for gate; deep import of dangerous APIs still caught by tokens.
            if _has_deep_token(ln):
                return False
            continue
        if _looks_type_hint_only(ln):
            continue
        if _LOG_LINE.match(ln) and not _has_deep_token(ln):
            continue
        if not ln.strip():
            continue
        return False
    return True
