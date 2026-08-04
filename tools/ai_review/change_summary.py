"""Deterministic "what changed" narrative from diffs and symbols.

No LLM. Bullets cite file paths and symbol names so a reviewer can verify
evidence in the pack excerpts. Product *why* stays with the agent (PR/user);
this module only states observable deltas.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from .classify import is_meta_tooling
from .diff import FileDiff
from .symbols import SymbolSpan

_CLASS_RE = re.compile(r"^(\s*)class\s+(\w+)\b")
_NEW_FUNC_RE = re.compile(r"^New function `([^`(]+)")
_API_FUNC_RE = re.compile(r"^API `([^`]+)`")
_REMOVED_FUNC_RE = re.compile(r"^Removed function `([^`]+)`")


@dataclass
class ChangeBullet:
    kind: str  # api | type | behavior | wiring | tests | docs | structure
    text: str
    evidence: list[str]


def build_change_summary(
    files: list[FileDiff],
    changed: list[SymbolSpan],
) -> list[ChangeBullet]:
    """Return ordered, deduplicated narrative bullets for the review pack."""
    bullets: list[ChangeBullet] = []
    bullets.extend(_api_and_type_bullets(files, changed))
    bullets.extend(_behavior_bullets(files, changed))
    bullets.extend(_wiring_bullets(files, changed))
    bullets.extend(_test_and_doc_bullets(files, changed))
    return _dedupe(bullets)


def is_contract_relevant_api(bullet: ChangeBullet) -> bool:
    """Whether an api bullet should become its own Must-check contract row.

    Skips brand-new private helpers (``_foo``) that blew up meta-tooling
    checklists. Keeps public news, removals, and public signature edits.
    """
    if bullet.kind != "api":
        return False
    name = _api_bullet_func_name(bullet.text)
    if name is None:
        return True
    if _is_private_leaf(name):
        # Private signature edits stay in the summary; tag-level C11 covers them.
        return False
    return True


def _api_and_type_bullets(
    files: list[FileDiff], changed: list[SymbolSpan]
) -> list[ChangeBullet]:
    out: list[ChangeBullet] = []
    by_file = {fd.path: fd for fd in files}

    for span in changed:
        leaf = span.qualname.split(".")[-1]
        if (
            span.kind == "class"
            and not _is_private_leaf(leaf)
            and _looks_new_type(span, by_file.get(span.file))
        ):
            out.append(
                ChangeBullet(
                    kind="type",
                    text=f"New type `{span.qualname}` in `{span.file}`.",
                    evidence=[f"{span.file}:{span.qualname}"],
                )
            )

        fd = by_file.get(span.file)
        if fd is None or span.kind not in {"function", "async_function"}:
            continue
        sig = _signature_delta(fd, leaf)
        if not sig:
            continue
        # Drop "New function `_helper`" noise; keep signature edits / removals
        # and new public APIs.
        if sig.startswith("New function ") and _is_private_leaf(leaf):
            continue
        out.append(
            ChangeBullet(
                kind="api",
                text=sig,
                evidence=[f"{span.file}:{span.qualname}"],
            )
        )
    return out


def _api_bullet_func_name(text: str) -> str | None:
    for pattern in (_NEW_FUNC_RE, _API_FUNC_RE, _REMOVED_FUNC_RE):
        match = pattern.match(text)
        if match:
            return match.group(1).split(".")[-1]
    return None


def _is_private_leaf(name: str) -> bool:
    if name.startswith("__") and name.endswith("__"):
        return False
    return name.startswith("_")


def _behavior_bullets(
    files: list[FileDiff], changed: list[SymbolSpan]
) -> list[ChangeBullet]:
    """Surface high-signal behavioral tokens with symbol citations."""
    patterns: list[tuple[re.Pattern[str], str]] = [
        (
            re.compile(r"\badmit|admission|FrameConsumer|refcount|lease\b", re.I),
            "Frame / lease admission or ownership logic changed.",
        ),
        (
            re.compile(r"\bLock\b|\bRLock\b|\.acquire\(|\.release\("),
            "Lock acquire/release behavior changed.",
        ),
        (
            re.compile(r"mission_override|force_off|perception_intent", re.I),
            "Perception intent / mission gating changed.",
        ),
        (
            re.compile(r"emergency.?stop|fail.?closed|set_speed", re.I),
            "Safety / motion command path changed.",
        ),
        (
            re.compile(r"create_subscription|create_publisher|QoS"),
            "ROS pub/sub or QoS wiring changed.",
        ),
    ]

    hits: dict[str, list[str]] = {}
    for span in changed:
        if is_meta_tooling(span.file):
            continue
        for pattern, label in patterns:
            if pattern.search(span.source):
                hits.setdefault(label, []).append(f"{span.file}:{span.qualname}")

    # Also scan raw hunks for files with no Python symbols (docs, etc.).
    for fd in files:
        if is_meta_tooling(fd.path):
            continue
        blob = "\n".join(fd.hunk_texts)
        for pattern, label in patterns:
            if label in hits:
                continue
            if pattern.search(blob):
                hits.setdefault(label, []).append(fd.path)

    out: list[ChangeBullet] = []
    for label, evidence in hits.items():
        out.append(
            ChangeBullet(
                kind="behavior",
                text=label,
                evidence=evidence[:6],
            )
        )
    return out


def _wiring_bullets(
    files: list[FileDiff], changed: list[SymbolSpan]
) -> list[ChangeBullet]:
    """Call-site consumers of frame-ring admission tiers (FrameConsumer)."""
    consumer_re = re.compile(
        r"acquire_latest_frame\(\s*(?:\n\s*)?consumer\s*=\s*FrameConsumer\.(\w+)",
        re.MULTILINE,
    )
    by_consumer: dict[str, list[str]] = {}

    def _add(tier: str, evidence: str) -> None:
        existing = by_consumer.setdefault(tier, [])
        # Avoid duplicates when the same path appears as path and path:qualname.
        path_only = evidence.split(":", 1)[0]
        if any(e == evidence or e.startswith(path_only + ":") for e in existing):
            if evidence not in existing and ":" in evidence:
                # Prefer the more specific symbol citation over a bare path.
                existing[:] = [e for e in existing if e != path_only]
                existing.append(evidence)
            return
        existing.append(evidence)

    for span in changed:
        if is_meta_tooling(span.file):
            continue
        for match in consumer_re.finditer(span.source):
            _add(match.group(1), f"{span.file}:{span.qualname}")

    # Fall back to hunks when excerpt missed a call (multi-line consumer=).
    for fd in files:
        if is_meta_tooling(fd.path):
            continue
        blob = "\n".join(
            line[1:]
            for hunk in fd.hunk_texts
            for line in hunk.splitlines()
            if line.startswith("+") and not line.startswith("+++")
        )
        for match in consumer_re.finditer(blob):
            _add(match.group(1), fd.path)

    if not by_consumer:
        return []

    parts = [
        f"`{tier}` ← {', '.join(f'`{e}`' for e in ev[:4])}"
        for tier, ev in sorted(by_consumer.items())
    ]
    return [
        ChangeBullet(
            kind="wiring",
            text="Frame acquire consumers wired: " + "; ".join(parts) + ".",
            evidence=[e for ev in by_consumer.values() for e in ev][:8],
        )
    ]


def _test_and_doc_bullets(
    files: list[FileDiff], changed: list[SymbolSpan]
) -> list[ChangeBullet]:
    out: list[ChangeBullet] = []
    source_files = [
        f
        for f in files
        if f.path.endswith(".py")
        and not f.path.startswith("tests/")
        and "/test_" not in f.path
        and not f.path.startswith("tools/ai_review/")
    ]
    test_files = [
        f
        for f in files
        if f.path.startswith("tests/")
        or "/test_" in f.path
        or f.path.startswith("scripts/")
        or f.path.endswith("smoke_test.py")
    ]
    doc_files = [f for f in files if f.path.endswith((".md", ".rst", ".txt"))]

    if source_files and not test_files:
        out.append(
            ChangeBullet(
                kind="tests",
                text=(
                    "Source changed without matching test/script updates "
                    f"({len(source_files)} source file(s))."
                ),
                evidence=[f.path for f in source_files[:6]],
            )
        )
    elif test_files:
        new_tests = [
            s.qualname
            for s in changed
            if s.file.startswith("tests/") and s.qualname.startswith("test_")
        ]
        if new_tests:
            out.append(
                ChangeBullet(
                    kind="tests",
                    text=f"Tests updated/added: {', '.join(f'`{t}`' for t in new_tests[:8])}"
                    + ("…" if len(new_tests) > 8 else "")
                    + ".",
                    evidence=[f.path for f in test_files[:6]],
                )
            )
        else:
            out.append(
                ChangeBullet(
                    kind="tests",
                    text=f"Test/script files touched ({len(test_files)}); confirm assertions match new behavior.",
                    evidence=[f.path for f in test_files[:6]],
                )
            )

    if doc_files:
        out.append(
            ChangeBullet(
                kind="docs",
                text=f"Docs updated: {', '.join(f'`{f.path}`' for f in doc_files[:4])}.",
                evidence=[f.path for f in doc_files],
            )
        )

    added = [f for f in files if f.status == "added"]
    deleted = [f for f in files if f.status == "deleted"]
    if added:
        out.append(
            ChangeBullet(
                kind="structure",
                text=f"Added files: {', '.join(f'`{f.path}`' for f in added[:6])}.",
                evidence=[f.path for f in added[:6]],
            )
        )
    if deleted:
        out.append(
            ChangeBullet(
                kind="structure",
                text=f"Deleted files: {', '.join(f'`{f.path}`' for f in deleted[:6])}.",
                evidence=[f.path for f in deleted[:6]],
            )
        )
    return out


def _looks_new_type(span: SymbolSpan, fd: FileDiff | None) -> bool:
    if fd is None:
        return span.kind == "class"
    name = span.qualname.split(".")[-1]
    for hunk in fd.hunk_texts:
        for line in hunk.splitlines():
            if line.startswith("+") and not line.startswith("+++"):
                if _CLASS_RE.match(line[1:]) and name in line:
                    # New if no matching removed class line in same hunk.
                    removed = any(
                        ln.startswith("-")
                        and not ln.startswith("---")
                        and name in ln
                        and "class " in ln
                        for ln in hunk.splitlines()
                    )
                    if not removed:
                        return True
    return False


def _signature_delta(fd: FileDiff, func_name: str) -> str | None:
    """Detect keyword-only or parameter shape changes for *func_name*.

    Handles multi-line ``def`` signatures by scanning contiguous added/removed
    hunk lines until parentheses balance.
    """
    old_sig = _extract_signature_from_hunks(fd, func_name, side="-")
    new_sig = _extract_signature_from_hunks(fd, func_name, side="+")
    if old_sig is None and new_sig is None:
        return None
    if old_sig is None and new_sig is not None:
        return f"New function `{func_name}({new_sig.strip()})` in `{fd.path}`."
    if old_sig is not None and new_sig is None:
        return f"Removed function `{func_name}` from `{fd.path}`."
    assert old_sig is not None and new_sig is not None
    if old_sig.strip() == new_sig.strip():
        return None
    old_params = _param_names(old_sig)
    new_params = _param_names(new_sig)
    added = [p for p in new_params if p not in old_params]
    removed = [p for p in old_params if p not in new_params]
    kwonly = "*" in new_sig and "*" not in old_sig
    bits: list[str] = []
    if kwonly:
        bits.append("now keyword-only")
    if added:
        bits.append("adds " + ", ".join(f"`{p}`" for p in added))
    if removed:
        bits.append("removes " + ", ".join(f"`{p}`" for p in removed))
    if not bits:
        bits.append(f"`({old_sig.strip()})` → `({new_sig.strip()})`")
    return f"API `{func_name}` in `{fd.path}`: " + "; ".join(bits) + "."


def _extract_signature_from_hunks(
    fd: FileDiff, func_name: str, *, side: str
) -> str | None:
    """Pull the parameter list for *func_name* from + or - hunk lines."""
    prefix = side
    lines: list[str] = []
    for hunk in fd.hunk_texts:
        for line in hunk.splitlines():
            if line.startswith(prefix) and not line.startswith(prefix * 3):
                lines.append(line[1:])

    text = "\n".join(lines)
    # Search for def name( ... ) possibly spanning lines.
    pattern = re.compile(
        rf"(?:async\s+)?def\s+{re.escape(func_name)}\s*\(",
        re.MULTILINE,
    )
    match = pattern.search(text)
    if not match:
        # Fallback: single-line regex on each line.
        for line in lines:
            m = re.search(
                rf"def\s+{re.escape(func_name)}\s*\(([^)]*)\)", line
            )
            if m:
                return m.group(1)
        return None

    start = match.end()
    depth = 1
    i = start
    while i < len(text) and depth > 0:
        ch = text[i]
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        i += 1
    if depth != 0:
        return None
    return text[start : i - 1]


def _param_names(sig: str) -> list[str]:
    # Drop *args / **kwargs markers; keep bare names.
    names: list[str] = []
    for part in sig.split(","):
        part = part.strip()
        if not part or part in {"*", "/"}:
            continue
        part = part.lstrip("*")
        name = part.split(":")[0].split("=")[0].strip()
        if name and name.isidentifier() and name != "self":
            names.append(name)
    return names


def _dedupe(bullets: list[ChangeBullet]) -> list[ChangeBullet]:
    seen: set[str] = set()
    out: list[ChangeBullet] = []
    for b in bullets:
        key = f"{b.kind}|{b.text}"
        if key in seen:
            continue
        seen.add(key)
        out.append(b)
    return out
