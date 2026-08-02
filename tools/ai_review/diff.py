"""Collect git diffs and map changed lines per file."""

from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class FileDiff:
    path: str
    status: str  # modified | added | deleted | renamed
    old_path: str | None = None
    added_lines: set[int] = field(default_factory=set)
    removed_context_lines: set[int] = field(default_factory=set)
    hunk_texts: list[str] = field(default_factory=list)
    raw_patch: str = ""


@dataclass
class DiffResult:
    mode: str
    base: str | None
    files: list[FileDiff]
    raw: str


_HUNK_RE = re.compile(r"^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@")


def _run_git(repo: Path, args: list[str]) -> str:
    completed = subprocess.run(
        ["git", *args],
        cwd=str(repo),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if completed.returncode != 0:
        err = (completed.stderr or completed.stdout or "").strip()
        raise RuntimeError(f"git {' '.join(args)} failed: {err}")
    return completed.stdout


def _working_tree_dirty(repo: Path) -> bool:
    out = _run_git(repo, ["status", "--porcelain"])
    return bool(out.strip())


def resolve_default_base(repo: Path) -> str:
    for candidate in ("origin/main", "main", "origin/master", "master"):
        probe = subprocess.run(
            ["git", "rev-parse", "--verify", candidate],
            cwd=str(repo),
            capture_output=True,
            text=True,
            check=False,
        )
        if probe.returncode == 0:
            return candidate
    return "HEAD"


def collect_diff(
    repo: Path,
    *,
    uncommitted: bool = False,
    staged: bool = False,
    base: str | None = None,
) -> DiffResult:
    """Collect a unified diff.

    Priority when flags are unset:
    - dirty working tree -> uncommitted (staged + unstaged vs HEAD)
    - else -> branch changes vs merge-base with default base
    """
    if staged:
        raw = _run_git(repo, ["diff", "--cached", "--no-ext-diff", "--find-renames"])
        return DiffResult(mode="staged", base="HEAD", files=_parse_unified_diff(raw), raw=raw)

    if uncommitted:
        raw = _run_git(repo, ["diff", "HEAD", "--no-ext-diff", "--find-renames"])
        return DiffResult(mode="uncommitted", base="HEAD", files=_parse_unified_diff(raw), raw=raw)

    if base is None and _working_tree_dirty(repo):
        raw = _run_git(repo, ["diff", "HEAD", "--no-ext-diff", "--find-renames"])
        return DiffResult(mode="uncommitted", base="HEAD", files=_parse_unified_diff(raw), raw=raw)

    resolved_base = base or resolve_default_base(repo)
    if resolved_base == "HEAD":
        raw = _run_git(repo, ["diff", "HEAD", "--no-ext-diff", "--find-renames"])
        return DiffResult(mode="uncommitted", base="HEAD", files=_parse_unified_diff(raw), raw=raw)

    merge_base = _run_git(repo, ["merge-base", resolved_base, "HEAD"]).strip()
    raw = _run_git(
        repo,
        ["diff", f"{merge_base}...HEAD", "--no-ext-diff", "--find-renames"],
    )
    # Include uncommitted on top of branch commits when dirty.
    if _working_tree_dirty(repo):
        dirty = _run_git(repo, ["diff", "HEAD", "--no-ext-diff", "--find-renames"])
        if dirty.strip():
            raw = raw + ("\n" if raw and not raw.endswith("\n") else "") + dirty
    return DiffResult(
        mode="branch",
        base=f"{resolved_base} (merge-base {merge_base[:12]})",
        files=_parse_unified_diff(raw),
        raw=raw,
    )


def _parse_unified_diff(raw: str) -> list[FileDiff]:
    if not raw.strip():
        return []

    files: list[FileDiff] = []
    current: FileDiff | None = None
    old_line = 0
    new_line = 0
    patch_lines: list[str] = []
    hunk_buf: list[str] = []

    def flush_hunk() -> None:
        nonlocal hunk_buf
        if current is not None and hunk_buf:
            current.hunk_texts.append("\n".join(hunk_buf))
            hunk_buf = []

    def flush_file() -> None:
        nonlocal current, patch_lines
        flush_hunk()
        if current is not None:
            current.raw_patch = "\n".join(patch_lines)
            files.append(current)
        current = None
        patch_lines = []

    for line in raw.splitlines():
        if line.startswith("diff --git "):
            flush_file()
            parts = line.split()
            # diff --git a/path b/path
            a_path = parts[2][2:] if len(parts) > 2 and parts[2].startswith("a/") else ""
            b_path = parts[3][2:] if len(parts) > 3 and parts[3].startswith("b/") else a_path
            current = FileDiff(path=b_path or a_path, status="modified", old_path=a_path or None)
            patch_lines = [line]
            continue

        if current is None:
            continue

        patch_lines.append(line)

        if line.startswith("new file mode"):
            current.status = "added"
            continue
        if line.startswith("deleted file mode"):
            current.status = "deleted"
            continue
        if line.startswith("rename from "):
            current.status = "renamed"
            current.old_path = line[len("rename from ") :]
            continue
        if line.startswith("rename to "):
            current.path = line[len("rename to ") :]
            continue
        if line.startswith("--- ") or line.startswith("+++ "):
            continue

        m = _HUNK_RE.match(line)
        if m:
            flush_hunk()
            hunk_buf = [line]
            old_line = int(m.group(1))
            new_line = int(m.group(3))
            continue

        if not hunk_buf and not line.startswith(("+", "-", " ")):
            continue

        if line.startswith("+") and not line.startswith("+++"):
            hunk_buf.append(line)
            current.added_lines.add(new_line)
            new_line += 1
        elif line.startswith("-") and not line.startswith("---"):
            hunk_buf.append(line)
            current.removed_context_lines.add(old_line)
            old_line += 1
        elif line.startswith(" "):
            hunk_buf.append(line)
            old_line += 1
            new_line += 1
        elif line.startswith("\\"):
            hunk_buf.append(line)

    flush_file()
    return files


def filter_review_paths(
    files: list[FileDiff], include_paths: list[str] | None = None
) -> list[FileDiff]:
    """Scope the diff to a review pass.

    ``include_paths`` restricts the pack to explicit path prefixes so a large
    working tree can be reviewed in focused passes.  Without it, prefer
    ``cat_follow/`` and ``ros_ws/`` and fall back to everything.
    """
    if include_paths:
        prefixes = [p.replace("\\", "/").rstrip("/") for p in include_paths]
        return [
            f
            for f in files
            if any(
                f.path == prefix or f.path.startswith(prefix + "/")
                for prefix in prefixes
            )
        ]

    preferred = [
        f
        for f in files
        if f.path.startswith("cat_follow/")
        or f.path.startswith("ros_ws/")
        or f.path.startswith("tools/ai_review/")
    ]
    return preferred if preferred else files
