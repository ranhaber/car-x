#!/usr/bin/env python3
"""Build a token-optimized AI code-review pack from a git diff.

Examples:
  python tools/ai_review/build_pack.py --uncommitted --out .cursor/review_pack
  python tools/ai_review/build_pack.py --base origin/main --out .cursor/review_pack
  python tools/ai_review/build_pack.py --staged --out .cursor/review_pack
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

# Allow running as `python tools/ai_review/build_pack.py` without install.
_TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from ai_review import MAX_CHANGED_SYMBOLS, MAX_EXCERPT_LINES, MAX_RELATED_SYMBOLS  # noqa: E402
from ai_review.change_summary import build_change_summary  # noqa: E402
from ai_review.checklist import build_must_check  # noqa: E402
from ai_review.classify import classify_diffs  # noqa: E402
from ai_review.diff import collect_diff, filter_review_paths  # noqa: E402
from ai_review.lenses import must_read_docs, select_lenses  # noqa: E402
from ai_review.pack import build_pack_object, write_pack  # noqa: E402
from ai_review.slice import slice_related_with_total  # noqa: E402
from ai_review.spotlight import rank_changed_symbols, spotlight_names  # noqa: E402
from ai_review.symbols import extract_changed_symbols  # noqa: E402


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _warn_untracked(repo: Path, include_paths: list[str] | None) -> None:
    """git diff never sees untracked files; tip the operator when likely omitted."""
    completed = subprocess.run(
        ["git", "status", "--porcelain", "-u"],
        cwd=str(repo),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if completed.returncode != 0:
        return
    untracked: list[str] = []
    for line in completed.stdout.splitlines():
        if not line.startswith("??"):
            continue
        path = line[3:].strip().replace("\\", "/")
        if include_paths:
            prefixes = [p.replace("\\", "/").rstrip("/") for p in include_paths]
            if not any(path == p or path.startswith(p + "/") for p in prefixes):
                continue
        untracked.append(path)
    if untracked:
        sample = ", ".join(untracked[:5])
        more = f" (+{len(untracked) - 5} more)" if len(untracked) > 5 else ""
        print(
            "WARNING: untracked files omitted by git diff; "
            f"run `git add -N <path>` first: {sample}{more}"
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build AI code-review pack (deterministic).")
    scope = parser.add_mutually_exclusive_group()
    scope.add_argument("--uncommitted", action="store_true", help="Diff working tree vs HEAD")
    scope.add_argument("--staged", action="store_true", help="Diff index vs HEAD")
    scope.add_argument(
        "--base",
        type=str,
        default=None,
        help="Branch/ref to diff against via merge-base (default: auto)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(".cursor/review_pack"),
        help="Output directory for review_pack.json and review_pack.md",
    )
    parser.add_argument(
        "--paths",
        nargs="+",
        default=None,
        metavar="PREFIX",
        help=(
            "Restrict the pack to these repo-relative path prefixes "
            "(review a large tree in focused passes)"
        ),
    )
    parser.add_argument(
        "--max-changed",
        type=int,
        default=MAX_CHANGED_SYMBOLS,
        help=f"Max changed symbols (default {MAX_CHANGED_SYMBOLS})",
    )
    parser.add_argument(
        "--max-related",
        type=int,
        default=MAX_RELATED_SYMBOLS,
        help=f"Max related symbols (default {MAX_RELATED_SYMBOLS})",
    )
    parser.add_argument(
        "--max-excerpt-lines",
        type=int,
        default=MAX_EXCERPT_LINES,
        help=f"Max excerpt lines per symbol (default {MAX_EXCERPT_LINES})",
    )
    args = parser.parse_args(argv)

    repo = _repo_root()
    out_dir = args.out if args.out.is_absolute() else repo / args.out

    _warn_untracked(repo, args.paths)

    diff = collect_diff(
        repo,
        uncommitted=args.uncommitted,
        staged=args.staged,
        base=args.base,
    )
    files = filter_review_paths(diff.files, args.paths)
    risk = classify_diffs(files)

    changed = []
    for fd in files:
        if fd.status == "deleted":
            continue
        lines = set(fd.added_lines)
        # For pure deletions inside a file, also use removed line numbers as hints
        # against current file (best-effort).
        if not lines and fd.removed_context_lines:
            lines = set(fd.removed_context_lines)
        changed.extend(
            extract_changed_symbols(
                repo,
                fd.path,
                lines,
                max_excerpt_lines=args.max_excerpt_lines,
            )
        )

    # Deduplicate by file+qualname, then rank so caps keep spotlight hunks.
    seen: set[tuple[str, str]] = set()
    uniq_changed = []
    for span in changed:
        key = (span.file, span.qualname)
        if key in seen:
            continue
        seen.add(key)
        uniq_changed.append(span)
    changed_total = len(uniq_changed)
    uniq_changed = rank_changed_symbols(uniq_changed, files, risk)
    uniq_changed = uniq_changed[: args.max_changed]
    spotlight = spotlight_names(uniq_changed)

    related = []
    related_total = 0
    if risk.level == "deep":
        related, related_total = slice_related_with_total(
            repo,
            uniq_changed,
            max_related=args.max_related,
            max_excerpt_lines=args.max_excerpt_lines,
        )

    clipped_symbols = sum(1 for span in uniq_changed if span.clipped)
    symbols_dropped = changed_total > len(uniq_changed) or related_total > len(related)
    caps = {
        "max_changed_symbols": args.max_changed,
        "max_related_symbols": args.max_related,
        "max_excerpt_lines": args.max_excerpt_lines,
        "changed_symbols_total": changed_total,
        "changed_symbols_included": len(uniq_changed),
        "related_symbols_total": related_total,
        "related_symbols_included": len(related),
        "excerpt_clipped_symbols": clipped_symbols,
        # Truncation means symbols were dropped by a cap — not excerpt clipping.
        "truncated": symbols_dropped,
    }

    summary = build_change_summary(files, uniq_changed)
    must_check = build_must_check(risk, files, uniq_changed, summary)
    lenses = select_lenses(risk)
    docs = must_read_docs(risk)
    pack = build_pack_object(
        repo=repo,
        mode=diff.mode,
        base=diff.base,
        risk=risk,
        changed=uniq_changed,
        related=related,
        lenses=lenses,
        docs=docs,
        files_touched=[f.path for f in files],
        caps=caps,
        include_paths=args.paths,
        change_summary=summary,
        must_check=must_check,
        spotlight=spotlight,
    )
    json_path, md_path = write_pack(out_dir, pack)

    print(f"risk={risk.level} tags={','.join(risk.tags) or '-'} files={len(files)}")
    print(
        f"changed_symbols={len(uniq_changed)}/{changed_total} "
        f"related_symbols={len(related)}/{related_total} "
        f"clipped_excerpts={clipped_symbols}"
    )
    if changed_total > len(uniq_changed):
        print("WARNING: changed symbols dropped; re-run with --paths for a focused pass")
    if related_total > len(related):
        print(
            "WARNING: related symbols dropped; raise --max-related or narrow --paths"
        )
    if clipped_symbols and not symbols_dropped:
        print(
            f"NOTE: {clipped_symbols} excerpt(s) clipped to changed lines; "
            "pack is complete — open the file only if skipped context matters"
        )
    elif clipped_symbols:
        print(
            f"NOTE: {clipped_symbols} excerpt(s) clipped to changed lines; "
            "read the file directly if surrounding context matters"
        )
    print(f"wrote {json_path}")
    print(f"wrote {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
