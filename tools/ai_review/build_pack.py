#!/usr/bin/env python3
"""Build a token-optimized AI code-review pack from a git diff.

Examples:
  python tools/ai_review/build_pack.py --uncommitted --out .cursor/review_pack
  python tools/ai_review/build_pack.py --base origin/main --out .cursor/review_pack
  python tools/ai_review/build_pack.py --staged --out .cursor/review_pack
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow running as `python tools/ai_review/build_pack.py` without install.
_TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from ai_review import MAX_CHANGED_SYMBOLS, MAX_EXCERPT_LINES, MAX_RELATED_SYMBOLS  # noqa: E402
from ai_review.classify import classify_diffs  # noqa: E402
from ai_review.diff import collect_diff, filter_review_paths  # noqa: E402
from ai_review.lenses import must_read_docs, select_lenses  # noqa: E402
from ai_review.pack import build_pack_object, write_pack  # noqa: E402
from ai_review.slice import slice_related  # noqa: E402
from ai_review.symbols import extract_changed_symbols  # noqa: E402


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


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

    # Deduplicate by file+qualname preserving order.
    seen: set[tuple[str, str]] = set()
    uniq_changed = []
    for span in changed:
        key = (span.file, span.qualname)
        if key in seen:
            continue
        seen.add(key)
        uniq_changed.append(span)
    changed_total = len(uniq_changed)
    uniq_changed = uniq_changed[: args.max_changed]

    related = []
    related_total = 0
    if risk.level == "deep":
        related = slice_related(
            repo,
            uniq_changed,
            max_related=args.max_related,
            max_excerpt_lines=args.max_excerpt_lines,
        )
        related_total = len(related)

    caps = {
        "max_changed_symbols": args.max_changed,
        "max_related_symbols": args.max_related,
        "max_excerpt_lines": args.max_excerpt_lines,
        "changed_symbols_total": changed_total,
        "changed_symbols_included": len(uniq_changed),
        "related_symbols_total": related_total,
        "related_symbols_included": len(related),
        "truncated": changed_total > len(uniq_changed),
    }

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
    )
    json_path, md_path = write_pack(out_dir, pack)

    print(f"risk={risk.level} tags={','.join(risk.tags) or '-'} files={len(files)}")
    print(
        f"changed_symbols={len(uniq_changed)}/{changed_total} "
        f"related_symbols={len(related)}"
    )
    if caps["truncated"]:
        print("WARNING: changed symbols truncated; re-run with --paths for a focused pass")
    print(f"wrote {json_path}")
    print(f"wrote {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
