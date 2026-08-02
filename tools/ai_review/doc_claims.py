#!/usr/bin/env python3
"""Verify documentation claims against the working tree.

Companion to build_pack.py for docs-only review passes. A markdown diff carries
no Python symbols, so the pack has nothing to excerpt; this tool supplies the
missing half by checking what the docs assert against what the code contains.

Absence of a symbol is only a defect when the document claims to describe the
implementation. Documents that declare a target/future state are classified
from their own ``**Status:**`` header and reported separately.

Examples:
  python tools/ai_review/doc_claims.py
  python tools/ai_review/doc_claims.py --base origin/main
  python tools/ai_review/doc_claims.py --paths cat_follow/docs
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from collections import Counter, defaultdict

SOURCE_EXTS = (".py", ".json", ".env", ".service", ".html", ".js", ".css",
               ".yaml", ".yml", ".txt", ".cfg", ".toml", ".sh", ".xml")
SKIP_DIRS = ("__pycache__", ".git", ".pytest_cache", "node_modules")

# Tokens that look like code but are never defined in this repo.
IGNORE_TOKENS = {"MUST", "MUST NOT", "SHOULD", "SHOULD NOT", "MAY", "TODO", "N/A"}
IGNORE_PREFIXES = ("rknn_", "VIDIOC_", "STREAM", "v4l2_", "gst_", "mpp")

STATUS_RE = re.compile(r"^\*\*Status:\*\*\s*(.+?)\s*$", re.M)
TARGET_HINTS = ("not implemented", "target", "pending", "ready for implementation",
                "in progress", "planned", "proposed")


def sh(args: list[str]) -> str:
    return subprocess.run(args, capture_output=True, text=True,
                          encoding="utf-8", errors="replace").stdout


def classify(path: str) -> str:
    """descriptive = claims to describe code; target = declares future state."""
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            head = fh.read(4000)
    except OSError:
        return "descriptive"
    m = STATUS_RE.search(head)
    if not m:
        return "descriptive"
    status = m.group(1).lower()
    if any(h in status for h in TARGET_HINTS):
        return "target"
    return "descriptive"


def changed_docs(base: str | None, paths: list[str] | None) -> list[str]:
    if base:
        out = sh(["git", "diff", "--name-status", f"{base}...HEAD"])
    else:
        out = sh(["git", "status", "--porcelain"])
    docs = []
    for line in out.splitlines():
        if len(line) < 4:
            continue
        status, p = line[:2], line[3:].strip().replace("\\", "/").strip('"')
        if not p.endswith(".md") or "D" in status or not os.path.exists(p):
            continue
        if paths and not any(p.startswith(pref.replace("\\", "/")) for pref in paths):
            continue
        docs.append(p)
    return sorted(set(docs))


def build_index() -> tuple[set[str], dict[str, str], str]:
    """One pass over the tree: paths, file text, and a single search blob."""
    paths: set[str] = set()
    texts: dict[str, str] = {}
    for root, dirs, files in os.walk("."):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for fn in files:
            p = os.path.relpath(os.path.join(root, fn)).replace("\\", "/")
            paths.add(p)
            if p.endswith(SOURCE_EXTS):
                try:
                    with open(p, encoding="utf-8", errors="replace") as fh:
                        texts[p] = fh.read()
                except OSError:
                    pass
    return paths, texts, "\n".join(texts.values())


def extract_claims(docs: list[str], base: str | None) -> dict[str, set[str]]:
    """Backtick-quoted, code-shaped tokens from lines the diff added."""
    claims: dict[str, set[str]] = defaultdict(set)
    for d in docs:
        diff = sh(["git", "diff", "-U0", base or "HEAD", "--", d])
        added = "\n".join(l[1:] for l in diff.splitlines()
                          if l.startswith("+") and not l.startswith("+++"))
        for tok in re.findall(r"`([^`\n]{2,80})`", added):
            tok = tok.strip()
            if tok in IGNORE_TOKENS or tok.startswith(IGNORE_PREFIXES):
                continue
            if re.search(r"[/\\]|\.[a-zA-Z]{1,5}\b|_[a-zA-Z]|[a-z][A-Z]|^[A-Z0-9_]{3,}$|\(\)$", tok):
                claims[tok].add(d)
    return claims


def verify(claims: dict[str, set[str]], paths: set[str], texts: dict[str, str],
           blob: str) -> list[tuple[str, str, str, str, str]]:
    token_index = set(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", blob))
    basenames: dict[str, list[str]] = defaultdict(list)
    dirs: set[str] = set()
    for p in paths:
        basenames[os.path.basename(p)].append(p)
        parent = os.path.dirname(p)
        while parent:
            dirs.add(parent)
            parent = os.path.dirname(parent)

    # Value assertions live in env, service, and *_config.py files. Keep the
    # separators newline-free so an empty value cannot absorb the next line.
    values: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for p, t in texts.items():
        if p.endswith((".env", ".service")) or p.endswith("_config.py") or "/scripts/" in p:
            for k, v in re.findall(r"^[ \t]*([A-Z][A-Z0-9_]{2,})[ \t]*=[ \t]*([^\s#]*)", t, re.M):
                values[k].append((p, v.strip("\"'")))
        if p.endswith(".py"):
            for k, v in re.findall(r"^([A-Z][A-Z0-9_]{2,})(?::[ \t]*\w+)?[ \t]*=[ \t]*([^\s#]*)", t, re.M):
                values[k].append((p, v.strip("\"'")))

    rows = []
    for tok in sorted(claims):
        docs = claims[tok]
        bucket = "descriptive" if any(classify(d) == "descriptive" for d in docs) else "target"
        where = ";".join(sorted(os.path.basename(d) for d in docs))

        m = re.match(r"^([A-Z][A-Z0-9_]{2,})\s*=\s*(.+)$", tok)
        if m:
            key, claimed = m.group(1), m.group(2).strip().strip("\"'")
            if key in values:
                if not any(claimed == v for _, v in values[key]):
                    rows.append(("VALUE-MISMATCH", tok, where, bucket,
                                 "actual " + ", ".join(f"{v or '(empty)'} ({p})"
                                                       for p, v in values[key][:2])))
            else:
                rows.append(("KEY-UNKNOWN", tok, where, bucket, ""))
            continue

        if re.match(r"^[A-Za-z0-9_./\\-]+\.(py|json|env|service|html|js|yaml|yml|txt|sh|md)$", tok):
            p = tok.replace("\\", "/").lstrip("./")
            if p in paths or os.path.exists(p):
                continue
            alt = basenames.get(os.path.basename(p), [])
            rows.append(("AMBIGUOUS-PATH" if len(alt) > 1 else "MOVED" if alt else "MISSING-PATH",
                         tok, where, bucket, ", ".join(alt[:3])))
            continue

        search = tok[:-2] if tok.endswith("()") else tok
        if " " in search:
            continue  # prose, not mechanically checkable
        if search.replace("\\", "/").rstrip("/") in dirs:
            continue  # directory reference
        if search in token_index or search in blob:
            continue
        rows.append(("NOT-FOUND", tok, where, bucket, ""))
    return rows


def check_links(docs: list[str], paths: set[str]) -> list[tuple[str, str]]:
    bad = []
    for d in docs:
        with open(d, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        for ref in set(re.findall(r"\]\(([^)#\s]+\.md)(?:#[^)]*)?\)", text)):
            if ref.startswith(("http://", "https://")):
                continue
            cand = os.path.normpath(os.path.join(os.path.dirname(d), ref)).replace("\\", "/")
            if not os.path.exists(cand):
                bad.append((d, ref))
    return bad


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Verify doc claims against the working tree.")
    ap.add_argument("--base", default=None, help="Diff against this ref instead of the working tree")
    ap.add_argument("--paths", nargs="+", default=None, metavar="PREFIX",
                    help="Restrict to these repo-relative path prefixes")
    ap.add_argument("--all", action="store_true",
                    help="Also print target-doc anomalies (default: descriptive only)")
    args = ap.parse_args(argv)

    docs = changed_docs(args.base, args.paths)
    if not docs:
        print("no changed markdown files in scope")
        return 0

    paths, texts, blob = build_index()
    claims = extract_claims(docs, args.base)
    rows = verify(claims, paths, texts, blob)
    links = check_links(docs, paths)

    desc = [d for d in docs if classify(d) == "descriptive"]
    print(f"docs={len(docs)} (descriptive={len(desc)}, target={len(docs) - len(desc)})  "
          f"claims={len(claims)}  anomalies={len(rows)}  broken_links={len(links)}")
    print("by status:", dict(Counter(r[0] for r in rows)) or "-")
    print()
    print("Target-state docs (absence is expected, not a defect):")
    for d in docs:
        if classify(d) == "target":
            print(f"  {d}")
    print()

    shown = [r for r in rows if args.all or r[3] == "descriptive"]
    for status, tok, where, bucket, detail in sorted(shown, key=lambda r: (r[3], r[0], r[1])):
        print(f"{bucket:12} {status:15} {tok:48} {where}" + (f"  | {detail}" if detail else ""))
    for d, ref in links:
        print(f"{'link':12} {'BROKEN-LINK':15} {ref:48} {d}")

    if not shown and not links:
        print("no anomalies in descriptive docs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
