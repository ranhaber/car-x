"""Emit review_pack.json and review_pack.md."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path

from .change_summary import ChangeBullet
from .checklist import CheckItem
from .classify import RiskAssessment
from .lenses import Lens
from .slice import RelatedSymbol
from .symbols import SymbolSpan


@dataclass
class ReviewPack:
    meta: dict
    risk: dict
    changed_symbols: list[dict] = field(default_factory=list)
    related_symbols: list[dict] = field(default_factory=list)
    change_summary: list[dict] = field(default_factory=list)
    must_check: list[dict] = field(default_factory=list)
    lenses: list[dict] = field(default_factory=list)
    must_read_docs: list[str] = field(default_factory=list)
    spotlight: list[str] = field(default_factory=list)
    skip_advice: str | None = None
    files_touched: list[str] = field(default_factory=list)


def build_pack_object(
    *,
    repo: Path,
    mode: str,
    base: str | None,
    risk: RiskAssessment,
    changed: list[SymbolSpan],
    related: list[RelatedSymbol],
    lenses: list[Lens],
    docs: list[str],
    files_touched: list[str],
    caps: dict | None = None,
    include_paths: list[str] | None = None,
    change_summary: list[ChangeBullet] | None = None,
    must_check: list[CheckItem] | None = None,
    spotlight: list[str] | None = None,
) -> ReviewPack:
    skip = None
    if risk.level == "shallow":
        skip = "Skim only; stop if no behavior change. Do not open unrelated subsystems."

    return ReviewPack(
        meta={
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "repo_root": str(repo.resolve()),
            "mode": mode,
            "git_base": base,
            "schema_version": 2,
            "include_paths": include_paths or [],
            "caps": caps or {},
        },
        risk={
            "level": risk.level,
            "tags": risk.tags,
            "reasons": risk.reasons,
        },
        changed_symbols=[_symbol_dict(s) for s in changed],
        related_symbols=[_related_dict(r) for r in related],
        change_summary=[_bullet_dict(b) for b in (change_summary or [])],
        must_check=[_check_dict(c) for c in (must_check or [])],
        lenses=[{"id": ln.id, "why": ln.why} for ln in lenses],
        must_read_docs=docs,
        spotlight=list(spotlight or []),
        skip_advice=skip,
        files_touched=files_touched,
    )


def write_pack(out_dir: Path, pack: ReviewPack) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "review_pack.json"
    md_path = out_dir / "review_pack.md"
    payload = asdict(pack)
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    md_path.write_text(render_markdown(pack), encoding="utf-8")
    return json_path, md_path


def render_markdown(pack: ReviewPack) -> str:
    risk = pack.risk
    lines: list[str] = [
        "# AI Review Pack",
        "",
        f"- Generated: `{pack.meta.get('generated_at')}`",
        f"- Mode: `{pack.meta.get('mode')}`",
        f"- Git base: `{pack.meta.get('git_base')}`",
        f"- Risk: **{risk.get('level', '?').upper()}**",
    ]
    include_paths = pack.meta.get("include_paths") or []
    if include_paths:
        lines.append(
            "- Scoped to: " + ", ".join(f"`{p}`" for p in include_paths)
        )
    caps = pack.meta.get("caps") or {}
    symbols_dropped = bool(caps.get("truncated"))
    clipped = caps.get("excerpt_clipped_symbols") or 0
    if symbols_dropped:
        lines.append(
            "- **Truncated**: "
            f"{caps.get('changed_symbols_included', 0)}/"
            f"{caps.get('changed_symbols_total', 0)} changed symbols and "
            f"{caps.get('related_symbols_included', 0)}/"
            f"{caps.get('related_symbols_total', 0)} related symbols included. "
            "Re-run with `--paths` to review the remainder."
        )
    if clipped:
        lines.append(
            f"- **Clipped excerpts**: {clipped} changed symbol(s) exceeded "
            f"`max_excerpt_lines` ({caps.get('max_excerpt_lines')}). Those "
            "excerpts keep every changed line and mark omitted ranges as "
            "`# ... skipped lines A-B ...`. Open the file for a finding "
            "that depends on skipped context."
        )
    lines += [
        "",
        "## Risk",
        "",
    ]
    tags = risk.get("tags") or []
    reasons = risk.get("reasons") or []
    lines.append(f"- Tags: {', '.join(f'`{t}`' for t in tags) if tags else '_none_'}")
    lines.append("- Reasons:")
    if reasons:
        for r in reasons:
            lines.append(f"  - {r}")
    else:
        lines.append("  - _none_")

    if pack.skip_advice:
        lines.extend(["", "## Skip advice", "", pack.skip_advice])

    lines.extend(
        [
            "",
            "## Review intent",
            "",
            "State in one sentence **why this change exists**, using the user "
            "request, PR description, or commit message. Do not invent product "
            "intent from the diff alone. Then deep-review Spotlight symbols first.",
            "",
            "## Spotlight",
            "",
        ]
    )
    if pack.spotlight:
        for cite in pack.spotlight:
            lines.append(f"- `{cite}`")
    else:
        lines.append("_No ranked symbols (non-Python or empty)._")

    lines.extend(["", "## Must-read docs", ""])
    for doc in pack.must_read_docs:
        lines.append(f"- `{doc}`")

    lines.extend(["", "## Lenses", ""])
    for ln in pack.lenses:
        lines.append(f"- `{ln['id']}` — {ln['why']}")

    lines.extend(["", "## Change summary", ""])
    if not pack.change_summary:
        lines.append("_No narrative bullets._")
    else:
        for bullet in pack.change_summary:
            kind = bullet.get("kind", "?")
            text = bullet.get("text", "")
            evidence = bullet.get("evidence") or []
            ev = ", ".join(f"`{e}`" for e in evidence[:6])
            lines.append(f"- **{kind}**: {text}")
            if ev:
                lines.append(f"  - evidence: {ev}")

    lines.extend(["", "## Must-check", ""])
    if not pack.must_check:
        lines.append("_No checklist items._")
    else:
        for item in pack.must_check:
            area = item.get("area", "?")
            question = item.get("question", "")
            lens = item.get("lens", "")
            evidence = item.get("evidence") or []
            ev = ", ".join(f"`{e}`" for e in evidence[:6])
            lines.append(f"- **{area}** [{lens}]: {question}")
            if ev:
                lines.append(f"  - evidence: {ev}")

    lines.extend(["", "## Files touched", ""])
    for path in pack.files_touched:
        lines.append(f"- `{path}`")

    lines.extend(["", "## Changed symbols", ""])
    if not pack.changed_symbols:
        lines.append("_No Python symbols extracted (non-Python or empty)._")
    for sym in pack.changed_symbols:
        lines.extend(_render_symbol(sym, heading="###"))

    lines.extend(["", "## Related symbols", ""])
    if not pack.related_symbols:
        lines.append("_None within slice caps._")
    for rel in pack.related_symbols:
        relation = rel.get("relation", "?")
        lines.append(f"### `{rel.get('qualname')}` ({relation})")
        lines.append("")
        lines.append(f"- File: `{rel.get('file')}` lines {rel.get('line_range')}")
        lines.append("")
        lines.append("```python")
        lines.append(rel.get("excerpt") or "")
        lines.append("```")
        lines.append("")

    lines.extend(
        [
            "## Agent handoff",
            "",
            "1. State Review intent (why), then read only the docs and excerpts above.",
            "2. Deep-review Spotlight symbols before lower-ranked excerpts.",
            "3. Answer every Must-check item before free-form findings.",
            "4. Do not open unrelated large files or whole subsystems.",
            "5. Apply listed lenses from `cat_follow/docs/Code_Review_Plan.md`.",
            "6. Report: `Severity | Dimension | Area | Location | Finding`.",
            "7. Note uncertainty when the slice is incomplete.",
            "8. Do not fix code unless the user asks.",
            "",
        ]
    )
    return "\n".join(lines)


def _symbol_dict(span: SymbolSpan) -> dict:
    return {
        "file": span.file,
        "qualname": span.qualname,
        "kind": span.kind,
        "line_range": span.line_range,
        "excerpt": span.source,
        "clipped": span.clipped,
    }


def _related_dict(rel: RelatedSymbol) -> dict:
    d = _symbol_dict(rel.span)
    d["relation"] = rel.relation
    return d


def _bullet_dict(bullet: ChangeBullet) -> dict:
    return {
        "kind": bullet.kind,
        "text": bullet.text,
        "evidence": list(bullet.evidence),
    }


def _check_dict(item: CheckItem) -> dict:
    return {
        "area": item.area,
        "question": item.question,
        "lens": item.lens,
        "evidence": list(item.evidence),
        "tag": item.tag,
    }


def _render_symbol(sym: dict, heading: str = "###") -> list[str]:
    out = [
        f"{heading} `{sym.get('qualname')}`",
        "",
        f"- File: `{sym.get('file')}` lines {sym.get('line_range')} ({sym.get('kind')})",
    ]
    if sym.get("clipped"):
        out.append("- Excerpt clipped: changed lines kept, gaps marked")
    out += [
        "",
        "```python",
        sym.get("excerpt") or "",
        "```",
        "",
    ]
    return out
