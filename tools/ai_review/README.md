# AI Review Pack Builder (MVP)

Deterministic (no LLM) tool that turns a git diff into a small **review pack**
for token-efficient AI code reviews.

Authority for what to check remains
[`cat_follow/docs/Code_Review_Plan.md`](../../cat_follow/docs/Code_Review_Plan.md).
This tool only scopes context.

## Run

From the repository root:

```text
python tools/ai_review/build_pack.py --uncommitted --out .cursor/review_pack
python tools/ai_review/build_pack.py --staged --out .cursor/review_pack
python tools/ai_review/build_pack.py --base origin/main --out .cursor/review_pack
```

Default (no flags): if the working tree is dirty, uses uncommitted diff vs
`HEAD`; otherwise diffs the branch against the merge-base with `main`/`master`.

Every mode runs `git diff`, which never reports untracked files. Stage a
brand-new file with `git add -N <path>` first, or the pack silently omits it.
The CLI prints a warning when untracked paths look relevant.

Without `--paths`, the pack prefers `cat_follow/`, `ros_ws/`, `tools/ai_review/`,
`.cursor/skills/`, `tests/`, and `scripts/`, falling back to every changed file
when none match. Tests are included on purpose: the Tests lens cannot be
applied to a change whose tests were filtered out of the pack.

Large working trees hit the symbol caps, so review them in focused passes with
`--paths`:

```text
python tools/ai_review/build_pack.py --uncommitted --paths cat_follow/control cat_follow/runtime/control_loop.py
python tools/ai_review/build_pack.py --uncommitted --paths cat_follow/perception cat_follow/memory cat_follow/threads
```

Caps:

- `meta.caps.truncated` — changed or related symbols were dropped by a cap.
  Re-run with `--paths`.
- `meta.caps.excerpt_clipped_symbols` — long symbols were excerpted around
  changed lines. The pack is still complete; open the file only if skipped
  context matters.

Outputs:

- `.cursor/review_pack/review_pack.json`
- `.cursor/review_pack/review_pack.md`

## Pack schema (`schema_version: 2`)

| Field | Meaning |
|---|---|
| `meta` | repo root, mode, git base, timestamp, `include_paths`, `caps` |
| `risk` | `shallow` \| `deep`, `tags[]`, `reasons[]` |
| `change_summary[]` | narrative bullets (`api` / `behavior` / `wiring` / …) with evidence |
| `must_check[]` | concrete checklist items with lens + evidence (public API deltas only) |
| `spotlight[]` | ranked `file:qualname` citations that deserve deep scrutiny first |
| `changed_symbols[]` | file, qualname, kind, line_range, excerpt, `clipped` (risk-ranked) |
| `related_symbols[]` | same + `relation` (`calls` / `called_by` / `same_module` / `keyword_hit`) |
| `lenses[]` | Code_Review_Plan lens ids + why |
| `must_read_docs[]` | always includes Code_Review_Plan |
| `skip_advice` | set for shallow packs |
| `files_touched[]` | filtered paths |

## Caps (token protection)

- Max changed symbols: 30 (`--max-changed`)
- Max related symbols: 40 (`--max-related`)
- Max excerpt lines per symbol: 80 (`--max-excerpt-lines`)

Changed symbols are ranked (path + ownership/safety tokens) before the changed
cap is applied, so Spotlight keeps high-risk hunks instead of first-seen order.
New private helpers (`_foo`) are omitted from api narrative / Must-check rows;
public signature edits still become contract checks (capped, with overflow
folded into one row).

A changed symbol longer than the excerpt cap is not clipped to its head, which
would hide the diff inside a large function. The excerpt always keeps every
changed line (even if that alone exceeds the budget), then a short head and
widening context, and marks omitted ranges as `# ... skipped lines A-B ...`.
Such symbols carry `clipped: true`; open the file when a finding depends on
skipped context.

Edits under `tools/ai_review/` are classified as tooling structure, not as
product `frame_ring` / `motor` / `ros` risk — the pack builder’s own pattern
tables would otherwise contaminate the lens list.

## Smoke test

```text
python tools/ai_review/smoke_test.py
```

## Later upgrade (light twin)

Keep this pack schema stable. Replace `slice.py` internals with a cached
import/call index without changing the Cursor skill contract.
