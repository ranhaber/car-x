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

Large working trees hit the symbol caps, so review them in focused passes with
`--paths`:

```text
python tools/ai_review/build_pack.py --uncommitted --paths cat_follow/control cat_follow/runtime/control_loop.py
python tools/ai_review/build_pack.py --uncommitted --paths cat_follow/perception cat_follow/memory cat_follow/threads
```

The pack reports `meta.caps.truncated` (and the CLI prints a warning) whenever
changed symbols were dropped by a cap.

Outputs:

- `.cursor/review_pack/review_pack.json`
- `.cursor/review_pack/review_pack.md`

## Pack schema (`schema_version: 1`)

| Field | Meaning |
|---|---|
| `meta` | repo root, mode, git base, timestamp, `include_paths`, `caps` |
| `risk` | `shallow` \| `deep`, `tags[]`, `reasons[]` |
| `changed_symbols[]` | file, qualname, kind, line_range, excerpt |
| `related_symbols[]` | same + `relation` (`calls` / `called_by` / `same_module` / `keyword_hit`) |
| `lenses[]` | Code_Review_Plan lens ids + why |
| `must_read_docs[]` | always includes Code_Review_Plan |
| `skip_advice` | set for shallow packs |
| `files_touched[]` | filtered paths |

## Caps (token protection)

- Max changed symbols: 30
- Max related symbols: 40
- Max excerpt lines per symbol: 80

## Smoke test

```text
python tools/ai_review/smoke_test.py
```

## Later upgrade (light twin)

Keep this pack schema stable. Replace `slice.py` internals with a cached
import/call index without changing the Cursor skill contract.
