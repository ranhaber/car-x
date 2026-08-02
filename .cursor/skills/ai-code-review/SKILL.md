---
name: ai-code-review
description: >-
  Token-optimized code review for this repo. Builds a deterministic review pack
  from the git diff, then reviews only that pack against Code_Review_Plan.md.
  Use when the user asks for a code review, token-optimized review, AI review
  pack, /ai-code-review, PR review, or Bugbot-style review with scoped context.
---

# AI Code Review (token-optimized)

## Authority

Follow [`cat_follow/docs/Code_Review_Plan.md`](../../../cat_follow/docs/Code_Review_Plan.md).
The pack scopes context; it does not replace the plan.

## Steps

1. From the repository root, build the pack:
   - Dirty tree / user asked uncommitted:  
     `python tools/ai_review/build_pack.py --uncommitted --out .cursor/review_pack`
   - Staged only:  
     `python tools/ai_review/build_pack.py --staged --out .cursor/review_pack`
   - Branch review (default when clean):  
     `python tools/ai_review/build_pack.py --out .cursor/review_pack`  
     or `python tools/ai_review/build_pack.py --base origin/main --out .cursor/review_pack`
   - Focused pass over a large tree (add to any of the above):  
     `--paths cat_follow/control cat_follow/runtime/control_loop.py`
2. Read `.cursor/review_pack/review_pack.md` (and `review_pack.json` if needed).
   If `meta.caps.truncated` is true, tell the user the pack is incomplete and
   review in `--paths` passes instead of trusting one pack.
3. Read **only** `must_read_docs` and the symbol excerpts in the pack.
4. Do **not** open unrelated large files or whole subsystems unless a finding
   requires one missing callee and the pack marks incomplete context.
5. If `risk.level` is `shallow`: skim for accidental behavior edits; report or
   say no findings; stop.
6. If `risk.level` is `deep`: apply only the listed lenses from Code_Review_Plan;
   then a short C1–C12 gap sweep on the touched surface.
7. Bugbot may seed findings but does not replace this review.
8. Report findings first, severity-sorted:  
   `Severity | Dimension | Area | Location | Finding`  
   Then list test-coverage gaps. Note uncertainty when the slice is incomplete.
9. Do not fix code unless the user asks.

## Docs-only pass

A markdown diff yields no Python symbols, so the pack has nothing to excerpt and
the classifier reports `shallow` with a single `Skim` lens. Do not stop there
when the docs assert behavior. Run the companion tool instead:

`python tools/ai_review/doc_claims.py`

It builds one in-process index of the tree, resolves every backtick-quoted
claim added by the diff, checks cross-document links, and prints only
anomalies. Never verify claims with one `git grep` per claim; that is orders of
magnitude slower and buys nothing.

Classify before judging absence. The tool reads each document's own
`**Status:**` header: a status containing `not implemented`, `target`,
`pending`, `in progress`, or `ready for implementation` marks the document as
target-state, where a missing symbol is expected rather than a defect. Absence
is only a finding for descriptive documents. Skipping this step turns an
approved target spec into hundreds of false positives.

For target-state documents, the real check is the reverse one and it is manual:
each carries a "current implementation gaps" section, and that list goes stale
as implementation lands. Compare the declared gaps against what now exists. An
implemented component still listed as absent is a defect in the document.

## Pack location

- `.cursor/review_pack/review_pack.md`
- `.cursor/review_pack/review_pack.json`

If the CLI fails, tell the user the error and the command to run; do not fall
back to reading the entire diff tree blindly.
