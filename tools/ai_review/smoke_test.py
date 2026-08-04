"""Smoke checks for ai_review classify / lenses / caps (no git required)."""

from __future__ import annotations

import ast
import sys
import tempfile
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[1]
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from ai_review import MAX_CHANGED_SYMBOLS, MAX_EXCERPT_LINES, MAX_RELATED_SYMBOLS
from ai_review.change_summary import (
    build_change_summary,
    is_contract_relevant_api,
    _signature_delta,
)
from ai_review.checklist import build_must_check
from ai_review.classify import classify_diffs
from ai_review.diff import FileDiff, filter_review_paths
from ai_review.lenses import must_read_docs, select_lenses
from ai_review.pack import build_pack_object, write_pack
from ai_review.slice import _imported_local_modules
from ai_review.spotlight import rank_changed_symbols
from ai_review.symbols import SymbolSpan, extract_changed_symbols


def _hunk(body_lines: list[str], new_start: int = 10) -> str:
    """Build a minimal unified hunk from +/-/space lines (without prefixes)."""
    # body_lines items already include leading + / - / space.
    return "\n".join([f"@@ -{new_start},0 +{new_start},{len(body_lines)} @@", *body_lines])


def test_comment_only_shallow() -> None:
    fd = FileDiff(
        path="cat_follow/util/helpers.py",
        status="modified",
        added_lines={12},
        hunk_texts=[
            _hunk(
                [
                    " # existing",
                    "+# clarify helper behavior",
                    " def helper():",
                ]
            )
        ],
    )
    risk = classify_diffs([fd])
    assert risk.level == "shallow", risk
    lenses = select_lenses(risk)
    assert lenses[0].id == "Skim"
    print("OK comment_only_shallow")


def test_lock_change_deep_c1() -> None:
    fd = FileDiff(
        path="cat_follow/runtime/shared_state.py",
        status="modified",
        added_lines={40, 41},
        hunk_texts=[
            _hunk(
                [
                    " def update():",
                    "+    with self._lock:",
                    "+        self.value += 1",
                    "     return self.value",
                ],
                new_start=40,
            )
        ],
    )
    risk = classify_diffs([fd])
    assert risk.level == "deep", risk
    assert "concurrency" in risk.tags or "shared_state" in risk.tags, risk.tags
    lens_ids = {ln.id for ln in select_lenses(risk)}
    assert "C1" in lens_ids, lens_ids
    print("OK lock_change_deep_c1")


def test_frame_ring_adds_audit_doc() -> None:
    fd = FileDiff(
        path="cat_follow/memory/pool.py",
        status="modified",
        added_lines={20},
        hunk_texts=[
            _hunk(
                [
                    " def acquire():",
                    "+    lease = self._ring.lease(generation=self._gen)",
                    "     return buf",
                ],
                new_start=20,
            )
        ],
    )
    risk = classify_diffs([fd])
    assert risk.level == "deep", risk
    docs = must_read_docs(risk)
    assert any("Frame_Ring" in d for d in docs), docs
    lens_ids = {ln.id for ln in select_lenses(risk)}
    assert "C2" in lens_ids, lens_ids
    print("OK frame_ring_adds_audit_doc")


def test_ros_bridge_lens() -> None:
    fd = FileDiff(
        path="cat_follow/navigation/ros_bridge.py",
        status="modified",
        added_lines={55},
        hunk_texts=[
            _hunk(
                [
                    " def setup(self):",
                    '+    self.create_subscription(Pose, "/odom", self._cb, 10)',
                    "     return",
                ],
                new_start=55,
            )
        ],
    )
    risk = classify_diffs([fd])
    assert risk.level == "deep", risk
    assert "ros" in risk.tags, risk.tags
    lens_ids = {ln.id for ln in select_lenses(risk)}
    assert "ROS2" in lens_ids, lens_ids
    print("OK ros_bridge_lens")


def test_meta_tooling_does_not_inflate_product_tags() -> None:
    """Pattern tables in tools/ai_review must not look like product frame_ring edits."""
    fd = FileDiff(
        path="tools/ai_review/change_summary.py",
        status="modified",
        added_lines={80, 81, 82},
        hunk_texts=[
            _hunk(
                [
                    '+    (re.compile(r"\\blease\\b|FrameConsumer|refcount", re.I),',
                    '+     "Frame / lease admission or ownership logic changed."),',
                    '+    (re.compile(r"set_speed|motor|servo|QoS|create_subscription"), "x"),',
                ],
                new_start=80,
            )
        ],
    )
    risk = classify_diffs([fd])
    assert risk.level == "deep", risk
    product = {"frame_ring", "motor", "ros", "fsm", "http_mutation", "rknn"}
    assert not (product & set(risk.tags)), risk.tags
    assert risk.tags == ["code_structure"] or set(risk.tags) <= {
        "code_structure",
        "format",
    }, risk.tags
    print("OK meta_tooling_does_not_inflate_product_tags")


def test_symbol_excerpt_cap() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        repo = Path(tmp)
        path = repo / "cat_follow" / "sample.py"
        path.parent.mkdir(parents=True)
        body = ["def big():"] + [f"    x{i} = {i}" for i in range(200)]
        path.write_text("\n".join(body) + "\n", encoding="utf-8")
        spans = extract_changed_symbols(repo, "cat_follow/sample.py", {150}, max_excerpt_lines=80)
        assert spans, "expected symbol"
        excerpt = spans[0].source
        excerpt_lines = excerpt.splitlines()
        # Sparse change: kept lines stay near the budget (+ gap markers).
        assert len(excerpt_lines) <= 84, len(excerpt_lines)
        assert spans[0].clipped
        assert "skipped lines" in excerpt
        # The changed line must survive clipping even though it is far from the
        # symbol head; head-only clipping used to hide it entirely.
        assert "x148 = 148" in excerpt, excerpt
        print("OK symbol_excerpt_cap")


def test_clipped_excerpt_keeps_every_changed_line() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        repo = Path(tmp)
        path = repo / "cat_follow" / "wide.py"
        path.parent.mkdir(parents=True)
        body = ["def wide():"] + [f"    y{i} = {i}" for i in range(300)]
        path.write_text("\n".join(body) + "\n", encoding="utf-8")
        changed = {30, 120, 210, 295}
        spans = extract_changed_symbols(
            repo, "cat_follow/wide.py", changed, max_excerpt_lines=80
        )
        excerpt = spans[0].source
        for line_no in changed:
            expected = f"y{line_no - 2} = {line_no - 2}"
            assert expected in excerpt, (line_no, excerpt)
        print("OK clipped_excerpt_keeps_every_changed_line")


def test_dense_changed_lines_exceed_budget_but_all_kept() -> None:
    """When changed lines alone exceed max_excerpt_lines, none are dropped."""
    with tempfile.TemporaryDirectory() as tmp:
        repo = Path(tmp)
        path = repo / "cat_follow" / "dense.py"
        path.parent.mkdir(parents=True)
        body = ["def dense():"] + [f"    z{i} = {i}" for i in range(200)]
        path.write_text("\n".join(body) + "\n", encoding="utf-8")
        # 100 changed lines inside the function (> max_excerpt_lines=80).
        changed = set(range(10, 110))
        spans = extract_changed_symbols(
            repo, "cat_follow/dense.py", changed, max_excerpt_lines=80
        )
        excerpt = spans[0].source
        assert spans[0].clipped
        for line_no in changed:
            expected = f"z{line_no - 2} = {line_no - 2}"
            assert expected in excerpt, (line_no, expected)
        print("OK dense_changed_lines_exceed_budget_but_all_kept")


def test_default_filter_keeps_tests_and_scripts() -> None:
    files = [
        FileDiff(path="cat_follow/memory/shared_state.py", status="modified"),
        FileDiff(path="tests/test_shared_state.py", status="modified"),
        FileDiff(path="scripts/board_soak_concurrent_h264.py", status="modified"),
        FileDiff(path="picar-x/picarx/preset_actions.py", status="modified"),
    ]
    kept = {f.path for f in filter_review_paths(files)}
    assert "tests/test_shared_state.py" in kept, kept
    assert "scripts/board_soak_concurrent_h264.py" in kept, kept
    assert "picar-x/picarx/preset_actions.py" not in kept, kept
    print("OK default_filter_keeps_tests_and_scripts")


def test_default_filter_keeps_cursor_skills() -> None:
    files = [
        FileDiff(path="cat_follow/memory/shared_state.py", status="modified"),
        FileDiff(path=".cursor/skills/ai-code-review/SKILL.md", status="modified"),
        FileDiff(path="picar-x/picarx/preset_actions.py", status="modified"),
    ]
    kept = {f.path for f in filter_review_paths(files)}
    assert ".cursor/skills/ai-code-review/SKILL.md" in kept, kept
    assert "cat_follow/memory/shared_state.py" in kept, kept
    assert "picar-x/picarx/preset_actions.py" not in kept, kept
    print("OK default_filter_keeps_cursor_skills")


def test_pack_write_and_caps_constants() -> None:
    assert MAX_CHANGED_SYMBOLS == 30
    assert MAX_RELATED_SYMBOLS == 40
    assert MAX_EXCERPT_LINES == 80
    with tempfile.TemporaryDirectory() as tmp:
        repo = Path(tmp)
        out = repo / "pack"
        from ai_review.classify import RiskAssessment

        risk = RiskAssessment(level="shallow", tags=["docs"], reasons=["fixture"])
        pack = build_pack_object(
            repo=repo,
            mode="fixture",
            base="HEAD",
            risk=risk,
            changed=[],
            related=[],
            lenses=select_lenses(risk),
            docs=must_read_docs(risk),
            files_touched=["cat_follow/x.py"],
        )
        json_path, md_path = write_pack(out, pack)
        assert json_path.is_file()
        assert md_path.is_file()
        text = md_path.read_text(encoding="utf-8")
        assert "SHALLOW" in text
        assert "Skim only" in text
        assert "## Change summary" in text
        assert "## Must-check" in text
        print("OK pack_write_and_caps_constants")


def test_include_paths_scope_pack() -> None:
    files = [
        FileDiff(path="cat_follow/control/fsm.py", status="modified"),
        FileDiff(path="cat_follow/runtime/control_loop.py", status="modified"),
        FileDiff(path="cat_follow/vision/backends.py", status="modified"),
    ]
    scoped = filter_review_paths(
        files, ["cat_follow/control", "cat_follow/runtime/control_loop.py"]
    )
    assert [f.path for f in scoped] == [
        "cat_follow/control/fsm.py",
        "cat_follow/runtime/control_loop.py",
    ], scoped
    assert len(filter_review_paths(files)) == 3
    print("OK include_paths_scope_pack")


def test_truncation_is_reported_in_markdown() -> None:
    from ai_review.classify import RiskAssessment

    risk = RiskAssessment(level="deep", tags=["concurrency"], reasons=["fixture"])
    with tempfile.TemporaryDirectory() as tmp:
        repo = Path(tmp)
        pack = build_pack_object(
            repo=repo,
            mode="fixture",
            base="HEAD",
            risk=risk,
            changed=[],
            related=[],
            lenses=select_lenses(risk),
            docs=must_read_docs(risk),
            files_touched=["cat_follow/control/fsm.py"],
            caps={
                "changed_symbols_total": 78,
                "changed_symbols_included": 30,
                "related_symbols_total": 11,
                "related_symbols_included": 11,
                "max_excerpt_lines": 80,
                "excerpt_clipped_symbols": 2,
                "truncated": True,
            },
            include_paths=["cat_follow/control"],
        )
        _, md_path = write_pack(repo / "pack", pack)
        text = md_path.read_text(encoding="utf-8")
        assert "Truncated" in text
        assert "30/78" in text
        assert "Scoped to" in text
        assert "Clipped excerpts" in text
        print("OK truncation_is_reported_in_markdown")


def test_clip_only_does_not_claim_truncation() -> None:
    from ai_review.classify import RiskAssessment

    risk = RiskAssessment(level="deep", tags=["code_structure"], reasons=["fixture"])
    with tempfile.TemporaryDirectory() as tmp:
        repo = Path(tmp)
        pack = build_pack_object(
            repo=repo,
            mode="fixture",
            base="HEAD",
            risk=risk,
            changed=[],
            related=[],
            lenses=select_lenses(risk),
            docs=must_read_docs(risk),
            files_touched=["tools/ai_review/symbols.py"],
            caps={
                "changed_symbols_total": 13,
                "changed_symbols_included": 13,
                "related_symbols_total": 5,
                "related_symbols_included": 5,
                "max_excerpt_lines": 80,
                "excerpt_clipped_symbols": 4,
                "truncated": False,
            },
        )
        _, md_path = write_pack(repo / "pack", pack)
        text = md_path.read_text(encoding="utf-8")
        assert "Truncated" not in text
        assert "Clipped excerpts" in text
        assert "Re-run with `--paths`" not in text
        print("OK clip_only_does_not_claim_truncation")


def test_change_summary_and_must_check_wired_into_pack() -> None:
    from ai_review.classify import RiskAssessment

    fd = FileDiff(
        path="cat_follow/memory/shared_state.py",
        status="modified",
        added_lines={10},
        hunk_texts=[
            _hunk(
                [
                    "-def acquire_latest_frame(self):",
                    "+def acquire_latest_frame(self, *, consumer: FrameConsumer):",
                ],
                new_start=10,
            )
        ],
    )
    span = SymbolSpan(
        file=fd.path,
        qualname="SharedState.acquire_latest_frame",
        kind="function",
        start_line=10,
        end_line=20,
        source="def acquire_latest_frame(self, *, consumer: FrameConsumer):\n    pass\n",
    )
    files = [fd]
    changed = [span]
    risk = RiskAssessment(
        level="deep",
        tags=["frame_ring", "code_structure"],
        reasons=["fixture"],
    )
    summary = build_change_summary(files, changed)
    checks = build_must_check(risk, files, changed, summary)
    assert any(b.kind == "api" for b in summary), summary
    assert any("consumer" in b.text for b in summary if b.kind == "api"), summary
    assert checks, checks

    with tempfile.TemporaryDirectory() as tmp:
        repo = Path(tmp)
        pack = build_pack_object(
            repo=repo,
            mode="fixture",
            base="HEAD",
            risk=risk,
            changed=changed,
            related=[],
            lenses=select_lenses(risk),
            docs=must_read_docs(risk),
            files_touched=[fd.path],
            change_summary=summary,
            must_check=checks,
        )
        _, md_path = write_pack(repo / "pack", pack)
        text = md_path.read_text(encoding="utf-8")
        assert "## Change summary" in text
        assert "**api**:" in text
        assert "## Must-check" in text
        assert "## Review intent" in text
        assert "## Spotlight" in text
        assert "## Agent handoff" in text
        assert pack.meta["schema_version"] == 2
        print("OK change_summary_and_must_check_wired_into_pack")


def test_private_new_helpers_skipped_from_contract_checks() -> None:
    from ai_review.classify import RiskAssessment

    fd = FileDiff(
        path="tools/ai_review/change_summary.py",
        status="modified",
        added_lines={40, 50},
        hunk_texts=[
            _hunk(
                [
                    "+def _dedupe(bullets: list[ChangeBullet]) -> list[ChangeBullet]:",
                    "+    return bullets",
                    "+def build_change_summary(files, changed):",
                    "+    return []",
                ],
                new_start=40,
            )
        ],
    )
    private = SymbolSpan(
        file=fd.path,
        qualname="_dedupe",
        kind="function",
        start_line=40,
        end_line=42,
        source="def _dedupe(bullets: list[ChangeBullet]) -> list[ChangeBullet]:\n    return bullets\n",
    )
    public = SymbolSpan(
        file=fd.path,
        qualname="build_change_summary",
        kind="function",
        start_line=50,
        end_line=52,
        source="def build_change_summary(files, changed):\n    return []\n",
    )
    summary = build_change_summary([fd], [private, public])
    api = [b for b in summary if b.kind == "api"]
    assert not any("_dedupe" in b.text for b in api), api
    assert any("build_change_summary" in b.text for b in api), api
    assert all(is_contract_relevant_api(b) for b in api if "build_change_summary" in b.text)
    assert not any(is_contract_relevant_api(b) for b in api if "_dedupe" in b.text)

    risk = RiskAssessment(level="deep", tags=["code_structure"], reasons=["fixture"])
    checks = build_must_check(risk, [fd], [private, public], summary)
    contract_qs = [c.question for c in checks if c.area == "Contracts"]
    assert not any("_dedupe" in q for q in contract_qs), contract_qs
    assert any("build_change_summary" in q for q in contract_qs), contract_qs
    print("OK private_new_helpers_skipped_from_contract_checks")


def test_spotlight_ranks_product_admission_ahead_of_meta_helper() -> None:
    from ai_review.classify import RiskAssessment

    helper = SymbolSpan(
        file="tools/ai_review/change_summary.py",
        qualname="_dedupe",
        kind="function",
        start_line=1,
        end_line=5,
        source="def _dedupe(bullets):\n    return bullets\n",
    )
    admit = SymbolSpan(
        file="cat_follow/memory/shared_state.py",
        qualname="SharedState._admit_acquire_locked",
        kind="function",
        start_line=10,
        end_line=40,
        source=(
            "def _admit_acquire_locked(self, consumer, latest_idx):\n"
            "    if self._slot_refcounts[latest_idx] > 0:\n"
            "        return True\n"
        ),
    )
    risk = RiskAssessment(level="deep", tags=["frame_ring"], reasons=["fixture"])
    ranked = rank_changed_symbols([helper, admit], [], risk)
    assert ranked[0].qualname == "SharedState._admit_acquire_locked", ranked
    print("OK spotlight_ranks_product_admission_ahead_of_meta_helper")


def test_multiline_signature_delta() -> None:
    fd = FileDiff(
        path="cat_follow/memory/shared_state.py",
        status="modified",
        hunk_texts=[
            _hunk(
                [
                    "-def acquire_latest_frame(self) -> Optional[FrameLease]:",
                    "+def acquire_latest_frame(",
                    "+    self, *, consumer: FrameConsumer",
                    "+) -> Optional[FrameLease]:",
                ],
                new_start=600,
            )
        ],
    )
    text = _signature_delta(fd, "acquire_latest_frame")
    assert text is not None, text
    assert "consumer" in text, text
    assert "keyword-only" in text or "adds" in text, text
    print("OK multiline_signature_delta")


def test_imported_local_modules_resolves_ai_review() -> None:
    source = "from .change_summary import ChangeBullet\nfrom ai_review.checklist import CheckItem\n"
    tree = ast.parse(source)
    imports = _imported_local_modules(tree, "tools/ai_review/pack.py")
    assert "tools/ai_review/change_summary.py" in imports, imports
    assert "tools/ai_review/checklist.py" in imports, imports
    print("OK imported_local_modules_resolves_ai_review")


def test_wiring_bullets_from_consumer_calls() -> None:
    fd = FileDiff(
        path="cat_follow/threads/detector.py",
        status="modified",
        added_lines={100},
        hunk_texts=[
            _hunk(
                [
                    "+tick_lease = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)",
                ],
                new_start=100,
            )
        ],
    )
    span = SymbolSpan(
        file=fd.path,
        qualname="run_detector_loop",
        kind="function",
        start_line=90,
        end_line=120,
        source=(
            "def run_detector_loop():\n"
            "    tick_lease = shared.acquire_latest_frame(consumer=FrameConsumer.DETECTOR)\n"
        ),
    )
    summary = build_change_summary([fd], [span])
    wiring = [b for b in summary if b.kind == "wiring"]
    assert wiring, summary
    assert "DETECTOR" in wiring[0].text, wiring[0].text
    print("OK wiring_bullets_from_consumer_calls")


def main() -> int:
    test_comment_only_shallow()
    test_lock_change_deep_c1()
    test_frame_ring_adds_audit_doc()
    test_ros_bridge_lens()
    test_meta_tooling_does_not_inflate_product_tags()
    test_symbol_excerpt_cap()
    test_clipped_excerpt_keeps_every_changed_line()
    test_dense_changed_lines_exceed_budget_but_all_kept()
    test_default_filter_keeps_tests_and_scripts()
    test_default_filter_keeps_cursor_skills()
    test_pack_write_and_caps_constants()
    test_include_paths_scope_pack()
    test_truncation_is_reported_in_markdown()
    test_clip_only_does_not_claim_truncation()
    test_change_summary_and_must_check_wired_into_pack()
    test_private_new_helpers_skipped_from_contract_checks()
    test_spotlight_ranks_product_admission_ahead_of_meta_helper()
    test_multiline_signature_delta()
    test_imported_local_modules_resolves_ai_review()
    test_wiring_bullets_from_consumer_calls()
    print("All smoke tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
