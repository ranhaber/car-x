"""Smoke checks for ai_review classify / lenses / caps (no git required)."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[1]
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from ai_review import MAX_CHANGED_SYMBOLS, MAX_EXCERPT_LINES, MAX_RELATED_SYMBOLS
from ai_review.classify import classify_diffs
from ai_review.diff import FileDiff, filter_review_paths
from ai_review.lenses import must_read_docs, select_lenses
from ai_review.pack import build_pack_object, write_pack
from ai_review.symbols import extract_changed_symbols


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


def test_symbol_excerpt_cap() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        repo = Path(tmp)
        path = repo / "cat_follow" / "sample.py"
        path.parent.mkdir(parents=True)
        body = ["def big():"] + [f"    x{i} = {i}" for i in range(200)]
        path.write_text("\n".join(body) + "\n", encoding="utf-8")
        spans = extract_changed_symbols(repo, "cat_follow/sample.py", {50}, max_excerpt_lines=80)
        assert spans, "expected symbol"
        excerpt_lines = spans[0].source.splitlines()
        assert len(excerpt_lines) <= 81  # 80 + possible truncation comment
        assert "truncated" in spans[0].source
        print("OK symbol_excerpt_cap")


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
                "truncated": True,
            },
            include_paths=["cat_follow/control"],
        )
        _, md_path = write_pack(repo / "pack", pack)
        text = md_path.read_text(encoding="utf-8")
        assert "Truncated" in text
        assert "30/78" in text
        assert "Scoped to" in text
        print("OK truncation_is_reported_in_markdown")


def main() -> int:
    test_comment_only_shallow()
    test_lock_change_deep_c1()
    test_frame_ring_adds_audit_doc()
    test_ros_bridge_lens()
    test_symbol_excerpt_cap()
    test_pack_write_and_caps_constants()
    test_include_paths_scope_pack()
    test_truncation_is_reported_in_markdown()
    print("All smoke tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
