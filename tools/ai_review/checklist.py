"""Deterministic must-check checklist from risk tags and change evidence.

Questions are concrete (memory, threads, timing, FSM, …), not just lens IDs.
The reviewing agent must answer each item before free-form findings.
"""

from __future__ import annotations

from dataclasses import dataclass

from .change_summary import ChangeBullet, is_contract_relevant_api
from .classify import RiskAssessment
from .diff import FileDiff
from .symbols import SymbolSpan

# Cap per-API contract rows so a large public renameset cannot dump 50 items.
_MAX_API_CONTRACT_CHECKS = 6

# tag -> list of (area, question, preferred_lens)
_TAG_CHECKS: dict[str, list[tuple[str, str, str]]] = {
    "concurrency": [
        (
            "Threads",
            "Are locks held only around shared metadata — never across camera, inference, network, or disk I/O?",
            "C1",
        ),
        (
            "Threads",
            "Is every new acquire/pin paired with a release on all paths (finally / context manager)?",
            "C1",
        ),
        (
            "Threads",
            "Can callback/thread re-entrancy or shutdown race leave shared state inconsistent?",
            "C1",
        ),
    ],
    "shared_state": [
        (
            "Memory",
            "Do compound read-modify-write and multi-field snapshots stay under the correct lock?",
            "C1",
        ),
        (
            "Contracts",
            "Do all producers and consumers agree on generation / freshness / ownership semantics?",
            "C11",
        ),
    ],
    "frame_ring": [
        (
            "Memory",
            "Does every new distinct pin leave ≥1 reclaimable slot for the camera writer?",
            "C2",
        ),
        (
            "Memory",
            "Under mission (SEARCH/CHASE), can recording/stream starve the detector of a distinct pin?",
            "C2",
        ),
        (
            "Memory",
            "Does same-slot multi-reader (e.g. detector + stream on latest) avoid counting as an extra slot?",
            "C2",
        ),
        (
            "Timing",
            "Do refused acquires return None (latest-wins) without blocking the camera?",
            "C5",
        ),
        (
            "Memory",
            "Can readers still observe torn pixels, or is generation/refcount handoff intact?",
            "C2",
        ),
    ],
    "rknn": [
        (
            "Hardware",
            "Are RKNN/NPU buffer ownership and model lifecycle still exclusive and fail-loud?",
            "C7",
        ),
        (
            "CPU",
            "Does inference stay off the critical path when idle / model unloaded?",
            "C12",
        ),
    ],
    "perception": [
        (
            "Degradation",
            "Does detection/tracking still run headless when stream or UI is denied/off?",
            "C10",
        ),
        (
            "Pipelines",
            "Is camera → motion → detector → publish still coherent after this change?",
            "Pipelines_Perception",
        ),
    ],
    "camera": [
        (
            "Hardware",
            "Is camera open/close and buffer ownership still exclusive across startup/shutdown?",
            "C7",
        ),
        (
            "Timing",
            "Do capture vs consumer rates still drop safely (writer never waits on readers)?",
            "C5",
        ),
    ],
    "ros": [
        (
            "Contracts",
            "Are topic types, QoS, frame IDs, and TF assumptions still matched across nodes?",
            "ROS2",
        ),
        (
            "Timing",
            "Are callback rates and stale-data policies still safe under load?",
            "C5",
        ),
    ],
    "motor": [
        (
            "Safety",
            "Do unexpected failures still fail closed (no motion left active)?",
            "C4",
        ),
        (
            "Hardware",
            "Are motor/servo units and limits still enforced on the command path?",
            "C7",
        ),
    ],
    "fsm": [
        (
            "FSM",
            "Does safety precedence / emergency stop still override navigation and follow?",
            "C4",
        ),
        (
            "FSM",
            "Are FSM transitions and decision-engine contracts intact for touched states?",
            "Pipelines_Control",
        ),
    ],
    "http_mutation": [
        (
            "Security",
            "Are mutating HTTP routes still authenticated and payload-validated?",
            "C6",
        ),
    ],
    "udp": [
        (
            "Security",
            "Are mutating UDP commands still authenticated and size-validated?",
            "C6",
        ),
    ],
    "hardware": [
        (
            "Hardware",
            "Is exclusive ownership preserved for touched hardware (camera, NPU, motors, lidar)?",
            "C7",
        ),
    ],
    "code_structure": [
        (
            "Contracts",
            "Were all call sites, Protocols, and fakes updated for signature / structural changes?",
            "C11",
        ),
    ],
}


@dataclass
class CheckItem:
    area: str
    question: str
    lens: str
    evidence: list[str]
    tag: str


def build_must_check(
    risk: RiskAssessment,
    files: list[FileDiff],
    changed: list[SymbolSpan],
    summary: list[ChangeBullet],
) -> list[CheckItem]:
    """Build a deduplicated, evidence-cited checklist for the pack."""
    if risk.level == "shallow":
        return [
            CheckItem(
                area="Skim",
                question="Confirm no accidental behavior edits slipped into a shallow change.",
                lens="Skim",
                evidence=[f.path for f in files[:5]],
                tag="docs",
            )
        ]

    items: list[CheckItem] = []
    seen: set[str] = set()

    for tag in risk.tags:
        for area, question, lens in _TAG_CHECKS.get(tag, []):
            key = f"{area}|{question}"
            if key in seen:
                continue
            seen.add(key)
            items.append(
                CheckItem(
                    area=area,
                    question=question,
                    lens=lens,
                    evidence=_evidence_for(tag, area, files, changed, summary),
                    tag=tag,
                )
            )

    # Always demand a test mapping for deep changes.
    test_key = "Tests|Is each new behavior / refusal / priority rule covered by a named test?"
    if test_key not in seen:
        items.append(
            CheckItem(
                area="Tests",
                question="Is each new behavior / refusal / priority rule covered by a named test?",
                lens="Tests",
                evidence=_test_evidence(files, changed, summary),
                tag="code_structure",
            )
        )

    # Public API deltas become explicit contract checks (private helpers skipped).
    api_added = 0
    api_overflow: list[ChangeBullet] = []
    for bullet in summary:
        if bullet.kind == "api" and is_contract_relevant_api(bullet):
            if api_added >= _MAX_API_CONTRACT_CHECKS:
                api_overflow.append(bullet)
                continue
            key = f"Contracts|{bullet.text}"
            if key in seen:
                continue
            seen.add(key)
            items.append(
                CheckItem(
                    area="Contracts",
                    question=f"Call sites and fakes match: {bullet.text}",
                    lens="C11",
                    evidence=bullet.evidence,
                    tag="code_structure",
                )
            )
            api_added += 1
        if bullet.kind == "tests" and "without matching" in bullet.text:
            key = "Tests|Source changed without tests — is that acceptable?"
            if key not in seen:
                seen.add(key)
                items.append(
                    CheckItem(
                        area="Tests",
                        question="Source changed without matching tests — is the gap acceptable, or missing coverage?",
                        lens="Tests",
                        evidence=bullet.evidence,
                        tag="code_structure",
                    )
                )

    if api_overflow:
        evidence = [e for b in api_overflow for e in b.evidence][:8]
        items.append(
            CheckItem(
                area="Contracts",
                question=(
                    f"Call sites and fakes match for {len(api_overflow)} additional "
                    "public API delta(s) listed in Change summary."
                ),
                lens="C11",
                evidence=evidence,
                tag="code_structure",
            )
        )

    return items


def _evidence_for(
    tag: str,
    area: str,
    files: list[FileDiff],
    changed: list[SymbolSpan],
    summary: list[ChangeBullet],
) -> list[str]:
    # Prefer symbols whose path/name matches the tag or area.
    needles = {
        "frame_ring": ("lease", "admit", "refcount", "frame", "FrameConsumer"),
        "shared_state": ("SharedState", "shared_state", "generation"),
        "concurrency": ("Lock", "acquire", "release", "Thread"),
        "perception": ("detector", "perception", "motion"),
        "camera": ("camera", "capture"),
        "rknn": ("rknn", "RKNN", "infer"),
        "fsm": ("fsm", "FSM", "decision"),
        "motor": ("motor", "servo", "speed"),
        "ros": ("create_subscription", "create_publisher", "QoS"),
        "http_mutation": ("route", "request", "auth"),
        "udp": ("udp", "comms"),
    }.get(tag, ())

    evidence: list[str] = []
    for span in changed:
        blob = f"{span.file} {span.qualname} {span.source}"
        if any(n.lower() in blob.lower() for n in needles) or not needles:
            evidence.append(f"{span.file}:{span.qualname}")
        if len(evidence) >= 5:
            break

    if not evidence:
        for bullet in summary:
            if bullet.kind in {"behavior", "wiring", "api"} and bullet.evidence:
                evidence.extend(bullet.evidence[:3])
                break

    if not evidence:
        evidence = [f.path for f in files[:3]]
    return evidence[:6]


def _test_evidence(
    files: list[FileDiff],
    changed: list[SymbolSpan],
    summary: list[ChangeBullet],
) -> list[str]:
    tests = [
        f"{s.file}:{s.qualname}"
        for s in changed
        if s.file.startswith("tests/") and s.qualname.startswith("test_")
    ]
    if tests:
        return tests[:8]
    for bullet in summary:
        if bullet.kind == "tests":
            return bullet.evidence[:6]
    return [f.path for f in files if f.path.startswith("tests/")][:6]
