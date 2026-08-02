"""Map risk tags to Code_Review_Plan lenses and must-read docs."""

from __future__ import annotations

from dataclasses import dataclass

from .classify import RiskAssessment

CODE_REVIEW_PLAN = "cat_follow/docs/Code_Review_Plan.md"
FRAME_RING_AUDIT = "cat_follow/docs/Frame_Ring_Ownership_Audit.md"

# tag -> list of (lens_id, why)
_TAG_LENSES: dict[str, list[tuple[str, str]]] = {
    "concurrency": [
        ("C1", "lock/thread/queue / shared mutable state"),
        ("Messages_IPC", "cross-thread handoff and backpressure"),
    ],
    "shared_state": [
        ("C1", "shared-state ownership"),
        ("C11", "shared-state contracts"),
        ("Messages_IPC", "generation / freshness contracts"),
    ],
    "frame_ring": [
        ("C2", "frame-ring / zero-copy ownership"),
        ("C12", "hot-path allocation / copy cost"),
        ("Pipelines_Perception", "camera → detector handoff"),
    ],
    "rknn": [
        ("C2", "NPU buffer / preprocess ownership"),
        ("C7", "RKNN/NPU hardware ownership"),
        ("C12", "inference CPU / lifecycle"),
        ("Pipelines_Perception", "detector pipeline"),
    ],
    "perception": [
        ("C2", "perception memory path"),
        ("C10", "headless detection must keep working"),
        ("C12", "detector lifecycle / idle CPU"),
        ("Pipelines_Perception", "perception pipeline"),
    ],
    "camera": [
        ("C7", "camera hardware ownership"),
        ("C2", "capture buffer ownership"),
        ("C5", "capture vs consumer rates"),
    ],
    "ros": [
        ("ROS2", "topics / QoS / bridge / launch"),
        ("C11", "ROS message contracts"),
        ("C5", "callback rates / freshness"),
        ("Pipelines_Nav", "nav / odometry / TF"),
    ],
    "motor": [
        ("C4", "fail-closed motion on errors"),
        ("C5", "motor command timing"),
        ("C7", "motor / servo ownership"),
        ("Pipelines_Control", "FSM → actuation"),
    ],
    "fsm": [
        ("C4", "safety precedence / fail-closed"),
        ("Pipelines_Control", "FSM and decision engine"),
        ("C11", "control contracts"),
    ],
    "http_mutation": [
        ("C6", "mutating HTTP auth / secrets"),
        ("C8", "request validation"),
        ("UI_HTTP", "Flask control plane"),
    ],
    "udp": [
        ("C6", "mutating UDP auth"),
        ("C8", "payload validation"),
        ("Messages_IPC", "UDP IPC contracts"),
    ],
    "hardware": [
        ("C7", "hardware exclusive ownership"),
        ("C4", "hardware failure must not leave motion active"),
    ],
    "code_structure": [
        ("C11", "producer/consumer contracts after structural change"),
        ("Tests", "map behavior to tests"),
    ],
}


@dataclass
class Lens:
    id: str
    why: str


def select_lenses(risk: RiskAssessment) -> list[Lens]:
    if risk.level == "shallow":
        return [Lens(id="Skim", why="shallow change — confirm no accidental behavior edits")]

    lenses: list[Lens] = []
    seen: set[str] = set()
    for tag in risk.tags:
        for lens_id, why in _TAG_LENSES.get(tag, []):
            if lens_id in seen:
                continue
            seen.add(lens_id)
            lenses.append(Lens(id=lens_id, why=why))

    if not lenses:
        lenses.append(Lens(id="C11", why="default contract sweep for deep unclassified change"))
        lenses.append(Lens(id="Tests", why="map changed behavior to tests"))
    return lenses


def must_read_docs(risk: RiskAssessment) -> list[str]:
    docs = [CODE_REVIEW_PLAN]
    tags = set(risk.tags)
    if tags & {"frame_ring", "rknn", "perception", "camera"}:
        docs.append(FRAME_RING_AUDIT)
    return docs
