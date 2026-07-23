"""SORT/ByteTrack-style constant-velocity tracking-by-detection."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from cat_follow.multitarget.geometry import dist

Detection = tuple[float, float, float, float, float, int]


@dataclass(frozen=True)
class TrackState:
    track_id: int
    class_id: int
    centroid: Tuple[float, float]
    predicted_centroid: Tuple[float, float]
    velocity: Tuple[float, float]
    confidence: float
    hits: int
    frames_since_update: int
    bbox: Optional[Tuple[float, float, float, float]] = None


def _centroid(detection: Detection) -> Tuple[float, float]:
    x1, y1, x2, y2 = detection[:4]
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


class PredictiveTracker:
    """Class-aware two-stage associator with constant-velocity coasting."""

    def __init__(
        self,
        max_distance: float = 300.0,
        max_disappeared: int = 25,
        high_conf: float = 0.30,
        low_conf: float = 0.10,
        velocity_alpha: float = 0.5,
    ) -> None:
        self.max_distance = float(max_distance)
        self.max_disappeared = int(max_disappeared)
        self.high_conf = float(high_conf)
        self.low_conf = float(low_conf)
        self.velocity_alpha = float(velocity_alpha)
        self._tracks: Dict[int, dict] = {}
        self._next_id = 1

    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1

    def remove(self, track_id: int) -> None:
        self._tracks.pop(track_id, None)

    def _match(self, track_ids, indexed_detections):
        pairs = []
        for track_id in track_ids:
            track = self._tracks[track_id]
            for detection_index, detection in indexed_detections:
                if detection[5] != track["class_id"]:
                    continue
                distance = dist(track["pred"], _centroid(detection))
                if distance <= self.max_distance:
                    pairs.append((distance, track_id, detection_index))
        pairs.sort(key=lambda item: item[0])
        matches = {}
        used_tracks = set()
        used_detections = set()
        for _distance, track_id, detection_index in pairs:
            if track_id in used_tracks or detection_index in used_detections:
                continue
            matches[track_id] = detection_index
            used_tracks.add(track_id)
            used_detections.add(detection_index)
        return matches, used_detections

    def _register(self, detection: Detection) -> None:
        track_id = self._next_id
        self._next_id += 1
        center = _centroid(detection)
        self._tracks[track_id] = {
            "class_id": detection[5],
            "estimate": center,
            "velocity": (0.0, 0.0),
            "pred": center,
            "bbox": tuple(detection[:4]),
            "confidence": detection[4],
            "hits": 1,
            "misses": 0,
        }

    def _apply_match(self, track_id: int, detection: Detection) -> None:
        track = self._tracks[track_id]
        measurement = _centroid(detection)
        instantaneous = (
            measurement[0] - track["estimate"][0],
            measurement[1] - track["estimate"][1],
        )
        if track["hits"] == 1:
            track["velocity"] = instantaneous
        else:
            alpha = self.velocity_alpha
            track["velocity"] = (
                alpha * instantaneous[0] + (1 - alpha) * track["velocity"][0],
                alpha * instantaneous[1] + (1 - alpha) * track["velocity"][1],
            )
        track["estimate"] = measurement
        track["bbox"] = tuple(detection[:4])
        track["confidence"] = detection[4]
        track["hits"] += 1
        track["misses"] = 0

    def update(self, detections, now=None) -> Dict[int, TrackState]:
        del now  # Kept for source-pipeline API compatibility.
        for track in self._tracks.values():
            x, y = track["estimate"]
            vx, vy = track["velocity"]
            track["pred"] = (x + vx, y + vy)

        filtered = [
            detection
            for detection in detections
            if detection is not None and detection[4] >= self.low_conf
        ]
        high = [
            (index, detection)
            for index, detection in enumerate(filtered)
            if detection[4] >= self.high_conf
        ]
        low = [
            (index, detection)
            for index, detection in enumerate(filtered)
            if detection[4] < self.high_conf
        ]
        track_ids = list(self._tracks)
        high_matches, used_high = self._match(track_ids, high)
        remaining = [track_id for track_id in track_ids if track_id not in high_matches]
        low_matches, used_low = self._match(remaining, low)
        matches = {**high_matches, **low_matches}
        used = used_high | used_low

        for track_id, detection_index in matches.items():
            self._apply_match(track_id, filtered[detection_index])
        for track_id in track_ids:
            if track_id not in matches:
                track = self._tracks[track_id]
                track["estimate"] = track["pred"]
                track["misses"] += 1
        for detection_index, detection in high:
            if detection_index not in used:
                self._register(detection)
        for track_id in list(self._tracks):
            if self._tracks[track_id]["misses"] > self.max_disappeared:
                del self._tracks[track_id]
        return self.snapshot()

    def snapshot(self) -> Dict[int, TrackState]:
        result = {}
        for track_id, track in self._tracks.items():
            x, y = track["estimate"]
            vx, vy = track["velocity"]
            result[track_id] = TrackState(
                track_id=track_id,
                class_id=track["class_id"],
                centroid=(x, y),
                predicted_centroid=(x + vx, y + vy),
                velocity=(vx, vy),
                confidence=track["confidence"],
                hits=track["hits"],
                frames_since_update=track["misses"],
                bbox=track["bbox"],
            )
        return result
