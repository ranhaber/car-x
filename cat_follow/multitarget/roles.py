"""Sticky role assignment for at most two cat tracks."""

from __future__ import annotations

from typing import Dict

PRIMARY_CAT = "PRIMARY_CAT"
SECONDARY_CAT = "SECONDARY_CAT"
CAT_CLASS_ID = 17


class RoleManager:
    """Keep roles stable and promote secondary when primary is lost."""

    def __init__(self, max_cats: int = 2) -> None:
        self.max_cats = int(max_cats)
        self._roles: Dict[int, str] = {}

    def reset(self) -> None:
        self._roles.clear()

    def update(self, tracks) -> Dict[int, str]:
        present_cats = {
            track_id
            for track_id, state in tracks.items()
            if state.class_id == CAT_CLASS_ID
        }
        for track_id in list(self._roles):
            if track_id not in present_cats:
                del self._roles[track_id]

        if PRIMARY_CAT not in self._roles.values():
            secondary = next(
                (
                    track_id
                    for track_id, role in self._roles.items()
                    if role == SECONDARY_CAT
                ),
                None,
            )
            if secondary is not None:
                self._roles[secondary] = PRIMARY_CAT

        candidates = sorted(
            (
                (track_id, tracks[track_id])
                for track_id in present_cats
                if track_id not in self._roles
            ),
            key=lambda item: item[1].confidence,
            reverse=True,
        )
        for track_id, _state in candidates:
            held = set(self._roles.values())
            if PRIMARY_CAT not in held:
                self._roles[track_id] = PRIMARY_CAT
            elif self.max_cats > 1 and SECONDARY_CAT not in held:
                self._roles[track_id] = SECONDARY_CAT
            else:
                break
        return dict(self._roles)

    def assignments(self) -> Dict[str, int]:
        return {role: track_id for track_id, role in self._roles.items()}

    def role_of(self, track_id: int):
        return self._roles.get(track_id)
