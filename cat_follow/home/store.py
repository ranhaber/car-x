"""Atomic durable home record store.

``SET_HOME`` must commit to disk before SharedState is updated and the ACK
is emitted.  Corrupt or mismatched records fail closed at startup.
"""

from __future__ import annotations

from dataclasses import replace
import hashlib
import json
import os
import tempfile
import time
from typing import Any, Optional

from cat_follow.control.types import HomeState
from cat_follow.runtime.shared_state import now_monotonic_ms


class HomePersistError(RuntimeError):
    """Raised when a durable home commit cannot be completed."""


def _sha256_payload(payload: dict[str, Any]) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode(
        "utf-8"
    )
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


class HomeStore:
    """Load and atomically persist versioned home records."""

    def __init__(
        self,
        path: str,
        *,
        map_id: str = "",
        calibration_version: int = 0,
    ) -> None:
        self._path = path
        self._map_id = map_id
        self._calibration_version = int(calibration_version)
        self._current: Optional[HomeState] = None

    @property
    def path(self) -> str:
        return self._path

    @property
    def current(self) -> Optional[HomeState]:
        return self._current

    def load(self) -> Optional[HomeState]:
        """Load and verify the durable home file, or return None if absent."""

        if not os.path.exists(self._path):
            self._current = None
            return None
        with open(self._path, "r", encoding="utf-8") as handle:
            raw = json.load(handle)
        home = self._from_dict(raw, verify=True)
        self._current = home
        return home

    def commit(
        self,
        *,
        x: float,
        y: float,
        frame_id: str = "yard",
        yaw_rad: float = 0.0,
        x_m: Optional[float] = None,
        y_m: Optional[float] = None,
        source_command_id: Optional[str] = None,
        timestamp_ms: Optional[int] = None,
    ) -> HomeState:
        """Bump version, persist atomically, and return the new HomeState."""

        previous = self._current or HomeState()
        next_version = int(previous.home_version) + 1
        map_x = float(x) / 100.0 if x_m is None else float(x_m)
        map_y = float(y) / 100.0 if y_m is None else float(y_m)
        wall_ms = (
            int(timestamp_ms)
            if timestamp_ms is not None
            else int(time.time() * 1000)
        )
        payload = {
            "home_version": next_version,
            "calibration_version": self._calibration_version,
            "map_id": self._map_id,
            "frame_id": frame_id,
            "x": float(x),
            "y": float(y),
            "x_m": map_x,
            "y_m": map_y,
            "yaw_rad": float(yaw_rad),
            "persisted_at_ms": wall_ms,
            "source_command_id": source_command_id,
        }
        checksum = _sha256_payload(payload)
        payload["checksum"] = checksum
        self._atomic_write(payload)
        home = HomeState(
            timestamp_ms=wall_ms,
            received_ms=now_monotonic_ms(),
            fresh=True,
            authority="HomeStore",
            set=True,
            valid=True,
            x=float(x),
            y=float(y),
            frame_id=frame_id,
            x_m=map_x,
            y_m=map_y,
            yaw_rad=float(yaw_rad),
            home_version=next_version,
            checksum=checksum,
            calibration_version=self._calibration_version,
            map_id=self._map_id,
            persisted_at_ms=wall_ms,
            source_command_id=source_command_id,
        )
        self._current = home
        return home

    def mark_frozen(self, frozen: bool) -> Optional[HomeState]:
        if self._current is None:
            return None
        self._current = replace(self._current, frozen_for_mission=frozen)
        return self._current

    def _atomic_write(self, payload: dict[str, Any]) -> None:
        directory = os.path.dirname(os.path.abspath(self._path)) or "."
        os.makedirs(directory, exist_ok=True)
        fd, tmp_path = tempfile.mkstemp(
            prefix=".home-", suffix=".tmp", dir=directory
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(tmp_path, self._path)
        except Exception as exc:  # noqa: BLE001
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise HomePersistError(str(exc)) from exc

    def _from_dict(self, raw: dict[str, Any], *, verify: bool) -> HomeState:
        if not isinstance(raw, dict):
            raise HomePersistError("home record must be an object")
        required = (
            "home_version",
            "checksum",
            "frame_id",
            "x",
            "y",
            "x_m",
            "y_m",
            "yaw_rad",
        )
        for key in required:
            if key not in raw:
                raise HomePersistError(f"home record missing field: {key}")
        payload = {
            key: raw[key]
            for key in (
                "home_version",
                "calibration_version",
                "map_id",
                "frame_id",
                "x",
                "y",
                "x_m",
                "y_m",
                "yaw_rad",
                "persisted_at_ms",
                "source_command_id",
            )
            if key in raw
        }
        expected = _sha256_payload(payload)
        if verify and str(raw.get("checksum")) != expected:
            raise HomePersistError("home record checksum mismatch")
        if (
            self._map_id
            and str(raw.get("map_id", "")) not in {"", self._map_id}
        ):
            raise HomePersistError("home record map_id mismatch")
        if (
            self._calibration_version
            and int(raw.get("calibration_version", 0))
            not in {0, self._calibration_version}
        ):
            raise HomePersistError("home record calibration_version mismatch")
        return HomeState(
            timestamp_ms=int(raw.get("persisted_at_ms", 0)),
            received_ms=now_monotonic_ms(),
            fresh=True,
            authority="HomeStore",
            set=True,
            valid=True,
            x=float(raw["x"]),
            y=float(raw["y"]),
            frame_id=str(raw["frame_id"]),
            x_m=float(raw["x_m"]),
            y_m=float(raw["y_m"]),
            yaw_rad=float(raw["yaw_rad"]),
            home_version=int(raw["home_version"]),
            checksum=str(raw["checksum"]),
            calibration_version=int(raw.get("calibration_version", 0)),
            map_id=str(raw.get("map_id", "")),
            persisted_at_ms=int(raw.get("persisted_at_ms", 0)),
            source_command_id=(
                str(raw["source_command_id"])
                if raw.get("source_command_id") is not None
                else None
            ),
        )


def default_home_path() -> str:
    """Resolve ``CAT_FOLLOW_HOME_FILE`` or a local development default."""

    override = os.getenv("CAT_FOLLOW_HOME_FILE")
    if override:
        return override
    return os.path.join("var", "lib", "cat_follow", "home.json")
