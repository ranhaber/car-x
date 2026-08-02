"""Recording segment index, quota enforcement, and crash recovery."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
import shutil
import time
from typing import Callable, List, Optional


def default_recording_dir() -> str:
    """Resolve ``CAT_FOLLOW_RECORDING_DIR`` or a local development default."""

    override = os.getenv("CAT_FOLLOW_RECORDING_DIR")
    if override:
        return override
    return os.path.join("var", "lib", "cat_follow", "recordings")


@dataclass(frozen=True)
class SegmentRecord:
    path: str
    bytes: int
    finalized_at_ms: int
    active: bool = False


class RecordingStore:
    """Manage finalized/active recording segments under a directory."""

    def __init__(
        self,
        directory: str,
        *,
        quota_bytes: Optional[int] = None,
        min_free_bytes: Optional[int] = None,
        disk_usage: Callable[[str], object] = shutil.disk_usage,
    ) -> None:
        self.directory = directory
        self.quota_bytes = quota_bytes
        self.min_free_bytes = min_free_bytes
        self._disk_usage = disk_usage
        self._index_path = os.path.join(directory, "index.json")
        self._active_path: Optional[str] = None
        os.makedirs(directory, exist_ok=True)

    def recover_incomplete(self) -> List[str]:
        """Finalize or mark recoverable ``*.mkv.part`` files from a crash."""

        recovered: List[str] = []
        for name in sorted(os.listdir(self.directory)):
            if not name.endswith(".mkv.part"):
                continue
            part_path = os.path.join(self.directory, name)
            final_path = part_path[: -len(".part")]
            size = os.path.getsize(part_path)
            if size <= 0:
                os.remove(part_path)
                continue
            os.replace(part_path, final_path)
            self._append_index(
                SegmentRecord(
                    path=final_path,
                    bytes=size,
                    finalized_at_ms=int(time.time() * 1000),
                )
            )
            recovered.append(final_path)
        self.enforce_retention()
        return recovered

    def begin_segment(self, *, wall_clock_ms: Optional[int] = None) -> str:
        """Open a new active segment path (``.mkv.part``).

        ``wall_clock_ms`` must be Unix epoch milliseconds: segment names are
        operator-facing timestamps, so a monotonic clock would produce
        1970-era names.
        """

        if not self.space_available_for_new_segment():
            raise OSError("recording storage reserve exhausted")
        stamp = time.strftime("%Y%m%d-%H%M%S")
        if wall_clock_ms is not None:
            stamp = time.strftime(
                "%Y%m%d-%H%M%S", time.localtime(wall_clock_ms / 1000.0)
            )
        base = os.path.join(self.directory, f"{stamp}.mkv")
        part = base + ".part"
        # Avoid collisions within the same second.
        suffix = 1
        while os.path.exists(part) or os.path.exists(base):
            base = os.path.join(self.directory, f"{stamp}-{suffix:03d}.mkv")
            part = base + ".part"
            suffix += 1
        with open(part, "wb"):
            pass
        self._active_path = part
        return part

    def append_bytes(self, path: str, payload: bytes) -> int:
        if self._active_path != path:
            raise ValueError("append target is not the active segment")
        with open(path, "ab") as handle:
            handle.write(payload)
            handle.flush()
        return os.path.getsize(path)

    def finalize_segment(self, path: str) -> str:
        if not path.endswith(".mkv.part"):
            raise ValueError("finalize expects a .mkv.part path")
        if not os.path.exists(path):
            raise FileNotFoundError(path)
        final_path = path[: -len(".part")]
        size = os.path.getsize(path)
        os.replace(path, final_path)
        if self._active_path == path:
            self._active_path = None
        self._append_index(
            SegmentRecord(
                path=final_path,
                bytes=size,
                finalized_at_ms=int(time.time() * 1000),
            )
        )
        self.enforce_retention()
        return final_path

    def active_path(self) -> Optional[str]:
        return self._active_path

    def active_bytes(self) -> int:
        """Bytes already written to the in-progress ``.mkv.part``, if any."""
        path = self._active_path
        if path is None or not os.path.exists(path):
            return 0
        return os.path.getsize(path)

    def list_finalized(self) -> List[SegmentRecord]:
        return [s for s in self._load_index() if not s.active]

    def total_finalized_bytes(self) -> int:
        return sum(s.bytes for s in self.list_finalized())

    def space_available(self) -> bool:
        """True when free disk space is at or above the configured reserve."""
        if self.min_free_bytes is None:
            return True
        usage = self._disk_usage(self.directory)
        if hasattr(usage, "free"):
            free = int(usage.free)
        else:
            free = int(usage[2])
        return free >= int(self.min_free_bytes)

    def space_available_for_new_segment(self) -> bool:
        return self.space_available()

    def over_quota(self) -> bool:
        """True when finalized plus in-progress bytes exceed the quota."""
        if self.quota_bytes is None:
            return False
        total = self.total_finalized_bytes() + self.active_bytes()
        return total > int(self.quota_bytes)

    def active_segment_over_quota(self) -> bool:
        """True when the in-progress segment alone breaches the quota.

        Retention only deletes finalized segments, so such a segment must be
        rotated before it can be reclaimed.
        """
        if self.quota_bytes is None:
            return False
        return self.active_bytes() >= int(self.quota_bytes)

    def enforce_retention(self) -> List[str]:
        """Delete oldest finalized segments until under quota.

        The in-progress ``.mkv.part`` counts toward the quota (so a long
        segment cannot grow past it unnoticed) but is never deleted.
        """

        if self.quota_bytes is None:
            return []
        deleted: List[str] = []
        segments = sorted(
            self.list_finalized(), key=lambda s: s.finalized_at_ms
        )
        total = sum(s.bytes for s in segments) + self.active_bytes()
        while segments and total > int(self.quota_bytes):
            victim = segments.pop(0)
            if victim.path == self._active_path:
                continue
            if os.path.exists(victim.path):
                os.remove(victim.path)
            deleted.append(victim.path)
            total -= victim.bytes
        if deleted:
            remaining = [
                s
                for s in self._load_index()
                if s.path not in deleted and not s.active
            ]
            self._write_index(remaining)
        return deleted

    def _append_index(self, record: SegmentRecord) -> None:
        records = self._load_index()
        records.append(record)
        self._write_index(records)

    def _load_index(self) -> List[SegmentRecord]:
        if not os.path.exists(self._index_path):
            return []
        with open(self._index_path, "r", encoding="utf-8") as handle:
            raw = json.load(handle)
        out: List[SegmentRecord] = []
        for item in raw.get("segments", []):
            out.append(
                SegmentRecord(
                    path=str(item["path"]),
                    bytes=int(item["bytes"]),
                    finalized_at_ms=int(item["finalized_at_ms"]),
                    active=bool(item.get("active", False)),
                )
            )
        return out

    def _write_index(self, records: List[SegmentRecord]) -> None:
        payload = {
            "segments": [
                {
                    "path": r.path,
                    "bytes": r.bytes,
                    "finalized_at_ms": r.finalized_at_ms,
                    "active": r.active,
                }
                for r in records
            ]
        }
        tmp = self._index_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, self._index_path)
