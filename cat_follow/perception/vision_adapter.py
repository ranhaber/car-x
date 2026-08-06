"""Vision adapter: prototype tracker bbox -> ``SharedState.vision``.

Reads the prototype's tracker bbox (from
``cat_follow.memory.shared_state.SharedState``) and publishes a
contract-conformant :class:`VisionState` into the new
``cat_follow.runtime.shared_state.SharedState.vision`` group.

Coordinate convention
---------------------
Per the Interface and Data Contract Specification, ``x_offset_norm``:
- ``-1.0`` means the cat is at the left edge of the camera frame
- ``0.0`` means the cat is centered horizontally
- ``+1.0`` means the cat is at the right edge

The prototype bbox is ``(x, y, w, h)`` in pixels, where ``(x, y)`` is the
top-left corner.  The cat center is therefore ``(x + w/2, y + h/2)``.

Stability
---------
``cat_visible_stable`` is true once the cat has been visible for at least
``stability_frames`` consecutive ``update()`` calls.  The default of 3
matches Interface spec section 10.4 (``cat_visible_stable >= 3 frames``).

Confidence
----------
The legacy tracker slot's fifth value is treated as confidence as well as
validity (positive means visible), preserving the detector's real score.

Freshness / generation
----------------------
The prototype tracker bbox carries a monotonic *generation* that bumps on
every publish.  The adapter counts stability and advances ``received_ms``
only when the generation changes (a genuinely new tracker observation), so a
frozen/dead tracker ages out via the DecisionEngine's freshness rules instead
of the adapter counting its own poll rate as "stable" frames.

Every age and freshness decision uses the monotonic ``received_ms``;
``timestamp_ms`` is wall-clock and exists only for operator-facing display, so
an NTP step cannot make an observation look fresh or stale.

If ``update()`` raises, the adapter retracts the channel (``fresh=False``,
``cat_visible=False``) rather than leaving the last good observation
published for the DecisionEngine to act on.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Protocol

from cat_follow.control.types import (
    TelemetryEventType,
    TelemetrySeverity,
    VisionState,
)
from cat_follow.runtime.shared_state import SharedState, now_monotonic_ms
from cat_follow.telemetry.async_logger import AsyncLogger


# Default polling rate.  Vision tracking typically runs at 15-30 fps in the
# prototype, so 30 Hz is a safe upper bound that still keeps adapter CPU
# usage trivial.
DEFAULT_POLL_RATE_HZ = 30.0

# Default consecutive-frame requirement before declaring a stable lock.
DEFAULT_STABILITY_FRAMES = 3

# Default age past which a published observation stops counting as fresh.
# Production wiring passes ``TargetRuntimeConfig.local_track_stale_ms`` so the
# reported flag matches the staleness rule DecisionEngine actually applies.
DEFAULT_FRESHNESS_TTL_MS = 350


class _PrototypeVisionStateLike(Protocol):
    """Subset of the prototype ``SharedState`` API used by the adapter."""

    def get_bbox_tracker(self) -> tuple:  # (x, y, w, h, valid)
        ...

    def get_bbox_tracker_with_gen(self) -> tuple:  # (x, y, w, h, valid, gen)
        ...


class VisionAdapter:
    """Bridge from the prototype tracker bbox to contract ``VisionState``."""

    def __init__(
        self,
        prototype_shared_state: _PrototypeVisionStateLike,
        contract_shared_state: SharedState,
        image_width: int,
        image_height: int,
        *,
        stability_frames: int = DEFAULT_STABILITY_FRAMES,
        poll_rate_hz: float = DEFAULT_POLL_RATE_HZ,
        freshness_ttl_ms: int = DEFAULT_FRESHNESS_TTL_MS,
        logger: Optional[AsyncLogger] = None,
        thread_name: str = "CatFollow-VisionAdapter",
        source: str = "VisionAdapter",
    ) -> None:
        if image_width <= 0:
            raise ValueError("image_width must be positive")
        if image_height <= 0:
            raise ValueError("image_height must be positive")
        if stability_frames < 1:
            raise ValueError("stability_frames must be >= 1")
        if poll_rate_hz <= 0:
            raise ValueError("poll_rate_hz must be positive")
        if freshness_ttl_ms <= 0:
            raise ValueError("freshness_ttl_ms must be positive")

        self._proto_ss = prototype_shared_state
        self._contract_ss = contract_shared_state
        self._image_width = float(image_width)
        self._image_height = float(image_height)
        self._stability_frames = stability_frames
        self._poll_rate_hz = poll_rate_hz
        self._freshness_ttl_ms = int(freshness_ttl_ms)
        self._logger = logger
        self._thread_name = thread_name
        self._source = source

        self._consecutive_visible = 0
        self._last_seen_ms = 0
        self._last_bbox_gen = -1
        self._last_received_ms = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ── lifecycle ───────────────────────────────────────────────────

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name=self._thread_name,
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=timeout)

    # ── single-step (for tests and synchronous use) ─────────────────

    def update(self) -> VisionState:
        # Prefer the generation-aware read so we can distinguish a new tracker
        # observation from a repeated poll of an unchanged buffer.
        getter = getattr(self._proto_ss, "get_bbox_tracker_with_gen", None)
        if getter is not None:
            bbox = getter()
            gen = int(bbox[5])
        else:  # pragma: no cover - fallback for minimal test doubles
            bbox = self._proto_ss.get_bbox_tracker()
            gen = self._last_bbox_gen + 1  # treat every poll as a new frame
        x, y, w, h, valid = (
            float(bbox[0]),
            float(bbox[1]),
            float(bbox[2]),
            float(bbox[3]),
            float(bbox[4]),
        )
        cat_visible = valid > 0.0
        new_observation = gen != self._last_bbox_gen
        self._last_bbox_gen = gen

        now = now_monotonic_ms()

        if not cat_visible:
            # No cat: reset stability and let freshness age from the last
            # genuine observation (do not advance received_ms).
            self._consecutive_visible = 0
        elif new_observation:
            # Genuinely new tracker frame with a visible cat: this is what
            # counts toward stability and refreshes the freshness clock.
            self._consecutive_visible += 1
            self._last_seen_ms = now
            self._last_received_ms = now
        # else: cat_visible but stale (no new tracker frame) -> hold counters
        #       and received_ms so DecisionEngine's age check can fail closed.

        x_offset_norm = self._compute_offset(x, w) if cat_visible else 0.0
        x_offset_px = (
            self._compute_offset_px(x, w) if cat_visible else None
        )
        cat_visible_stable = (
            cat_visible and self._consecutive_visible >= self._stability_frames
        )
        received_ms = self._last_received_ms if cat_visible else now
        active_target_id = self._contract_ss.get_mission().active_target_id

        # A coasting tracker keeps reporting the cat without advancing
        # ``received_ms``, so freshness must be judged by observation age, not
        # by visibility. Otherwise status would show "fresh" for a channel the
        # DecisionEngine has already aged out.
        fresh = (
            cat_visible
            and received_ms > 0
            and (now - received_ms) <= self._freshness_ttl_ms
        )

        new_state = VisionState(
            timestamp_ms=int(time.time() * 1000),
            received_ms=received_ms,
            fresh=fresh,
            authority=self._source,
            cat_visible=cat_visible,
            cat_visible_stable=cat_visible_stable,
            x_offset_norm=x_offset_norm,
            x_offset_px=x_offset_px,
            confidence=max(0.0, min(1.0, valid)) if cat_visible else 0.0,
            last_seen_ms=self._last_seen_ms,
            observation_sequence=gen,
            # The current prototype tracker owns one PRIMARY_CAT slot. During
            # migration, bind that slot to the active protocol target; the
            # future association layer will replace this adapter evidence.
            associated_target_id=active_target_id if cat_visible else None,
            association_ambiguous=False,
        )
        self._contract_ss.update_vision(new_state)
        self._log_update(new_state)
        return new_state

    # ── internals ───────────────────────────────────────────────────

    def _run(self) -> None:
        period_s = 1.0 / max(self._poll_rate_hz, 1e-3)
        while not self._stop.is_set():
            try:
                self.update()
            except Exception:
                # Adapter errors must never kill the thread; log via
                # thread_health and continue.
                self._log_thread_exception()
                self._publish_fault_state()
            self._stop.wait(period_s)

    def _publish_fault_state(self) -> None:
        """Retract the vision channel after a failed update.

        Leaving the last good observation published would let the
        DecisionEngine keep steering toward a cat this adapter can no longer
        confirm.
        """
        self._consecutive_visible = 0
        try:
            self._contract_ss.update_vision(
                VisionState(
                    timestamp_ms=int(time.time() * 1000),
                    received_ms=self._last_received_ms,
                    fresh=False,
                    authority=self._source,
                    cat_visible=False,
                    cat_visible_stable=False,
                    x_offset_norm=0.0,
                    confidence=0.0,
                    last_seen_ms=self._last_seen_ms,
                    observation_sequence=self._last_bbox_gen,
                    associated_target_id=None,
                    association_ambiguous=False,
                )
            )
        except Exception:  # noqa: BLE001
            # Shared state itself is failing; the thread must still survive.
            self._log_thread_exception()

    def _compute_offset(self, x: float, w: float) -> float:
        cat_center_x = x + w / 2.0
        half_width = self._image_width / 2.0
        if half_width <= 0:
            return 0.0
        offset = (cat_center_x - half_width) / half_width
        if offset > 1.0:
            return 1.0
        if offset < -1.0:
            return -1.0
        return offset

    def _compute_offset_px(self, x: float, w: float) -> float:
        """Horizontal pixel error from frame center (positive = cat right)."""

        return (x + w / 2.0) - (self._image_width / 2.0)

    def _log_update(self, state: VisionState) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.VISION_UPDATE,
            severity=TelemetrySeverity.DEBUG,
            source=self._source,
            state=None,
            data={
                "cat_visible": state.cat_visible,
                "cat_visible_stable": state.cat_visible_stable,
                "x_offset_norm": state.x_offset_norm,
                "confidence": state.confidence,
            },
        )

    def _log_thread_exception(self) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.THREAD_HEALTH,
            severity=TelemetrySeverity.ERROR,
            source=self._source,
            state=None,
            data={"event": "vision_adapter_exception"},
        )


__all__ = [
    "DEFAULT_POLL_RATE_HZ",
    "DEFAULT_STABILITY_FRAMES",
    "VisionAdapter",
]
