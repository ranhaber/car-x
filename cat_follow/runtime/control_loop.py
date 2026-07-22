"""ControlLoop: 50 Hz heartbeat that wires the contract-driven runtime.

Each tick:

1. Capture a coherent :class:`SharedSnapshot`.
2. Build a :class:`DecisionInput` (which deliberately omits the previous
   decision group).
3. Call :py:meth:`DecisionEngine.tick`.
4. Publish the resulting :class:`FSMSnapshot` back into ``SharedState.fsm``.
5. Publish the resulting :class:`DecisionState` back into
   ``SharedState.decision``.
6. Apply the :class:`DecisionOutput` to :class:`MotorInterface`.
7. Emit ``decision`` and (when applicable) ``control_tick_overrun``
   telemetry.

The loop owns its own thread (``CatFollow-Control``); :py:meth:`tick` is
exposed publicly so unit tests can drive a single deterministic tick
without sleeps.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from cat_follow.control.decision_engine import DecisionEngine
from cat_follow.control.fsm import FSM
from cat_follow.control.types import (
    DecisionInput,
    DecisionOutput,
    DecisionState,
    FsmState,
    ReasonCode,
    TelemetryEventType,
    TelemetrySeverity,
)
from cat_follow.motion.motor_interface import MotorInterface
from cat_follow.runtime.shared_state import SharedState, now_monotonic_ms
from cat_follow.telemetry.async_logger import AsyncLogger


# Defaults from Interface spec section 13.14 / 15.
DEFAULT_TARGET_RATE_HZ = 50.0
DEFAULT_TICK_BUDGET_MS = 20
DEFAULT_CONSECUTIVE_OVERRUN_LIMIT = 3
DEFAULT_CRITICAL_OVERRUN_MS = 100
DEFAULT_MIN_DEGRADED_RATE_HZ = 20.0


class ControlLoop:
    """50 Hz control heartbeat for the contract-driven runtime."""

    def __init__(
        self,
        shared_state: SharedState,
        decision_engine: DecisionEngine,
        fsm: FSM,
        motor_interface: MotorInterface,
        logger: Optional[AsyncLogger] = None,
        target_rate_hz: float = DEFAULT_TARGET_RATE_HZ,
        tick_budget_ms: int = DEFAULT_TICK_BUDGET_MS,
        consecutive_overrun_limit: int = DEFAULT_CONSECUTIVE_OVERRUN_LIMIT,
        critical_overrun_ms: int = DEFAULT_CRITICAL_OVERRUN_MS,
        min_degraded_rate_hz: float = DEFAULT_MIN_DEGRADED_RATE_HZ,
        thread_name: str = "CatFollow-Control",
    ) -> None:
        self._ss = shared_state
        self._engine = decision_engine
        self._fsm = fsm
        self._motor = motor_interface
        self._logger = logger
        self._target_rate_hz = target_rate_hz
        self._tick_budget_ms = tick_budget_ms
        self._consecutive_overrun_limit = consecutive_overrun_limit
        self._critical_overrun_ms = critical_overrun_ms
        self._min_degraded_rate_hz = min_degraded_rate_hz
        self._thread_name = thread_name

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._consecutive_overruns = 0
        self._tick_count = 0

    # ── lifecycle ───────────────────────────────────────────────────

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._consecutive_overruns = 0
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

    @property
    def tick_count(self) -> int:
        return self._tick_count

    @property
    def consecutive_overruns(self) -> int:
        return self._consecutive_overruns

    # ── single tick (test entry point and run-loop step) ────────────

    def tick(self, now_ms: Optional[int] = None) -> DecisionOutput:
        tick_start = now_ms if now_ms is not None else now_monotonic_ms()

        snapshot = self._ss.get_snapshot()
        decision_input = DecisionInput(
            now_ms=tick_start,
            overhead=snapshot.overhead,
            home=snapshot.home,
            vision=snapshot.vision,
            range=snapshot.range,
            lidar=snapshot.lidar,
            navigation=snapshot.navigation,
            system=snapshot.system,
            fsm=snapshot.fsm,
            command=snapshot.command,
        )

        output = self._engine.tick(decision_input)

        # Publish FSM snapshot before motor apply so external observers
        # can see the FSM result that drove the upcoming actuator command.
        fsm_snapshot = self._fsm.snapshot(received_ms=tick_start)
        self._ss.update_fsm(fsm_snapshot)

        decision_state = DecisionState(
            timestamp_ms=int(time.time() * 1000),
            received_ms=tick_start,
            fresh=True,
            authority="DecisionEngine",
            requested_state=output.requested_state,
            speed=output.speed,
            steering=output.steering,
            brake=output.brake,
            reason=output.reason,
            active_constraints=output.active_constraints,
        )
        self._ss.update_decision(decision_state)

        self._motor.apply(output)
        self._log_decision(output)

        elapsed = now_monotonic_ms() - tick_start
        self._track_overrun(elapsed, output)
        self._tick_count += 1
        return output

    # ── internals ───────────────────────────────────────────────────

    def _run(self) -> None:
        period_s = 1.0 / max(self._target_rate_hz, 1e-3)
        while not self._stop.is_set():
            tick_start_ms = now_monotonic_ms()
            try:
                self.tick(tick_start_ms)
            except Exception:
                # A tick exception means we cannot produce a fresh, trusted
                # decision.  Fail closed: latch FAILSAFE and emergency-stop so
                # the next tick cannot re-drive on stale state.
                self._log_thread_exception()
                self._latch_failsafe(reason="control_tick_exception")
            elapsed_ms = now_monotonic_ms() - tick_start_ms
            remaining_s = period_s - (elapsed_ms / 1000.0)
            if remaining_s > 0:
                self._stop.wait(remaining_s)

    def _latch_failsafe(self, *, reason: str) -> None:
        """Latch FAILSAFE and inhibit motors until an operator CLEAR_FAILSAFE.

        Forcing the shared FSM to FAILSAFE makes every subsequent
        ``DecisionEngine.tick`` emit a safe-stop (only ``clear_failsafe``
        leaves FAILSAFE), so the latch holds across ticks instead of allowing
        an immediate re-drive.
        """
        try:
            self._motor.emergency_stop(reason=reason)
        except Exception:
            pass
        try:
            if self._fsm.state != FsmState.FAILSAFE:
                self._fsm.force_state(
                    FsmState.FAILSAFE, reason=ReasonCode.FAILSAFE_TRIGGERED
                )
                self._ss.update_fsm(self._fsm.snapshot())
        except Exception:
            pass

    def _log_decision(self, output: DecisionOutput) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.DECISION,
            severity=TelemetrySeverity.DEBUG,
            source="DecisionEngine",
            state=output.requested_state,
            data={
                "speed": output.speed,
                "steering": output.steering,
                "brake": output.brake,
                "reason": output.reason.value,
                "active_constraints": list(output.active_constraints),
                "rejected_transition": output.rejected_transition,
                "target_x": output.target_x,
                "target_y": output.target_y,
                "target_source": (
                    output.target_source.value
                    if output.target_source is not None
                    else None
                ),
            },
        )

    def _track_overrun(
        self, elapsed_ms: int, output: DecisionOutput
    ) -> None:
        if elapsed_ms <= self._tick_budget_ms:
            self._consecutive_overruns = 0
            return

        self._consecutive_overruns += 1
        critical = elapsed_ms >= self._critical_overrun_ms
        # A single critical overrun, or repeated overruns beyond the configured
        # limit, both indicate the loop can no longer meet its deadline.  Latch
        # FAILSAFE (not just a one-shot e-stop) so motors stay inhibited until
        # an operator CLEAR_FAILSAFE rather than re-driving on the next tick.
        limit_exceeded = (
            self._consecutive_overruns >= self._consecutive_overrun_limit
        )
        if critical or limit_exceeded:
            self._latch_failsafe(
                reason=(
                    "control_tick_overrun_critical"
                    if critical
                    else "control_consecutive_overruns"
                )
            )

        if self._logger is None:
            return
        severity = (
            TelemetrySeverity.CRITICAL
            if (critical or limit_exceeded)
            else TelemetrySeverity.WARNING
        )
        self._logger.log(
            event_type=TelemetryEventType.THREAD_HEALTH,
            severity=severity,
            source="ControlLoop",
            state=output.requested_state,
            data={
                "event": "control_tick_overrun",
                "elapsed_ms": elapsed_ms,
                "tick_budget_ms": self._tick_budget_ms,
                "consecutive_overruns": self._consecutive_overruns,
                "critical": critical,
            },
        )

    def _log_thread_exception(self) -> None:
        if self._logger is None:
            return
        self._logger.log(
            event_type=TelemetryEventType.THREAD_HEALTH,
            severity=TelemetrySeverity.ERROR,
            source="ControlLoop",
            state=None,
            data={"event": "control_tick_exception"},
        )


__all__ = [
    "ControlLoop",
    "DEFAULT_TARGET_RATE_HZ",
    "DEFAULT_TICK_BUDGET_MS",
    "DEFAULT_CONSECUTIVE_OVERRUN_LIMIT",
    "DEFAULT_CRITICAL_OVERRUN_MS",
    "DEFAULT_MIN_DEGRADED_RATE_HZ",
]
