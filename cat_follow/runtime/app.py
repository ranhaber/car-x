"""Standalone entry point for the contract-driven runtime.

Constructs ``SharedState``, ``FSM``, ``DecisionEngine``, ``MotorInterface``
(no-op backend in V2), ``AsyncLogger`` (JSONL file sink), ``ControlLoop``,
and ``CommsManager``.  Wires them together and runs until SIGINT/SIGTERM.

Run with::

    python -m cat_follow.runtime.app
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, List, Optional, Tuple

from cat_follow.comms.comms_manager import CommsManager
from cat_follow.comms.udp_receiver import UdpReceiver
from cat_follow.comms.udp_sender import UdpSender
from cat_follow.control.decision_engine import DecisionEngine
from cat_follow.control.fsm import FSM
from cat_follow.motion.motor_interface import (
    MotorBackend,
    MotorInterface,
    NoOpMotorBackend,
)
from cat_follow.perception.range_adapter import RangeAdapter
from cat_follow.perception.vision_adapter import VisionAdapter
from cat_follow.runtime.control_loop import ControlLoop
from cat_follow.runtime.shared_state import SharedState
from cat_follow.telemetry.async_logger import (
    AsyncLogger,
    JsonlFileSink,
    default_jsonl_path,
)


@dataclass
class App:
    """Bundle of runtime components, mostly for tests and lifecycle clarity."""

    shared_state: SharedState
    fsm: FSM
    decision_engine: DecisionEngine
    motor: MotorInterface
    motor_backend: MotorBackend
    logger: AsyncLogger
    control_loop: ControlLoop
    comms_manager: CommsManager
    stop_event: threading.Event
    udp_receiver: Optional[UdpReceiver] = None
    udp_sender: Optional[UdpSender] = None
    vision_adapter: Optional[VisionAdapter] = None
    range_adapter: Optional[RangeAdapter] = None
    prototype_perception_stop_event: Optional[threading.Event] = None
    prototype_perception_threads: Tuple[threading.Thread, ...] = field(
        default_factory=tuple
    )

    def start(self) -> None:
        self.logger.start()
        for thread in self.prototype_perception_threads:
            thread.start()
        if self.vision_adapter is not None:
            self.vision_adapter.start()
        if self.range_adapter is not None:
            self.range_adapter.start()
        self.control_loop.start()
        if self.udp_receiver is not None:
            self.udp_receiver.start()

    def stop(self, timeout: float = 2.0) -> None:
        if self.udp_receiver is not None:
            self.udp_receiver.stop(timeout=timeout)
        self.control_loop.stop(timeout=timeout)
        if self.range_adapter is not None:
            self.range_adapter.stop(timeout=timeout)
        if self.vision_adapter is not None:
            self.vision_adapter.stop(timeout=timeout)
        if self.prototype_perception_stop_event is not None:
            self.prototype_perception_stop_event.set()
            for thread in self.prototype_perception_threads:
                thread.join(timeout=timeout)
        self.logger.stop(timeout=timeout)
        if self.udp_sender is not None:
            self.udp_sender.close()


def build_app(
    *,
    log_path: Optional[Path] = None,
    stop_event: Optional[threading.Event] = None,
    target_rate_hz: float = 50.0,
    motor_backend: Optional[MotorBackend] = None,
    use_picarx: bool = False,
    udp_listen_host: Optional[str] = None,
    udp_listen_port: Optional[int] = None,
    udp_target_host: Optional[str] = None,
    udp_target_port: Optional[int] = None,
    vision_adapter: Optional[VisionAdapter] = None,
    prototype_vision_shared_state: Optional[object] = None,
    vision_image_width: Optional[int] = None,
    vision_image_height: Optional[int] = None,
    range_adapter: Optional[RangeAdapter] = None,
    range_read_distance: Optional[Callable[[], Optional[float]]] = None,
    prototype_perception_threads: Tuple[threading.Thread, ...] = (),
    prototype_perception_stop_event: Optional[threading.Event] = None,
) -> App:
    """Construct the runtime stack without starting any threads.

    Backend selection:
    - If ``motor_backend`` is provided, use it as-is.
    - Else if ``use_picarx`` is True, instantiate a real ``PiCarXBackend``.
    - Else fall back to ``NoOpMotorBackend`` (the default for tests and
      non-Pi development machines).

    UDP transport:
    - If ``udp_listen_host``/``udp_listen_port`` are set, build a
      ``UdpReceiver``.
    - If ``udp_target_host``/``udp_target_port`` are set, build a
      ``UdpSender`` and use it as the ``CommsManager`` ack sink.
    - Otherwise the comms layer stays in-process only.

    Vision:
    - If ``vision_adapter`` is provided, the runtime uses it as-is.
    - Else if ``prototype_vision_shared_state`` is provided together with
      both ``vision_image_width`` and ``vision_image_height``, build_app
      constructs a :class:`VisionAdapter` wired to the contract
      ``SharedState`` it just created.  This is the common path for
      production wiring.
    - Otherwise ``SharedState.vision`` stays at its default and
      DecisionEngine treats it as "no cat visible".

    Range:
    - If ``range_adapter`` is provided, the runtime uses it as-is.
    - Else if ``range_read_distance`` is provided, build_app constructs a
      :class:`RangeAdapter` wired to the contract ``SharedState``.
    - Otherwise ``SharedState.range`` stays at its default and the
      obstacle-veto rules in DecisionEngine remain inactive.
    """

    shared_state = SharedState()
    fsm = FSM()
    decision_engine = DecisionEngine(fsm)
    if motor_backend is None:
        motor_backend = _make_default_backend(use_picarx=use_picarx)

    sink_path = log_path if log_path is not None else default_jsonl_path("logs")
    logger = AsyncLogger(sink=JsonlFileSink(sink_path))

    motor = MotorInterface(backend=motor_backend, logger=logger)
    control_loop = ControlLoop(
        shared_state=shared_state,
        decision_engine=decision_engine,
        fsm=fsm,
        motor_interface=motor,
        logger=logger,
        target_rate_hz=target_rate_hz,
    )

    udp_sender: Optional[UdpSender] = None
    if udp_target_host is not None and udp_target_port is not None:
        udp_sender = UdpSender(
            target_host=udp_target_host,
            target_port=udp_target_port,
            logger=logger,
        )
        ack_sink = udp_sender
    else:
        ack_sink = lambda ack: None  # noqa: E731 - in-process placeholder

    comms_manager = CommsManager(
        shared_state=shared_state,
        ack_sink=ack_sink,
        logger=logger,
    )

    udp_receiver: Optional[UdpReceiver] = None
    if udp_listen_port is not None:
        udp_receiver = UdpReceiver(
            comms_manager=comms_manager,
            bind_host=udp_listen_host or "0.0.0.0",
            bind_port=udp_listen_port,
            logger=logger,
        )

    if (
        vision_adapter is None
        and prototype_vision_shared_state is not None
        and vision_image_width is not None
        and vision_image_height is not None
    ):
        vision_adapter = VisionAdapter(
            prototype_shared_state=prototype_vision_shared_state,
            contract_shared_state=shared_state,
            image_width=vision_image_width,
            image_height=vision_image_height,
            logger=logger,
        )

    if range_adapter is None and range_read_distance is not None:
        range_adapter = RangeAdapter(
            contract_shared_state=shared_state,
            read_distance=range_read_distance,
            logger=logger,
        )

    return App(
        shared_state=shared_state,
        fsm=fsm,
        decision_engine=decision_engine,
        motor=motor,
        motor_backend=motor_backend,
        logger=logger,
        control_loop=control_loop,
        comms_manager=comms_manager,
        stop_event=stop_event or threading.Event(),
        udp_receiver=udp_receiver,
        udp_sender=udp_sender,
        vision_adapter=vision_adapter,
        range_adapter=range_adapter,
        prototype_perception_threads=prototype_perception_threads,
        prototype_perception_stop_event=prototype_perception_stop_event,
    )


def _try_make_picarx() -> Optional[Any]:
    """Return a ``Picarx()`` instance if available, else ``None``.

    Used by both the motor backend and the prototype-perception bootstrap.
    A single instance is shared between them so we never construct two
    ``Picarx`` objects (which would fight over the I2C/PWM hardware).
    """

    try:
        from picarx import Picarx  # type: ignore[import-not-found]
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write(
            f"warning: picarx import failed ({exc!r})\n"
        )
        return None
    try:
        return Picarx()
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write(
            f"warning: Picarx() construction failed ({exc!r})\n"
        )
        return None


def _make_default_backend(
    *, use_picarx: bool, picarx_instance: Optional[Any] = None
) -> MotorBackend:
    """Return the configured motor backend.

    ``--picarx`` requires the SunFounder ``picarx`` package and the robot
    hardware.  On any other platform (or when the import fails), fall back
    to the no-op backend with a stderr warning so the CLI still runs.
    If a ``picarx_instance`` is supplied (e.g. shared with the prototype
    perception bootstrap), it is used instead of constructing a new one.
    """

    if not use_picarx:
        return NoOpMotorBackend()

    if picarx_instance is None:
        picarx_instance = _try_make_picarx()
    if picarx_instance is None:
        sys.stderr.write(
            "warning: --picarx requested but Picarx unavailable; "
            "falling back to NoOpMotorBackend\n"
        )
        return NoOpMotorBackend()

    from cat_follow.motion.picarx_backend import PiCarXBackend

    return PiCarXBackend(picarx_instance)


@dataclass
class _PrototypePerception:
    """Bundle of prototype perception resources for the runtime app."""

    shared_state: Any
    threads: List[threading.Thread]
    stop_event: threading.Event
    image_width: int
    image_height: int
    range_read_distance: Callable[[], Optional[float]]


def _build_prototype_perception(
    *,
    picarx_instance: Optional[Any] = None,
) -> Optional[_PrototypePerception]:
    """Allocate the prototype memory pool / SharedState / threads.

    Returns ``None`` (with a stderr warning) on platforms where the
    prototype dependencies cannot be imported.  When ``picarx_instance``
    is supplied, it is also injected into ``cat_follow.range_sensor`` so
    ultrasonic reads work on real hardware.
    """

    try:
        from cat_follow import range_sensor as proto_range_sensor
        from cat_follow.calibration import CALIBRATION_IMAGE_SIZE
        from cat_follow.memory.pool import allocate_pool
        from cat_follow.memory.shared_state import (
            SharedState as PrototypeSharedState,
        )
        from cat_follow.threads.camera import run_camera_loop
        from cat_follow.threads.detector import run_detector_loop
        from cat_follow.threads.tracker import run_tracker_loop
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write(
            f"warning: --with-prototype-perception unavailable ({exc!r}); skipping\n"
        )
        return None

    pool = allocate_pool()
    proto_ss = PrototypeSharedState(pool)

    if picarx_instance is not None:
        try:
            proto_range_sensor.set_car(picarx_instance)
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(
                f"warning: range_sensor.set_car failed ({exc!r}); "
                "ultrasonic reads will return None\n"
            )

    stop_event = threading.Event()
    threads = [
        threading.Thread(
            target=run_camera_loop,
            args=(proto_ss, stop_event),
            name="CatFollow-Proto-Camera",
            daemon=True,
        ),
        threading.Thread(
            target=run_tracker_loop,
            args=(proto_ss, stop_event),
            name="CatFollow-Proto-Tracker",
            daemon=True,
        ),
        threading.Thread(
            target=run_detector_loop,
            args=(proto_ss, stop_event),
            name="CatFollow-Proto-Detector",
            daemon=True,
        ),
    ]

    image_width, image_height = CALIBRATION_IMAGE_SIZE

    return _PrototypePerception(
        shared_state=proto_ss,
        threads=threads,
        stop_event=stop_event,
        image_width=image_width,
        image_height=image_height,
        range_read_distance=proto_range_sensor.get_distance_cm,
    )


def _install_signal_handlers(stop_event: threading.Event) -> None:
    def handler(signum, frame):  # noqa: ARG001
        stop_event.set()

    try:
        signal.signal(signal.SIGINT, handler)
    except (ValueError, OSError):
        # SIGINT may not be installable in non-main threads (rare, but keep
        # the failure quiet so unit tests embedding the app don't break).
        pass

    if hasattr(signal, "SIGTERM"):
        try:
            signal.signal(signal.SIGTERM, handler)
        except (ValueError, OSError, AttributeError):
            pass


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(description="Cat Follow contract runtime")
    parser.add_argument(
        "--log-path",
        type=Path,
        default=None,
        help="Override the default JSONL telemetry path",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=50.0,
        help="Target control loop rate in Hz",
    )
    parser.add_argument(
        "--picarx",
        action="store_true",
        help="Drive the real PiCar-X chassis (requires SunFounder picarx package)",
    )
    parser.add_argument(
        "--with-prototype-perception",
        action="store_true",
        help=(
            "Spin up the prototype camera/tracker/detector threads and the "
            "range sensor wiring, and connect them to the runtime via the "
            "VisionAdapter and RangeAdapter.  Intended for live operation "
            "on the PiCar-X hardware."
        ),
    )
    parser.add_argument(
        "--udp-listen-host",
        type=str,
        default=None,
        help="Bind host for the UDP receiver (default 0.0.0.0 when --udp-listen-port is set)",
    )
    parser.add_argument(
        "--udp-listen-port",
        type=int,
        default=None,
        help="Bind port for the UDP receiver (enables UDP ingress)",
    )
    parser.add_argument(
        "--udp-target-host",
        type=str,
        default=None,
        help="Target host for outbound ACKs (enables UDP egress when paired with --udp-target-port)",
    )
    parser.add_argument(
        "--udp-target-port",
        type=int,
        default=None,
        help="Target port for outbound ACKs",
    )
    args = parser.parse_args(argv)

    stop_event = threading.Event()
    _install_signal_handlers(stop_event)

    # Single Picarx instance shared between motor backend and prototype
    # perception (range sensor injection).  Constructed only when needed.
    picarx_instance: Optional[Any] = None
    if args.picarx or args.with_prototype_perception:
        picarx_instance = _try_make_picarx()

    motor_backend = _make_default_backend(
        use_picarx=args.picarx,
        picarx_instance=picarx_instance,
    )

    proto = None
    if args.with_prototype_perception:
        proto = _build_prototype_perception(picarx_instance=picarx_instance)

    app_kwargs: dict = {
        "log_path": args.log_path,
        "stop_event": stop_event,
        "target_rate_hz": args.rate_hz,
        "motor_backend": motor_backend,
        "udp_listen_host": args.udp_listen_host,
        "udp_listen_port": args.udp_listen_port,
        "udp_target_host": args.udp_target_host,
        "udp_target_port": args.udp_target_port,
    }
    if proto is not None:
        app_kwargs.update(
            prototype_vision_shared_state=proto.shared_state,
            vision_image_width=proto.image_width,
            vision_image_height=proto.image_height,
            range_read_distance=proto.range_read_distance,
            prototype_perception_threads=tuple(proto.threads),
            prototype_perception_stop_event=proto.stop_event,
        )

    app = build_app(**app_kwargs)

    app.start()
    try:
        # Wait for SIGINT/SIGTERM.
        stop_event.wait()
    finally:
        app.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
