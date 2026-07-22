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
from cat_follow.control.types import (
    FsmState,
    ReasonCode,
    TelemetryEventType,
    TelemetrySeverity,
)
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
    prototype_detector_handshake: Optional[Any] = None
    ros_nav: bool = False
    ros_bridge_thread: Optional[threading.Thread] = None
    web_ui_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.logger.start()
        for thread in self.prototype_perception_threads:
            thread.start()
        # Startup handshake: block until the detector worker validates its
        # RKNN backend (or reports the dev/CI stub).  A validation failure
        # raises here and aborts startup instead of running the app blind.
        if self.prototype_detector_handshake is not None:
            try:
                self.prototype_detector_handshake.wait_ready()
            except Exception:
                self.stop_event.set()
                if self.prototype_perception_stop_event is not None:
                    self.prototype_perception_stop_event.set()
                raise
        if self.vision_adapter is not None:
            self.vision_adapter.start()
        if self.range_adapter is not None:
            self.range_adapter.start()
        if self.ros_nav:
            self._start_ros_bridge()
        if self.web_ui_thread is not None:
            self.web_ui_thread.start()
        self.control_loop.start()
        if self.udp_receiver is not None:
            self.udp_receiver.start()

    def _start_ros_bridge(self) -> None:
        try:
            from cat_follow.navigation.ros_bridge import spin_in_thread

            self.ros_bridge_thread = spin_in_thread(self.shared_state)
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(
                f"warning: --ros-nav requested but the ROS bridge could not "
                f"start ({exc!r}); navigation constraints will be inactive\n"
            )

    def stop(self, timeout: float = 2.0) -> None:
        if self.ros_nav:
            try:
                from cat_follow.navigation.ros_bridge import request_shutdown

                request_shutdown()
                if self.ros_bridge_thread is not None:
                    self.ros_bridge_thread.join(timeout=timeout)
            except Exception:  # noqa: BLE001
                pass
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
    prototype_detector_handshake: Optional[Any] = None,
    prototype_detector_fatal_hook: Optional[Any] = None,
    ros_nav: bool = False,
    web_ui: bool = False,
    web_ui_port: int = 5000,
    web_ui_shared_state: Optional[object] = None,
    web_ui_picarx: Optional[Any] = None,
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

    Web UI:
    - If ``web_ui`` is True, a Flask monitoring thread is prepared and
      started from :meth:`App.start`. Requires a prototype
      ``web_ui_shared_state`` (frame ring) for streaming.
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

    app_stop_event = stop_event or threading.Event()

    def _enter_failsafe(reason: str) -> None:
        """Synchronously stop motors and latch the FSM into FAILSAFE.

        Shared by the comms emergency-stop path and the perception fatal-error
        escalation.  Latching means only an operator ``clear_failsafe`` leaves
        FAILSAFE, so subsequent control ticks keep emitting a safe stop.
        """
        # Emit CRITICAL telemetry first so the failsafe reason is durable even
        # if the motor/FSM calls below raise (CRITICAL triggers a sync flush).
        try:
            logger.log(
                event_type=TelemetryEventType.FAILSAFE,
                severity=TelemetrySeverity.CRITICAL,
                source="Runtime",
                state=fsm.state,
                data={"event": "failsafe_latched", "reason": reason},
            )
        except Exception:  # noqa: BLE001
            pass
        try:
            motor.emergency_stop(reason=reason)
        except Exception:  # noqa: BLE001
            pass
        try:
            if fsm.state != FsmState.FAILSAFE:
                fsm.force_state(
                    FsmState.FAILSAFE, reason=ReasonCode.FAILSAFE_TRIGGERED
                )
                shared_state.update_fsm(fsm.snapshot())
        except Exception:  # noqa: BLE001
            pass

    # Wire the perception fatal-error escalation to e-stop + FAILSAFE + app
    # teardown.  A detector escalation (failed NPU reload, repeated inference
    # failures) must stop the whole vehicle, not just the perception threads.
    if prototype_detector_fatal_hook is not None:
        def _on_detector_fatal(message: str) -> None:
            _enter_failsafe(f"perception_fatal: {message}")
            app_stop_event.set()
            if prototype_perception_stop_event is not None:
                prototype_perception_stop_event.set()

        prototype_detector_fatal_hook.set_handler(_on_detector_fatal)

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
        on_emergency_stop=lambda: _enter_failsafe("comms_emergency_stop"),
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

    web_ui_thread: Optional[threading.Thread] = None
    if web_ui:
        web_ui_thread = _build_web_ui_thread(
            runtime_shared=shared_state,
            comms_manager=comms_manager,
            memory_shared=web_ui_shared_state or prototype_vision_shared_state,
            picarx=web_ui_picarx,
            port=web_ui_port,
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
        stop_event=app_stop_event,
        udp_receiver=udp_receiver,
        udp_sender=udp_sender,
        vision_adapter=vision_adapter,
        range_adapter=range_adapter,
        prototype_perception_threads=prototype_perception_threads,
        prototype_perception_stop_event=prototype_perception_stop_event,
        prototype_detector_handshake=prototype_detector_handshake,
        ros_nav=ros_nav,
        web_ui_thread=web_ui_thread,
    )


def _build_web_ui_thread(
    *,
    runtime_shared: SharedState,
    comms_manager: CommsManager,
    memory_shared: Optional[object],
    picarx: Optional[Any],
    port: int,
) -> Optional[threading.Thread]:
    """Build a daemon Flask thread for contract-runtime monitoring."""
    if memory_shared is None:
        try:
            from cat_follow.memory.pool import allocate_pool
            from cat_follow.memory.shared_state import (
                SharedState as PrototypeSharedState,
            )

            memory_shared = PrototypeSharedState(allocate_pool())
            sys.stderr.write(
                "warning: --web-ui without prototype perception; "
                "stream frames will stay blank until a camera publishes\n"
            )
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(
                f"warning: --web-ui unavailable ({exc!r}); skipping\n"
            )
            return None

    try:
        from cat_follow.calibration import Calibration
        from cat_follow.web_ui.app import create_app
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write(
            f"warning: --web-ui import failed ({exc!r}); skipping\n"
        )
        return None

    try:
        calibration = Calibration()
    except Exception:  # noqa: BLE001
        calibration = None

    flask_app = create_app(
        shared=memory_shared,
        state_machine=None,
        calibration=calibration,
        picarx=picarx,
        runtime_shared=runtime_shared,
        comms_manager=comms_manager,
    )

    def _run() -> None:
        # threaded=True so MJPEG + status polls don't block each other.
        flask_app.run(
            host="0.0.0.0",
            port=port,
            debug=False,
            use_reloader=False,
            threaded=True,
        )

    return threading.Thread(
        target=_run,
        name="CatFollow-Flask",
        daemon=True,
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
    detector_handshake: Any = None
    detector_fatal_hook: Any = None


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
        from cat_follow.threads.detector import (
            DetectorFatalHook,
            DetectorHandshake,
            run_detector_loop,
        )
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
    detector_handshake = DetectorHandshake()
    detector_fatal_hook = DetectorFatalHook()
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
            kwargs={
                "handshake": detector_handshake,
                "on_fatal": detector_fatal_hook,
            },
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
        detector_handshake=detector_handshake,
        detector_fatal_hook=detector_fatal_hook,
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
        "--ros-nav",
        action="store_true",
        help=(
            "Start the ROS 2 navigation bridge (rclpy) on a background thread, "
            "feeding NavigationState + lidar RangeState from /scan, /odom and "
            "Nav2 /cmd_vel into the DecisionEngine.  Requires ROS 2 Jazzy."
        ),
    )
    parser.add_argument(
        "--web-ui",
        action="store_true",
        help=(
            "Start the Flask monitoring UI on a background thread. Prefer "
            "pairing with --with-prototype-perception so the live stream has "
            "camera frames. The UI is non-authoritative monitoring/config only."
        ),
    )
    parser.add_argument(
        "--web-ui-port",
        type=int,
        default=5000,
        help="TCP port for --web-ui (default 5000)",
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
        "ros_nav": args.ros_nav,
        "web_ui": args.web_ui,
        "web_ui_port": args.web_ui_port,
        "web_ui_picarx": picarx_instance,
    }
    if proto is not None:
        app_kwargs.update(
            prototype_vision_shared_state=proto.shared_state,
            vision_image_width=proto.image_width,
            vision_image_height=proto.image_height,
            range_read_distance=proto.range_read_distance,
            prototype_perception_threads=tuple(proto.threads),
            prototype_perception_stop_event=proto.stop_event,
            prototype_detector_handshake=proto.detector_handshake,
            prototype_detector_fatal_hook=proto.detector_fatal_hook,
            web_ui_shared_state=proto.shared_state,
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
