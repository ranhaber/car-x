"""Standalone entry point for the contract-driven runtime.

Constructs ``SharedState``, ``FSM``, ``DecisionEngine``, ``MotorInterface``
(no-op backend in V2), ``AsyncLogger`` (JSONL file sink), ``ControlLoop``,
and ``CommsManager``.  Wires them together and runs until SIGINT/SIGTERM.

Run with::

    python -m cat_follow.runtime.app
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
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
from cat_follow.motion.sequence_executor import MotionSequenceExecutor
from cat_follow.navigation.manager import NavigationManager
from cat_follow.navigation.map_config import resolve_map_file
from cat_follow.navigation.startup_seed import (
    apply_startup_to_shared_state,
    load_startup_artifacts,
)
from cat_follow.home.store import HomeStore, default_home_path
from cat_follow.navigation.geofence import default_geofence_path
from cat_follow.perception.perception_lifecycle_manager import (
    PerceptionLifecycleManager,
)
from cat_follow.perception.recording_store import (
    RecordingStore,
    default_recording_dir,
)
from cat_follow.memory.pool import FRAME_H, FRAME_W
from cat_follow.perception.recording_encoder import create_recording_encoder
from cat_follow.perception.recording_writer import RecordingWriter
from cat_follow.perception_config import load_perception_config
from cat_follow.active_config import active_runtime_config_dict
from cat_follow.safety_config import (
    apply_safety_config_to_runtime,
    resolve_safety_config,
)
from cat_follow.target_config import TargetRuntimeConfig, load_target_runtime_config
from cat_follow.web_ui.control_policy import load_control_auth_policy
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
    navigation_manager: NavigationManager
    stop_event: threading.Event
    perception_lifecycle_manager: Optional[PerceptionLifecycleManager] = None
    recording_writer: Optional[RecordingWriter] = None
    udp_receiver: Optional[UdpReceiver] = None
    udp_sender: Optional[UdpSender] = None
    vision_adapter: Optional[VisionAdapter] = None
    range_adapter: Optional[RangeAdapter] = None
    range_source: Optional[Any] = None
    prototype_perception_stop_event: Optional[threading.Event] = None
    prototype_perception_threads: Tuple[threading.Thread, ...] = field(
        default_factory=tuple
    )
    prototype_detector_handshake: Optional[Any] = None
    prototype_camera_handshake: Optional[Any] = None
    ros_nav: bool = False
    start_bicycle_odom: bool = False
    ros_bridge_thread: Optional[threading.Thread] = None
    ros_bridge_holder: dict = field(default_factory=dict)
    web_ui_thread: Optional[threading.Thread] = None
    apply_safety_config: Optional[Callable[..., Any]] = None
    safety_config: Optional[Any] = None
    target_runtime_config: Optional[TargetRuntimeConfig] = None

    def start(self) -> None:
        self.logger.start()
        try:
            if self.range_source is not None:
                self.range_source.start()
            camera_threads = [
                t for t in self.prototype_perception_threads if "Camera" in t.name
            ]
            other_threads = [
                t for t in self.prototype_perception_threads if "Camera" not in t.name
            ]
            for thread in camera_threads:
                thread.start()
            if self.prototype_camera_handshake is not None:
                self.prototype_camera_handshake.wait_ready()
            elif camera_threads:
                time.sleep(0.2)
            for thread in other_threads:
                thread.start()
            # Startup handshake: block until the detector worker validates its
            # RKNN backend (or reports the dev/CI stub). A validation failure
            # aborts startup instead of running the app blind.
            if self.prototype_detector_handshake is not None:
                self.prototype_detector_handshake.wait_ready()
            if self.vision_adapter is not None:
                self.vision_adapter.start()
            if self.range_adapter is not None:
                self.range_adapter.start()
            if self.recording_writer is not None:
                self.recording_writer.start()
            if self.ros_nav:
                self._start_ros_bridge()
            self.control_loop.start()
            if self.udp_receiver is not None:
                self.udp_receiver.start()
            if self.web_ui_thread is not None:
                try:
                    self.web_ui_thread.start()
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(
                        f"warning: web UI thread could not start ({exc!r}); "
                        "core runtime remains active\n"
                    )
        except BaseException:
            # A partially-started runtime must release every resource it
            # acquired. In particular, ultrasonic/RT-policy failures must not
            # leave telemetry or perception workers orphaned.
            self.stop_event.set()
            if self.prototype_perception_stop_event is not None:
                self.prototype_perception_stop_event.set()
            try:
                self.stop()
            finally:
                raise

    def _start_ros_bridge(self) -> None:
        try:
            from cat_follow.navigation.ros_bridge import spin_in_thread

            self.ros_bridge_thread = spin_in_thread(
                self.shared_state,
                navigation_manager=self.navigation_manager,
                start_bicycle_odom=self.start_bicycle_odom,
                safety_config=self.safety_config,
                bridge_holder=self.ros_bridge_holder,
            )
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
        if self.recording_writer is not None:
            self.recording_writer.stop(timeout=timeout)
        if self.range_adapter is not None:
            self.range_adapter.stop(timeout=timeout)
        if self.range_source is not None:
            self.range_source.stop(timeout=timeout)
        if self.vision_adapter is not None:
            self.vision_adapter.stop(timeout=timeout)
        if self.prototype_perception_stop_event is not None:
            self.prototype_perception_stop_event.set()
            camera_threads = [
                t
                for t in self.prototype_perception_threads
                if "Camera" in t.name
            ]
            other_threads = [
                t
                for t in self.prototype_perception_threads
                if "Camera" not in t.name
            ]
            for thread in other_threads:
                thread.join(timeout=timeout)
            for thread in camera_threads:
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
    range_source: Optional[Any] = None,
    prototype_perception_threads: Tuple[threading.Thread, ...] = (),
    prototype_perception_stop_event: Optional[threading.Event] = None,
    prototype_detector_handshake: Optional[Any] = None,
    prototype_detector_fatal_hook: Optional[Any] = None,
    prototype_camera_handshake: Optional[Any] = None,
    prototype_camera_fatal_hook: Optional[Any] = None,
    ros_nav: bool = False,
    start_bicycle_odom: bool = False,
    web_ui: bool = False,
    web_ui_port: int = 5000,
    web_ui_shared_state: Optional[object] = None,
    web_ui_picarx: Optional[Any] = None,
    calibration: Optional[Any] = None,
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

    if web_ui or udp_listen_port is not None:
        # Missing production credentials disable mutating control planes, but
        # monitoring must never prevent the camera/detector/tracker/control
        # runtime from starting. Web routes return 503 through their auth
        # decorator; UDP ingress is omitted below unless it can fail closed.
        auth_policy = load_control_auth_policy()
        if not auth_policy.is_production_ready:
            sys.stderr.write(
                "warning: control authentication is incomplete; mutating web "
                "control and unauthenticated UDP ingress are disabled\n"
            )
    else:
        auth_policy = None

    if calibration is None:
        try:
            from cat_follow.calibration import Calibration

            calibration = Calibration()
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError("failed to load runtime calibration") from exc

    shared_state = SharedState()
    fsm = FSM()
    sequence_executor = MotionSequenceExecutor()
    # Persisted calibration overrides must be applied before any control,
    # range, or ROS thread starts. Invalid persisted safety values fail startup
    # rather than silently falling back to less conservative defaults.
    safety_config = resolve_safety_config(calibration)
    # Canonical knobs are activated incrementally; telemetry identifies the
    # fields currently wired into DecisionEngine and NavigationManager.
    target_runtime_config = load_target_runtime_config()
    home_path = target_runtime_config.home_file or default_home_path()
    home_store = HomeStore(
        home_path,
        map_id=resolve_map_file(),
        calibration_version=0,
    )
    geofence_path = (
        target_runtime_config.geofence_file or default_geofence_path()
    )
    startup_artifacts = load_startup_artifacts(
        home_store=home_store,
        geofence_path=geofence_path,
        require_home=target_runtime_config.startup_require_home,
        require_geofence=target_runtime_config.startup_require_geofence,
    )
    apply_startup_to_shared_state(shared_state, startup_artifacts)
    geofence_polygon = startup_artifacts.geofence
    ros_bridge_holder: dict = {}
    decision_engine = DecisionEngine(
        fsm,
        sequence_executor=sequence_executor,
        obstacle_too_close_cm=safety_config.obstacle_too_close_cm,
        target_runtime_config=target_runtime_config,
    )
    if motor_backend is None:
        motor_backend = _make_default_backend(
            use_picarx=use_picarx,
            pan_forward_deg=target_runtime_config.look_pan_forward_deg,
        )

    sink_path = log_path if log_path is not None else default_jsonl_path("logs")
    logger = AsyncLogger(sink=JsonlFileSink(sink_path))
    navigation_manager = NavigationManager(
        target_runtime_config, logger=logger
    )
    perception_lifecycle_manager = PerceptionLifecycleManager(
        target_runtime_config, logger=logger
    )
    recording_dir = (
        target_runtime_config.recording_dir or default_recording_dir()
    )
    recording_store = RecordingStore(
        recording_dir,
        quota_bytes=target_runtime_config.recording_quota_bytes,
        min_free_bytes=target_runtime_config.recording_min_free_bytes,
    )
    recovered_segments = recording_store.recover_incomplete()
    recording_writer = RecordingWriter(
        recording_store,
        encoder=create_recording_encoder(
            prototype_vision_shared_state,
            width=FRAME_W,
            height=FRAME_H,
        ),
        segment_duration_ms=int(
            target_runtime_config.recording_segment_sec * 1000
        ),
    )
    logger.log(
        event_type=TelemetryEventType.CONFIGURATION,
        severity=TelemetrySeverity.INFO,
        source="Runtime",
        state=fsm.state,
        data={
            "target_runtime": target_runtime_config.telemetry_dict(),
            "active_runtime": active_runtime_config_dict(
                safety_config, target_runtime_config
            ),
            "startup": {
                "ready": startup_artifacts.ready,
                "home_loaded": startup_artifacts.home is not None,
                "geofence_loaded": startup_artifacts.geofence is not None,
                "degraded_reason": startup_artifacts.degraded_reason,
                "home_path": home_path,
                "geofence_path": geofence_path,
                "recording_dir": recording_dir,
                "recovered_recording_segments": recovered_segments,
            },
        },
    )

    motor = MotorInterface(
        backend=motor_backend,
        logger=logger,
        pan_forward_deg=target_runtime_config.look_pan_forward_deg,
    )
    control_loop = ControlLoop(
        shared_state=shared_state,
        decision_engine=decision_engine,
        fsm=fsm,
        motor_interface=motor,
        logger=logger,
        navigation_manager=navigation_manager,
        geofence_polygon=geofence_polygon,
        perception_lifecycle_manager=perception_lifecycle_manager,
        recording_writer=recording_writer,
        prototype_shared_state=prototype_vision_shared_state,
        target_rate_hz=target_rate_hz,
    )

    app_stop_event = stop_event or threading.Event()
    fatal_lock = threading.Lock()

    def _enter_failsafe(
        reason: str, *, latch_runtime_fatal: bool = False
    ) -> None:
        """Synchronously stop motors and latch the FSM into FAILSAFE.

        Shared by the comms emergency-stop path and the perception fatal-error
        escalation.  Latching means only an operator ``clear_failsafe`` leaves
        FAILSAFE, so subsequent control ticks keep emitting a safe stop.
        """
        if latch_runtime_fatal:
            with fatal_lock:
                first_fatal = shared_state.get_runtime_fatal_reason() is None
                shared_state.set_runtime_fatal_reason(reason)
            if not first_fatal:
                return
        # Queue CRITICAL telemetry first and wake the async writer promptly.
        # Motor/FSM safety does not wait for disk I/O.
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
            _enter_failsafe(
                f"perception_fatal: {message}", latch_runtime_fatal=True
            )
            app_stop_event.set()
            if prototype_perception_stop_event is not None:
                prototype_perception_stop_event.set()

        prototype_detector_fatal_hook.set_handler(_on_detector_fatal)

    if prototype_camera_fatal_hook is not None:
        def _on_camera_fatal(message: str) -> None:
            _enter_failsafe(
                f"camera_fatal: {message}", latch_runtime_fatal=True
            )
            app_stop_event.set()
            if prototype_perception_stop_event is not None:
                prototype_perception_stop_event.set()

        prototype_camera_fatal_hook.set_handler(_on_camera_fatal)

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
        on_start_chase=(
            prototype_vision_shared_state.request_detector_warmup
            if prototype_vision_shared_state is not None
            and hasattr(prototype_vision_shared_state, "request_detector_warmup")
            else None
        ),
        home_store=home_store,
        geofence_polygon=geofence_polygon,
    )
    control_loop.attach_comms_manager(comms_manager)
    comms_manager.bind_runtime(
        control_loop=control_loop,
        decision_engine=decision_engine,
        fsm=fsm,
    )

    udp_receiver: Optional[UdpReceiver] = None
    if (
        udp_listen_port is not None
        and auth_policy is not None
        and (auth_policy.comms_token is not None or auth_policy.allow_unauthenticated)
    ):
        udp_receiver = UdpReceiver(
            comms_manager=comms_manager,
            bind_host=udp_listen_host or "0.0.0.0",
            bind_port=udp_listen_port,
            logger=logger,
            command_token=auth_policy.comms_token if auth_policy is not None else None,
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
            freshness_ttl_ms=target_runtime_config.local_track_stale_ms,
            logger=logger,
        )

    if range_adapter is None and range_read_distance is not None:
        range_adapter = RangeAdapter(
            contract_shared_state=shared_state,
            read_distance=range_read_distance,
            logger=logger,
            obstacle_detected_cm=safety_config.obstacle_detected_cm,
            obstacle_critical_cm=safety_config.obstacle_too_close_cm,
            health_error=(
                (lambda: getattr(range_source, "runtime_error", None))
                if range_source is not None
                else None
            ),
        )

    def apply_runtime_safety(calib=None):
        cfg = resolve_safety_config(calib)
        apply_safety_config_to_runtime(
            cfg,
            decision_engine=decision_engine,
            range_adapter=range_adapter,
            ros_bridge=ros_bridge_holder.get("node"),
        )
        return cfg

    web_ui_thread: Optional[threading.Thread] = None
    if web_ui:
        web_ui_thread = _build_web_ui_thread(
            runtime_shared=shared_state,
            comms_manager=comms_manager,
            memory_shared=web_ui_shared_state or prototype_vision_shared_state,
            picarx=web_ui_picarx,
            port=web_ui_port,
            sequence_executor=sequence_executor,
            apply_safety_config=apply_runtime_safety,
            calibration=calibration,
            target_runtime_config=target_runtime_config,
            perception_lifecycle_manager=perception_lifecycle_manager,
        )

    if prototype_vision_shared_state is not None:
        control_loop.attach_prototype_shared_state(prototype_vision_shared_state)

    return App(
        shared_state=shared_state,
        fsm=fsm,
        decision_engine=decision_engine,
        motor=motor,
        motor_backend=motor_backend,
        logger=logger,
        control_loop=control_loop,
        comms_manager=comms_manager,
        navigation_manager=navigation_manager,
        stop_event=app_stop_event,
        perception_lifecycle_manager=perception_lifecycle_manager,
        recording_writer=recording_writer,
        udp_receiver=udp_receiver,
        udp_sender=udp_sender,
        vision_adapter=vision_adapter,
        range_adapter=range_adapter,
        range_source=range_source,
        prototype_perception_threads=prototype_perception_threads,
        prototype_perception_stop_event=prototype_perception_stop_event,
        prototype_detector_handshake=prototype_detector_handshake,
        prototype_camera_handshake=prototype_camera_handshake,
        ros_nav=ros_nav,
        start_bicycle_odom=start_bicycle_odom,
        ros_bridge_holder=ros_bridge_holder,
        web_ui_thread=web_ui_thread,
        apply_safety_config=apply_runtime_safety,
        safety_config=safety_config,
        target_runtime_config=target_runtime_config,
    )


def _build_web_ui_thread(
    *,
    runtime_shared: SharedState,
    comms_manager: CommsManager,
    memory_shared: Optional[object],
    picarx: Optional[Any],
    port: int,
    sequence_executor: MotionSequenceExecutor,
    apply_safety_config: Optional[Callable[..., Any]] = None,
    calibration: Optional[Any] = None,
    target_runtime_config: Optional[TargetRuntimeConfig] = None,
    perception_lifecycle_manager: Optional[PerceptionLifecycleManager] = None,
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
        from cat_follow.web_ui.app import create_app
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write(
            f"warning: --web-ui import failed ({exc!r}); skipping\n"
        )
        return None

    try:
        flask_app = create_app(
            shared=memory_shared,
            state_machine=None,
            calibration=calibration,
            picarx=picarx,
            runtime_shared=runtime_shared,
            comms_manager=comms_manager,
            sequence_executor=sequence_executor,
            apply_safety_config=apply_safety_config,
            target_runtime_config=target_runtime_config,
            perception_lifecycle_manager=perception_lifecycle_manager,
        )
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write(
            f"warning: --web-ui initialization failed ({exc!r}); skipping\n"
        )
        return None

    def _run() -> None:
        # threaded=True so H.264 WS + status polls don't block each other.
        # WebCodecs requires a secure context; enable TLS with
        # CAT_FOLLOW_WEB_SSL_CERTFILE / CAT_FOLLOW_WEB_SSL_KEYFILE (or
        # CAT_FOLLOW_WEB_SSL_ADHOC=1 with pyOpenSSL).
        run_kwargs: dict[str, Any] = {
            "host": "0.0.0.0",
            "port": port,
            "debug": False,
            "use_reloader": False,
            "threaded": True,
        }
        certfile = os.environ.get("CAT_FOLLOW_WEB_SSL_CERTFILE", "").strip()
        keyfile = os.environ.get("CAT_FOLLOW_WEB_SSL_KEYFILE", "").strip()
        if certfile and keyfile:
            if Path(certfile).is_file() and Path(keyfile).is_file():
                run_kwargs["ssl_context"] = (certfile, keyfile)
            else:
                sys.stderr.write(
                    "warning: web UI TLS certificate/key is missing; "
                    "web UI disabled; core runtime remains active\n"
                )
                return
        elif certfile or keyfile:
            sys.stderr.write(
                "warning: web UI TLS requires both certificate and key; "
                "web UI disabled; core runtime remains active\n"
            )
            return
        elif os.environ.get("CAT_FOLLOW_WEB_SSL_ADHOC", "").strip() in {
            "1",
            "true",
            "yes",
            "on",
        }:
            run_kwargs["ssl_context"] = "adhoc"
        try:
            flask_app.run(**run_kwargs)
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(
                f"warning: web UI server stopped ({exc!r}); "
                "core runtime remains active\n"
            )

    return threading.Thread(
        target=_run,
        name="CatFollow-Flask",
        daemon=True,
    )


def _try_make_picarx(*, enable_ultrasonic: bool = True) -> Optional[Any]:
    """Return a ``Picarx()`` instance if available, else ``None``.

    Used by both the motor backend and the prototype-perception bootstrap.
    A single instance is shared between them so we never construct two
    ``Picarx`` objects (which would fight over the I2C/PWM hardware).

    When the libgpiod edge reader owns D2/D3, pass
    ``enable_ultrasonic=False`` so Picarx does not claim those GPIO lines.
    """

    try:
        from picarx import Picarx  # type: ignore[import-not-found]
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write(
            f"warning: picarx import failed ({exc!r})\n"
        )
        return None
    try:
        return Picarx(enable_ultrasonic=enable_ultrasonic)
    except Exception as exc:  # noqa: BLE001
        sys.stderr.write(
            f"warning: Picarx() construction failed ({exc!r})\n"
        )
        return None


def _make_default_backend(
    *,
    use_picarx: bool,
    picarx_instance: Optional[Any] = None,
    pan_forward_deg: float = 0.0,
) -> MotorBackend:
    """Return the configured motor backend.

    ``--picarx`` requires the SunFounder ``picarx`` package and the robot
    hardware.  On any other platform (or when the import fails), fall back
    to the no-op backend with a stderr warning so the CLI still runs.
    If a ``picarx_instance`` is supplied (e.g. shared with the prototype
    perception bootstrap), it is used instead of constructing a new one.
    """

    if not use_picarx:
        return NoOpMotorBackend(pan_forward_deg=pan_forward_deg)

    if picarx_instance is None:
        picarx_instance = _try_make_picarx()
    if picarx_instance is None:
        sys.stderr.write(
            "warning: --picarx requested but Picarx unavailable; "
            "falling back to NoOpMotorBackend\n"
        )
        return NoOpMotorBackend(pan_forward_deg=pan_forward_deg)

    from cat_follow.motion.picarx_backend import PiCarXBackend

    return PiCarXBackend(picarx_instance, pan_forward_deg=pan_forward_deg)


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
    camera_handshake: Any = None
    camera_fatal_hook: Any = None


def _build_prototype_perception(
    *,
    range_read_distance: Optional[Callable[[], Optional[float]]] = None,
) -> Optional[_PrototypePerception]:
    """Allocate the prototype memory pool / SharedState / threads.

    Returns ``None`` (with a stderr warning) on platforms where the
    prototype dependencies cannot be imported.  Hardware range acquisition is
    supplied separately so this bundle never calls the PiCar-X polling
    ultrasonic implementation.
    """

    try:
        from cat_follow.calibration import CALIBRATION_IMAGE_SIZE
        from cat_follow.memory.pool import allocate_pool
        from cat_follow.memory.shared_state import (
            SharedState as PrototypeSharedState,
        )
        from cat_follow.threads.camera import run_camera_loop
        from cat_follow.threads.camera import CameraFatalHook, CameraHandshake
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
    perception_config = load_perception_config()

    stop_event = threading.Event()
    detector_handshake = DetectorHandshake()
    detector_fatal_hook = DetectorFatalHook()
    camera_handshake = CameraHandshake()
    camera_fatal_hook = CameraFatalHook()
    threads = [
        threading.Thread(
            target=run_camera_loop,
            args=(proto_ss, stop_event),
            kwargs={
                "handshake": camera_handshake,
                "on_fatal": camera_fatal_hook,
            },
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
                "config": perception_config,
                "score_threshold": perception_config.score_threshold,
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
        range_read_distance=range_read_distance or (lambda: None),
        detector_handshake=detector_handshake,
        detector_fatal_hook=detector_fatal_hook,
        camera_handshake=camera_handshake,
        camera_fatal_hook=camera_fatal_hook,
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
        "--odom-source",
        choices=("lidar", "bicycle"),
        default=None,
        help=(
            "Local odometry source for --ros-nav. "
            "'lidar' expects RF2O (or another external scan matcher) to publish "
            "/odom and odom->base_link; 'bicycle' starts cat_follow's internal "
            "OdomPublisher. Overrides CAT_FOLLOW_ODOM_SOURCE (default: lidar)."
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

    use_bicycle_odom = False
    if args.odom_source is not None and not args.ros_nav:
        sys.stderr.write(
            "warning: --odom-source is ignored without --ros-nav\n"
        )
    if args.ros_nav or args.odom_source is not None:
        from cat_follow.navigation.odom_source import (
            resolve_odom_source_or_default,
            uses_bicycle_odom_source,
        )

        odom_source = resolve_odom_source_or_default(override=args.odom_source)
        use_bicycle_odom = uses_bicycle_odom_source(odom_source)

    stop_event = threading.Event()
    _install_signal_handlers(stop_event)

    # Single Picarx instance shared between motor backend and prototype
    # perception (range sensor injection).  Constructed only when needed.
    picarx_instance: Optional[Any] = None
    if args.picarx or args.with_prototype_perception:
        picarx_instance = _try_make_picarx(
            enable_ultrasonic=not args.with_prototype_perception,
        )

    range_source: Optional[Any] = None
    range_read_callback: Optional[Callable[[], Optional[float]]] = None
    if args.with_prototype_perception and picarx_instance is not None:
        from cat_follow import range_sensor
        from cat_follow.perception.edge_ultrasonic import EdgeTimedUltrasonic

        range_source = EdgeTimedUltrasonic.from_env()
        range_sensor.set_reader(range_source.latest_distance_cm)
        range_read_callback = range_sensor.get_distance_cm

    # Load config before constructing the backend so calibrated pan forward
    # matches MotorInterface (CLI injects this backend into build_app).
    cli_target_config = load_target_runtime_config()
    motor_backend = _make_default_backend(
        use_picarx=args.picarx,
        picarx_instance=picarx_instance,
        pan_forward_deg=cli_target_config.look_pan_forward_deg,
    )

    proto = None
    if args.with_prototype_perception:
        proto = _build_prototype_perception(
            range_read_distance=range_read_callback
        )

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
        "start_bicycle_odom": use_bicycle_odom,
        "web_ui": args.web_ui,
        "web_ui_port": args.web_ui_port,
        "web_ui_picarx": picarx_instance,
        "range_source": range_source,
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
            prototype_camera_handshake=proto.camera_handshake,
            prototype_camera_fatal_hook=proto.camera_fatal_hook,
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
