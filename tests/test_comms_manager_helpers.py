"""Shared helpers for comms manager protocol tests."""

import tempfile

from cat_follow.comms.comms_manager import CommsManager
from cat_follow.comms.messages import (
    CommandMessage,
    MissionEventMessage,
    TrackingCar,
    TrackingCat,
    TrackingMessage,
)
from cat_follow.control.decision_engine import DecisionEngine
from cat_follow.control.fsm import FSM
from cat_follow.control.types import (
    CommandName,
    CommandState,
    DecisionInput,
    FSMSnapshot,
    HomeState,
    MissionEventName,
    NavigationState,
    OverheadState,
    RangeBackend,
    RangeState,
    SystemState,
    VisionState,
)
from cat_follow.home.store import HomeStore
from cat_follow.runtime.shared_state import SharedState, now_monotonic_ms

DEFAULT_TARGET_ID = "cat-17"
DEFAULT_PERIMETER_ID = "yard-v3"


def make_manager(*, sensors_healthy=True, command_id_cache_size=100, home="default"):
    ss = SharedState()
    if sensors_healthy:
        now_ms = now_monotonic_ms()
        ss.update_range(
            RangeState(
                received_ms=now_ms,
                fresh=True,
                distance_cm=100.0,
                confidence=1.0,
            )
        )
        ss.update_lidar_range(
            RangeState(
                received_ms=now_ms,
                fresh=True,
                backend=RangeBackend.LIDAR_C1,
                distance_cm=100.0,
                confidence=1.0,
            )
        )
    if home == "default":
        ss.update_home(durable_home())
    elif home is not None:
        ss.update_home(home)
    fsm = FSM()
    engine = DecisionEngine(fsm)
    received = []
    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".json")
    tmp.close()
    home_store = HomeStore(tmp.name)
    manager = CommsManager(
        shared_state=ss,
        ack_sink=received.append,
        command_id_cache_size=command_id_cache_size,
        home_store=home_store,
    )
    manager.bind_runtime(decision_engine=engine, fsm=fsm)
    return manager, ss, received, fsm, engine


def durable_home(
    *,
    x=0.0,
    y=0.0,
    home_version=1,
    frame_id="yard",
):
    return HomeState(
        set=True,
        valid=True,
        x=x,
        y=y,
        x_m=x / 100.0,
        y_m=y / 100.0,
        frame_id=frame_id,
        home_version=home_version,
    )


def tracking_message(
    sequence=1,
    *,
    car_conf=1.0,
    cat_conf=1.0,
    target_id=DEFAULT_TARGET_ID,
    perimeter_id=DEFAULT_PERIMETER_ID,
):
    return TrackingMessage(
        sequence=sequence,
        timestamp_ms=sequence * 100,
        perimeter_id=perimeter_id,
        selected_target_id=target_id,
        car=TrackingCar(
            x=10.0,
            y=20.0,
            heading=0.0,
            heading_valid=False,
            confidence=car_conf,
        ),
        cat=TrackingCat(
            x=30.0,
            y=40.0,
            confidence=cat_conf,
            target_id=target_id,
            inside_perimeter=True,
        ),
    )


def command_message(name, *, sequence=2001, command_id=None, params=None):
    return CommandMessage(
        sequence=sequence,
        timestamp_ms=sequence * 10,
        command_id=command_id or f"cmd-{sequence}",
        command=name,
        params=params or {},
    )


def start_chase_command(**kwargs):
    params = dict(kwargs.get("params") or {})
    params.setdefault("target_id", DEFAULT_TARGET_ID)
    kwargs["params"] = params
    return command_message(CommandName.START_CHASE, **kwargs)


def drive_into_brake_reverse(engine, *, now_ms=1000):
    """Tick ``engine`` against a blocking obstacle so the FSM enters BRAKE_REVERSE.

    Lets command/mission-event tests reach the state whose saved objective
    gates ``STOP_CHASE`` and ``PRIMARY_CAT_LEFT_PERIMETER`` acceptance.
    """

    blocked_cm = engine.close_obstacle_trigger_cm - 1.0
    return engine.tick(
        DecisionInput(
            now_ms=now_ms,
            overhead=OverheadState(),
            home=HomeState(set=True, valid=True),
            vision=VisionState(),
            range=RangeState(
                received_ms=now_ms,
                fresh=True,
                distance_cm=blocked_cm,
                confidence=1.0,
            ),
            lidar=RangeState(
                received_ms=now_ms,
                fresh=True,
                backend=RangeBackend.LIDAR_C1,
                distance_cm=blocked_cm,
                confidence=1.0,
            ),
            navigation=NavigationState(),
            system=SystemState(),
            fsm=FSMSnapshot(),
            command=CommandState(),
        )
    )


def mission_event_message(**kwargs):
    defaults = {
        "event_id": "evt-1",
        "mission_id": "mission-1",
        "timestamp_ms": 1000,
        "name": MissionEventName.PRIMARY_CAT_LEFT_PERIMETER,
        "target_id": DEFAULT_TARGET_ID,
        "perimeter_id": DEFAULT_PERIMETER_ID,
        "observation_sequence": 10,
    }
    defaults.update(kwargs)
    return MissionEventMessage(**defaults)
