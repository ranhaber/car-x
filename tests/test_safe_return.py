"""Safe-return predicate and mission home freeze tests."""

from cat_follow.control.types import (
    GeofenceState,
    HomeState,
    MissionState,
    NavigationState,
)
from cat_follow.navigation.safe_return import (
    frozen_home_pose,
    home_map_pose_m,
    safe_return_possible,
)
from tests.test_comms_manager_helpers import (
    durable_home,
    make_manager,
    start_chase_command,
    tracking_message,
)
from cat_follow.comms.messages import CommandMessage
from cat_follow.control.types import AckStatus, CommandName, RejectionCause


def test_safe_return_requires_home_and_sensors():
    home = HomeState(set=True, valid=True)
    mission = MissionState()
    ok, reason = safe_return_possible(
        home=home,
        mission=mission,
        range_healthy=True,
        lidar_healthy=True,
    )
    assert ok and reason == "ok"

    ok, reason = safe_return_possible(
        home=HomeState(),
        mission=mission,
        range_healthy=True,
        lidar_healthy=True,
    )
    assert not ok and reason == "home_missing"

    ok, reason = safe_return_possible(
        home=home,
        mission=mission,
        range_healthy=False,
        lidar_healthy=True,
    )
    assert not ok and reason == "sensors_unhealthy"


def test_configured_geofence_blocks_unobservable_return():
    ok, reason = safe_return_possible(
        home=HomeState(set=True, valid=True),
        mission=MissionState(),
        range_healthy=True,
        lidar_healthy=True,
        geofence=GeofenceState(
            configured=True,
            localization_valid_for_containment=False,
        ),
    )
    assert not ok and reason == "geofence_unobservable"


def test_versioned_home_at_map_origin_keeps_its_meter_pose():
    """(0, 0) m in a versioned record is the origin, not an unset value."""

    home = HomeState(
        set=True,
        valid=True,
        x=250.0,
        y=125.0,
        x_m=0.0,
        y_m=0.0,
        home_version=2,
    )
    assert home_map_pose_m(home) == (0.0, 0.0)
    pose = frozen_home_pose(MissionState(), home)
    assert pose is not None
    assert pose[4:] == (0.0, 0.0)


def test_unversioned_home_falls_back_to_centimeter_conversion():
    home = HomeState(set=True, x=250.0, y=125.0)
    assert home_map_pose_m(home) == (2.5, 1.25)


def test_start_chase_freezes_home_version_and_blocks_set_home():
    manager, ss, _, _, _ = make_manager(home=durable_home(home_version=4))
    manager.submit_tracking(tracking_message(sequence=1))
    ack = manager.submit_command(start_chase_command())
    assert ack.status == AckStatus.ACCEPTED
    mission = ss.get_mission()
    assert mission.home_version_frozen == 4
    assert mission.frozen_home_x == 0.0
    pose = frozen_home_pose(mission, ss.get_home())
    assert pose is not None

    rejected = manager.submit_command(
        CommandMessage(
            sequence=3001,
            timestamp_ms=30,
            command_id="cmd-set-during-mission",
            command=CommandName.SET_HOME,
            params={"home": {"x": 9.0, "y": 9.0, "frame_id": "yard"}},
        )
    )
    assert rejected.status == AckStatus.REJECTED
    assert rejected.cause == RejectionCause.INVALID_STATE
