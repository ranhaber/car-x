"""Slice 6 NavigationManager goal lifecycle and completion tests."""

from dataclasses import replace
import threading

from cat_follow.control.types import (
    CarTrackingState,
    CommandName,
    CommandState,
    FsmState,
    HomeState,
    NavigationFailureClass,
    NavigationResultStatus,
    NavigationState,
    MissionState,
    OverheadState,
    SharedSnapshot,
    TrackingObjectState,
)
from cat_follow.navigation.manager import (
    MAX_EXPECTED_REPLACEMENTS,
    NavigationManager,
)
from cat_follow.target_config import TargetRuntimeConfig


class FakeTransport:
    def __init__(self):
        self.submitted = []
        self.canceled = []

    def submit_goal(self, intent):
        action_id = f"ag-{len(self.submitted) + 1}"
        self.submitted.append((action_id, intent))
        return action_id

    def cancel_goal(self, action_goal_id):
        self.canceled.append(action_goal_id)


def _overhead(x_cm=100.0, target_id="cat-17"):
    return OverheadState(
        received_ms=1000,
        fresh=True,
        frame_id="yard",
        selected_target_id=target_id,
        car=CarTrackingState(confidence=1.0),
        cat=TrackingObjectState(
            x=x_cm,
            confidence=1.0,
            target_id=target_id,
        ),
    )


def test_moving_goal_requires_rate_and_displacement():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    snapshot = SharedSnapshot(overhead=_overhead(100.0))

    first = manager.tick(snapshot, FsmState.GETTING_CLOSE, 1000)
    manager.tick(
        replace(snapshot, overhead=_overhead(140.0)),
        FsmState.GETTING_CLOSE,
        1200,
    )
    assert len(transport.submitted) == 1  # under 500 ms rate limit

    second = manager.tick(
        replace(snapshot, overhead=_overhead(120.0)),
        FsmState.GETTING_CLOSE,
        1600,
    )
    assert len(transport.submitted) == 1  # under 25 cm displacement

    refreshed = manager.tick(
        replace(snapshot, overhead=_overhead(140.0)),
        FsmState.GETTING_CLOSE,
        1600,
    )
    assert transport.canceled == ["ag-1"]
    assert len(transport.submitted) == 2
    assert first.goal_intent.goal_intent_id == second.goal_intent.goal_intent_id
    assert refreshed.goal_intent.refresh_count == 1
    assert refreshed.goal_intent.action_goal_id == "ag-2"


def test_expected_replacement_result_is_neutral():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    snapshot = SharedSnapshot(overhead=_overhead(100.0))
    first = manager.tick(snapshot, FsmState.GETTING_CLOSE, 1000)
    manager.tick(
        replace(snapshot, overhead=_overhead(140.0)),
        FsmState.GETTING_CLOSE,
        1600,
    )

    accepted = manager.handle_result(
        goal_intent_id=first.goal_intent.goal_intent_id,
        action_goal_id="ag-1",
        status=NavigationResultStatus.CANCELED,
        completed_at_ms=1700,
    )
    assert accepted is False
    state = manager.tick(
        replace(snapshot, overhead=_overhead(140.0)),
        FsmState.GETTING_CLOSE,
        1700,
    )
    assert state.failures_exhausted is False


def test_late_wrong_correlation_is_ignored():
    manager = NavigationManager(
        TargetRuntimeConfig(), transport=FakeTransport()
    )
    manager.tick(
        SharedSnapshot(overhead=_overhead()),
        FsmState.GETTING_CLOSE,
        1000,
    )
    assert (
        manager.handle_result(
            goal_intent_id="gi-wrong",
            action_goal_id="ag-wrong",
            status=NavigationResultStatus.ABORTED,
            completed_at_ms=1100,
        )
        is False
    )
    assert manager.ignored_late_results == 1


def test_completion_requires_success_pose_and_continuous_dwell():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    command = CommandState(
        last_command=CommandName.GO_TO,
        objective_x_cm=100.0,
        objective_y_cm=0.0,
        objective_frame_id="yard",
    )
    snapshot = SharedSnapshot(command=command)
    state = manager.tick(snapshot, FsmState.GOTO, 1000)
    intent = state.goal_intent
    manager.handle_result(
        goal_intent_id=intent.goal_intent_id,
        action_goal_id=intent.action_goal_id,
        status=NavigationResultStatus.SUCCEEDED,
        completed_at_ms=1100,
    )

    in_pose = replace(
        snapshot,
        navigation=NavigationState(
            pose_x_m=1.0,
            pose_y_m=0.0,
            pose_yaw_rad=0.0,
            pose_received_ms=1200,
        ),
    )
    assert not manager.tick(in_pose, FsmState.GOTO, 1200).completion_qualified
    # Losing tolerance resets dwell.
    out_pose = replace(
        in_pose,
        navigation=replace(in_pose.navigation, pose_x_m=2.0, pose_received_ms=1600),
    )
    assert not manager.tick(out_pose, FsmState.GOTO, 1600).completion_qualified
    in_pose = replace(
        in_pose,
        navigation=replace(in_pose.navigation, pose_received_ms=1700),
    )
    assert not manager.tick(in_pose, FsmState.GOTO, 1700).completion_qualified
    in_pose = replace(
        in_pose,
        navigation=replace(in_pose.navigation, pose_received_ms=2700),
    )
    qualified = manager.tick(in_pose, FsmState.GOTO, 2700)
    assert qualified.completion_qualified
    assert qualified.last_result.pose_qualified
    assert qualified.last_result.dwell_qualified


def test_retries_then_reports_exhaustion():
    transport = FakeTransport()
    manager = NavigationManager(
        TargetRuntimeConfig(), transport=transport, max_failures=2
    )
    snapshot = SharedSnapshot(
        home=HomeState(set=True, x=0.0, y=0.0, frame_id="yard")
    )
    first = manager.tick(snapshot, FsmState.RETURN_HOME, 1000)
    manager.handle_result(
        goal_intent_id=first.goal_intent.goal_intent_id,
        action_goal_id=first.goal_intent.action_goal_id,
        status=NavigationResultStatus.ABORTED,
        failure_class=NavigationFailureClass.PLANNER_FAILURE,
        completed_at_ms=1100,
    )
    retry = manager.tick(snapshot, FsmState.RETURN_HOME, 1200)
    assert retry.failures_exhausted is False
    assert retry.goal_intent.action_goal_id == "ag-2"

    manager.handle_result(
        goal_intent_id=retry.goal_intent.goal_intent_id,
        action_goal_id=retry.goal_intent.action_goal_id,
        status=NavigationResultStatus.ABORTED,
        failure_class=NavigationFailureClass.PLANNER_FAILURE,
        completed_at_ms=1300,
    )
    exhausted = manager.tick(snapshot, FsmState.RETURN_HOME, 1400)
    assert exhausted.failures_exhausted


def test_stationary_state_cancels_immediately():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    manager.tick(
        SharedSnapshot(overhead=_overhead()),
        FsmState.GETTING_CLOSE,
        1000,
    )
    state = manager.tick(SharedSnapshot(), FsmState.FAILSAFE, 1001)
    assert transport.canceled == ["ag-1"]
    assert state.goal_intent is None


def test_search_second_stage_uses_exactly_one_observation_waypoint():
    transport = FakeTransport()
    requested = []

    def choose(goal):
        requested.append(goal)
        return 2.0, 1.0, 0.2, "map"

    manager = NavigationManager(
        TargetRuntimeConfig(),
        transport=transport,
        observation_waypoint_provider=choose,
    )
    snapshot = SharedSnapshot(
        overhead=_overhead(),
        mission=MissionState(search_stage=1),
    )
    first = manager.tick(snapshot, FsmState.SEARCH, 1000)
    second = manager.tick(snapshot, FsmState.SEARCH, 1100)

    assert len(requested) == 1
    assert len(transport.submitted) == 1
    assert (
        first.goal_intent.objective_type.value == "SEARCH_OBSERVATION"
    )
    assert second.goal_intent.goal_intent_id == first.goal_intent.goal_intent_id


def test_search_without_safe_observation_waypoint_stays_stationary():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    snapshot = SharedSnapshot(
        overhead=_overhead(),
        mission=MissionState(search_stage=1),
        navigation=NavigationState(
            received_ms=1000,
            fresh=True,
            path_viable=True,
            speed_limit=0.5,
        ),
    )
    state = manager.tick(snapshot, FsmState.SEARCH, 1000)
    assert state.goal_intent is None
    assert state.path_viable is False


def test_moving_goal_refresh_at_inclusive_rate_and_displacement_thresholds():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    snapshot = SharedSnapshot(overhead=_overhead(100.0))
    manager.tick(snapshot, FsmState.GETTING_CLOSE, 1000)

    refreshed = manager.tick(
        replace(snapshot, overhead=_overhead(125.0)),
        FsmState.GETTING_CLOSE,
        1500,
    )
    assert len(transport.submitted) == 2
    assert refreshed.goal_intent.refresh_count == 1


def test_moving_goal_holds_at_exclusive_sub_thresholds():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    snapshot = SharedSnapshot(overhead=_overhead(100.0))
    manager.tick(snapshot, FsmState.GETTING_CLOSE, 1000)

    manager.tick(
        replace(snapshot, overhead=_overhead(124.9)),
        FsmState.GETTING_CLOSE,
        1500,
    )
    assert len(transport.submitted) == 1

    manager.tick(
        replace(snapshot, overhead=_overhead(125.0)),
        FsmState.GETTING_CLOSE,
        1499,
    )
    assert len(transport.submitted) == 1


def test_safety_cancel_bypasses_moving_goal_rate_hold():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    manager.tick(
        SharedSnapshot(overhead=_overhead(100.0)),
        FsmState.GETTING_CLOSE,
        1000,
    )
    state = manager.tick(SharedSnapshot(), FsmState.IDLE, 1100)
    assert transport.canceled == ["ag-1"]
    assert state.goal_intent is None


def test_succeeded_without_pose_tolerance_never_qualifies():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    command = CommandState(
        last_command=CommandName.GO_TO,
        objective_x_cm=100.0,
        objective_y_cm=0.0,
        objective_frame_id="yard",
    )
    snapshot = SharedSnapshot(command=command)
    state = manager.tick(snapshot, FsmState.GOTO, 1000)
    manager.handle_result(
        goal_intent_id=state.goal_intent.goal_intent_id,
        action_goal_id=state.goal_intent.action_goal_id,
        status=NavigationResultStatus.SUCCEEDED,
        completed_at_ms=1100,
    )
    far_pose = replace(
        snapshot,
        navigation=NavigationState(
            pose_x_m=5.0,
            pose_y_m=0.0,
            pose_yaw_rad=0.0,
            pose_received_ms=1200,
        ),
    )
    assert not manager.tick(far_pose, FsmState.GOTO, 2200).completion_qualified


def test_completion_not_qualified_before_dwell_duration():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    command = CommandState(
        last_command=CommandName.GO_TO,
        objective_x_cm=100.0,
        objective_y_cm=0.0,
        objective_frame_id="yard",
    )
    snapshot = SharedSnapshot(command=command)
    state = manager.tick(snapshot, FsmState.GOTO, 1000)
    manager.handle_result(
        goal_intent_id=state.goal_intent.goal_intent_id,
        action_goal_id=state.goal_intent.action_goal_id,
        status=NavigationResultStatus.SUCCEEDED,
        completed_at_ms=1100,
    )
    in_pose = replace(
        snapshot,
        navigation=NavigationState(
            pose_x_m=1.0,
            pose_y_m=0.0,
            pose_yaw_rad=0.0,
            pose_received_ms=1200,
        ),
    )
    assert not manager.tick(in_pose, FsmState.GOTO, 2199).completion_qualified


def test_chase_state_does_not_submit_moving_goal():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    state = manager.tick(
        SharedSnapshot(overhead=_overhead()),
        FsmState.CHASE,
        1000,
    )
    assert transport.submitted == []
    assert state.goal_intent is None


def test_exhausted_failures_stop_moving_goal_refresh():
    transport = FakeTransport()
    manager = NavigationManager(
        TargetRuntimeConfig(), transport=transport, max_failures=1
    )
    first = manager.tick(
        SharedSnapshot(overhead=_overhead(100.0)),
        FsmState.GETTING_CLOSE,
        1000,
    )
    manager.handle_result(
        goal_intent_id=first.goal_intent.goal_intent_id,
        action_goal_id=first.goal_intent.action_goal_id,
        status=NavigationResultStatus.ABORTED,
        failure_class=NavigationFailureClass.PLANNER_FAILURE,
        completed_at_ms=1100,
    )

    # A moving target keeps producing refresh candidates; the spent retry
    # budget must stop them from reaching Nav2.
    exhausted = manager.tick(
        SharedSnapshot(overhead=_overhead(300.0)),
        FsmState.GETTING_CLOSE,
        2000,
    )
    assert exhausted.failures_exhausted
    assert len(transport.submitted) == 1
    assert transport.canceled == []


def test_new_objective_restores_failure_budget():
    transport = FakeTransport()
    manager = NavigationManager(
        TargetRuntimeConfig(), transport=transport, max_failures=1
    )
    first = manager.tick(
        SharedSnapshot(overhead=_overhead(100.0)),
        FsmState.GETTING_CLOSE,
        1000,
    )
    manager.handle_result(
        goal_intent_id=first.goal_intent.goal_intent_id,
        action_goal_id=first.goal_intent.action_goal_id,
        status=NavigationResultStatus.ABORTED,
        completed_at_ms=1100,
    )
    fresh = manager.tick(
        SharedSnapshot(overhead=_overhead(100.0, target_id="cat-99")),
        FsmState.GETTING_CLOSE,
        1200,
    )
    assert fresh.failures_exhausted is False
    assert len(transport.submitted) == 2


def test_durable_home_at_map_origin_uses_map_pose():
    """A versioned home at (0, 0) m is the map origin, not an unset value."""

    transport = FakeTransport()
    manager = NavigationManager(
        TargetRuntimeConfig(),
        transport=transport,
        # A calibrated yard transform would move the goal if the meter fields
        # were mistaken for "unset".
        yard_to_map=lambda x_cm, y_cm, yaw, frame: (
            x_cm / 100.0 + 5.0,
            y_cm / 100.0 + 5.0,
            yaw,
            "map",
        ),
    )
    home = HomeState(
        set=True,
        valid=True,
        x=250.0,
        y=125.0,
        x_m=0.0,
        y_m=0.0,
        home_version=3,
    )
    state = manager.tick(
        SharedSnapshot(home=home), FsmState.RETURN_HOME, 1000
    )
    assert (state.goal_intent.x_m, state.goal_intent.y_m) == (0.0, 0.0)
    assert state.goal_intent.frame_id == "map"


def test_legacy_home_without_version_uses_yard_transform():
    transport = FakeTransport()
    manager = NavigationManager(
        TargetRuntimeConfig(),
        transport=transport,
        yard_to_map=lambda x_cm, y_cm, yaw, frame: (
            x_cm / 100.0 + 5.0,
            y_cm / 100.0,
            yaw,
            "map",
        ),
    )
    state = manager.tick(
        SharedSnapshot(home=HomeState(set=True, x=250.0, y=0.0)),
        FsmState.RETURN_HOME,
        1000,
    )
    assert state.goal_intent.x_m == 7.5


def test_incomplete_home_freeze_yields_no_return_home_goal():
    """Matches safe_return_possible()'s ``frozen_home_invalid`` verdict."""

    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    mission = MissionState(
        home_version_frozen=7,
        frozen_home_x_m=1.0,
        frozen_home_y_m=2.0,
    )
    state = manager.tick(
        SharedSnapshot(
            home=HomeState(set=True, valid=True, home_version=7),
            mission=mission,
        ),
        FsmState.RETURN_HOME,
        1000,
    )
    assert state.goal_intent is None
    assert transport.submitted == []


def test_transport_io_runs_without_holding_the_manager_lock():
    """A blocking transport must not stall result callbacks (C1)."""

    entered = threading.Event()
    release = threading.Event()
    observed = []

    class BlockingTransport(FakeTransport):
        def submit_goal(self, intent):
            entered.set()
            release.wait(timeout=2.0)
            return super().submit_goal(intent)

    manager = NavigationManager(
        TargetRuntimeConfig(), transport=BlockingTransport()
    )
    worker = threading.Thread(
        target=lambda: manager.tick(
            SharedSnapshot(overhead=_overhead()),
            FsmState.GETTING_CLOSE,
            1000,
        )
    )
    worker.start()
    assert entered.wait(timeout=2.0)
    # Reads and result handling stay responsive while the submit is in flight.
    observed.append(manager.active_intent)
    observed.append(
        manager.handle_result(
            goal_intent_id="gi-000001",
            action_goal_id="ag-late",
            status=NavigationResultStatus.ABORTED,
            completed_at_ms=1100,
        )
    )
    release.set()
    worker.join(timeout=2.0)
    assert not worker.is_alive()
    assert observed[1] is False
    assert manager.active_intent.action_goal_id == "ag-1"


def test_cancel_during_submit_cancels_the_superseded_action():
    entered = threading.Event()
    release = threading.Event()

    class BlockingTransport(FakeTransport):
        def submit_goal(self, intent):
            entered.set()
            release.wait(timeout=2.0)
            return super().submit_goal(intent)

    transport = BlockingTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    worker = threading.Thread(
        target=lambda: manager.tick(
            SharedSnapshot(overhead=_overhead()),
            FsmState.GETTING_CLOSE,
            1000,
        )
    )
    worker.start()
    assert entered.wait(timeout=2.0)
    manager.cancel()
    release.set()
    worker.join(timeout=2.0)

    assert manager.active_intent is None
    assert transport.canceled == ["ag-1"]
    # The stale action's terminal result must stay neutral.
    accepted = manager.handle_result(
        goal_intent_id="gi-000001",
        action_goal_id="ag-1",
        status=NavigationResultStatus.CANCELED,
        completed_at_ms=1200,
    )
    assert accepted is False


def test_expected_replacements_stay_bounded():
    transport = FakeTransport()
    manager = NavigationManager(TargetRuntimeConfig(), transport=transport)
    now = 1000
    x_cm = 100.0
    # Refresh far more times than the cap without ever reporting a result.
    for _ in range(MAX_EXPECTED_REPLACEMENTS + 10):
        now += 600
        x_cm += 50.0
        manager.tick(
            SharedSnapshot(overhead=_overhead(x_cm)),
            FsmState.GETTING_CLOSE,
            now,
        )
    assert len(manager._expected_replacements) <= MAX_EXPECTED_REPLACEMENTS


def test_manager_missing_costmap_fails_closed_when_getter_wired():
    transport = FakeTransport()
    cfg = TargetRuntimeConfig(envelope_provider="costmap_sweep")
    manager = NavigationManager(
        cfg, transport=transport, costmap_getter=lambda: None
    )
    snapshot = SharedSnapshot(
        overhead=_overhead(100.0),
        navigation=NavigationState(
            received_ms=1000,
            fresh=True,
            path_correction=0.2,
            path_viable=True,
            pose_x_m=0.0,
            pose_y_m=0.0,
            pose_yaw_rad=0.0,
        ),
    )
    state = manager.tick(snapshot, FsmState.GETTING_CLOSE, 1000)
    assert state.path_viable is False
    assert state.envelope_source == "none"
    assert state.safe_steering_min == 0.0
    assert state.safe_steering_max == 0.0


def test_manager_stale_costmap_fails_closed():
    from cat_follow.navigation.steering_envelope import OccupancyGridSnapshot

    transport = FakeTransport()
    cfg = TargetRuntimeConfig(
        envelope_provider="costmap_sweep",
        envelope_stale_ttl_ms=100,
    )
    stale = OccupancyGridSnapshot(
        width=10,
        height=10,
        resolution=0.05,
        origin_x=-0.25,
        origin_y=-0.25,
        origin_yaw=0.0,
        data=[0] * 100,
        received_ms=100,
    )
    manager = NavigationManager(
        cfg, transport=transport, costmap_getter=lambda: stale
    )
    snapshot = SharedSnapshot(
        overhead=_overhead(100.0),
        navigation=NavigationState(
            received_ms=1000,
            fresh=True,
            path_correction=0.0,
            path_viable=True,
            pose_x_m=0.0,
            pose_y_m=0.0,
            pose_yaw_rad=0.0,
        ),
    )
    state = manager.tick(snapshot, FsmState.GETTING_CLOSE, 1000)
    assert state.path_viable is False
    assert state.envelope_source == "none"
