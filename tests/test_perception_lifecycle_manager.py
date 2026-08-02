"""PerceptionLifecycleManager FSM consumer-policy tests (LIFE-01..13 host)."""

from cat_follow.control.types import FsmState, MissionState
from cat_follow.perception.perception_lifecycle_manager import (
    LifecycleMissionContext,
    PerceptionLifecycleManager,
)
from cat_follow.target_config import TargetRuntimeConfig


def _tick(fsm, *, context=None, mission=None, now_ms=1000, clients=0):
    mgr = PerceptionLifecycleManager(TargetRuntimeConfig())
    for _ in range(clients):
        mgr.register_stream_client()
    return mgr.tick(
        fsm_state=fsm,
        mission=mission or MissionState(),
        context=context or LifecycleMissionContext(),
        now_ms=now_ms,
    )


def test_life_01_home_forces_all_off():
    state = _tick(FsmState.HOME, clients=1)
    assert state.detector.requested is False
    assert state.recording.requested is False
    assert state.stream_forced_off is True
    assert state.stream_active_clients == 0
    assert state.camera_hardware_state.value == "ready_inactive"
    assert state.capture_active is False


def test_life_09_failsafe_forces_all_off():
    state = _tick(FsmState.FAILSAFE, clients=2)
    assert state.detector.requested is False
    assert state.recording.requested is False
    assert state.stream_forced_off is True
    assert state.capture_active is False


def test_life_03_getting_close_detector_off_with_recording():
    state = _tick(
        FsmState.GETTING_CLOSE,
        context=LifecycleMissionContext(chase_recording_requested=True),
    )
    assert state.detector.requested is False
    assert state.recording.requested is True
    assert state.capture_active is True


def test_life_04_search_detector_required():
    state = _tick(FsmState.SEARCH)
    assert state.detector.requested is True
    assert state.detector.reason == "SEARCH_required"
    assert state.detector_mission_override is True


def test_life_05_chase_detector_required():
    state = _tick(FsmState.CHASE)
    assert state.detector.requested is True
    assert state.detector_mission_override is True


def test_life_07_goto_exact_flags():
    state = _tick(
        FsmState.GOTO,
        context=LifecycleMissionContext(
            goto_request_yolo=False,
            goto_request_recording=True,
        ),
    )
    assert state.detector.requested is False
    assert state.recording.requested is True


def test_life_08_return_home_retains_postroll():
    state = _tick(
        FsmState.RETURN_HOME,
        context=LifecycleMissionContext(
            recording_postroll_deadline_ms=5000
        ),
        now_ms=1000,
    )
    assert state.detector.requested is False
    assert state.recording.requested is True


def test_life_06_brake_reverse_inherits_saved():
    state = _tick(
        FsmState.BRAKE_REVERSE,
        context=LifecycleMissionContext(
            brake_saved_detector=True,
            brake_saved_recording=True,
        ),
    )
    assert state.detector.requested is True
    assert state.recording.requested is True


def test_life_02_idle_postroll_only():
    active = _tick(
        FsmState.IDLE,
        context=LifecycleMissionContext(
            recording_postroll_deadline_ms=2000
        ),
        now_ms=1000,
    )
    assert active.recording.requested is True
    assert active.detector.requested is False

    expired = _tick(
        FsmState.IDLE,
        context=LifecycleMissionContext(
            recording_postroll_deadline_ms=2000
        ),
        now_ms=2500,
    )
    assert expired.recording.requested is False


def test_life_10_stream_refcount_race():
    mgr = PerceptionLifecycleManager(TargetRuntimeConfig())
    errors = []

    def worker(n):
        try:
            for _ in range(n):
                mgr.register_stream_client()
                mgr.unregister_stream_client()
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)

    threads = [__import__("threading").Thread(target=worker, args=(200,)) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert errors == []
    assert mgr.stream_clients == 0
    state = mgr.tick(
        fsm_state=FsmState.SEARCH,
        mission=MissionState(),
        context=LifecycleMissionContext(),
        now_ms=1,
    )
    assert state.detector.requested is True


def test_life_11_ready_inactive_when_no_consumers():
    state = _tick(FsmState.IDLE)
    assert state.capture_active is False
    assert state.camera_hardware_state.value == "ready_inactive"


def test_life_12_camera_fault_marks_faulted():
    mgr = PerceptionLifecycleManager(TargetRuntimeConfig())
    mgr.note_camera_fault(now_ms=10)
    state = mgr.tick(
        fsm_state=FsmState.SEARCH,
        mission=MissionState(),
        context=LifecycleMissionContext(),
        now_ms=20,
    )
    assert state.camera_fatal_fault is True
    assert state.camera_hardware_state.value == "faulted"
    assert state.capture_active is False


def test_stop_chase_starts_postroll_via_engine():
    from cat_follow.control.decision_engine import DecisionEngine
    from cat_follow.control.fsm import FSM

    fsm = FSM()
    engine = DecisionEngine(fsm, target_runtime_config=TargetRuntimeConfig())
    engine.request_chase_recording()
    engine.start_recording_postroll(1000)
    mission = engine.mission_state
    assert mission.chase_recording_requested is False
    assert mission.recording_postroll_deadline_ms == 11000


def test_clear_expired_recording_postroll():
    from cat_follow.control.decision_engine import DecisionEngine
    from cat_follow.control.fsm import FSM

    fsm = FSM()
    engine = DecisionEngine(fsm, target_runtime_config=TargetRuntimeConfig())
    engine.start_recording_postroll(1000)
    engine.clear_expired_recording_postroll(5000)
    assert engine.mission_state.recording_postroll_deadline_ms == 11000
    engine.clear_expired_recording_postroll(11000)
    assert engine.mission_state.recording_postroll_deadline_ms is None


def test_str_02_no_mjpeg_routes_in_web_ui_package():
    import pathlib

    web_ui = pathlib.Path(__file__).resolve().parents[1] / "cat_follow" / "web_ui"
    route_files = list(web_ui.glob("routes_*.py"))
    forbidden = ("mjpeg", "multipart/x-mixed-replace")
    hits = []
    for path in route_files:
        text = path.read_text(encoding="utf-8").lower()
        for token in forbidden:
            if token in text:
                hits.append(f"{path.name}:{token}")
    assert hits == []