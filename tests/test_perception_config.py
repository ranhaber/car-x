"""Tests for perception processing configuration and phase gating."""

import pytest

from cat_follow.perception_config import PerceptionConfig, load_perception_config
from cat_follow.perception.phase import Phase, PhaseMachine


_ENV_NAMES = (
    "MOTION_GATING",
    "MOTION_SCALE",
    "MOTION_THRESHOLD",
    "MOTION_MIN_AREA",
    "DETECT_FPS",
    "DETECT_INTERVAL_TRACKING",
    "IDLE_UNLOAD_SEC",
    "WARMUP_ON_START",
    "OPENCV_THREADS_IDLE",
    "OPENCV_THREADS_ACTIVE",
    "AFFINITY_ENABLED",
    "CAMERA_CORES",
    "DETECTOR_CORES",
    "RKNN_MODEL_PATH",
    "RKNN_INPUT",
)


@pytest.fixture(autouse=True)
def _clear_env(monkeypatch):
    for name in _ENV_NAMES:
        monkeypatch.delenv(f"CAT_FOLLOW_PERCEPTION_{name}", raising=False)


def test_defaults_are_rknn_only():
    config = load_perception_config()
    assert config.rknn_model_path == "models/ssd_mobilenet_v2.rknn"
    assert config.rknn_input_size == (320, 320)
    assert config.motion_gating is True
    assert config.opencv_threads_idle == 1
    assert config.opencv_threads_active == 4
    assert config.affinity_enabled is False
    assert config.camera_cores == ()
    assert config.detector_cores == ()
    # The backend is fixed to RKNN; there is no backend/uses_rknn selector.
    assert not hasattr(config, "backend")
    assert not hasattr(config, "uses_rknn")


def test_env_driven_config(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_MOTION_GATING", "0")
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_AFFINITY_ENABLED", "1")
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_CAMERA_CORES", "4,5")
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_DETECTOR_CORES", "6, 7")
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_DETECT_INTERVAL_TRACKING", "3")
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_RKNN_MODEL_PATH", "models/cat.rknn")
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_RKNN_INPUT", "320,240")

    config = load_perception_config()
    assert config.motion_gating is False
    assert config.affinity_enabled is True
    assert config.camera_cores == (4, 5)
    assert config.detector_cores == (6, 7)
    assert config.detect_interval_tracking == 3
    assert config.rknn_model_path == "models/cat.rknn"
    assert config.rknn_input_size == (320, 240)


def test_rknn_input_square_shorthand(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_PERCEPTION_RKNN_INPUT", "416")
    assert load_perception_config().rknn_input_size == (416, 416)


@pytest.mark.parametrize(
    ("name", "value"),
    (
        ("MOTION_THRESHOLD", "0"),
        ("DETECT_INTERVAL_TRACKING", "0"),
        ("OPENCV_THREADS_IDLE", "0"),
        ("RKNN_INPUT", "0,0"),
    ),
)
def test_below_minimum_rejected(monkeypatch, name, value):
    monkeypatch.setenv(f"CAT_FOLLOW_PERCEPTION_{name}", value)
    with pytest.raises(ValueError):
        load_perception_config()


def test_phase_idle_to_acquisition_on_motion():
    pm = PhaseMachine()
    assert pm.phase is Phase.IDLE
    assert pm.should_detect(0) is False
    pm.update(now_s=0.0, motion=True, detected=False)
    assert pm.phase is Phase.ACQUISITION
    assert pm.should_detect(1) is True  # every frame while acquiring


def test_phase_acquisition_to_tracking_on_detection():
    pm = PhaseMachine(tracking_interval=2)
    pm.update(now_s=0.0, motion=True, detected=False)
    pm.update(now_s=0.1, motion=True, detected=True)
    assert pm.phase is Phase.TRACKING
    assert pm.should_detect(2) is True
    assert pm.should_detect(3) is False  # reduced cadence while tracking


def test_phase_tracking_times_out_to_idle():
    pm = PhaseMachine(detection_timeout_s=1.0)
    pm.update(now_s=0.0, motion=True, detected=False)  # IDLE -> ACQUISITION
    pm.update(now_s=0.1, motion=True, detected=True)  # ACQUISITION -> TRACKING
    assert pm.phase is Phase.TRACKING
    pm.update(now_s=2.0, motion=False, detected=False)
    assert pm.phase in (Phase.WATCH, Phase.IDLE)
    pm.update(now_s=4.0, motion=False, detected=False)
    assert pm.phase is Phase.IDLE
