"""Live-frame cat injection tests (pixels only; no detector fallback)."""

from pathlib import Path

import numpy as np

from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState
from cat_follow.perception.live_cat_injector import LiveCatInjector
from cat_follow.web_ui.app import create_app


ROOT = Path(__file__).resolve().parents[1]
CAT_IMAGE = ROOT / "models" / "cat_1_320.png"


def test_injector_alpha_blends_real_asset_and_moves():
    frame = np.full((480, 640, 3), 40, dtype=np.uint8)
    before = frame.copy()
    injector = LiveCatInjector(str(CAT_IMAGE), speed_px_s=60.0)
    injector.set_enabled(True)

    bbox0 = injector.apply(frame, now=10.0)
    assert bbox0 is not None
    assert np.any(frame != before)
    x1, y1, x2, y2 = bbox0
    assert 0 <= x1 < x2 <= 640
    assert 0 <= y1 < y2 <= 480

    bbox1 = injector.apply(frame, now=11.0)
    assert bbox1 is not None
    assert bbox1[0] > bbox0[0]


def test_shared_injection_status_has_no_detection_fallback():
    shared = SharedState(allocate_pool())
    assert shared.get_cat_injection_status() == {
        "enabled": False,
        "bbox": None,
        "detection_fallback": False,
    }
    shared.set_cat_injection_enabled(True)
    shared.set_cat_injection_bbox((10, 20, 30, 40))
    assert shared.get_cat_injection_status() == {
        "enabled": True,
        "bbox": [10, 20, 30, 40],
        "detection_fallback": False,
    }
    shared.set_cat_injection_enabled(False)
    assert shared.get_cat_injection_status()["bbox"] is None


def test_injection_api_toggles_camera_owned_flag(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", "1")
    monkeypatch.delenv("CAT_FOLLOW_WEB_CONTROL_TOKEN", raising=False)
    monkeypatch.delenv("CAT_FOLLOW_COMMS_TOKEN", raising=False)
    shared = SharedState(allocate_pool())
    app = create_app(shared=shared)
    client = app.test_client()

    response = client.post("/api/dev/inject_cat", json={"action": "start"})
    assert response.status_code == 200
    assert response.get_json()["enabled"] is True
    assert shared.cat_injection_enabled() is True

    response = client.post("/api/dev/inject_cat", json={"action": "stop"})
    assert response.status_code == 200
    assert response.get_json()["enabled"] is False
    assert shared.cat_injection_enabled() is False


def test_injection_api_rejects_invalid_action(monkeypatch):
    monkeypatch.setenv("CAT_FOLLOW_ALLOW_UNAUTHENTICATED_CONTROL", "1")
    shared = SharedState(allocate_pool())
    client = create_app(shared=shared).test_client()
    response = client.post("/api/dev/inject_cat", json={"action": "fake"})
    assert response.status_code == 400
