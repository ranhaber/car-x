#!/usr/bin/env python3
"""Smoke-test H.264-only web UI prerequisites on the board."""
from cat_follow.memory.pool import allocate_pool
from cat_follow.memory.shared_state import SharedState
from cat_follow.perception.h264_encoder import MppH264Encoder
from cat_follow.web_ui.app import create_app

print("mpp_available", MppH264Encoder.available())
app = create_app(shared=SharedState(allocate_pool()))
client = app.test_client()
res = client.get("/api/stream/capabilities")
print("capabilities_status", res.status_code, res.get_json())
res2 = client.get("/")
print("home_status", res2.status_code)
print("h264_ui_ok")
