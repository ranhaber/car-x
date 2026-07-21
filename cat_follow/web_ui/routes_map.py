"""Live occupancy map API for the Control page.

Routes:
    GET /api/map — Downsampled OccupancyGrid + robot pose + scan overlay
"""

from __future__ import annotations

from flask import Blueprint, jsonify

from cat_follow.navigation.map_snapshot import map_snapshot_dict

map_bp = Blueprint("map", __name__)

_ctx = None


def init_map_routes(ctx):
    """Bind map context (unused today; snapshot is process-global)."""
    global _ctx
    _ctx = ctx


@map_bp.route("/api/map", methods=["GET"])
def api_map():
    return jsonify(map_snapshot_dict())
