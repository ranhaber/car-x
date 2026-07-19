"""
Page routes: main (Control) and Calibration tabs.

Routes:
    GET /            — Control tab (main.html)
    GET /calibration — Calibration tab (calibration.html)
"""

from flask import Blueprint, render_template

from cat_follow import __version__

pages_bp = Blueprint("pages", __name__)


def init_pages_routes():
    """Register page routes."""

    @pages_bp.route("/")
    def index():
        return render_template("main.html", version=__version__)

    @pages_bp.route("/calibration")
    def calibration_page():
        """Serve the Calibration tab page."""
        return render_template("calibration.html", version=__version__)
