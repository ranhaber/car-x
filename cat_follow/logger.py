"""
Centralized logging for cat-follow.

- Writes to both the **terminal** (live) and a **log file**.
- Log file: ~/logs_car_x/<YYYY-MM-DD_HH-MM-SS>.log
- All modules use:  ``from cat_follow.logger import log``
  then call ``log.info(...)``, ``log.warning(...)``, etc.
- Flask / werkzeug access logs are also captured.
"""

import logging
import os
import sys
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Log directory and file name
# ---------------------------------------------------------------------------
_LOG_DIR = Path.home() / "logs_car_x"
_LOG_DIR.mkdir(parents=True, exist_ok=True)

_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE = _LOG_DIR / f"{_timestamp}.log"

# ---------------------------------------------------------------------------
# Format
# ---------------------------------------------------------------------------
_FMT = "%(asctime)s [%(levelname)-5s] %(name)-20s  %(message)s"
_DATE_FMT = "%Y-%m-%d %H:%M:%S"

# ---------------------------------------------------------------------------
# Build the root logger once at import time
# ---------------------------------------------------------------------------
_formatter = logging.Formatter(_FMT, datefmt=_DATE_FMT)

# File handler — captures everything (DEBUG and above)
_file_handler = logging.FileHandler(str(LOG_FILE), encoding="utf-8")
_file_handler.setLevel(logging.DEBUG)
_file_handler.setFormatter(_formatter)

# Console handler — INFO and above (live terminal output)
_console_handler = logging.StreamHandler(sys.stdout)
_console_handler.setLevel(logging.INFO)
_console_handler.setFormatter(_formatter)

# Root logger
_root = logging.getLogger()
_root.setLevel(logging.DEBUG)
# Avoid duplicate handlers if this module is re-imported
if not _root.handlers:
    _root.addHandler(_file_handler)
    _root.addHandler(_console_handler)


def get_logger(name: str) -> logging.Logger:
    """Return a named child logger (e.g. ``get_logger('main_loop')``)."""
    return logging.getLogger(name)


# Convenience: a default logger named 'cat_follow'
log = get_logger("cat_follow")

# Log the startup
log.info("Logging started -> %s", LOG_FILE)
