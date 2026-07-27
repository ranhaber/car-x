#!/usr/bin/env python3
"""Production WebSocket H.264 soak against running cat-follow.service.

Connects to the live Flask /ws/h264 route (not a headless encoder loop),
measures binary stream FPS, parses [DETECT-PERF] capture stalls from journal,
and checks FD stability across connect/disconnect cycles.

Usage on ROCK 4D::

    python3 scripts/board_soak_ws_h264_prod.py --seconds 300 --cycles 3
"""

from __future__ import annotations

import argparse
import json
import os
import re
import ssl
import subprocess
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.soak_h264_metrics import (  # noqa: E402
    capture_stats,
    contiguous_generations,
    count_requeue_errors,
    parse_detect_perf,
    percentile,
)

DETECT_RE = re.compile(
    r"\[DETECT-PERF\] gen=(\d+) capture=([\d.]+)ms queue=([\d.]+)ms"
)


def _service_pid() -> int:
    out = subprocess.check_output(
        ["systemctl", "show", "-p", "MainPID", "--value", "cat-follow.service"],
        text=True,
    ).strip()
    return int(out)


def _fd_count(pid: int) -> int:
    try:
        return len(os.listdir(f"/proc/{pid}/fd"))
    except OSError:
        return -1


def _api_status(base: str) -> dict[str, Any]:
    ctx = ssl._create_unverified_context()
    with urllib.request.urlopen(f"{base}/api/status", context=ctx, timeout=5) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _journal_text(since: str) -> str:
    cmd = [
        "journalctl",
        "-u",
        "cat-follow.service",
        "--since",
        since,
        "--no-pager",
        "-o",
        "cat",
    ]
    try:
        return subprocess.check_output(cmd, text=True, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError:
        return ""


def _journal_detect_perf(since: str) -> list[tuple[int, float, float]]:
    return parse_detect_perf(_journal_text(since))


def _open_ws(url: str):
    try:
        import websocket  # type: ignore
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "websocket-client required: pip install websocket-client"
        ) from exc
    return websocket.create_connection(
        url,
        sslopt={"cert_reqs": ssl.CERT_NONE},
        timeout=10,
    )


def _stream_once(url: str, seconds: float) -> dict[str, Any]:
    ws = _open_ws(url)
    binary = 0
    overlay = 0
    bytes_total = 0
    gaps_ge_50 = 0
    last_bin_t: float | None = None
    inter_ms: list[float] = []
    t0 = time.monotonic()
    deadline = t0 + seconds
    try:
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            ws.settimeout(min(1.0, max(0.05, remaining)))
            try:
                msg = ws.recv()
            except Exception:  # noqa: BLE001 - timeout / transient
                continue
            now = time.monotonic()
            if isinstance(msg, (bytes, bytearray)):
                binary += 1
                bytes_total += len(msg)
                if last_bin_t is not None:
                    gap = (now - last_bin_t) * 1000.0
                    inter_ms.append(gap)
                    if gap >= 50.0:
                        gaps_ge_50 += 1
                last_bin_t = now
            else:
                overlay += 1
    finally:
        try:
            ws.close()
        except Exception:  # noqa: BLE001
            pass
    elapsed = max(1e-6, time.monotonic() - t0)
    return {
        "elapsed_s": round(elapsed, 2),
        "binary_frames": binary,
        "overlay_frames": overlay,
        "stream_fps": round(binary / elapsed, 2),
        "stream_bytes": bytes_total,
        "interarrival_p95_ms": round(percentile(inter_ms, 95.0), 2),
        "interarrival_max_ms": round(max(inter_ms) if inter_ms else 0.0, 2),
        "stream_gaps_ge_50ms": gaps_ge_50,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="https://127.0.0.1:5000")
    parser.add_argument("--seconds", type=float, default=300.0)
    parser.add_argument("--cycles", type=int, default=3, help="connect/disconnect cycles")
    parser.add_argument(
        "--warmup-s",
        type=float,
        default=5.0,
        help="seconds of detector rows to ignore before capture stats",
    )
    args = parser.parse_args()

    ws_url = args.base.replace("https://", "wss://").replace("http://", "ws://")
    if not ws_url.endswith("/ws/h264"):
        ws_url = ws_url.rstrip("/") + "/ws/h264"

    pid = _service_pid()
    if pid <= 0:
        print(json.dumps({"status": "fail", "error": "cat-follow not running"}))
        return 1

    status0 = _api_status(args.base)
    fd_baseline = _fd_count(pid)
    cycle_fds: list[dict[str, int]] = []

    for i in range(max(1, args.cycles)):
        fd_before = _fd_count(pid)
        short = _stream_once(ws_url, 3.0)
        time.sleep(1.0)
        fd_after = _fd_count(pid)
        cycle_fds.append(
            {
                "cycle": i + 1,
                "fd_before": fd_before,
                "fd_after": fd_after,
                "fd_delta": fd_after - fd_before,
                "binary": short["binary_frames"],
            }
        )

    since = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    time.sleep(0.2)
    long_run = _stream_once(ws_url, max(10.0, args.seconds))
    time.sleep(1.0)
    fd_final = _fd_count(pid)
    status1 = _api_status(args.base)
    journal_text = _journal_text(since)
    rows = _journal_detect_perf(since)
    if rows and args.warmup_s > 0:
        first_gen = rows[0][0]
        skip = int(args.warmup_s * 30)
        rows = [row for row in rows if row[0] >= first_gen + skip]

    cap = capture_stats(rows)
    steady_elapsed = max(1e-6, long_run["elapsed_s"] - max(0.0, args.warmup_s))
    detect_fps = len(rows) / steady_elapsed if rows else 0.0
    requeue_errors = count_requeue_errors(journal_text.splitlines())

    later_cycles = cycle_fds[1:] if len(cycle_fds) > 1 else cycle_fds
    fd_after_cycles = cycle_fds[-1]["fd_after"] if cycle_fds else fd_baseline
    gates = {
        "capture_p95_le_34": cap["capture_p95_ms"] <= 34.0 if rows else False,
        "capture_max_lt_40": cap["capture_max_ms"] < 40.0 if rows else False,
        "zero_stalls_ge_50": cap["stalls_ge_50ms"] == 0 if rows else False,
        "detect_fps_ge_29_5": detect_fps >= 29.5 if rows else False,
        "contiguous_detector_gens": contiguous_generations(rows),
        "stream_fps_ge_28": long_run["stream_fps"] >= 28.0,
        "stream_gaps_bounded": long_run["stream_gaps_ge_50ms"] <= 150,
        "fd_stable_after_encoder_open": all(c["fd_delta"] <= 0 for c in later_cycles)
        and fd_final <= fd_after_cycles,
        "stream_clients_cleared": int(status1.get("stream_clients", -1)) == 0,
        "zero_requeue_errors": requeue_errors == 0,
    }
    passed = all(gates.values())

    report = {
        "status": "pass" if passed else "fail",
        "service_pid": pid,
        "ws_url": ws_url,
        "status_before": {
            "stream_clients": status0.get("stream_clients"),
            "perception": status0.get("perception"),
            "tracker_fps": status0.get("tracker_fps"),
        },
        "status_after": {
            "stream_clients": status1.get("stream_clients"),
            "perception": status1.get("perception"),
            "tracker_fps": status1.get("tracker_fps"),
        },
        "connect_disconnect_cycles": cycle_fds,
        "fd_baseline": fd_baseline,
        "fd_final": fd_final,
        "fd_delta_total": fd_final - fd_baseline,
        "long_stream": long_run,
        "detect_samples": len(rows),
        "detect_fps": round(detect_fps, 2),
        "capture_p95_ms": cap["capture_p95_ms"],
        "capture_max_ms": cap["capture_max_ms"],
        "stalls_ge_50ms": cap["stalls_ge_50ms"],
        "requeue_errors": requeue_errors,
        "gates": gates,
    }
    print(json.dumps(report, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
