#!/usr/bin/env python3
"""Standalone Radxa Camera 4K (IMX415) diagnostic.

Checks the Rockchip 3A/AWB installation and IQ-file binding, inspects V4L2
formats, captures packed NV12, measures luma/chroma, and writes side-by-side
NV12/NV21 previews with JSON and HTML reports.

Run on the board while the production camera service is stopped:

    sudo systemctl stop cat-follow
    /opt/car-x/venv/bin/python scripts/diagnose_radxa_camera_4k.py
    sudo systemctl start cat-follow

Exit status is 0 for PASS, 2 for WARN, and 1 for FAIL.
"""

from __future__ import annotations

import argparse
import base64
import html
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def run(command: list[str], timeout: float = 10.0) -> dict[str, Any]:
    """Run a command and return a JSON-safe result."""
    try:
        proc = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return {
            "command": command,
            "returncode": proc.returncode,
            "stdout": proc.stdout.strip(),
            "stderr": proc.stderr.strip(),
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {
            "command": command,
            "returncode": None,
            "stdout": "",
            "stderr": str(exc),
        }


def read_dt_string(path: Path) -> str:
    try:
        return path.read_bytes().rstrip(b"\x00").decode("utf-8", "replace")
    except OSError:
        return ""


def find_sensor(sensor: str) -> dict[str, str]:
    root = Path("/sys/bus/i2c/drivers") / sensor
    result = {"sensor": sensor, "sysfs": "", "module": "", "lens": ""}
    if not root.is_dir():
        return result
    for child in sorted(root.iterdir()):
        if "-" not in child.name or not (child / "of_node").exists():
            continue
        node = child / "of_node"
        result.update(
            {
                "sysfs": str(child),
                "module": read_dt_string(
                    node / "rockchip,camera-module-name"
                ),
                "lens": read_dt_string(
                    node / "rockchip,camera-module-lens-name"
                ),
            }
        )
        break
    return result


def inspect_rkaiq(sensor: dict[str, str]) -> dict[str, Any]:
    binary = shutil.which("rkaiq_3A_server") or ""
    service = run(
        [
            "systemctl",
            "show",
            "rkaiq_3A.service",
            "--property=LoadState,ActiveState,SubState",
        ]
    )
    service_props: dict[str, str] = {}
    for line in service["stdout"].splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            service_props[key] = value

    iq_dir = Path("/etc/iqfiles")
    iq_files = sorted(str(path) for path in iq_dir.glob("imx415*.json"))
    expected_name = ""
    if sensor["module"] and sensor["lens"]:
        expected_name = (
            f"{sensor['sensor']}_{sensor['module']}_{sensor['lens']}.json"
        )
    expected_path = iq_dir / expected_name if expected_name else None
    return {
        "binary": binary,
        "service": service_props,
        "service_probe": service,
        "iq_directory": str(iq_dir),
        "iq_files": iq_files,
        "expected_iq": str(expected_path) if expected_path else "",
        "expected_iq_exists": bool(expected_path and expected_path.is_file()),
    }


def inspect_v4l2(device: str) -> dict[str, Any]:
    if shutil.which("v4l2-ctl") is None:
        return {"error": "v4l2-ctl not installed"}
    return {
        "info": run(["v4l2-ctl", "-d", device, "--info"]),
        "format": run(["v4l2-ctl", "-d", device, "--get-fmt-video"]),
        "formats": run(["v4l2-ctl", "-d", device, "--list-formats-ext"]),
    }


def channel_stats(array: np.ndarray) -> dict[str, float]:
    values = array.astype(np.float32, copy=False)
    return {
        "mean": round(float(values.mean()), 3),
        "median": round(float(np.median(values)), 3),
        "std": round(float(values.std()), 3),
        "min": round(float(values.min()), 3),
        "max": round(float(values.max()), 3),
        "p01": round(float(np.percentile(values, 1)), 3),
        "p99": round(float(np.percentile(values, 99)), 3),
    }


def capture_nv12(
    device: str,
    width: int,
    height: int,
    frames: int,
) -> tuple[np.ndarray, dict[str, Any]]:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"NV12"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(
            f"cannot open {device}; stop cat-follow or another camera owner"
        )

    expected = width * height * 3 // 2
    latest: np.ndarray | None = None
    started = time.monotonic()
    successful = 0
    try:
        for _ in range(max(1, frames)):
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            flat = np.asarray(frame, dtype=np.uint8).reshape(-1)
            if flat.size != expected:
                raise RuntimeError(
                    f"NV12 frame has {flat.size} bytes; expected {expected}"
                )
            latest = flat.copy().reshape(height * 3 // 2, width)
            successful += 1
    finally:
        cap.release()
    if latest is None:
        raise RuntimeError(f"{device} produced no frames")

    y = latest[:height]
    uv = latest[height:]
    u = uv[:, 0::2]
    v = uv[:, 1::2]
    return latest, {
        "requested_frames": frames,
        "successful_frames": successful,
        "elapsed_s": round(time.monotonic() - started, 3),
        "shape": list(latest.shape),
        "y": channel_stats(y),
        "u": channel_stats(u),
        "v": channel_stats(v),
    }


def save_previews(
    frame: np.ndarray,
    output_dir: Path,
) -> dict[str, str]:
    output_dir.mkdir(parents=True, exist_ok=True)
    nv12 = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_NV12)
    nv21 = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_NV21)

    # Diagnostic-only gray-world preview. It demonstrates what proper AWB
    # should broadly correct without pretending to replace an IQ profile.
    corrected = frame.copy()
    height = frame.shape[0] * 2 // 3
    uv = corrected[height:]
    for plane in (uv[:, 0::2], uv[:, 1::2]):
        shifted = plane.astype(np.int16) - int(plane.mean()) + 128
        plane[:] = np.clip(shifted, 0, 255).astype(np.uint8)
    gray_world = cv2.cvtColor(corrected, cv2.COLOR_YUV2BGR_NV12)

    paths = {
        "nv12": output_dir / "preview_nv12.png",
        "nv21": output_dir / "preview_nv21.png",
        "gray_world": output_dir / "preview_gray_world.png",
        "raw": output_dir / "frame.nv12",
    }
    cv2.imwrite(str(paths["nv12"]), nv12)
    cv2.imwrite(str(paths["nv21"]), nv21)
    cv2.imwrite(str(paths["gray_world"]), gray_world)
    paths["raw"].write_bytes(frame.tobytes())
    return {key: str(value) for key, value in paths.items()}


def diagnose(report: dict[str, Any]) -> tuple[str, list[dict[str, str]]]:
    findings: list[dict[str, str]] = []

    def add(severity: str, check: str, message: str) -> None:
        findings.append(
            {"severity": severity, "check": check, "message": message}
        )

    sensor = report["sensor"]
    rkaiq = report["rkaiq"]
    capture = report.get("capture")

    if not sensor["sysfs"]:
        add("FAIL", "sensor", "IMX415 was not found in sysfs.")
    elif not sensor["module"] or not sensor["lens"]:
        add(
            "WARN",
            "device-tree",
            "Camera module/lens names are missing; IQ-file matching is ambiguous.",
        )
    else:
        add(
            "PASS",
            "device-tree",
            f"IMX415 module={sensor['module']} lens={sensor['lens']}.",
        )

    if not rkaiq["binary"]:
        add(
            "FAIL",
            "rkaiq",
            "rkaiq_3A_server is not installed; AWB/AE/CCM will not run.",
        )
    if rkaiq["service"].get("ActiveState") != "active":
        add(
            "FAIL",
            "rkaiq-service",
            "rkaiq_3A.service is not active.",
        )
    if not rkaiq["expected_iq_exists"]:
        add(
            "FAIL",
            "iq-file",
            f"Expected tuning file is missing: {rkaiq['expected_iq'] or 'unknown'}",
        )

    if "capture_error" in report:
        add("FAIL", "capture", report["capture_error"])
    elif capture:
        y_mean = capture["y"]["mean"]
        u_mean = capture["u"]["mean"]
        v_mean = capture["v"]["mean"]
        if y_mean < 30:
            add(
                "FAIL",
                "exposure",
                f"Very dark frame (mean Y={y_mean:.1f}/255).",
            )
        elif y_mean > 225:
            add(
                "WARN",
                "exposure",
                f"Likely overexposed frame (mean Y={y_mean:.1f}/255).",
            )
        else:
            add("PASS", "exposure", f"Mean Y={y_mean:.1f}/255.")

        chroma_bias = max(abs(u_mean - 128), abs(v_mean - 128))
        if chroma_bias >= 15:
            add(
                "WARN",
                "white-balance",
                f"Strong mean chroma bias U={u_mean:.1f}, V={v_mean:.1f}; "
                "compare previews and verify rkaiq/IQ tuning.",
            )
        else:
            add(
                "PASS",
                "chroma",
                f"Mean chroma U={u_mean:.1f}, V={v_mean:.1f}.",
            )

    worst = "PASS"
    if any(item["severity"] == "FAIL" for item in findings):
        worst = "FAIL"
    elif any(item["severity"] == "WARN" for item in findings):
        worst = "WARN"
    return worst, findings


def image_data_uri(path: Path) -> str:
    try:
        encoded = base64.b64encode(path.read_bytes()).decode("ascii")
        return f"data:image/png;base64,{encoded}"
    except OSError:
        return ""


def write_html(report: dict[str, Any], path: Path) -> None:
    cards = []
    for name in ("nv12", "nv21", "gray_world"):
        image_path = Path(report.get("previews", {}).get(name, ""))
        uri = image_data_uri(image_path)
        cards.append(
            f"<section><h2>{html.escape(name)}</h2>"
            f"<img src='{uri}' alt='{html.escape(name)} preview'></section>"
        )
    rows = "".join(
        "<tr>"
        f"<td class='{item['severity'].lower()}'>{item['severity']}</td>"
        f"<td>{html.escape(item['check'])}</td>"
        f"<td>{html.escape(item['message'])}</td>"
        "</tr>"
        for item in report["findings"]
    )
    document = f"""<!doctype html>
<meta charset="utf-8">
<title>Radxa Camera 4K diagnostic</title>
<style>
body{{font:15px system-ui;background:#101827;color:#e5e7eb;margin:24px}}
.images{{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:16px}}
section{{background:#1f2937;padding:12px;border-radius:8px}}
img{{width:100%;height:auto}} table{{width:100%;border-collapse:collapse;margin:18px 0}}
td,th{{border:1px solid #475569;padding:8px;text-align:left}}
.pass{{color:#4ade80}}.warn{{color:#facc15}}.fail{{color:#fb7185}}
pre{{white-space:pre-wrap;background:#111827;padding:12px;border-radius:8px}}
</style>
<h1>Radxa Camera 4K diagnostic: {html.escape(report["result"])}</h1>
<table><tr><th>Severity</th><th>Check</th><th>Finding</th></tr>{rows}</table>
<div class="images">{''.join(cards)}</div>
<h2>Raw report</h2><pre>{html.escape(json.dumps(report, indent=2))}</pre>
"""
    path.write_text(document, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video11")
    parser.add_argument("--sensor", default="imx415")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frames", type=int, default=12)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("radxa-camera-diagnostic"),
    )
    parser.add_argument(
        "--no-capture",
        action="store_true",
        help="Only inspect installation/device metadata; do not open camera.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report: dict[str, Any] = {
        "device": args.device,
        "geometry": [args.width, args.height],
        "sensor": find_sensor(args.sensor),
    }
    report["rkaiq"] = inspect_rkaiq(report["sensor"])
    report["v4l2"] = inspect_v4l2(args.device)

    if not args.no_capture:
        try:
            frame, stats = capture_nv12(
                args.device, args.width, args.height, args.frames
            )
            report["capture"] = stats
            report["previews"] = save_previews(frame, args.output_dir)
        except Exception as exc:  # noqa: BLE001
            report["capture_error"] = str(exc)

    report["result"], report["findings"] = diagnose(report)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / "report.json"
    html_path = args.output_dir / "report.html"
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    write_html(report, html_path)

    for item in report["findings"]:
        print(
            f"{item['severity']:4} | {item['check']:16} | {item['message']}"
        )
    print(f"\nResult: {report['result']}")
    print(f"JSON:   {json_path}")
    print(f"HTML:   {html_path}")
    return {"PASS": 0, "WARN": 2, "FAIL": 1}[report["result"]]


if __name__ == "__main__":
    sys.exit(main())
