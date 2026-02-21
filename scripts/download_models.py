"""Download helper for recommended TFLite models.

This script does not assume any particular network environment; it will
attempt to download recommended model URLs but falls back to printing
manual instructions if automatic download fails. Edit the URLS dict if
you prefer other mirrors.

Usage:
    python scripts/download_models.py --all
    python scripts/download_models.py --model ssd_mobilenet_v2
"""

import argparse
import os
import sys
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(ROOT, "models")

# Suggested model URLs. For robustness we keep a list of mirrors/variants
# for each model. The script will try them in order and report failures.
URLS = {
    # SSD MobileNet V2 FPNLite 320x320 (quantized) - multiple candidate URLs
    "ssd_mobilenet_v2_fpnlite_320x320": [
        "https://storage.googleapis.com/download.tensorflow.org/models/tflite/ssd_mobilenet_v2_320x320_coco_quant_postprocess.tflite",
        "https://storage.googleapis.com/download.tensorflow.org/models/tflite/ssd_mobilenet_v2_fpnlite_320x320_coco25_postprocess.tflite",
        # TF Model Zoo tarball (may contain a tflite inside the release archive)
        "https://storage.googleapis.com/tfhub-modules/google/ssd_mobilenet_v2_fpnlite_320x320/1.tar.gz",
    ],

    # EfficientDet D0 512x512 (TF2 model zoo entry) - try tflite, then tarball
    "efficientdet_d0_512x512": [
        # Common standalone tflite link (may not exist for all exporters)
        "https://storage.googleapis.com/download.tensorflow.org/models/tflite/efficientdet_d0_512x512.tflite",
        # TF2 Detection Model Zoo tarball for D0 (may need manual extraction)
        "https://storage.googleapis.com/download.tensorflow.org/models/tf2/efficientdet/efficientdet_d0_coco17_tpu-32.tar.gz",
    ],
}

MODEL_MAP = {
    "ssd_mobilenet_v2_fpnlite_320x320": "ssd_mobilenet_v2_320x320.tflite",
    "efficientdet_d0_512x512": "efficientdet_d0_512x512.tflite",
}


def _download_url(url: str, out_path: str) -> bool:
    """Download a single URL with a User-Agent header and save to out_path."""
    req = urllib.request.Request(url, headers={
        "User-Agent": "car-x-model-downloader/1.0",
        "Accept": "*/*",
    })
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            with open(out_path, "wb") as f:
                f.write(resp.read())
        return True
    except Exception as e:
        print(f"  -> URL failed: {url}  ({e})")
        return False


def download(name: str, urls: list, out_path: str) -> bool:
    print(f"Downloading {name} -> {out_path}")
    for url in urls:
        print(f" Trying: {url}")
        ok = _download_url(url, out_path)
        if ok:
            print("Downloaded successfully")
            return True
    print(f"All download attempts failed for {name}.")
    return False


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--all", action="store_true")
    p.add_argument("--model", choices=list(URLS.keys()))
    args = p.parse_args()

    if not args.all and args.model is None:
        p.print_help()
        sys.exit(1)

    os.makedirs(MODELS_DIR, exist_ok=True)

    targets = list(URLS.keys()) if args.all else [args.model]
    for k in targets:
        url = URLS.get(k)
        filename = MODEL_MAP.get(k)
        if not url or not filename:
            print(f"No URL configured for {k}; please add one to this script.")
            continue
        out = os.path.join(MODELS_DIR, filename)
        if os.path.exists(out):
            print(f"Model already exists: {out}")
            continue
        ok = download(k, url, out)
        if not ok:
            print("Automatic download failed. Please obtain the .tflite file manually and place it in:")
            print("  ", out)


if __name__ == "__main__":
    main()
