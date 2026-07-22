"""Shared SSD detection output parsing.

Backend-agnostic post-processing for SSD-MobileNet style detectors: given the
model's output tensors it returns the single best detection as
``(x, y, w, h, valid)`` in full-frame pixels.  Used by the RKNN NPU backend
(:mod:`cat_follow.vision.rknn_backend`).
"""

from typing import Any, List, Tuple

import numpy as np


def validate_ssd_output_contract(outputs: List[Any]) -> None:
    """Raise ``ValueError`` unless *outputs* match the finalized SSD contract
    that :func:`parse_ssd_outputs` consumes.

    :func:`parse_ssd_outputs` reads ``outputs[0]`` as boxes ``(N, 4)`` and
    ``outputs[2]`` as scores ``(N,)`` (the standard
    ``TFLite_Detection_PostProcess`` ordering: boxes / classes / scores /
    count).  A loose check (e.g. "any four-element tensor") would let an
    incompatible model pass preflight and then silently produce no detections,
    so this validates:

    - at least 4 output tensors are present;
    - ``boxes`` (``outputs[0]``) squeezes to ``(N, 4)``;
    - ``scores`` (``outputs[2]``) squeezes to ``(N,)`` with the **same** ``N``;
    - ``classes`` (``outputs[1]``) squeezes to ``(N,)`` with the same ``N``;
    - scores look like probabilities (finite, within ``[0, 1.5]`` to allow for
      minor quantization overshoot).
    """
    if not outputs:
        raise ValueError("model returned no output tensors")
    arrs = [np.asarray(o) for o in outputs]
    shapes = [tuple(a.shape) for a in arrs]

    if len(arrs) < 4:
        raise ValueError(
            f"expected >=4 SSD detection tensors (boxes/classes/scores/count), "
            f"got {len(arrs)} with shapes {shapes}"
        )

    boxes = np.squeeze(arrs[0])
    # A single-detection model squeezes (1,1,4) -> (4,); normalize to (1, 4).
    if boxes.ndim == 1 and boxes.shape[0] == 4:
        boxes = boxes.reshape(1, 4)
    if boxes.ndim != 2 or boxes.shape[1] != 4:
        raise ValueError(
            f"boxes tensor (outputs[0]) must be (N, 4); got shape "
            f"{tuple(np.asarray(arrs[0]).shape)} -> squeezed {boxes.shape}"
        )
    num = boxes.shape[0]

    classes = np.atleast_1d(np.squeeze(arrs[1]))
    scores = np.atleast_1d(np.squeeze(arrs[2]))
    if scores.ndim != 1 or scores.shape[0] != num:
        raise ValueError(
            f"scores tensor (outputs[2]) must be (N,) aligned with boxes N={num}; "
            f"got shape {tuple(np.asarray(arrs[2]).shape)} -> squeezed {scores.shape}"
        )
    if classes.ndim != 1 or classes.shape[0] != num:
        raise ValueError(
            f"classes tensor (outputs[1]) must be (N,) aligned with boxes N={num}; "
            f"got shape {tuple(np.asarray(arrs[1]).shape)} -> squeezed {classes.shape}"
        )

    finite = scores[np.isfinite(scores)]
    if finite.size == 0:
        raise ValueError("scores tensor contains no finite values")
    if float(finite.min()) < 0.0 or float(finite.max()) > 1.5:
        raise ValueError(
            "scores are outside the expected probability range [0, 1.5]; "
            f"got min={float(finite.min()):.3f}, max={float(finite.max()):.3f} "
            "(the model may not be a finalized SSD with decoded scores, or the "
            "output ordering differs from boxes/classes/scores/count)"
        )


def parse_ssd_outputs(
    outputs: List[Any],
    frame_h: int,
    frame_w: int,
    score_thresh: float = 0.5,
) -> Tuple[float, float, float, float, float]:
    """Parse SSD detection outputs to ``(x, y, w, h, valid)``.

    ``valid`` is ``1.0`` if a detection above ``score_thresh`` was found, else
    ``0.0``.
    """
    # SSD-style: boxes [1,N,4] (ymin,xmin,ymax,xmax) normalized, scores [1,N]
    if len(outputs) >= 4:
        boxes = outputs[0]
        scores = outputs[2]
        if isinstance(boxes, np.ndarray):
            boxes = np.squeeze(boxes)
        if isinstance(scores, np.ndarray):
            scores = np.squeeze(scores)
        if boxes.ndim == 2 and scores.ndim == 1:
            best_idx = int(np.argmax(scores))
            if float(scores[best_idx]) >= score_thresh:
                bymin, bxmin, bymax, bxmax = boxes[best_idx]
                xmin = int(bxmin * frame_w)
                ymin = int(bymin * frame_h)
                xmax = int(bxmax * frame_w)
                ymax = int(bymax * frame_h)
                w = max(0, xmax - xmin)
                h = max(0, ymax - ymin)
                return (float(xmin), float(ymin), float(w), float(h), 1.0)
    # Fallback: single-box output length 4
    for out in outputs:
        arr = np.array(out).squeeze()
        if arr.size == 4:
            a0, a1, a2, a3 = arr.tolist()
            if max(arr) <= 1.01:
                xmin = int(a1 * frame_w)
                ymin = int(a0 * frame_h)
                xmax = int(a3 * frame_w)
                ymax = int(a2 * frame_h)
                w = max(0, xmax - xmin)
                h = max(0, ymax - ymin)
                return (float(xmin), float(ymin), float(w), float(h), 1.0)
            else:
                xmin = int(min(a0, a2))
                ymin = int(min(a1, a3))
                xmax = int(max(a0, a2))
                ymax = int(max(a1, a3))
                w = max(0, xmax - xmin)
                h = max(0, ymax - ymin)
                return (float(xmin), float(ymin), float(w), float(h), 1.0)
    return (0.0, 0.0, 0.0, 0.0, 0.0)


__all__ = ["parse_ssd_outputs", "validate_ssd_output_contract"]
