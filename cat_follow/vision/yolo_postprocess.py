"""YOLOv8 RKNN model-zoo output decoding.

The production model exposes nine tensors: three scales, each containing a
64-channel DFL box branch, an 80-class COCO score branch, and a one-channel
score-sum branch.  The score-sum branch cheaply rejects empty cells before DFL
decode, matching the active pipeline in ``picarx_cat tracker``.
"""

from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np

Detection = tuple[int, int, int, int, float, int]

# Official COCO cat category id (what the tracker/roles treat as PRIMARY_CAT).
CAT_COCO_ID = 17

# Animal-mode class collapse: raw YOLO 0-indexed COCO-80 classes that a stock
# nano model confuses a small/far cat with -- cat=15, dog=16, horse=17,
# sheep=18, cow=19, bear=21.  When animal mode is on, any of these is relabelled
# as CAT_COCO_ID so a cat-sized animal still counts as a cat regardless of the
# fine-grained guess (matches ``picarx_cat tracker`` config.ANIMAL_MODE).
ANIMAL_CLASS_IDS_0IDX: frozenset[int] = frozenset({15, 16, 17, 18, 19, 21})

# YOLO's contiguous COCO-80 index -> official COCO category id.
_YOLO80_TO_COCO = np.asarray(
    (
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19,
        20, 21, 22, 23, 24, 25, 27, 28, 31, 32, 33, 34, 35, 36, 37, 38,
        39, 40, 41, 42, 43, 44, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55,
        56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 67, 70, 72, 73, 74, 75,
        76, 77, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88, 89, 90,
    ),
    dtype=np.int32,
)


def yolo80_to_coco_ids(class_indices: np.ndarray) -> np.ndarray:
    """Map zero-based YOLO class indices to official COCO category ids."""
    indices = np.asarray(class_indices, dtype=np.int32)
    result = indices + 1
    valid = (indices >= 0) & (indices < _YOLO80_TO_COCO.size)
    if np.any(valid):
        result = np.array(result, copy=True)
        result[valid] = _YOLO80_TO_COCO[indices[valid]]
    return result


def validate_yolo_output_contract(outputs: Sequence[object]) -> None:
    """Raise unless *outputs* match the 9-tensor YOLOv8 model-zoo contract."""
    if outputs is None or len(outputs) != 9:
        count = 0 if outputs is None else len(outputs)
        raise ValueError(f"expected 9 YOLO output tensors, got {count}")

    expected_grids = ((40, 40), (20, 20), (10, 10))
    for branch, (grid_h, grid_w) in enumerate(expected_grids):
        box = np.asarray(outputs[branch * 3])
        score = np.asarray(outputs[branch * 3 + 1])
        score_sum = np.asarray(outputs[branch * 3 + 2])
        expected = (
            (1, 64, grid_h, grid_w),
            (1, 80, grid_h, grid_w),
            (1, 1, grid_h, grid_w),
        )
        actual = (box.shape, score.shape, score_sum.shape)
        if actual != expected:
            raise ValueError(
                f"YOLO branch {branch} shapes must be {expected}, got {actual}"
            )
        if not all(np.issubdtype(arr.dtype, np.number) for arr in (box, score, score_sum)):
            raise ValueError(f"YOLO branch {branch} contains non-numeric tensors")


def _decode_cells_dfl(
    box_branch: np.ndarray,
    cells: np.ndarray,
    grid_h: int,
    grid_w: int,
    input_w: int,
    input_h: int,
) -> np.ndarray:
    """Decode only cells that survived score filtering into model-space xyxy."""
    box_branch = np.asarray(box_branch)
    channels = box_branch.shape[1]
    candidates = box_branch[0].reshape(channels, -1).T[cells].astype(
        np.float32, copy=False
    )
    bins = channels // 4
    logits = candidates.reshape(candidates.shape[0], 4, bins)
    logits -= np.max(logits, axis=2, keepdims=True)
    exp_logits = np.exp(logits, dtype=np.float32)
    weights = exp_logits / np.sum(exp_logits, axis=2, keepdims=True)
    decoded = (weights * np.arange(bins, dtype=np.float32)).sum(axis=2)

    col = (cells % grid_w).astype(np.float32)
    row = (cells // grid_w).astype(np.float32)
    stride_x = np.float32(input_w / grid_w)
    stride_y = np.float32(input_h / grid_h)
    return np.stack(
        (
            (col + 0.5 - decoded[:, 0]) * stride_x,
            (row + 0.5 - decoded[:, 1]) * stride_y,
            (col + 0.5 + decoded[:, 2]) * stride_x,
            (row + 0.5 + decoded[:, 3]) * stride_y,
        ),
        axis=1,
    )


def _unletterbox(
    box: Iterable[float],
    *,
    frame_w: int,
    frame_h: int,
    scale: float,
    pad_x: int,
    pad_y: int,
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = (float(value) for value in box)
    x1 = max(0, min(int(round((x1 - pad_x) / scale)), frame_w))
    y1 = max(0, min(int(round((y1 - pad_y) / scale)), frame_h))
    x2 = max(0, min(int(round((x2 - pad_x) / scale)), frame_w))
    y2 = max(0, min(int(round((y2 - pad_y) / scale)), frame_h))
    return x1, y1, x2, y2


def decode_yolov8_outputs(
    outputs: Sequence[object],
    *,
    input_w: int,
    input_h: int,
    frame_w: int,
    frame_h: int,
    scale: float,
    pad_x: int,
    pad_y: int,
    score_threshold: float,
    target_class_ids: frozenset[int] = frozenset({17}),
    nms_threshold: float = 0.45,
    animal_class_ids_0idx: frozenset[int] = frozenset(),
) -> list[Detection]:
    """Decode, filter and NMS YOLO outputs into frame-space detections.

    When *animal_class_ids_0idx* is non-empty, any detection whose raw YOLO
    0-indexed class is in that set is relabelled as :data:`CAT_COCO_ID` before
    the target-class filter runs, so a cat-sized quadruped counts as a cat.
    """
    animal_mask_0idx = (
        np.asarray(sorted(animal_class_ids_0idx), dtype=np.int32)
        if animal_class_ids_0idx
        else None
    )
    validate_yolo_output_contract(outputs)
    box_parts: list[np.ndarray] = []
    score_parts: list[np.ndarray] = []
    class_parts: list[np.ndarray] = []

    for branch in range(3):
        box_branch = np.asarray(outputs[branch * 3])
        score_branch = np.asarray(outputs[branch * 3 + 1])
        score_sum = np.asarray(outputs[branch * 3 + 2], dtype=np.float32)
        grid_h, grid_w = box_branch.shape[2:4]
        preliminary = np.nonzero(score_sum.reshape(-1) >= score_threshold)[0]
        if preliminary.size == 0:
            continue

        class_scores = score_branch[0].reshape(80, -1)[:, preliminary].T.astype(
            np.float32, copy=False
        )
        class_indices = np.argmax(class_scores, axis=1)
        best_scores = class_scores[np.arange(preliminary.size), class_indices]
        coco_ids = yolo80_to_coco_ids(class_indices)
        if animal_mask_0idx is not None:
            is_animal = np.isin(class_indices, animal_mask_0idx)
            if np.any(is_animal):
                coco_ids = np.where(is_animal, CAT_COCO_ID, coco_ids)
        keep = (best_scores >= score_threshold) & np.isin(
            coco_ids, tuple(target_class_ids)
        )
        if not np.any(keep):
            continue

        cells = preliminary[keep]
        box_parts.append(
            _decode_cells_dfl(
                box_branch, cells, grid_h, grid_w, input_w, input_h
            )
        )
        score_parts.append(best_scores[keep])
        class_parts.append(coco_ids[keep])

    if not box_parts:
        return []

    boxes = np.concatenate(box_parts)
    scores = np.concatenate(score_parts)
    class_ids = np.concatenate(class_parts)
    cv2 = _cv2()
    result: list[Detection] = []

    for class_id in np.unique(class_ids):
        indices = np.where(class_ids == class_id)[0]
        candidates = boxes[indices]
        candidate_scores = scores[indices]
        rectangles = [
            [float(x1), float(y1), float(x2 - x1), float(y2 - y1)]
            for x1, y1, x2, y2 in candidates
        ]
        selected = cv2.dnn.NMSBoxes(
            rectangles,
            candidate_scores.tolist(),
            score_threshold,
            nms_threshold,
        )
        for selected_index in np.asarray(selected).reshape(-1):
            index = int(selected_index)
            x1, y1, x2, y2 = _unletterbox(
                candidates[index],
                frame_w=frame_w,
                frame_h=frame_h,
                scale=scale,
                pad_x=pad_x,
                pad_y=pad_y,
            )
            if x2 > x1 and y2 > y1:
                result.append(
                    (
                        x1,
                        y1,
                        x2,
                        y2,
                        float(candidate_scores[index]),
                        int(class_id),
                    )
                )
    return result


def _cv2():
    import cv2

    return cv2


__all__ = [
    "ANIMAL_CLASS_IDS_0IDX",
    "CAT_COCO_ID",
    "Detection",
    "decode_yolov8_outputs",
    "validate_yolo_output_contract",
    "yolo80_to_coco_ids",
]
