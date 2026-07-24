"""Packed NV12 frame helpers.

Frames use the conventional two-plane packed NumPy layout ``(height * 3/2,
width)``: the full-resolution Y plane first, followed by interleaved UV rows.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


def nv12_shape(width: int, height: int) -> tuple[int, int]:
    """Return the packed array shape for an even NV12 geometry."""
    width = int(width)
    height = int(height)
    if width <= 0 or height <= 0 or width % 2 or height % 2:
        raise ValueError(f"NV12 geometry must be positive and even, got {width}x{height}")
    return (height * 3 // 2, width)


def validate_nv12(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Validate and return *frame* reshaped to packed NV12."""
    expected_shape = nv12_shape(width, height)
    if frame.dtype != np.uint8:
        raise ValueError(f"NV12 dtype must be uint8, got {frame.dtype}")
    if frame.size != expected_shape[0] * expected_shape[1]:
        raise ValueError(
            f"NV12 frame has {frame.size} bytes, expected "
            f"{expected_shape[0] * expected_shape[1]}"
        )
    return frame.reshape(expected_shape)


def pack_nv12_from_buffer(
    buffer,
    width: int,
    height: int,
    y_stride: int,
    uv_stride: int,
    uv_offset: int,
    *,
    y_offset: int = 0,
    mapped_size: Optional[int] = None,
    dst: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Pack a mapped, possibly stride-padded NV12 buffer into tight storage."""
    width = int(width)
    height = int(height)
    y_stride = int(y_stride)
    uv_stride = int(uv_stride)
    uv_offset = int(uv_offset)
    y_offset = int(y_offset)
    expected_shape = nv12_shape(width, height)
    if dst is None:
        dst = np.empty(expected_shape, dtype=np.uint8)
    elif dst.shape != expected_shape or dst.dtype != np.uint8:
        raise ValueError(
            f"NV12 destination must be uint8 {expected_shape}, "
            f"got {dst.dtype} {dst.shape}"
        )

    if y_stride < width or uv_stride < width:
        raise ValueError(
            f"NV12 strides must be >= width {width}, "
            f"got y_stride={y_stride}, uv_stride={uv_stride}"
        )
    if y_offset < 0 or uv_offset < 0:
        raise ValueError(
            f"NV12 offsets must be non-negative, got y={y_offset}, uv={uv_offset}"
        )

    source = np.frombuffer(buffer, dtype=np.uint8)
    y_end = y_offset + (height - 1) * y_stride + width
    uv_end = uv_offset + (height // 2 - 1) * uv_stride + width
    required = max(y_end, uv_end)
    if mapped_size is not None and required > int(mapped_size):
        raise ValueError(
            f"NV12 layout needs {required} bytes but mapped region is "
            f"{int(mapped_size)} bytes"
        )
    if source.size < required:
        raise ValueError(
            f"mapped NV12 buffer has {source.size} bytes, needs at least {required}"
        )

    if (
        y_offset == 0
        and y_stride == width
        and uv_stride == width
        and uv_offset == width * height
    ):
        np.copyto(dst.reshape(-1), source[: width * height * 3 // 2])
        return dst

    y_dst = dst[:height]
    uv_dst = dst[height:]
    for row in range(height):
        start = y_offset + row * y_stride
        y_dst[row] = source[start : start + width]
    for row in range(height // 2):
        start = uv_offset + row * uv_stride
        uv_dst[row] = source[start : start + width]
    return dst


def y_plane(frame_nv12: np.ndarray, width: int, height: int) -> np.ndarray:
    """Return a zero-copy view of the full-resolution luma plane."""
    packed = validate_nv12(frame_nv12, width, height)
    return packed[:height, :width]


def align_nv12_crop(
    x: int,
    y: int,
    width: int,
    height: int,
    frame_width: int,
    frame_height: int,
) -> tuple[int, int, int, int]:
    """Clamp a crop to the frame and align all edges for 4:2:0 chroma."""
    if width <= 0 or height <= 0:
        raise ValueError("NV12 crop dimensions must be positive")
    x = max(0, min(int(x), int(frame_width) - 2)) & ~1
    y = max(0, min(int(y), int(frame_height) - 2)) & ~1
    width = min(int(width), int(frame_width) - x) & ~1
    height = min(int(height), int(frame_height) - y) & ~1
    if width <= 0 or height <= 0:
        raise ValueError("NV12 crop is empty after alignment")
    return x, y, width, height


def center_bottom_nv12_region(
    frame_width: int,
    frame_height: int,
    crop_width: int,
    crop_height: int,
) -> tuple[int, int, int, int]:
    """Return the even-aligned center-bottom crop for a full NV12 frame."""
    return align_nv12_crop(
        (frame_width - crop_width) // 2,
        frame_height - crop_height,
        crop_width,
        crop_height,
        frame_width,
        frame_height,
    )


def extract_nv12_crop(
    source: np.ndarray,
    frame_width: int,
    frame_height: int,
    region: tuple[int, int, int, int],
    *,
    dst: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Copy an even-aligned rectangular crop into packed NV12 storage."""
    packed = validate_nv12(source, frame_width, frame_height)
    x, y, width, height = region
    if (x | y | width | height) & 1:
        raise ValueError(f"NV12 crop must be even-aligned, got {region}")
    if x < 0 or y < 0 or x + width > frame_width or y + height > frame_height:
        raise ValueError(f"NV12 crop {region} exceeds {frame_width}x{frame_height}")

    expected_shape = nv12_shape(width, height)
    if dst is None:
        dst = np.empty(expected_shape, dtype=np.uint8)
    elif dst.shape != expected_shape or dst.dtype != np.uint8:
        raise ValueError(
            f"NV12 crop destination must be uint8 {expected_shape}, "
            f"got {dst.dtype} {dst.shape}"
        )

    np.copyto(dst[:height], packed[y : y + height, x : x + width])
    uv_start = frame_height + y // 2
    np.copyto(
        dst[height:],
        packed[uv_start : uv_start + height // 2, x : x + width],
    )
    return dst


def nv12_crop_to_bgr(
    source: np.ndarray,
    region: tuple[int, int, int, int],
    frame_width: int,
    frame_height: int,
    *,
    dst_nv12: Optional[np.ndarray] = None,
    dst_bgr: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Extract an aligned NV12 crop and convert it to BGR."""
    x, y, width, height = region
    crop = extract_nv12_crop(
        source, frame_width, frame_height, region, dst=dst_nv12
    )
    return nv12_to_bgr(crop, width, height, dst=dst_bgr)


def nv12_to_bgr(
    source: np.ndarray,
    width: int,
    height: int,
    *,
    dst: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Convert packed NV12 to BGR, optionally into a reusable destination."""
    import cv2

    packed = validate_nv12(source, width, height)
    if dst is not None:
        expected = (height, width, 3)
        if dst.shape != expected or dst.dtype != np.uint8:
            raise ValueError(
                f"BGR destination must be uint8 {expected}, got {dst.dtype} {dst.shape}"
            )
        cv2.cvtColor(packed, cv2.COLOR_YUV2BGR_NV12, dst=dst)
        return dst
    return cv2.cvtColor(packed, cv2.COLOR_YUV2BGR_NV12)


def nv12_to_rgb(
    source: np.ndarray,
    width: int,
    height: int,
    *,
    dst: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Convert packed NV12 directly to RGB into an optional reusable buffer."""
    import cv2

    packed = validate_nv12(source, width, height)
    if dst is not None:
        expected = (height, width, 3)
        if dst.shape != expected or dst.dtype != np.uint8:
            raise ValueError(
                f"RGB destination must be uint8 {expected}, got {dst.dtype} {dst.shape}"
            )
        cv2.cvtColor(packed, cv2.COLOR_YUV2RGB_NV12, dst=dst)
        return dst
    return cv2.cvtColor(packed, cv2.COLOR_YUV2RGB_NV12)


def bgr_to_nv12(source: np.ndarray, *, dst: Optional[np.ndarray] = None) -> np.ndarray:
    """Convert an even-sized BGR image to packed NV12."""
    import cv2

    if source.ndim != 3 or source.shape[2] != 3 or source.dtype != np.uint8:
        raise ValueError(f"BGR source must be uint8 HxWx3, got {source.shape}")
    height, width = source.shape[:2]
    expected_shape = nv12_shape(width, height)
    if dst is None:
        dst = np.empty(expected_shape, dtype=np.uint8)
    elif dst.shape != expected_shape or dst.dtype != np.uint8:
        raise ValueError(
            f"NV12 destination must be uint8 {expected_shape}, "
            f"got {dst.dtype} {dst.shape}"
        )

    i420 = cv2.cvtColor(source, cv2.COLOR_BGR2YUV_I420)
    y_size = width * height
    chroma_size = y_size // 4
    flat = i420.reshape(-1)
    np.copyto(dst[:height].reshape(-1), flat[:y_size])
    uv = dst[height:].reshape(-1)
    uv[0::2] = flat[y_size : y_size + chroma_size]
    uv[1::2] = flat[y_size + chroma_size :]
    return dst


__all__ = [
    "align_nv12_crop",
    "bgr_to_nv12",
    "center_bottom_nv12_region",
    "extract_nv12_crop",
    "nv12_crop_to_bgr",
    "nv12_shape",
    "nv12_to_bgr",
    "nv12_to_rgb",
    "pack_nv12_from_buffer",
    "validate_nv12",
    "y_plane",
]
