import numpy as np
import pytest

from cat_follow.vision.nv12_utils import (
    align_nv12_crop,
    bgr_to_nv12,
    center_bottom_nv12_region,
    extract_nv12_crop,
    nv12_crop_to_bgr,
    nv12_shape,
    nv12_to_bgr,
    nv12_to_rgb,
    pack_nv12_from_buffer,
    y_plane,
)


def test_nv12_shape_requires_even_geometry():
    assert nv12_shape(640, 480) == (720, 640)
    with pytest.raises(ValueError):
        nv12_shape(641, 480)


def test_y_plane_is_zero_copy_view():
    frame = np.zeros(nv12_shape(8, 4), dtype=np.uint8)
    luma = y_plane(frame, 8, 4)
    luma[1, 2] = 37
    assert luma.shape == (4, 8)
    assert np.shares_memory(luma, frame)
    assert frame[1, 2] == 37


def test_align_nv12_crop_clamps_and_even_aligns():
    assert align_nv12_crop(161, 159, 321, 321, 640, 480) == (
        160,
        158,
        320,
        320,
    )


def test_extract_nv12_crop_maps_luma_and_chroma_rows():
    frame = np.arange(12 * 8, dtype=np.uint8).reshape(12, 8)
    crop = extract_nv12_crop(frame, 8, 8, (2, 2, 4, 4))

    assert crop.shape == (6, 4)
    assert np.array_equal(crop[:4], frame[2:6, 2:6])
    assert np.array_equal(crop[4:], frame[9:11, 2:6])


def test_extract_nv12_crop_reuses_destination():
    frame = np.zeros(nv12_shape(8, 8), dtype=np.uint8)
    dst = np.empty(nv12_shape(4, 4), dtype=np.uint8)
    result = extract_nv12_crop(frame, 8, 8, (0, 0, 4, 4), dst=dst)
    assert result is dst


def test_pack_nv12_from_padded_gstreamer_buffer():
    width, height = 4, 4
    y_stride = 8
    uv_stride = 8
    uv_offset = y_stride * height
    mapped = np.full(uv_offset + uv_stride * (height // 2), 255, dtype=np.uint8)
    for row in range(height):
        mapped[row * y_stride : row * y_stride + width] = row + 1
    for row in range(height // 2):
        start = uv_offset + row * uv_stride
        mapped[start : start + width] = row + 10

    packed = pack_nv12_from_buffer(
        mapped, width, height, y_stride, uv_stride, uv_offset
    )

    assert packed.shape == (6, 4)
    assert np.array_equal(packed[:height, 0], [1, 2, 3, 4])
    assert np.array_equal(packed[height:, 0], [10, 11])
    assert 255 not in packed


def test_pack_nv12_rejects_stride_shorter_than_width():
    width, height = 4, 4
    mapped = np.zeros(64, dtype=np.uint8)
    with pytest.raises(ValueError, match="strides must be >= width"):
        pack_nv12_from_buffer(mapped, width, height, 3, 4, 16)


def test_pack_nv12_rejects_layout_beyond_mapped_size():
    width, height = 4, 4
    y_stride = 8
    uv_stride = 8
    uv_offset = y_stride * height
    mapped = np.zeros(uv_offset + uv_stride * (height // 2), dtype=np.uint8)
    with pytest.raises(ValueError, match="mapped region is"):
        pack_nv12_from_buffer(
            mapped,
            width,
            height,
            y_stride,
            uv_stride,
            uv_offset,
            mapped_size=30,
        )


def test_center_bottom_region_matches_production_geometry():
    assert center_bottom_nv12_region(640, 480, 320, 320) == (160, 160, 320, 320)


def test_center_bottom_crop_bgr_matches_full_frame_slice():
    pytest.importorskip("cv2")
    frame_w, frame_h = 640, 480
    crop_w, crop_h = 320, 320
    bgr = np.zeros((frame_h, frame_w, 3), dtype=np.uint8)
    bgr[:, :, 0] = np.linspace(0, 255, frame_w, dtype=np.uint8)
    bgr[:, :, 1] = np.linspace(0, 255, frame_h, dtype=np.uint8)[:, None]
    bgr[:, :, 2] = ((np.arange(frame_w) + np.arange(frame_h)[:, None]) % 256).astype(
        np.uint8
    )

    full_nv12 = bgr_to_nv12(bgr)
    full_bgr = nv12_to_bgr(full_nv12, frame_w, frame_h)
    region = center_bottom_nv12_region(frame_w, frame_h, crop_w, crop_h)
    crop_bgr = nv12_crop_to_bgr(full_nv12, region, frame_w, frame_h)

    x, y, width, height = region
    np.testing.assert_array_equal(crop_bgr, full_bgr[y : y + height, x : x + width])


def test_align_nv12_crop_required_before_odd_region_extract():
    pytest.importorskip("cv2")
    frame_w, frame_h = 640, 480
    bgr = np.zeros((frame_h, frame_w, 3), dtype=np.uint8)
    bgr[::2, ::2, 0] = 255
    bgr[1::2, 1::2, 2] = 255
    full_nv12 = bgr_to_nv12(bgr)
    full_bgr = nv12_to_bgr(full_nv12, frame_w, frame_h)

    odd_request = (161, 161, 320, 320)
    aligned_region = align_nv12_crop(*odd_request, frame_w, frame_h)
    with pytest.raises(ValueError, match="even-aligned"):
        extract_nv12_crop(full_nv12, frame_w, frame_h, odd_request)

    crop_bgr = nv12_crop_to_bgr(full_nv12, aligned_region, frame_w, frame_h)
    x, y, width, height = aligned_region
    np.testing.assert_array_equal(
        crop_bgr, full_bgr[y : y + height, x : x + width]
    )
    assert aligned_region != odd_request


@pytest.mark.parametrize(
    "color",
    ((128, 128, 128), (255, 0, 0), (0, 255, 0), (0, 0, 255)),
)
def test_bgr_nv12_round_trip_preserves_chroma_order(color):
    bgr = np.full((8, 10, 3), color, dtype=np.uint8)
    nv12 = bgr_to_nv12(bgr)
    restored = nv12_to_bgr(nv12, 10, 8)

    assert nv12.shape == (12, 10)
    assert restored.shape == bgr.shape
    assert np.max(np.abs(restored.astype(np.int16) - bgr.astype(np.int16))) <= 2


def test_nv12_to_rgb_matches_bgr_channel_swap_and_reuses_destination():
    cv2 = pytest.importorskip("cv2")
    bgr = np.zeros((8, 10, 3), dtype=np.uint8)
    bgr[:, :, 0] = 31
    bgr[:, :, 1] = 127
    bgr[:, :, 2] = 219
    nv12 = bgr_to_nv12(bgr)
    expected = cv2.cvtColor(
        nv12_to_bgr(nv12, 10, 8), cv2.COLOR_BGR2RGB
    )
    dst = np.empty_like(bgr)

    result = nv12_to_rgb(nv12, 10, 8, dst=dst)

    assert result is dst
    np.testing.assert_array_equal(result, expected)
