import pytest

from cat_follow.perception.h264_encoder import MppH264Encoder


def test_nv12_pipeline_avoids_videoconvert():
    encoder = MppH264Encoder(640, 480, fps=15, pixel_format="NV12")
    pipeline = encoder._pipeline_description()

    assert "format=NV12" in pipeline
    assert "videoconvert" not in pipeline
    assert "mpph264enc" in pipeline


def test_bgr_compatibility_pipeline_keeps_videoconvert():
    encoder = MppH264Encoder(640, 480, pixel_format="BGR")
    pipeline = encoder._pipeline_description()

    assert "format=BGR" in pipeline
    assert "videoconvert" in pipeline


def test_h264_encoder_rejects_unknown_input_format():
    with pytest.raises(ValueError, match="expected NV12 or BGR"):
        MppH264Encoder(640, 480, pixel_format="RGB")
