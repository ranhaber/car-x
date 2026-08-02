from scripts.compare_zerocopy_vs_numpy import _iou, _parity_failures


def test_parity_thresholds_accept_equal_counts_and_boundary_iou():
    assert (
        _parity_failures(
            frame_count=3,
            count_delta=1,
            top_box_iou_p50=0.9,
            max_count_delta=1,
            min_top_box_iou=0.9,
        )
        == []
    )


def test_parity_thresholds_report_each_failed_gate():
    failures = _parity_failures(
        frame_count=2,
        count_delta=2,
        top_box_iou_p50=0.75,
        max_count_delta=1,
        min_top_box_iou=0.8,
    )

    assert len(failures) == 2
    assert failures[0].startswith("detection_count_delta")
    assert failures[1].startswith("top_box_iou_p50")


def test_parity_requires_at_least_one_compared_frame():
    assert _parity_failures(
        frame_count=0,
        count_delta=0,
        top_box_iou_p50=None,
        max_count_delta=0,
        min_top_box_iou=0.9,
    ) == ["no frames were compared"]


def test_iou_for_identical_boxes_is_one():
    assert _iou((1, 2, 11, 12), (1, 2, 11, 12)) == 1.0
