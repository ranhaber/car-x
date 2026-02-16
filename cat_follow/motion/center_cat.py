"""
Car centers cat in frame (camera straight): bbox + image size -> steer, speed.
Lateral: bbox center X vs image center X -> steering.
Distance: bbox area -> forward/back to approach or hold ~15 cm (uses calibration).
"""
from typing import Optional, Tuple

from . import driver
from . import limits


def center_cat_control(
    bbox: Tuple[float, float, float, float],  # x, y, w, h (pixels)
    image_width: int,
    image_height: int,
    calibration,
    target_distance_cm: float = 15.0,
    approach_speed: int = 40,
    dead_zone_px: float = 20.0,
) -> None:
    """
    Compute steer and forward/back so the cat stays in the middle of the frame.
    Calls driver.set_steer(), driver.forward() or driver.backward().
    """
    x, y, w, h = bbox
    cx_cat = x + w / 2
    cy_cat = y + h / 2
    cx_img = image_width / 2
    cy_img = image_height / 2

    # Lateral: steer so cx_cat -> cx_img
    error_x = cx_cat - cx_img
    # Normalize to degrees (tune gain as needed)
    steer_deg = error_x * 0.08  # pixels -> rough degrees
    steer_deg = limits.clamp_steer(steer_deg, calibration)
    driver.set_steer(steer_deg)

    # Distance: use bbox area -> distance (if calibration has it)
    area = w * h
    dist_cm = None
    if calibration is not None:
        dist_cm = calibration.get_distance_cm_from_bbox_area(area)
    if dist_cm is None:
        # No calibration: drive forward slowly when bbox is small (far), else stop
        if area < image_width * image_height * 0.1:
            driver.forward(limits.clamp_speed(approach_speed))
        else:
            driver.stop()
        return

    # We have distance estimate
    if dist_cm > target_distance_cm + 5:
        driver.forward(limits.clamp_speed(approach_speed))
    elif dist_cm < target_distance_cm - 5:
        driver.backward(limits.clamp_speed(approach_speed // 2))
    else:
        driver.stop()
