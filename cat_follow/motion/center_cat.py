"""
Car centers cat in frame (camera straight): bbox + image size -> steer, speed.
Lateral: bbox center X vs image center X -> steering.
Distance: ultrasonic only (no bbox-based fallback). Forward/back to hold target_distance_cm.
"""
from typing import Tuple

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
    Distance is from ultrasonic only; if no reading, we only steer and stop (no forward/back).
    """
    x, y, w, h = bbox
    cx_cat = x + w / 2
    cy_cat = y + h / 2
    cx_img = image_width / 2
    cy_img = image_height / 2

    # Lateral: steer so cx_cat -> cx_img
    error_x = cx_cat - cx_img
    steer_deg = error_x * 0.08
    steer_deg = limits.clamp_steer(steer_deg, calibration)
    driver.set_steer(steer_deg)

    # Distance: ultrasonic only (no bbox fallback)
    try:
        from cat_follow import range_sensor
        dist_cm = range_sensor.get_distance_cm()
    except Exception:
        dist_cm = None
    if dist_cm is None:
        driver.stop()
        return
    if dist_cm > target_distance_cm + 5:
        driver.forward(limits.clamp_speed(approach_speed))
    elif dist_cm < target_distance_cm - 5:
        driver.backward(limits.clamp_speed(approach_speed // 2))
    else:
        driver.stop()
