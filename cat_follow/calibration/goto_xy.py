"""
Goto motion logic used when calibrating the car's movement on a goto command.

This module is for calibration runs (e.g. testing how the car drives toward a target
when it receives a goto command). The main loop uses motion/goto_xy.py for actual
operation; use this module when calibrating goto behavior (speed, steering, arrival).
"""

import math
from typing import Tuple

def compute_goto(
    current_x: float,
    current_y: float,
    current_heading: float,
    target_x: float,
    target_y: float,
    calib
) -> Tuple[float, float, bool]:
    """
    Calculate steering and speed to drive toward target.
    
    Args:
        current_x, current_y: Current position (cm).
        current_heading: Current heading (degrees).
        target_x, target_y: Target position (cm).
        calib: Calibration object for limits.
        
    Returns:
        (steer_angle, speed, arrived)
        steer_angle: degrees (negative=left, positive=right).
        speed: motor speed value (0-100).
        arrived: True if within threshold distance.
    """
    dx = target_x - current_x
    dy = target_y - current_y
    dist = math.sqrt(dx * dx + dy * dy)

    # Arrival threshold (e.g. 10 cm)
    if dist < 10.0:
        return 0.0, 0.0, True

    # Calculate desired heading
    desired_heading = math.degrees(math.atan2(dy, dx))
    
    # Calculate heading error (shortest path)
    error = desired_heading - current_heading
    while error > 180: error -= 360
    while error < -180: error += 360

    # Proportional control for steering
    # Clamp to max steering angle
    max_steer = calib.get_max_steer_angle_deg()
    steer = max(-max_steer, min(max_steer, error))

    # Speed control: slow down if turning sharply or close to target
    base_speed = 40
    if abs(error) > 20:
        speed = 30
    elif dist < 20:
        speed = 25
    else:
        speed = base_speed

    return steer, speed, False