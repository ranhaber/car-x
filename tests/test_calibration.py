"""
Test calibration loader. Run from car-x: python -m pytest tests/test_calibration.py -v
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cat_follow.calibration import Calibration


def test_cm_per_sec():
    calib = Calibration()
    # Default JSON has 30->12, 50->22, 70->32
    assert calib.get_cm_per_sec(30) == 12.0
    assert calib.get_cm_per_sec(50) == 22.0
    assert 21 < calib.get_cm_per_sec(45) < 23


def test_max_steer():
    calib = Calibration()
    assert calib.get_max_steer_angle_deg() == 25.0


def test_target_distance():
    calib = Calibration()
    assert calib.get_target_distance_cm() == 15.0
