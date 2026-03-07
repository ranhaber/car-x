"""
Functions to physically run the car for calibration measurements.
These are called by the web UI and command-line scripts.
"""
import time

try:
    from picarx import Picarx
except ImportError:
    print("WARN: 'picarx' module not found. Using mock for PC testing.")

    class Picarx:
        """Mock Picarx class for running on a PC without the hardware."""
        def set_dir_servo_angle(self, angle):
            print(f"MOCK: Set steer angle to {angle}")
        def forward(self, speed):
            print(f"MOCK: Move forward at speed {speed}")
        def stop(self):
            print("MOCK: Stop")

def run_speed_test(px: Picarx, speed: int, duration: float = 1.0):
    """Drive the car straight for a fixed duration to measure speed."""
    print(f"CALIBRATION: Driving forward for {duration}s at speed {speed}...")
    px.set_dir_servo_angle(0)
    px.forward(speed)
    time.sleep(duration)
    px.stop()
    print("CALIBRATION: Speed test finished.")

def run_steer_test(px: Picarx, angle: int, speed: int = 30, duration: float = 4.0):
    """Drive the car in a circle to measure turning radius."""
    print(f"CALIBRATION: Driving with steering {angle} at speed {speed} for {duration}s...")
    px.set_dir_servo_angle(angle)
    px.forward(speed)
    time.sleep(duration)
    px.stop()
    # Reset steering to neutral after the test
    px.set_dir_servo_angle(0)
    print("CALIBRATION: Steer test finished.")