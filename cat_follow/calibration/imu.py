"""
IMU stub for future use.
Intended to wrap a hardware IMU (e.g. BNO055, MPU6050) to provide
heading and acceleration data for odometry fusion.
"""

class IMU:
    def __init__(self):
        self._connected = False
        # Future: Initialize I2C device here

    def is_connected(self) -> bool:
        return self._connected

    def read_heading(self) -> float:
        """Return heading in degrees (0-360) or None if not available."""
        # Future: Read from hardware
        return None

    def read_acceleration(self):
        """Return (ax, ay, az) in m/s^2 or None."""
        # Future: Read from hardware
        return None