#!/usr/bin/env python3
"""
BNO085 orientation example for Raspberry Pi (PiCar-X / Robot HAT).
Reads quaternion and Euler angles (roll, pitch, yaw) for orientation.

Hardware: BNO085 on Robot HAT I2C/QWIIC port.
Setup:   sudo pip3 install adafruit-blinka adafruit-circuitpython-bno08x
         Add dtparam=i2c_arm_baudrate=400000 to /boot/config.txt and reboot.
"""

import math
import time

try:
    import board
    import busio
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
except ImportError:
    print("Install: sudo pip3 install adafruit-blinka adafruit-circuitpython-bno08x")
    raise


def quaternion_to_euler_degrees(quat_i, quat_j, quat_k, quat_real):
    """Convert quaternion (i, j, k, real) to Euler (roll, pitch, yaw) in degrees."""
    x, y, z, w = quat_i, quat_j, quat_k, quat_real
    # Roll (x-axis)
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    # Pitch (y-axis)
    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)
    # Yaw (z-axis) — heading
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    print("BNO085 orientation (Ctrl+C to stop)")
    print("Heading (yaw): 0 = forward, +90 = left, -90 = right, ±180 = back")
    print("-" * 60)

    while True:
        time.sleep(0.05)
        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        roll, pitch, yaw = quaternion_to_euler_degrees(quat_i, quat_j, quat_k, quat_real)
        print(
            "Roll: %6.1f°  Pitch: %6.1f°  Yaw (heading): %6.1f°  "
            "| Quat I: %+.3f J: %+.3f K: %+.3f R: %+.3f"
            % (roll, pitch, yaw, quat_i, quat_j, quat_k, quat_real)
        )


if __name__ == "__main__":
    main()
