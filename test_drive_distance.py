#!/usr/bin/env python3
"""
Simple straight-line drive test.

Enter a distance in cm and the robot drives straight ahead that far,
using IMU dead-reckoning + gyro correction.

Usage:
    python test_drive_distance.py
"""
from __future__ import annotations

from motor import IMUReader, MotorController, Navigator


def main() -> None:
    motor     = MotorController()
    imu       = IMUReader()
    navigator = Navigator(motor, imu)

    try:
        distance_cm = float(input("Enter distance (cm): "))
        print("[IMU] Calibrating — keep the robot still...")
        off_x, off_y, off_gz = imu.calibrate()
        print(f"[Drive] Going straight {distance_cm} cm")
        navigator.drive_dist(distance_cm, off_x, off_y, off_gz)
        print("[Drive] Done.")
    except KeyboardInterrupt:
        print("\n[Drive] Interrupted.")
    finally:
        motor.stop()
        motor.cleanup()


if __name__ == "__main__":
    main()
