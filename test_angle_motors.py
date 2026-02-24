#!/usr/bin/env python3
"""
Standalone test harness for the motor and IMU subsystems.

Run modes (interactive prompt):
  test   — rotate to a fixed set of test angles and print results
  normal — manually enter an angle (0–180°) and distance (cm) then drive

Usage:
    python test_angle_motors.py
"""
from __future__ import annotations

import time

from motor import IMUReader, MotorController, Navigator


def test_rotate_to_angle(navigator: Navigator, imu: IMUReader) -> None:
    """Rotate to each of several predefined angles and pause between them."""
    print("[Test] Calibrating IMU for rotation test...")
    _, _, off_gz = imu.calibrate()

    test_angles = [0, 45, 90, 135, 180]
    for angle in test_angles:
        print(f"[Test] Rotating to {angle}°")
        navigator.rotate_to_angle(angle, off_gz)
        time.sleep(1.0)

    # Extra check: full right then full left
    print("[Test] Rotating right (0°) then left (180°)")
    navigator.rotate_to_angle(0, off_gz)
    time.sleep(1.0)
    navigator.rotate_to_angle(180, off_gz)
    time.sleep(1.0)

    print("[Test] Done.")


def run_normal(navigator: Navigator) -> None:
    """Ask the user for an angle and distance, then drive there."""
    angle       = float(input("Enter angle (0–180°): "))
    distance_cm = float(input("Enter distance (cm): "))
    navigator.move_by_angle_and_distance(angle, distance_cm)


def main() -> None:
    motor     = MotorController()
    imu       = IMUReader()
    navigator = Navigator(motor, imu)

    try:
        mode = input("Choose mode (test / normal): ").strip().lower()
        if mode == "test":
            test_rotate_to_angle(navigator, imu)
        else:
            run_normal(navigator)
    except KeyboardInterrupt:
        print("\n[Test] Interrupted by user.")
    finally:
        motor.stop()
        motor.cleanup()


if __name__ == "__main__":
    main()
