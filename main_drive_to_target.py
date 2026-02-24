#!/usr/bin/env python3
"""
Entry point: scan for a red target, then physically drive to it.

Workflow
--------
1. Run a stereo-vision sweep to find the target (angle + distance).
2. Hand the result to :class:`Navigator` which drives the robot there.

Usage:
    python main_drive_to_target.py --target Cube
    python main_drive_to_target.py               # any red object
"""
from __future__ import annotations

from main import build_arg_parser, build_config
from motor import IMUReader, MotorController, Navigator
from scanner import Scanner


def integrated_system() -> None:
    """
    Full pipeline: scan → detect → drive.

    Parses the same CLI flags as ``main.py`` so the user can set target
    class, camera resolution, servo range, etc.
    """
    # ── Build config from CLI ────────────────────────────────────────────────
    parser = build_arg_parser()
    args   = parser.parse_args()
    config = build_config(args)

    # ── Scan for the target ──────────────────────────────────────────────────
    print("[Drive] Starting scan...")
    scanner = Scanner(config)
    result  = scanner.run()

    if result is None:
        print("[Drive] No object detected. Stopping.")
        return

    angle       = result.angle_deg
    distance_cm = result.distance_cm

    print(f"[Drive] Target found — angle={angle}°, distance={distance_cm} cm")

    # ── Drive to the target ──────────────────────────────────────────────────
    if distance_cm is None or distance_cm <= 2.5:
        print("[Drive] Distance too short or unavailable — not moving.")
        return

    motor     = MotorController()
    imu       = IMUReader()
    navigator = Navigator(motor, imu)

    try:
        # Approach to within 2.5 cm of the object surface
        navigator.move_by_angle_and_distance(angle, distance_cm - 2.5)
    except KeyboardInterrupt:
        print("[Drive] Interrupted by user.")
    finally:
        motor.stop()
        motor.cleanup()


if __name__ == "__main__":
    try:
        integrated_system()
    except KeyboardInterrupt:
        print("[Drive] Stopping.")
    finally:
        print("[Drive] Cleaning up and exiting.")
