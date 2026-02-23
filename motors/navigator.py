"""High-level navigator: drives straight to a measured distance target."""
from __future__ import annotations

import math
import time

from config import MotorConfig
from motors.motor_driver import MotorDriver
from motors.imu_sensor import ImuSensor


class Navigator:
    """Composes MotorDriver and ImuSensor to drive straight to a distance."""

    def __init__(self, config: MotorConfig) -> None:
        self._cfg = config
        self._motor = MotorDriver(config)
        self._imu = ImuSensor(config)

    def start(self) -> None:
        """Wake IMU, calibrate, and prepare motors."""
        self._imu.wake_up()
        self._imu.calibrate()
        self._motor.start()

    def drive_straight_mm(self, target_mm: float, speed: float | None = None) -> None:
        """Drive forward until the IMU reports >= *target_mm* of travel."""
        if speed is None:
            speed = self._cfg.drive_speed

        vel_x, vel_y = 0.0, 0.0
        dist_x, dist_y = 0.0, 0.0
        still_count = 0
        last_t = time.time()

        try:
            print(f"[Navigator] Driving to {target_mm:.1f} mm at speed {speed:.0f}%")
            self._motor.drive_forward(speed)

            while math.sqrt(dist_x ** 2 + dist_y ** 2) < target_mm:
                now = time.time()
                dt = now - last_t
                last_t = now
                if dt <= 0:
                    continue

                acc_x, acc_y = self._imu.read_acceleration_mm_s2()
                threshold = self._cfg.imu_noise_threshold

                if abs(acc_x) < threshold and abs(acc_y) < threshold:
                    still_count += 1
                    if still_count > self._cfg.imu_still_count_limit:
                        vel_x, vel_y = 0.0, 0.0
                else:
                    still_count = 0
                    vel_x += acc_x * dt
                    vel_y += acc_y * dt

                dist_x += vel_x * dt
                dist_y += vel_y * dt
                total_dist = math.sqrt(dist_x ** 2 + dist_y ** 2)

                print(
                    f"Current distance: {total_dist:5.1f} / {target_mm:.1f} mm",
                    end="\r",
                )

            total_dist = math.sqrt(dist_x ** 2 + dist_y ** 2)
            print(f"\n[Navigator] Target reached ({total_dist:.1f} mm). Stopping.")
            self._motor.stop()
            time.sleep(1)

        finally:
            self._motor.stop()

    def cleanup(self) -> None:
        """Stop motors, close I2C, release GPIO."""
        self._motor.cleanup()
        self._imu.close()
