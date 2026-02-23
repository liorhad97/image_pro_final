"""MPU-6050 I2C accelerometer reading and calibration."""
from __future__ import annotations

import time
from typing import Tuple

import smbus2

from config import MotorConfig

_POWER_MGMT_REG = 0x6B
_ACCEL_X_REG = 0x3B
_ACCEL_Y_REG = 0x3D


class ImuSensor:
    """Reads XY acceleration from an MPU-6050 over I2C."""

    def __init__(self, config: MotorConfig) -> None:
        self._cfg = config
        self._bus = smbus2.SMBus(config.imu_bus)
        self._offset_x: float = 0.0
        self._offset_y: float = 0.0
        self._calibrated: bool = False

    def wake_up(self) -> None:
        """Exit sleep mode on the MPU-6050."""
        self._bus.write_byte_data(self._cfg.imu_addr, _POWER_MGMT_REG, 0x00)
        time.sleep(0.1)

    def calibrate(self, samples: int | None = None) -> None:
        """Measure XY offset while stationary."""
        n = samples if samples is not None else self._cfg.imu_calibration_samples
        print("Calibrating XY plane... do not move the car")
        sum_x, sum_y = 0.0, 0.0
        for _ in range(n):
            sum_x += self._read_raw(_ACCEL_X_REG)
            sum_y += self._read_raw(_ACCEL_Y_REG)
            time.sleep(0.002)
        self._offset_x = sum_x / n
        self._offset_y = sum_y / n
        self._calibrated = True

    def read_acceleration_mm_s2(self) -> Tuple[float, float]:
        """Return calibrated (acc_x, acc_y) in mm/s²."""
        raw_x = self._read_raw(_ACCEL_X_REG)
        raw_y = self._read_raw(_ACCEL_Y_REG)
        scale = self._cfg.imu_accel_scale
        gravity = self._cfg.imu_accel_ms2
        acc_x = ((raw_x - self._offset_x) / scale) * gravity
        acc_y = ((raw_y - self._offset_y) / scale) * gravity
        return acc_x, acc_y

    def close(self) -> None:
        """Close the I2C bus."""
        self._bus.close()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _read_raw(self, reg: int) -> int:
        h = self._bus.read_byte_data(self._cfg.imu_addr, reg)
        l = self._bus.read_byte_data(self._cfg.imu_addr, reg + 1)
        val = (h << 8) + l
        return val - 65536 if val > 32768 else val
