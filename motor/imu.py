"""
MPU-6050 IMU reader via I2C (smbus2).

Reads raw 16-bit signed values from the accelerometer and gyroscope
registers, and provides a blocking calibration routine that must be
run while the robot is stationary.
"""
from __future__ import annotations

import time

import smbus2

import hparams as HP


class IMUReader:
    """
    Reads raw accelerometer and gyroscope data from an MPU-6050 over I2C.

    The sensor is woken from sleep immediately on construction.  Call
    :meth:`calibrate` before any movement to compute bias offsets.

    Parameters
    ----------
    bus_num : int
        I2C bus number (default 1 → /dev/i2c-1).
    addr : int
        7-bit I2C address (default 0x68, AD0 pulled LOW).
    """

    # ── MPU-6050 register addresses ──────────────────────────────────────────
    _REG_PWR_MGMT = 0x6B   # Power management (write 0 to wake from sleep)
    _REG_ACCEL_X  = 0x3B   # Accelerometer X-axis high byte
    _REG_ACCEL_Y  = 0x3D   # Accelerometer Y-axis high byte
    _REG_GYRO_Z   = 0x47   # Gyroscope Z-axis high byte

    def __init__(
        self,
        bus_num: int = HP.IMU_I2C_BUS,
        addr: int = HP.IMU_I2C_ADDR,
    ) -> None:
        self._addr = addr
        self._bus = smbus2.SMBus(bus_num)

        # Wake the sensor by clearing the sleep bit in PWR_MGMT_1
        self._bus.write_byte_data(addr, self._REG_PWR_MGMT, 0)
        time.sleep(0.1)

    # ------------------------------------------------------------------ public

    def read_raw(self, reg: int) -> int:
        """
        Read a signed 16-bit value from two consecutive registers.

        Parameters
        ----------
        reg : int
            Address of the high byte (low byte is at ``reg + 1``).

        Returns
        -------
        int
            Signed 16-bit sensor reading.
        """
        high = self._bus.read_byte_data(self._addr, reg)
        low  = self._bus.read_byte_data(self._addr, reg + 1)
        raw  = (high << 8) + low
        return raw - 65536 if raw > 32768 else raw

    def calibrate(self, samples: int = HP.IMU_CALIBRATION_SAMPLES):
        """
        Compute bias offsets for accelerometer X/Y and gyroscope Z.

        The robot **must remain stationary** for the full duration of
        calibration.

        Parameters
        ----------
        samples : int
            Number of samples to average (default 1000, ~2 s).

        Returns
        -------
        offset_ax, offset_ay, offset_gz : float
            Raw-unit bias offsets to subtract from future readings.
        """
        print(f"[IMU] Calibrating ({samples} samples) — keep the robot still...")
        sum_x, sum_y, sum_gz = 0, 0, 0
        for _ in range(samples):
            sum_x  += self.read_raw(self._REG_ACCEL_X)
            sum_y  += self.read_raw(self._REG_ACCEL_Y)
            sum_gz += self.read_raw(self._REG_GYRO_Z)
            time.sleep(0.002)
        return sum_x / samples, sum_y / samples, sum_gz / samples
