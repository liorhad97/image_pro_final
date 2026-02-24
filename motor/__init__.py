"""
Motor control package.

Exposes three public classes:
- MotorController  — low-level L298N H-bridge / GPIO wrapper
- IMUReader        — MPU-6050 accelerometer + gyroscope reader via I2C
- Navigator        — high-level movement using motor + IMU feedback loops
"""
from .controller import MotorController
from .imu import IMUReader
from .navigator import Navigator

__all__ = ["MotorController", "IMUReader", "Navigator"]
