"""
Central place for default constants and configuration dataclasses.

Import from here rather than hard-coding values spread across modules.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

import hparams as HP


# ──────────────────────────────────────────────────────────────
# Camera
# ──────────────────────────────────────────────────────────────

@dataclass
class CameraConfig:
    width: int = HP.CAMERA_WIDTH
    height: int = HP.CAMERA_HEIGHT
    fps: int = HP.CAMERA_FPS
    cam0_index: int = HP.CAMERA_CAM0_INDEX
    cam1_index: int = HP.CAMERA_CAM1_INDEX


# ──────────────────────────────────────────────────────────────
# Servo
# ──────────────────────────────────────────────────────────────

@dataclass
class ServoConfig:
    pwm_channel: int = HP.SERVO_PWM_CHANNEL
    pwm_chip: int = HP.SERVO_PWM_CHIP
    min_us: int = HP.SERVO_MIN_US
    max_us: int = HP.SERVO_MAX_US


# ──────────────────────────────────────────────────────────────
# Detection / HSV
# ──────────────────────────────────────────────────────────────

@dataclass
class DetectionConfig:
    hsv_lo: Tuple[int, int, int] = HP.HSV_LO
    hsv_hi: Tuple[int, int, int] = HP.HSV_HI
    min_area: int = HP.MIN_AREA
    downscale_width: int = HP.DOWNSCALE_WIDTH


# ──────────────────────────────────────────────────────────────
# Stereo distance
# ──────────────────────────────────────────────────────────────

@dataclass
class StereoConfig:
    baseline_m: float = HP.STEREO_BASELINE_M
    fx_px: float = HP.STEREO_FX_PX
    min_disparity_px: float = HP.STEREO_MIN_DISPARITY_PX
    max_vertical_offset_px: float = HP.STEREO_MAX_VERTICAL_OFFSET_PX


# ──────────────────────────────────────────────────────────────
# Scan behaviour
# ──────────────────────────────────────────────────────────────

@dataclass
class ScanConfig:
    step_deg: int = HP.SCAN_STEP_DEG
    settle_s: float = HP.SCAN_SETTLE_S
    edge_margin_px: int = HP.SCAN_EDGE_MARGIN_PX
    outdir: str = HP.SCAN_OUTDIR
    view: bool = False
    target: Optional[str] = None   # "Cube" | "Cylinder" | "Pyramid" | None


# ──────────────────────────────────────────────────────────────
# Motor driver + IMU
# ──────────────────────────────────────────────────────────────

@dataclass
class MotorConfig:
    ena: int = HP.MOTOR_ENA
    in1: int = HP.MOTOR_IN1
    in2: int = HP.MOTOR_IN2
    enb: int = HP.MOTOR_ENB
    in3: int = HP.MOTOR_IN3
    in4: int = HP.MOTOR_IN4
    pwm_frequency_hz: int = HP.MOTOR_PWM_FREQUENCY_HZ
    imu_bus: int = HP.IMU_I2C_BUS
    imu_addr: int = HP.IMU_I2C_ADDR
    imu_accel_scale: float = HP.IMU_ACCEL_SCALE
    imu_accel_ms2: float = HP.IMU_ACCEL_MS2
    imu_noise_threshold: float = HP.IMU_NOISE_THRESHOLD
    imu_still_count_limit: int = HP.IMU_STILL_COUNT_LIMIT
    imu_calibration_samples: int = HP.IMU_CALIBRATION_SAMPLES
    drive_speed: float = HP.MOTOR_DRIVE_SPEED


# ──────────────────────────────────────────────────────────────
# Convenience: bundle all configs together
# ──────────────────────────────────────────────────────────────

@dataclass
class AppConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    servo: ServoConfig = field(default_factory=ServoConfig)
    detection: DetectionConfig = field(default_factory=DetectionConfig)
    stereo: StereoConfig = field(default_factory=StereoConfig)
    scan: ScanConfig = field(default_factory=ScanConfig)
    motor: MotorConfig = field(default_factory=MotorConfig)
