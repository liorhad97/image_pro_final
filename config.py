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
    hsv_lo: Tuple[int, int, int] = HP.HSV_LO1
    hsv_hi: Tuple[int, int, int] = HP.HSV_HI1
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
# Alignment (follow-mode)
# ──────────────────────────────────────────────────────────────

@dataclass
class AlignmentConfig:
    """Pixel-offset thresholds used by the tracker's alignment check."""

    max_offset_px: int = HP.ALIGNMENT_MAX_OFFSET_PX
    """Maximum single-camera offset from frame centre before realigning [px]."""

    average_threshold_px: int = HP.ALIGNMENT_AVERAGE_THRESHOLD_PX
    """Maximum average offset across both cameras before realigning [px]."""


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
    alignment: AlignmentConfig = field(default_factory=AlignmentConfig)
