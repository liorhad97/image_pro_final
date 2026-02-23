"""
Central place for default constants and configuration dataclasses.

Import from here rather than hard-coding values spread across modules.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple


# ──────────────────────────────────────────────────────────────
# Camera
# ──────────────────────────────────────────────────────────────

@dataclass
class CameraConfig:
    width: int = 2328
    height: int = 1748
    fps: int = 30
    cam0_index: int = 0
    cam1_index: int = 1


# ──────────────────────────────────────────────────────────────
# Servo
# ──────────────────────────────────────────────────────────────

@dataclass
class ServoConfig:
    pwm_channel: int = 0
    pwm_chip: int = 0
    min_us: int = 1000
    max_us: int = 2000


# ──────────────────────────────────────────────────────────────
# Detection / HSV
# ──────────────────────────────────────────────────────────────

@dataclass
class DetectionConfig:
    hsv_lo: Tuple[int, int, int] = (105, 120, 50)
    hsv_hi: Tuple[int, int, int] = (123, 255, 255)
    min_area: int = 50
    downscale_width: int = 640


# ──────────────────────────────────────────────────────────────
# Stereo distance
# ──────────────────────────────────────────────────────────────

@dataclass
class StereoConfig:
    baseline_m: float = 0.075    # camera centre-to-centre separation [m]
    fx_px: float = 1893.0        # horizontal focal length [px] — calibrate this!
    min_disparity_px: float = 2.0
    max_vertical_offset_px: float = 60.0


# ──────────────────────────────────────────────────────────────
# Scan behaviour
# ──────────────────────────────────────────────────────────────

@dataclass
class ScanConfig:
    step_deg: int = 5
    settle_s: float = 0.05
    edge_margin_px: int = 10
    outdir: str = "outputs"
    view: bool = False
    target: Optional[str] = None   # "Cube" | "Cylinder" | "Pyramid" | None


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
