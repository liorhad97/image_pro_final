"""
Hyperparameters for all vision / detection components.

All tunable numbers live here so they can be changed in one place without
touching business logic in detector.py, config.py, or any other module.
"""
from typing import Tuple

# ──────────────────────────────────────────────────────────────
# HSV colour thresholds  (hue 0-179, sat 0-255, val 0-255)
# ──────────────────────────────────────────────────────────────
HSV_LO: Tuple[int, int, int] = (105, 120, 50)
HSV_HI: Tuple[int, int, int] = (123, 255, 255)

# ──────────────────────────────────────────────────────────────
# Detection filtering
# ──────────────────────────────────────────────────────────────
MIN_AREA: int = 50            # minimum contour area [px²] to consider a hit
DOWNSCALE_WIDTH: int = 640    # resize frames wider than this before processing

# ──────────────────────────────────────────────────────────────
# Morphological cleanup
# ──────────────────────────────────────────────────────────────
MORPH_KERNEL_SIZE: Tuple[int, int] = (7, 7)
MORPH_CLOSE_ITER: int = 2
MORPH_OPEN_ITER: int = 1

# ──────────────────────────────────────────────────────────────
# Display / preview
# ──────────────────────────────────────────────────────────────
SHOW_PREVIEW: bool = True       # set True only when a display is available
PREVIEW_MAX_WIDTH: int = 1920   # max combined width of side-by-side preview [px]

# ──────────────────────────────────────────────────────────────
# Shape-classification thresholds
# ──────────────────────────────────────────────────────────────
# Extent  = blob area / bounding-box area  (1.0 = perfectly fills its box)
# Circularity = 4π·area / perimeter²        (1.0 = perfect circle)
CLASSIFY_DP_EPSILON: float = 0.02          # Douglas-Peucker approximation factor
CLASSIFY_PYRAMID_MAX_EXTENT: float = 0.58  # extent below this → Pyramid
CLASSIFY_PYRAMID_MAX_VERTICES: int = 6     # AND vertex count ≤ this → Pyramid
CLASSIFY_CYLINDER_MIN_CIRCULARITY: float = 0.78  # circularity above this → Cylinder
CLASSIFY_CYLINDER_MIN_VERTICES: int = 8    # OR vertex count ≥ this → Cylinder

# ──────────────────────────────────────────────────────────────
# Camera
# ──────────────────────────────────────────────────────────────
CAMERA_WIDTH: int = 2328
CAMERA_HEIGHT: int = 1748
CAMERA_FPS: int = 30
CAMERA_CAM0_INDEX: int = 0
CAMERA_CAM1_INDEX: int = 1
CAMERA_WARMUP_FRAMES: int = 8      # frames discarded during AE/AWB warmup
CAMERA_STARTUP_SLEEP_S: float = 0.3   # seconds to wait after starting cameras
CAMERA_WARMUP_SLEEP_S: float = 0.01   # seconds between warmup frame captures

# ──────────────────────────────────────────────────────────────
# Servo PWM
# ──────────────────────────────────────────────────────────────
SERVO_PWM_CHANNEL: int = 0
SERVO_PWM_CHIP: int = 0
SERVO_MIN_US: int = 1000      # pulse width [µs] at 0°
SERVO_MAX_US: int = 2000      # pulse width [µs] at 180°
SERVO_PWM_FREQUENCY_HZ: int = 50        # standard RC servo frequency
SERVO_PERIOD_US: int = 20_000           # 1 / 50 Hz = 20 ms = 20 000 µs

# ──────────────────────────────────────────────────────────────
# Stereo distance
# ──────────────────────────────────────────────────────────────
STEREO_BASELINE_M: float = 0.075        # camera centre-to-centre separation [m]
STEREO_FX_PX: float = 1893.0            # horizontal focal length [px] — calibrate!
STEREO_MIN_DISPARITY_PX: float = 2.0    # below this disparity, Z is unreliable
STEREO_MAX_VERTICAL_OFFSET_PX: float = 60.0  # max |yL-yR| for a valid estimate

# ──────────────────────────────────────────────────────────────
# Scan behaviour
# ──────────────────────────────────────────────────────────────
SCAN_STEP_DEG: int = 5           # servo step size per sweep position [degrees]
SCAN_SETTLE_S: float = 0.05      # wait time after each servo move [s]
SCAN_EDGE_MARGIN_PX: int = 10    # min pixels from frame edge for a valid bbox
SCAN_OUTDIR: str = "outputs"     # directory for saved detections

# ──────────────────────────────────────────────────────────────
# Target tracker (follow-mode)
# ──────────────────────────────────────────────────────────────
TRACKER_DEADBAND_MIN_PX: float = 12.0   # minimum deadband [px]
TRACKER_DEADBAND_FRAC: float = 0.02     # deadband as fraction of frame width
TRACKER_KP_DEG: float = 8.0            # proportional gain [degrees per unit error]
TRACKER_MAX_STEP_DEG: float = 4.0      # maximum pan step per update [degrees]
TRACKER_LOST_LIMIT: int = 20           # consecutive missed frames before giving up
TRACKER_STEP_SLEEP_S: float = 0.03     # sleep after a pan move [s]
TRACKER_LOOP_SLEEP_S: float = 0.01     # base loop sleep [s]
