"""
Hyperparameters for all vision / detection components.

All tunable numbers live here so they can be changed in one place without
touching business logic in detector.py, config.py, or any other module.
"""
from typing import Tuple

# ──────────────────────────────────────────────────────────────
# HSV colour thresholds  (hue 0-179, sat 0-255, val 0-255)
# ──────────────────────────────────────────────────────────────
""" blue
HSV_LO: Tuple[int, int, int] = (105, 120, 50)
HSV_HI: Tuple[int, int, int] = (123, 255, 255)
"""
# red upper range
HSV_LO1: Tuple[int, int, int] = (0, 150, 50)
HSV_HI1: Tuple[int, int, int] = (5, 255, 255)

# red lower range
# deep red / burgundy
HSV_LO2: Tuple[int, int, int] = (170, 150, 50)
HSV_HI2: Tuple[int, int, int] = (180, 255, 255)
# ──────────────────────────────────────────────────────────────
# Detection filtering
# ──────────────────────────────────────────────────────────────
MIN_AREA: int = 200           # minimum contour area [px²] to consider a hit
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
# Motor driver (L298N) GPIO pins  [BCM numbering]
# ──────────────────────────────────────────────────────────────
MOTOR_ENA: int = 18   # Motor A PWM enable  → Physical Pin 12
MOTOR_IN1: int = 17   # Motor A direction 1 → Physical Pin 11
MOTOR_IN2: int = 27   # Motor A direction 2 → Physical Pin 13
MOTOR_ENB: int = 13   # Motor B PWM enable  → Physical Pin 33
MOTOR_IN3: int = 23   # Motor B direction 1 → Physical Pin 16
MOTOR_IN4: int = 24   # Motor B direction 2 → Physical Pin 18
MOTOR_PWM_FREQUENCY_HZ: int = 1000  # PWM frequency for motor speed control

# ──────────────────────────────────────────────────────────────
# IMU — MPU-6050  (I2C bus 1, SDA=GPIO2/Pin3, SCL=GPIO3/Pin5)
# ──────────────────────────────────────────────────────────────
IMU_I2C_BUS: int = 1       # /dev/i2c-1
IMU_I2C_ADDR: int = 0x68   # AD0 pulled LOW; use 0x69 if AD0 is HIGH
IMU_ACCEL_SCALE: float = 16384.0   # LSB/g at ±2 g full-scale range
IMU_ACCEL_MS2: float = 9806.6      # 1 g in mm/s²
IMU_NOISE_THRESHOLD: float = 400.0 # mm/s² below which motion is ignored
IMU_STILL_COUNT_LIMIT: int = 5     # consecutive still samples before zeroing velocity
IMU_CALIBRATION_SAMPLES: int = 1000

# ──────────────────────────────────────────────────────────────
# Servo PWM
# ──────────────────────────────────────────────────────────────
SERVO_PWM_CHANNEL: int = 0
SERVO_PWM_CHIP: int = 0
SERVO_MIN_US: int = 590       # pulse width [µs] at 0°  (lowered from 800 – servo was sitting ~15° high at 0°)
SERVO_MAX_US: int = 2500      # pulse width [µs] at 180°
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
SCAN_SETTLE_S: float = 0.5      # wait time after each servo move [s]
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
TARGET_LOCK_ITERATIONS: int = 5        # lock-center correction steps before returning angle

# Maximum allowed distance from the center in pixels for object alignment
ALIGNMENT_MAX_OFFSET_PX: int = 50

# Maximum average offset allowed between the object and the center [px]
ALIGNMENT_AVERAGE_THRESHOLD_PX: int = 30
