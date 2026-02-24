"""
Hyperparameters for all vision / detection components.

All tunable numbers live here so they can be changed in one place without
touching business logic in detector.py, config.py, or any other module.
"""
from typing import Dict, Tuple

# ──────────────────────────────────────────────────────────────
# Colour selection  (hue 0-179, sat 0-255, val 0-255)
# Change DETECT_COLOUR to switch the target colour globally.
# For single-hue colours lo2/hi2 mirrors lo1/hi1 (OR is a no-op).
# ──────────────────────────────────────────────────────────────
DETECT_COLOUR: str = "red"   # "red" | "blue" | "green" | "yellow"

_COLOUR_PRESETS: Dict[str, Dict[str, Tuple[int, int, int]]] = {
    "red": {                              # wraps around the 0°/180° boundary
        "hsv_lo1": (0,   60,  40),
        "hsv_hi1": (12,  255, 255),
        "hsv_lo2": (165, 60,  40),
        "hsv_hi2": (180, 255, 255),
    },
    "blue": {
        "hsv_lo1": (100, 80,  50),
        "hsv_hi1": (130, 255, 255),
        "hsv_lo2": (100, 80,  50),        # same — blue is a single hue range
        "hsv_hi2": (130, 255, 255),
    },
    "green": {
        "hsv_lo1": (40,  60,  40),
        "hsv_hi1": (85,  255, 255),
        "hsv_lo2": (40,  60,  40),
        "hsv_hi2": (85,  255, 255),
    },
    "yellow": {
        "hsv_lo1": (20,  80,  80),
        "hsv_hi1": (35,  255, 255),
        "hsv_lo2": (20,  80,  80),
        "hsv_hi2": (35,  255, 255),
    },
}

_preset = _COLOUR_PRESETS[DETECT_COLOUR]
HSV_LO1: Tuple[int, int, int] = _preset["hsv_lo1"]
HSV_HI1: Tuple[int, int, int] = _preset["hsv_hi1"]
HSV_LO2: Tuple[int, int, int] = _preset["hsv_lo2"]
HSV_HI2: Tuple[int, int, int] = _preset["hsv_hi2"]
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
GYRO_SENSITIVITY: float = 131.0    # LSB / (°/s) at ±250 °/s full-scale range
GYRO_RATE_DEADBAND: float = 0.5    # °/s below which gyro rate is treated as zero (noise floor)
GYRO_CORRECTION_GAIN: float = 1.5  # motor duty-cycle adjustment per °/s of rotation rate

# ──────────────────────────────────────────────────────────────
# Navigator — movement tuning
# ──────────────────────────────────────────────────────────────
NAV_DRIVE_SPEED: float = 100.0        # base motor duty-cycle [%] for straight driving
MOTOR_LEFT_TRIM: float = 50.0          # static offset for left motor [%]: positive = speed up left, negative = slow down left
NAV_ROTATE_SPEED: float = 45.0       # motor duty-cycle [%] while rotating
NAV_ROTATE_TOLERANCE_DEG: float = 2.0  # stop rotating when within this many degrees of target
NAV_STOP_MARGIN_MM: float = 15.0     # stop this many mm before target (overshoot compensation)
NAV_CLEARANCE_CM: float = 5.1        # forward clearance drive before rotating [cm]
NAV_LOOP_SLEEP_S: float = 0.01       # control loop sleep [s] (both drive and rotate loops)
NAV_SETTLE_S: float = 0.5            # pause after rotation before driving [s]

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

# Maximum allowed distance from the center in pixels for object alignment
ALIGNMENT_MAX_OFFSET_PX: int = 50

# Maximum average offset allowed between the object and the center [px]
ALIGNMENT_AVERAGE_THRESHOLD_PX: int = 30
