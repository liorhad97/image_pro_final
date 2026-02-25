from typing import Tuple

# HSV is used instead of BGR for color detection because hue is stable under changing light conditions
# red wraps around hue 0 in the HSV wheel so two separate ranges are needed to cover the full red spectrum
# stereo distance formula is Z = (fx * baseline) / abs(xLeft - xRight)

HSV_LO1: Tuple[int, int, int] = (0, 150, 50)
HSV_HI1: Tuple[int, int, int] = (5, 255, 255)

HSV_LO2: Tuple[int, int, int] = (170, 150, 50)
HSV_HI2: Tuple[int, int, int] = (180, 255, 255)

MIN_AREA: int = 25
DOWNSCALE_WIDTH: int = 2328

MORPH_KERNEL_SIZE: Tuple[int, int] = (3, 3)
MORPH_CLOSE_ITER: int = 1
MORPH_OPEN_ITER: int = 1

SHOW_PREVIEW: bool = True
PREVIEW_MAX_WIDTH: int = 1920

CLASSIFY_DP_EPSILON: float = 0.02
CLASSIFY_PYRAMID_MAX_EXTENT: float = 0.58
CLASSIFY_PYRAMID_MAX_VERTICES: int = 6
CLASSIFY_CYLINDER_MIN_CIRCULARITY: float = 0.78
CLASSIFY_CYLINDER_MIN_VERTICES: int = 8

CAMERA_WIDTH: int = 2328
CAMERA_HEIGHT: int = 1748
CAMERA_FPS: int = 30
CAMERA_CAM0_INDEX: int = 0
CAMERA_CAM1_INDEX: int = 1
CAMERA_WARMUP_FRAMES: int = 8
CAMERA_STARTUP_SLEEP_S: float = 0.3
CAMERA_WARMUP_SLEEP_S: float = 0.01

MOTOR_ENA: int = 18
MOTOR_IN1: int = 17
MOTOR_IN2: int = 27
MOTOR_ENB: int = 13
MOTOR_IN3: int = 23
MOTOR_IN4: int = 24
MOTOR_PWM_FREQUENCY_HZ: int = 1000

IMU_I2C_BUS: int = 1
IMU_I2C_ADDR: int = 0x68
IMU_ACCEL_SCALE: float = 16384.0
IMU_ACCEL_MS2: float = 9806.6
IMU_NOISE_THRESHOLD: float = 400.0
IMU_STILL_COUNT_LIMIT: int = 5
IMU_CALIBRATION_SAMPLES: int = 1000

SERVO_PWM_CHANNEL: int = 0
SERVO_PWM_CHIP: int = 0
SERVO_MIN_US: int = 590
SERVO_MAX_US: int = 2500
SERVO_PWM_FREQUENCY_HZ: int = 50
SERVO_PERIOD_US: int = 20_000

STEREO_BASELINE_M: float = 0.075
STEREO_FX_PX: float = 1950.0
STEREO_MIN_DISPARITY_PX: float = 1.0
STEREO_MAX_VERTICAL_OFFSET_PX: float = 80.0

SCAN_STEP_DEG: int = 1
SCAN_SETTLE_S: float = 0.1
SCAN_EDGE_MARGIN_PX: int = 10
SCAN_OUTDIR: str = "outputs"
SCAN_CENTER_LEFT_FRAC: float = 0.35
SCAN_CENTER_RIGHT_FRAC: float = 0.65

TRACKER_DEADBAND_MIN_PX: float = 12.0
TRACKER_DEADBAND_FRAC: float = 0.02
TRACKER_KP_DEG: float = 8.0
TRACKER_MAX_STEP_DEG: float = 4.0
TRACKER_LOST_LIMIT: int = 20
TRACKER_STEP_SLEEP_S: float = 0.03
TRACKER_LOOP_SLEEP_S: float = 0.01

ALIGNMENT_MAX_OFFSET_PX: int = 50
ALIGNMENT_AVERAGE_THRESHOLD_PX: int = 30
