import math
import time
from typing import Optional, Tuple

import RPi.GPIO as GPIO
import smbus2

import hparams as HP
from detection.detector import RedObjectDetector
from servo.controller import ServoController
from stereo.camera import StereoCams
from stereo.distance import StereoDistanceEstimator

CAMERA_SERVO_CENTER_DEG = 90.0
CAMERA_SERVO_TURN_OFFSET_DEG = 20.0
DRIVE_BASE_SPEED = 50.0
DRIVE_YAW_GAIN = 1.8
DRIVE_LOOP_SLEEP_S = 0.01
VISION_STEER_GAIN = 18.0
VISION_MAX_STEER = 16.0
VISION_POLL_INTERVAL_S = 0.08

# --- Your Motor Setup ---
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
ENA, IN1, IN2 =  13, 23, 24
ENB, IN3, IN4 =  18, 17, 27  # Update ENB to a valid value

pins = [ENA, IN1, IN2, ENB, IN3, IN4]
for pin in pins: GPIO.setup(pin, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0); pwm_b.start(0)


def set_motors(speed_l, speed_r, direction="forward"):
    if direction == "forward":
        GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
    elif direction == "rotate_left":
        GPIO.output(IN1, GPIO.LOW);  GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
    elif direction == "rotate_right":
        GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(max(0, min(100, speed_l)))
    pwm_b.ChangeDutyCycle(max(0, min(100, speed_r)))


def stop_motors():
    pwm_a.ChangeDutyCycle(1); pwm_b.ChangeDutyCycle(1)
    for p in [IN1, IN2, IN3, IN4]: GPIO.output(p, GPIO.LOW)


# --- IMU Setup and Full Calibration ---
bus = smbus2.SMBus(1)
ADDR = 0x68
bus.write_byte_data(ADDR, 0x6B, 0)
time.sleep(0.1)


def read_raw(reg):
    h = bus.read_byte_data(ADDR, reg)
    l = bus.read_byte_data(ADDR, reg + 1)
    val = (h << 8) + l
    return val - 65536 if val > 32768 else val


# Full calibration of accelerometer (X,Y,Z) and gyroscope (Z)
def calibrate_all(samples=1000):
    print("Full calibration (X, Y, GZ)... Do not move the car")
    sum_x, sum_y, sum_gz = 0, 0, 0
    for _ in range(samples):
        sum_x += read_raw(0x3B) # Accel X
        sum_y += read_raw(0x3D) # Accel Y
        sum_gz += read_raw(0x47) # Gyro Z
        time.sleep(0.002)
    return sum_x/samples, sum_y/samples, sum_gz/samples


class DriveVisionTracker:
    """Provides steering correction + live stereo distance while driving."""

    def __init__(self, target_cls: str) -> None:
        self._target_cls = target_cls
        self._frame_mid_x = HP.CAMERA_WIDTH / 2.0
        self._detector = RedObjectDetector(
            hsv_lo1=HP.HSV_LO1,
            hsv_hi1=HP.HSV_HI1,
            hsv_lo2=HP.HSV_LO2,
            hsv_hi2=HP.HSV_HI2,
            min_area=HP.MIN_AREA,
            downscale_width=HP.DOWNSCALE_WIDTH,
        )
        self._distance_estimator = StereoDistanceEstimator(
            fx_px=HP.STEREO_FX_PX,
            baseline_m=HP.STEREO_BASELINE_M,
            min_disparity_px=HP.STEREO_MIN_DISPARITY_PX,
            max_vertical_offset_px=HP.STEREO_MAX_VERTICAL_OFFSET_PX,
        )
        self._cams = StereoCams(
            cam0=HP.CAMERA_CAM0_INDEX,
            cam1=HP.CAMERA_CAM1_INDEX,
            size=(HP.CAMERA_WIDTH, HP.CAMERA_HEIGHT),
            fps=HP.CAMERA_FPS,
        )
        self._started = False
        self._last_poll_ts = 0.0
        self._last_steer = 0.0
        self._last_distance_cm: Optional[float] = None

    def start(self) -> None:
        if self._started:
            return
        self._cams.start()
        self._started = True

    def stop(self) -> None:
        if not self._started:
            return
        self._cams.stop()
        self._started = False

    def poll(self) -> Tuple[float, Optional[float]]:
        if not self._started:
            return 0.0, None

        now = time.time()
        if (now - self._last_poll_ts) < VISION_POLL_INTERVAL_S:
            return self._last_steer, self._last_distance_cm

        self._last_poll_ts = now

        frame_left, frame_right = self._cams.capture_bgr()
        det_left = self._detector.detect(frame_left)
        det_right = self._detector.detect(frame_right)

        centers_x = []
        if det_left.is_valid and det_left.cls == self._target_cls and det_left.center is not None:
            centers_x.append(float(det_left.center[0]))
        if det_right.is_valid and det_right.cls == self._target_cls and det_right.center is not None:
            centers_x.append(float(det_right.center[0]))

        steer_correction = 0.0
        if centers_x:
            error_px = (sum(centers_x) / len(centers_x)) - self._frame_mid_x
            steer_correction = -(error_px / self._frame_mid_x) * VISION_STEER_GAIN
            steer_correction = max(-VISION_MAX_STEER, min(VISION_MAX_STEER, steer_correction))

        same_target = (
            det_left.is_valid
            and det_right.is_valid
            and det_left.cls == self._target_cls
            and det_right.cls == self._target_cls
        )
        if same_target:
            dist_result = self._distance_estimator.estimate(det_left, det_right)
            if dist_result.is_valid and dist_result.distance_cm is not None:
                self._last_distance_cm = dist_result.distance_cm

        self._last_steer = steer_correction
        return self._last_steer, self._last_distance_cm


def _center_camera_servo(camera_servo: Optional[ServoController]) -> None:
    if camera_servo is not None:
        camera_servo.set_angle(CAMERA_SERVO_CENTER_DEG)


def _align_servo_for_rotation(
    camera_servo: Optional[ServoController],
    direction: str,
) -> None:
    if camera_servo is None:
        return
    if direction == "rotate_left":
        camera_servo.set_angle(CAMERA_SERVO_CENTER_DEG + CAMERA_SERVO_TURN_OFFSET_DEG)
    elif direction == "rotate_right":
        camera_servo.set_angle(CAMERA_SERVO_CENTER_DEG - CAMERA_SERVO_TURN_OFFSET_DEG)


def _align_servo_with_wheels(
    camera_servo: Optional[ServoController],
    speed_l: float,
    speed_r: float,
) -> None:
    if camera_servo is None:
        return

    wheel_delta = speed_r - speed_l
    if abs(wheel_delta) < 0.75:
        return

    if wheel_delta > 0:
        camera_servo.set_angle(CAMERA_SERVO_CENTER_DEG + CAMERA_SERVO_TURN_OFFSET_DEG)
    else:
        camera_servo.set_angle(CAMERA_SERVO_CENTER_DEG - CAMERA_SERVO_TURN_OFFSET_DEG)


# --- Movement Functions ---
def rotate_to(
    target_angle_fixed,
    off_gz,
    camera_servo: Optional[ServoController] = None,
):
    relative_target = target_angle_fixed - 90
    curr_yaw = 0.0
    last_t = time.time()
    while abs(relative_target - curr_yaw) > 2.0:
        dt = time.time() - last_t
        last_t = time.time()
        gz = (read_raw(0x47) - off_gz) / 131.0
        curr_yaw += gz * dt
        direction = "rotate_left" if (relative_target - curr_yaw) > 0 else "rotate_right"
        _align_servo_for_rotation(camera_servo, direction)
        set_motors(45, 45, direction)
        time.sleep(DRIVE_LOOP_SLEEP_S)
        _center_camera_servo(camera_servo)
    stop_motors()
    _center_camera_servo(camera_servo)


def rotate_to_angle(
    target_angle,
    off_gz,
    camera_servo: Optional[ServoController] = None,
):
    if target_angle < 0 or target_angle > 180:
        print("Error: Angle must be between 0 and 180 degrees.")
        return

    relative_target = target_angle - 90  # 90 degrees is straight
    curr_yaw = 0.0
    last_t = time.time()

    while abs(relative_target - curr_yaw) > 2.0:  # 2 degree tolerance
        dt = time.time() - last_t
        last_t = time.time()
        gz = (read_raw(0x47) - off_gz) / 131.0  # Gyroscope reading
        curr_yaw -= gz * dt

        direction = "rotate_left" if (relative_target - curr_yaw) > 0 else "rotate_right"
        _align_servo_for_rotation(camera_servo, direction)
        set_motors(45, 45, direction)
        time.sleep(DRIVE_LOOP_SLEEP_S)
        _center_camera_servo(camera_servo)

    stop_motors()
    _center_camera_servo(camera_servo)


def drive_dist(
    target_cm,
    off_x,
    off_y,
    off_gz,
    vision_tracker: Optional[DriveVisionTracker] = None,
    camera_servo: Optional[ServoController] = None,
    stop_distance_cm: Optional[float] = None,
):
    target_mm = target_cm * 10  # Convert cm to mm
    vx, vy, dx, dy, yaw, last_t = 0, 0, 0, 0, 0, time.time()
    while math.sqrt(dx**2 + dy**2) < (target_mm - 15):
        dt = time.time() - last_t
        last_t = time.time()
        ax = ((read_raw(0x3B) - off_x) / 16384.0) * 9806.6
        ay = ((read_raw(0x3D) - off_y) / 16384.0) * 9806.6
        gz = (read_raw(0x47) - off_gz) / 131.0

        # Deadzone to prevent noise
        ax = 0 if abs(ax) < 350 else ax
        ay = 0 if abs(ay) < 350 else ay

        vx += ax * dt; vy += ay * dt
        dx += vx * dt; dy += vy * dt
        yaw += gz * dt

        vision_steer = 0.0
        if vision_tracker is not None:
            vision_steer, live_distance_cm = vision_tracker.poll()
            if (
                stop_distance_cm is not None
                and live_distance_cm is not None
                and live_distance_cm <= stop_distance_cm
            ):
                break

        # Correction for deviation while driving
        correction = (yaw * DRIVE_YAW_GAIN) + vision_steer
        speed_l = DRIVE_BASE_SPEED - correction
        speed_r = DRIVE_BASE_SPEED + correction
        _align_servo_with_wheels(camera_servo, speed_l, speed_r)
        set_motors(speed_l, speed_r)
        time.sleep(DRIVE_LOOP_SLEEP_S)
        _center_camera_servo(camera_servo)
    stop_motors()
    _center_camera_servo(camera_servo)


def move_by_angle_and_distance(
    angle,
    distance_cm,
    target_cls: Optional[str] = None,
):
    ox, oy, ogz = calibrate_all()
    print(f"Moving to angle {angle} degrees and distance {distance_cm} cm")
    camera_servo: Optional[ServoController] = None
    vision_tracker: Optional[DriveVisionTracker] = None

    try:
        camera_servo = ServoController()
        camera_servo.start(angle=CAMERA_SERVO_CENTER_DEG)
    except Exception as exc:
        print(f"[WARN] Camera servo unavailable ({exc}). Continuing without servo alignment.")
        camera_servo = None

    if target_cls:
        try:
            vision_tracker = DriveVisionTracker(target_cls)
            vision_tracker.start()
        except Exception as exc:
            print(f"[WARN] Vision tracker unavailable ({exc}). Continuing with IMU-only drive.")
            if vision_tracker is not None:
                try:
                    vision_tracker.stop()
                except Exception:
                    pass
            vision_tracker = None

    try:
        drive_dist(5.1, ox, oy, ogz, camera_servo=camera_servo)
        rotate_to_angle(angle, ogz, camera_servo=camera_servo)
        time.sleep(0.5)

        # Move to the desired distance with continuous steering corrections.
        drive_dist(
            distance_cm,
            ox,
            oy,
            ogz,
            vision_tracker=vision_tracker,
            camera_servo=camera_servo,
            stop_distance_cm=2.5 if target_cls else None,
        )
    finally:
        if vision_tracker is not None:
            try:
                vision_tracker.stop()
            except Exception:
                pass
        if camera_servo is not None:
            try:
                _center_camera_servo(camera_servo)
                camera_servo.stop()
            except Exception:
                pass

# --- Test Functions ---
def test_rotate_to_angle():
    print("Testing rotate_to_angle function...")
    ox, oy, ogz = calibrate_all()
    """"""
    test_angles = [0, 45, 90, 135, 180]  # Angles to test
    for angle in test_angles:
        print(f"Testing angle: {angle}")
        rotate_to_angle(angle, ogz)
        time.sleep(1)  # Delay between tests

    # Additional test: rotate right then left
    print("Testing rotation to the right (0 degrees) and to the left (180 degrees)")
    rotate_to_angle(0, ogz)  # Rotate right
    time.sleep(1)
    rotate_to_angle(180, ogz)  # Rotate left
    time.sleep(1)

    print("Test finished.")

# --- Execution ---
if __name__ == "__main__":
    try:
        mode = input("Choose mode (test/normal): ").strip().lower()
        if mode == "test":
            test_rotate_to_angle()
        else:
            t_angle = float(input("Enter angle (0-180): "))
            t_dist = float(input("Distance (cm): "))
            move_by_angle_and_distance(t_angle, t_dist)

    except KeyboardInterrupt:
        stop_motors()
    finally:
        stop_motors()
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
