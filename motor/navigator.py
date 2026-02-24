"""
High-level navigation using MotorController + IMUReader feedback loops.

Navigator combines dead-reckoning (accelerometer) and gyro-corrected
steering to achieve angle-accurate rotation and distance-accurate
straight-line drives.
"""
from __future__ import annotations

import math
import time

import hparams as HP

from .controller import MotorController
from .imu import IMUReader


class Navigator:
    """
    High-level movement controller that fuses motor commands with IMU
    feedback to rotate to a heading and drive a measured distance.

    Parameters
    ----------
    motor : MotorController
        Initialised motor driver instance.
    imu : IMUReader
        Initialised IMU reader instance.
    """

    _GYRO_SENSITIVITY = HP.GYRO_SENSITIVITY

    def __init__(self, motor: MotorController, imu: IMUReader) -> None:
        self._motor = motor
        self._imu   = imu

    # ------------------------------------------------------------------ public

    def rotate_to_angle(
        self,
        target_angle: float,
        off_gz: float,
        tolerance_deg: float = HP.NAV_ROTATE_TOLERANCE_DEG,
        speed: float = HP.NAV_ROTATE_SPEED,
    ) -> None:
        """
        Rotate the robot to *target_angle* degrees using gyro integration.

        Reference frame: 90° = straight ahead, < 90° = right, > 90° = left.

        Parameters
        ----------
        target_angle : float
            Desired heading in [0, 180] degrees.
        off_gz : float
            Gyro-Z calibration offset (raw units) from :meth:`IMUReader.calibrate`.
        tolerance_deg : float
            Stop when within this many degrees of target (default 2°).
        speed : float
            Motor duty-cycle while rotating (default 45 %).
        """
        if not (0.0 <= target_angle <= 180.0):
            print("[Navigator] Error: angle must be between 0 and 180 degrees.")
            return

        # Relative target: 0° = turn fully right, 90° = straight, 180° = left
        relative_target = target_angle - 90.0
        curr_yaw = 0.0
        last_t = time.time()

        while abs(relative_target - curr_yaw) > tolerance_deg:
            dt = time.time() - last_t
            last_t = time.time()

            # Integrate gyro reading to estimate current yaw
            gz = (self._imu.read_raw(self._imu._REG_GYRO_Z) - off_gz) / self._GYRO_SENSITIVITY
            curr_yaw -= gz * dt

            direction = "rotate_left" if (relative_target - curr_yaw) > 0 else "rotate_right"
            self._motor.set_motors(speed, speed, direction)
            time.sleep(HP.NAV_LOOP_SLEEP_S)

        self._motor.stop()

    def drive_dist(
        self,
        target_cm: float,
        off_x: float,
        off_y: float,
        off_gz: float,
        speed: float = HP.NAV_DRIVE_SPEED,
        stop_margin_mm: float = HP.NAV_STOP_MARGIN_MM,
    ) -> None:
        """
        Drive straight for *target_cm* centimetres using accelerometer
        dead-reckoning and gyro-corrected steering.

        Parameters
        ----------
        target_cm : float
            Distance to travel [cm].
        off_x, off_y : float
            Accelerometer X/Y calibration offsets (raw units).
        off_gz : float
            Gyro-Z calibration offset (raw units).
        speed : float
            Base motor duty-cycle while driving (default 50 %).
        stop_margin_mm : float
            Stop this many mm before reaching *target_cm* (overshoot margin).
        """
        target_mm = target_cm * 10.0
        vx, vy = 0.0, 0.0     # velocity [mm/s]
        dx, dy = 0.0, 0.0     # displacement [mm]
        last_t = time.time()

        while math.sqrt(dx ** 2 + dy ** 2) < (target_mm - stop_margin_mm):
            dt = time.time() - last_t
            last_t = time.time()

            # Convert raw accelerometer to mm/s²
            ax = ((self._imu.read_raw(self._imu._REG_ACCEL_X) - off_x) / HP.IMU_ACCEL_SCALE) * HP.IMU_ACCEL_MS2
            ay = ((self._imu.read_raw(self._imu._REG_ACCEL_Y) - off_y) / HP.IMU_ACCEL_SCALE) * HP.IMU_ACCEL_MS2
            # Current rotation rate [°/s] — apply a deadband to ignore gyro noise
            gz = (self._imu.read_raw(self._imu._REG_GYRO_Z) - off_gz) / self._GYRO_SENSITIVITY
            gz = 0.0 if abs(gz) < HP.GYRO_RATE_DEADBAND else gz

            # Suppress accelerometer noise below the deadzone threshold
            ax = 0.0 if abs(ax) < HP.IMU_NOISE_THRESHOLD else ax
            ay = 0.0 if abs(ay) < HP.IMU_NOISE_THRESHOLD else ay

            # Integrate acceleration → velocity → displacement
            vx += ax * dt
            vy += ay * dt
            dx += vx * dt
            dy += vy * dt

            # Proportional correction on current rotation rate (not accumulated yaw).
            # gz < 0 → robot turning left  → speed - gz*gain speeds up left motor  ✓
            # gz > 0 → robot turning right → speed - gz*gain slows  left motor     ✓
            self._motor.set_motors(speed - gz * HP.GYRO_CORRECTION_GAIN, speed + gz * HP.GYRO_CORRECTION_GAIN)
            time.sleep(HP.NAV_LOOP_SLEEP_S)

        self._motor.stop()

    def move_by_angle_and_distance(
        self,
        angle: float,
        distance_cm: float,
        forward_clearance_cm: float = HP.NAV_CLEARANCE_CM,
    ) -> None:
        """
        Full movement sequence:

        1. Calibrate the IMU (robot must be stationary).
        2. Drive forward *forward_clearance_cm* to clear nearby obstacles.
        3. Rotate to *angle* degrees.
        4. Drive straight for *distance_cm*.

        Parameters
        ----------
        angle : float
            Target heading in [0, 180] degrees (90 = straight).
        distance_cm : float
            Distance to travel after rotating [cm].
        forward_clearance_cm : float
            Short forward drive before turning (default 5.1 cm).
        """
        off_x, off_y, off_gz = self._imu.calibrate()
        print(f"[Navigator] Moving → angle={angle:.1f}°, distance={distance_cm:.1f} cm")

        # Clearance drive to get away from the start position
        self.drive_dist(forward_clearance_cm, off_x, off_y, off_gz)

        # Rotate to the desired heading
        self.rotate_to_angle(angle, off_gz)
        time.sleep(HP.NAV_SETTLE_S)   # brief pause to let the robot settle after rotation

        # Drive to the target distance
        self.drive_dist(distance_cm, off_x, off_y, off_gz)
