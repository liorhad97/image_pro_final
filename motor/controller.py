"""
L298N dual H-bridge motor controller.

Wraps RPi.GPIO and PWM setup into a clean class so the rest of the code
never touches GPIO registers directly.
"""
from __future__ import annotations

import RPi.GPIO as GPIO

import hparams as HP


class MotorController:
    """
    Controls two DC motors via an L298N H-bridge driver board.

    Wiring (BCM pin numbers, configurable via hparams.py):
    --------------------------------------------------------
    Motor A (left wheel) : ENA (PWM enable), IN1, IN2
    Motor B (right wheel): ENB (PWM enable), IN3, IN4

    Parameters
    ----------
    ena, in1, in2 : int
        BCM GPIO pins for Motor A enable and direction.
    enb, in3, in4 : int
        BCM GPIO pins for Motor B enable and direction.
    pwm_freq_hz : int
        PWM frequency for speed control (default 1 kHz).
    """

    def __init__(
        self,
        ena: int = HP.MOTOR_ENA,
        in1: int = HP.MOTOR_IN1,
        in2: int = HP.MOTOR_IN2,
        enb: int = HP.MOTOR_ENB,
        in3: int = HP.MOTOR_IN3,
        in4: int = HP.MOTOR_IN4,
        pwm_freq_hz: int = HP.MOTOR_PWM_FREQUENCY_HZ,
    ) -> None:
        # Store direction-control pins for use in set_motors()
        self._in1 = in1
        self._in2 = in2
        self._in3 = in3
        self._in4 = in4

        # Initialise GPIO
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in (ena, in1, in2, enb, in3, in4):
            GPIO.setup(pin, GPIO.OUT)

        # Start PWM at 0 % duty cycle (motors off)
        self._pwm_a = GPIO.PWM(ena, pwm_freq_hz)
        self._pwm_b = GPIO.PWM(enb, pwm_freq_hz)
        self._pwm_a.start(0)
        self._pwm_b.start(0)

    # ------------------------------------------------------------------ public

    def set_motors(
        self,
        speed_l: float,
        speed_r: float,
        direction: str = "forward",
    ) -> None:
        """
        Set motor speeds and direction.

        Parameters
        ----------
        speed_l : float
            Duty-cycle [0–100] for Motor A (left wheel).
        speed_r : float
            Duty-cycle [0–100] for Motor B (right wheel).
        direction : str
            One of ``"forward"``, ``"rotate_left"``, or ``"rotate_right"``.
        """
        if direction == "forward":
            GPIO.output(self._in1, GPIO.HIGH)
            GPIO.output(self._in2, GPIO.LOW)
            GPIO.output(self._in3, GPIO.HIGH)
            GPIO.output(self._in4, GPIO.LOW)
        elif direction == "rotate_left":
            GPIO.output(self._in1, GPIO.LOW)
            GPIO.output(self._in2, GPIO.HIGH)
            GPIO.output(self._in3, GPIO.HIGH)
            GPIO.output(self._in4, GPIO.LOW)
        elif direction == "rotate_right":
            GPIO.output(self._in1, GPIO.HIGH)
            GPIO.output(self._in2, GPIO.LOW)
            GPIO.output(self._in3, GPIO.LOW)
            GPIO.output(self._in4, GPIO.HIGH)

        self._pwm_a.ChangeDutyCycle(max(0.0, min(100.0, speed_l)))
        self._pwm_b.ChangeDutyCycle(max(0.0, min(100.0, speed_r)))

    def stop(self) -> None:
        """Brake both motors to a near-stop (low duty cycle, all IN pins LOW)."""
        self._pwm_a.ChangeDutyCycle(1)
        self._pwm_b.ChangeDutyCycle(1)
        for pin in (self._in1, self._in2, self._in3, self._in4):
            GPIO.output(pin, GPIO.LOW)

    def cleanup(self) -> None:
        """Stop PWM output and release all GPIO resources."""
        self._pwm_a.stop()
        self._pwm_b.stop()
        GPIO.cleanup()
