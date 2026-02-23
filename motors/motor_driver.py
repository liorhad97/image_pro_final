"""Low-level L298N GPIO/PWM motor control."""
from __future__ import annotations

import RPi.GPIO as GPIO

from config import MotorConfig


class MotorDriver:
    """Controls two DC motors via an L298N driver over GPIO/PWM."""

    def __init__(self, config: MotorConfig) -> None:
        self._cfg = config
        self._pwm_a = None
        self._pwm_b = None

    def start(self) -> None:
        """Set up GPIO pins and start PWM at 0% duty cycle."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        pins = [
            self._cfg.ena, self._cfg.in1, self._cfg.in2,
            self._cfg.enb, self._cfg.in3, self._cfg.in4,
        ]
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)

        self._pwm_a = GPIO.PWM(self._cfg.ena, self._cfg.pwm_frequency_hz)
        self._pwm_b = GPIO.PWM(self._cfg.enb, self._cfg.pwm_frequency_hz)
        self._pwm_a.start(0)
        self._pwm_b.start(0)

    def drive_forward(self, speed: float) -> None:
        """Spin both motors forward at *speed* (0-100)."""
        GPIO.output(self._cfg.in1, GPIO.HIGH)
        GPIO.output(self._cfg.in2, GPIO.LOW)
        GPIO.output(self._cfg.in3, GPIO.HIGH)
        GPIO.output(self._cfg.in4, GPIO.LOW)
        self._pwm_a.ChangeDutyCycle(speed)
        self._pwm_b.ChangeDutyCycle(speed)

    def stop(self) -> None:
        """Zero duty cycle and pull all direction pins LOW."""
        if self._pwm_a:
            self._pwm_a.ChangeDutyCycle(0)
        if self._pwm_b:
            self._pwm_b.ChangeDutyCycle(0)
        for pin in (self._cfg.in1, self._cfg.in2, self._cfg.in3, self._cfg.in4):
            GPIO.output(pin, GPIO.LOW)

    def cleanup(self) -> None:
        """Stop PWM and release GPIO resources."""
        self.stop()
        if self._pwm_a:
            self._pwm_a.stop()
            self._pwm_a = None
        if self._pwm_b:
            self._pwm_b.stop()
            self._pwm_b = None
        GPIO.cleanup()
