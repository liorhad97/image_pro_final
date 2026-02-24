from __future__ import annotations

from rpi_hardware_pwm import HardwarePWM

import hparams as HP

# controls a standard RC servo over hardware PWM, maps 0-180 degrees to the configured min/max pulse width
class ServoController:

    _PWM_FREQUENCY_HZ = HP.SERVO_PWM_FREQUENCY_HZ
    _PERIOD_US = HP.SERVO_PERIOD_US

    def __init__(
        self,
        channel: int = HP.SERVO_PWM_CHANNEL,
        chip: int = HP.SERVO_PWM_CHIP,
        min_us: int = HP.SERVO_MIN_US,
        max_us: int = HP.SERVO_MAX_US,
    ) -> None:
        self._min_us = min_us
        self._max_us = max_us
        self._pwm = HardwarePWM(
            pwm_channel=channel,
            hz=self._PWM_FREQUENCY_HZ,
            chip=chip,
        )

    def start(self, angle: float = 90.0) -> None:
        self._pwm.start(self._angle_to_duty(angle))

    def set_angle(self, angle: float) -> None:
        self._pwm.change_duty_cycle(self._angle_to_duty(angle))

    def stop(self) -> None:
        self._pwm.stop()

    def _angle_to_duty(self, angle: float) -> float:
        angle = max(0.0, min(180.0, float(angle)))
        pulse_us = self._min_us + (self._max_us - self._min_us) * (angle / 180.0)
        return (pulse_us / self._PERIOD_US) * 100.0
