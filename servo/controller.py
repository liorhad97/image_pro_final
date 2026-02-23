from __future__ import annotations

from rpi_hardware_pwm import HardwarePWM


class ServoController:
    """
    Thin wrapper around :class:`rpi_hardware_pwm.HardwarePWM` that adds
    human-friendly angle-based control for a standard 50 Hz RC servo.

    The pulse-width range (``min_us`` / ``max_us``) maps linearly to
    0 – 180 degrees.

    Parameters
    ----------
    channel : int
        Hardware PWM channel (e.g. 0 → GPIO12 on RPi).
    chip : int
        PWM chip number (usually 0).
    min_us : int
        Pulse width (µs) corresponding to 0°.
    max_us : int
        Pulse width (µs) corresponding to 180°.
    """

    _PWM_FREQUENCY_HZ = 50
    _PERIOD_US = 20_000  # 1 / 50 Hz = 20 ms = 20 000 µs

    def __init__(
        self,
        channel: int = 0,
        chip: int = 0,
        min_us: int = 1000,
        max_us: int = 2000,
    ) -> None:
        self._min_us = min_us
        self._max_us = max_us
        self._pwm = HardwarePWM(
            pwm_channel=channel,
            hz=self._PWM_FREQUENCY_HZ,
            chip=chip,
        )

    # ------------------------------------------------------------------ public
    def start(self, angle: float = 90.0) -> None:
        """Start the PWM signal at *angle* degrees."""
        self._pwm.start(self._angle_to_duty(angle))

    def set_angle(self, angle: float) -> None:
        """Move servo to *angle* degrees (clamped to [0, 180])."""
        self._pwm.change_duty_cycle(self._angle_to_duty(angle))

    def stop(self) -> None:
        """Stop PWM output and release the hardware resource."""
        self._pwm.stop()

    # ----------------------------------------------------------------- private
    def _angle_to_duty(self, angle: float) -> float:
        """
        Convert *angle* [0 – 180°] to a duty-cycle percentage for a 50 Hz
        PWM signal.
        """
        angle = max(0.0, min(180.0, float(angle)))
        pulse_us = self._min_us + (self._max_us - self._min_us) * (angle / 180.0)
        return (pulse_us / self._PERIOD_US) * 100.0
