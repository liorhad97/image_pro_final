"""
Interactive servo manual-control script.

Reads all PWM parameters from hparams.py so there is one source of truth.

Usage (on the Raspberry Pi):
    python servo/manual_control.py

Commands at the prompt:
    <number>   – move to that angle in degrees (0-180)
    c          – run a full 0 → 180 → 0 calibration sweep
    q          – quit
"""

from time import sleep

from rpi_hardware_pwm import HardwarePWM

import hparams as HP

# ── PWM setup ──────────────────────────────────────────────────────────────────
pwm = HardwarePWM(
    pwm_channel=HP.SERVO_PWM_CHANNEL,
    hz=HP.SERVO_PWM_FREQUENCY_HZ,
    chip=HP.SERVO_PWM_CHIP,
)


def angle_to_duty(angle: float) -> float:
    angle = max(0.0, min(180.0, float(angle)))
    pulse_us = HP.SERVO_MIN_US + (HP.SERVO_MAX_US - HP.SERVO_MIN_US) * (angle / 180.0)
    return (pulse_us / HP.SERVO_PERIOD_US) * 100.0


def move(angle: float) -> str:
    duty = angle_to_duty(angle)
    pwm.change_duty_cycle(duty)
    return f"→ {angle:.1f}°  (duty={duty:.2f}%,  pulse={HP.SERVO_MIN_US + (HP.SERVO_MAX_US - HP.SERVO_MIN_US) * angle / 180:.0f} µs)"


# ── main ───────────────────────────────────────────────────────────────────────
try:
    pwm.start(angle_to_duty(90))
    print(f"HW-PWM servo ready on GPIO12  (channel={HP.SERVO_PWM_CHANNEL}, "
          f"min={HP.SERVO_MIN_US}µs, max={HP.SERVO_MAX_US}µs)")
    print("Commands:  <angle 0-180>  |  c = calibration sweep  |  q = quit\n")

    while True:
        raw = input("angle> ").strip().lower()
        if raw in ("q", "quit", "exit"):
            break
        if raw == "c":
            for target in (0, 90, 180, 90, 0):
                print(move(target))
                sleep(1)
            continue
        if raw == "":
            continue
        try:
            print(move(float(raw)))
        except ValueError:
            print("  ! enter a number between 0 and 180")

except KeyboardInterrupt:
    pass
finally:
    pwm.stop()
    print("\nPWM stopped.")
