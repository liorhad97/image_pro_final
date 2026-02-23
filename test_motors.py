import RPi.GPIO as GPIO
import smbus2
import time
import math

# --- Motor Settings (GPIO) ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

ENA, IN1, IN2 = 12, 17, 27
ENB, IN3, IN4 = 13, 23, 24

pins = [ENA, IN1, IN2, ENB, IN3, IN4]
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)


def set_motors(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)


def stop_motors():
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)


# --- IMU Settings ---
bus = smbus2.SMBus(1)
ADDR = 0x68
bus.write_byte_data(ADDR, 0x6B, 0)  # Wake up sensor
time.sleep(0.1)


def read_raw(reg):
    h = bus.read_byte_data(ADDR, reg)
    l = bus.read_byte_data(ADDR, reg + 1)
    val = (h << 8) + l
    return val - 65536 if val > 32768 else val


def calibrate_xy(samples=1000):
    print("Calibrating XY plane... do not move the car")
    sum_x, sum_y = 0, 0
    for _ in range(samples):
        sum_x += read_raw(0x3B)  # X axis
        sum_y += read_raw(0x3D)  # Y axis
        time.sleep(0.002)
    return sum_x / samples, sum_y / samples


def get_target_distance():
    while True:
        try:
            dist = float(input("Enter target distance in millimeters (e.g. 400): "))
            if dist <= 0:
                print("Error: distance must be a positive number.")
                continue
            return dist
        except ValueError:
            print("Error: please enter a valid number.")


if __name__ == "__main__":
    TARGET_DIST_MM = get_target_distance()
    offset_x, offset_y = calibrate_xy()

    # State variables
    vel_x, vel_y = 0.0, 0.0
    dist_x, dist_y = 0.0, 0.0
    total_dist = 0.0
    still_count = 0
    last_t = time.time()

    try:
        print(f"Starting drive to target: {TARGET_DIST_MM} mm...")
        set_motors(100)

        while total_dist < TARGET_DIST_MM:
            now = time.time()
            dt = now - last_t
            last_t = now
            if dt <= 0:
                continue

            # Read sensor data
            raw_x = read_raw(0x3B)
            raw_y = read_raw(0x3D)

            # Convert to acceleration (mm/s^2) at 2g scale
            acc_x = ((raw_x - offset_x) / 16384.0) * 9806.6
            acc_y = ((raw_y - offset_y) / 16384.0) * 9806.6

            # Noise filter and integration
            if abs(acc_x) < 400 and abs(acc_y) < 400:
                still_count += 1
                if still_count > 5:
                    vel_x, vel_y = 0, 0
            else:
                still_count = 0
                vel_x += acc_x * dt
                vel_y += acc_y * dt

            dist_x += vel_x * dt
            dist_y += vel_y * dt
            total_dist = math.sqrt(dist_x**2 + dist_y**2)

            print(f"Current distance: {total_dist:5.1f} / {TARGET_DIST_MM} mm", end="\r")

        print(f"\nTarget reached ({total_dist:.1f} mm). Stopping motors.")
        stop_motors()
        time.sleep(1)  # Wait for car to fully stop

    except KeyboardInterrupt:
        print("\nEmergency stop!")
    finally:
        stop_motors()
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
