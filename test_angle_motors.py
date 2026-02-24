import RPi.GPIO as GPIO
import smbus2
import time
import math

# motor direction is set by IN1-IN4 GPIO pins, speed is set by PWM duty cycle on ENA and ENB
# gyroscope Z axis is integrated over time for rotation, accelerometer is double-integrated for distance
# move_by_angle_and_distance drives 5 cm forward first, then rotates to the servo angle, then drives to the target

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
ENA, IN1, IN2 =  13, 23, 24
ENB, IN3, IN4 =  18, 17, 27

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

bus = smbus2.SMBus(1)
ADDR = 0x68
bus.write_byte_data(ADDR, 0x6B, 0)
time.sleep(0.1)

def read_raw(reg):
    h = bus.read_byte_data(ADDR, reg)
    l = bus.read_byte_data(ADDR, reg + 1)
    val = (h << 8) + l
    return val - 65536 if val > 32768 else val

def calibrate_all(samples=1000):
    print("Full calibration (X, Y, GZ)... Do not move the car")
    sum_x, sum_y, sum_gz = 0, 0, 0
    for _ in range(samples):
        sum_x += read_raw(0x3B)
        sum_y += read_raw(0x3D)
        sum_gz += read_raw(0x47)
        time.sleep(0.002)
    return sum_x/samples, sum_y/samples, sum_gz/samples

def rotate_to(target_angle_fixed, off_gz):
    relative_target = target_angle_fixed - 90
    curr_yaw = 0.0
    last_t = time.time()
    while abs(relative_target - curr_yaw) > 2.0:
        dt = time.time() - last_t
        last_t = time.time()
        gz = (read_raw(0x47) - off_gz) / 131.0
        curr_yaw += gz * dt
        direction = "rotate_left" if (relative_target - curr_yaw) > 0 else "rotate_right"
        set_motors(45, 45, direction)
        time.sleep(0.01)
    stop_motors()

def rotate_to_angle(target_angle, off_gz):
    if target_angle < 0 or target_angle > 180:
        print("Error: Angle must be between 0 and 180 degrees.")
        return

    relative_target = target_angle - 90
    curr_yaw = 0.0
    last_t = time.time()

    while abs(relative_target - curr_yaw) > 2.0:
        dt = time.time() - last_t
        last_t = time.time()
        gz = (read_raw(0x47) - off_gz) / 131.0
        curr_yaw -= gz * dt

        direction = "rotate_left" if (relative_target - curr_yaw) > 0 else "rotate_right"
        set_motors(45, 45, direction)
        time.sleep(0.01)

    stop_motors()

def drive_dist(target_cm, off_x, off_y, off_gz):
    target_mm = target_cm * 10
    vx, vy, dx, dy, yaw, last_t = 0, 0, 0, 0, 0, time.time()
    while math.sqrt(dx**2 + dy**2) < (target_mm - 15):
        dt = time.time() - last_t
        last_t = time.time()
        ax = ((read_raw(0x3B) - off_x) / 16384.0) * 9806.6
        ay = ((read_raw(0x3D) - off_y) / 16384.0) * 9806.6
        gz = (read_raw(0x47) - off_gz) / 131.0

        ax = 0 if abs(ax) < 350 else ax
        ay = 0 if abs(ay) < 350 else ay

        vx += ax * dt; vy += ay * dt
        dx += vx * dt; dy += vy * dt
        yaw += gz * dt

        set_motors(50 - (yaw * 1.8), 50 + (yaw * 1.8))
        time.sleep(0.01)
    stop_motors()

def move_by_angle_and_distance(angle, distance_cm):
    ox, oy, ogz = calibrate_all()
    print(f"Moving to angle {angle} degrees and distance {distance_cm} cm")
    drive_dist(5.1, ox, oy, ogz)
    rotate_to_angle(angle, ogz)
    time.sleep(0.5)
    drive_dist(distance_cm, ox, oy, ogz)

def test_rotate_to_angle():
    print("Testing rotate_to_angle function...")
    ox, oy, ogz = calibrate_all()
    test_angles = [0, 45, 90, 135, 180]
    for angle in test_angles:
        print(f"Testing angle: {angle}")
        rotate_to_angle(angle, ogz)
        time.sleep(1)

    print("Testing rotation to the right (0 degrees) and to the left (180 degrees)")
    rotate_to_angle(0, ogz)
    time.sleep(1)
    rotate_to_angle(180, ogz)
    time.sleep(1)

    print("Test finished.")

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
