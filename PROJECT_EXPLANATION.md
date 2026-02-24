# Project Explanation - Autonomous Target Detection & Navigation Robot

## What the project does in one sentence
A Raspberry Pi robot scans its environment with two cameras on a rotating servo, detects a colored 3D object, classifies its shape, calculates exactly how far it is, then autonomously drives to it.

---

## Hardware used
- Raspberry Pi (runs all the code)
- 2x Raspberry Pi cameras (stereo pair, side by side)
- 1x Pan servo (rotates both cameras left/right, 0-180 degrees)
- L298N H-bridge motor driver (controls the two drive wheels)
- MPU-6050 IMU (inertial measurement unit - accelerometer + gyroscope chip on I2C)
- rpi_hardware_pwm library to generate the servo PWM signal from hardware (not software)

---

## Software / Libraries used
- **OpenCV (cv2)** - all image processing: color masking, morphology, contour detection, drawing
- **Picamera2** - Raspberry Pi camera interface, captures BGR frames
- **NumPy** - array math, mask operations
- **RPi.GPIO** - controls the motor driver GPIO pins
- **smbus2** - reads the IMU sensor over I2C
- **rpi_hardware_pwm** - hardware PWM for precise servo control

---

## Full pipeline, step by step

### Step 1 - Servo sweep (scanner/scanner.py)
The servo starts at 0 degrees and sweeps to 180 degrees, then back, in 5-degree steps.
At every position it waits 0.5 seconds for the camera image to stabilize, then captures one frame from each camera.

### Step 2 - Color detection (detection/detector.py)
Each frame is converted from BGR color space to HSV color space.
HSV is used instead of BGR because color in HSV is separated from brightness, making it much more stable under different lighting.

Two HSV ranges are used for red because red wraps around 0 in the hue wheel:
- Range 1: hue 0-5 (red at the low end of the wheel)
- Range 2: hue 170-180 (red at the high end of the wheel)

`cv2.inRange` creates a binary mask where white pixels are the detected color.
Then morphological operations clean up the mask:
- MORPH_CLOSE fills small holes inside the blob
- MORPH_OPEN removes small noise pixels outside the blob

Then `cv2.findContours` finds all connected white regions in the mask.
The largest contour by area is selected as the candidate object.

### Step 3 - Shape classification (detection/detector.py `_classify`)
Three geometric measurements are computed from the contour:

**Extent** = blob area / bounding box area
A pyramid only fills part of its bounding box so it has a low extent (below 0.58).

**Circularity** = 4π × area / perimeter²
A circle gives 1.0. A cylinder seen from the front looks like a circle so it has high circularity (above 0.78).

**Vertex count** = from `cv2.approxPolyDP` (Douglas-Peucker polygon approximation)
A pyramid has few vertices (≤6). A cylinder has many (≥8).

Classification rules:
- low extent AND few vertices → **Pyramid**
- high circularity OR many vertices → **Cylinder**
- otherwise → **Cube**

The result is a `DetResult` dataclass with: found flag, class name, bounding box (x,y,w,h), center pixel (cx,cy), and the masks.

### Step 4 - Confirmation (scanner/scanner.py)
A detection is only accepted when ALL four conditions are true at the same time:
1. Both left and right cameras detected something
2. Both cameras report the same shape class
3. The detected class matches the target class requested (or any if not specified)
4. The bounding box is at least 10 pixels away from the frame edge (not partially cut off)

This dual-camera confirmation eliminates false positives. One noisy frame cannot trigger a move.

### Step 5 - Distance estimation (stereo/distance.py)
This uses the standard stereo disparity formula:

```
Z = (fx × B) / |disparity|
```

Where:
- **fx** = horizontal focal length of the camera in pixels (1893 px, from calibration)
- **B** = baseline = physical distance between the two cameras (7.5 cm)
- **disparity** = xLeft - xRight = horizontal pixel difference between the same object center in the two frames

Because the two cameras are separated horizontally, a nearby object appears shifted further apart (large disparity) and a far object appears almost in the same position (small disparity). The formula converts this pixel shift into real-world distance in meters.

Sanity checks before accepting the estimate:
- disparity must be above 1 pixel (below this is numerical noise)
- vertical offset |yLeft - yRight| must be below 80 px (large vertical offset means the cameras are not well aligned)

### Step 6 - Robot navigation (test_angle_motors.py)
Once angle and distance are confirmed, `move_by_angle_and_distance(angle, distance_cm)` is called.

First the IMU is calibrated: 1000 samples are averaged to get the bias offset of accelerometer X, Y and gyroscope Z.

Then the robot:
1. Drives forward 5 cm to clear its own camera mount shadow
2. Rotates to the servo angle (the angle where the target was seen)
3. Drives forward to (distance - 2.5 cm) stopping just before the object

**Rotation** uses the gyroscope Z axis. The code reads gyro Z in a tight loop, integrates it over time to get yaw angle, and keeps rotating until yaw matches the target angle within 2 degrees.

**Distance driving** uses the accelerometer. It double-integrates acceleration into position (ax → vx → dx, ay → vy → dy) and drives until the total displacement matches the target. A yaw correction term keeps the robot going straight by adjusting left/right motor duty cycle proportionally to drift.

Motors are controlled by an L298N H-bridge. Motor direction is set with GPIO HIGH/LOW on IN1-IN4 pins. Motor speed is set with PWM duty cycle on ENA/ENB pins.

---

## Data flow summary

```
main_drive_target.py
    |
    v
Scanner.run()  [scanner/scanner.py]
    |-- servo sweeps 0->180->0 degrees
    |-- at each step: StereoCams.capture_bgr()  [stereo/camera.py]
    |-- RedObjectDetector.detect(frame_left)   [detection/detector.py]
    |-- RedObjectDetector.detect(frame_right)
    |       |-- BGR->HSV conversion
    |       |-- cv2.inRange -> binary mask
    |       |-- morphological cleanup
    |       |-- cv2.findContours -> largest contour
    |       |-- _classify() -> Pyramid / Cube / Cylinder
    |       |-- returns DetResult
    |-- StereoDistanceEstimator.estimate()     [stereo/distance.py]
    |       |-- Z = (fx * B) / |xL - xR|
    |-- confirmation check (both cameras, same class, edge margin)
    |-- returns ScanResult (angle_deg, distance_cm, class)
    |
    v
move_by_angle_and_distance(angle, distance_cm)  [test_angle_motors.py]
    |-- IMU calibration (1000 samples)
    |-- drive 5 cm forward
    |-- rotate to angle (gyro Z integration loop)
    |-- drive to distance (accelerometer double integration loop)
```

---

## Key design decisions

**Why HSV instead of BGR/RGB for color detection?**
HSV separates hue (the actual color) from saturation and value (brightness). This means lighting changes barely affect the hue channel, making the threshold robust across different environments.

**Why two cameras instead of one?**
A single camera cannot measure depth. Two cameras separated by a known distance (baseline) create parallax - the same object appears at different horizontal positions in each image. The disparity between these positions is directly inversely proportional to depth.

**Why confirm in both cameras before moving?**
Reduces false positives to near zero. A reflection or lighting artifact that fools one camera is extremely unlikely to fool both at exactly the same time in the same position.

**Why use gyro + accelerometer separately for rotation vs distance?**
Gyroscope measures angular velocity directly, so integrating it once gives angle - this is clean and fast for rotation. Accelerometer measures linear acceleration, but for distance you need to integrate twice (acceleration -> velocity -> position), which accumulates error. The deadzone (ignoring acceleration below 350 mm/s²) reduces the noise integration problem.
