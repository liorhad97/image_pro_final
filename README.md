# image_pro_final

Command guide for running the stereo vision scanner.

## Main scanner (`main.py`)

Show all available CLI options:

```bash
python main.py --help
```

Run with defaults (accept any blue target):

```bash
python main.py
```

Run your example (target cube + live preview):

```bash
python main.py --target cube --view  ######this works fine 
```

Other common runs:

```bash
# Target options: cube, cylinder, pyramid, none
python main.py --target cylinder
python main.py --target pyramid --step 3 --settle 0.03
python main.py --target none --view

# Save outputs to a custom folder
python main.py --target cube --outdir outputs_cube

# Camera settings
python main.py --width 2328 --height 1748 --fps 30

# Servo settings
python main.py --min-us 1000 --max-us 2000

# Stereo distance settings
python main.py --baseline-m 0.075 --fx-px 1893 --min-disp 2.0 --max-dy 60.0
```

Full example with all main options:

```bash
python main.py \
  --target cube \
  --step 5 \
  --settle 0.05 \
  --view \
  --outdir outputs \
  --edge-margin 10 \
  --width 2328 \
  --height 1748 \
  --fps 30 \
  --min-us 1000 \
  --max-us 2000 \
  --baseline-m 0.075 \
  --fx-px 1893 \
  --min-disp 2.0 \
  --max-dy 60.0
```

## Servo manual control (`servo/manual_control.py`)

Interactive script to move the servo by typing angles. Useful for calibration.

```bash
python servo/manual_control.py
```

Commands at the prompt:

| Input | Action |
|-------|--------|
| `0`–`180` | Move servo to that angle |
| `c` | Calibration sweep: 0 → 90 → 180 → 90 → 0 |
| `q` | Quit |

## Straight-line drive test (`test_drive_distance.py`)

Drive straight ahead a given distance using IMU dead-reckoning and gyro correction.

```bash
python test_drive_distance.py
```

You will be prompted to enter a distance in centimetres:

```
Enter distance (cm): 20
```

The robot calibrates the IMU, drives straight to the target distance, then stops.
Press `Ctrl+C` at any time for an emergency stop.

## Angle + distance drive test (`test_angle_motors.py`)

Two modes — rotate to an angle then drive, or run a preset rotation sweep.

```bash
python test_angle_motors.py
```

```
Choose mode (test / normal): normal
Enter angle (0–180°): 90
Enter distance (cm): 20
```

| Mode | What it does |
|------|--------------|
| `normal` | Rotate to the given angle then drive the given distance |
| `test` | Automatic sweep through 0°, 45°, 90°, 135°, 180° |

## Distance tool (`measure_distance.py`)

Show options:

```bash
python measure_distance.py --help
```

Run with defaults:

```bash
python measure_distance.py
```

Run with custom stereo params:

```bash
python measure_distance.py --baseline-m 0.075 --fx-px 1893 --min-disp 2.0 --max-dy 60.0
```
