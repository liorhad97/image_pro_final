# Potential Issues — image_pro_final

> Reviewed: 2026-02-23

---

## Bugs

### 1. `measure_distance.py:53` — `--downscale` CLI arg ignored by detector

`args.downscale` is parsed but never passed to `BlueObjectDetector`. The detector always uses `HP.DOWNSCALE_WIDTH`. The flag only ends up controlling the display window width (`args.downscale * 2`), not actual detection processing.

```python
# current (broken)
detector = BlueObjectDetector()   # ignores args.downscale

# should be
detector = BlueObjectDetector(downscale_width=args.downscale)
```

---

### 2. `stereo/camera.py:51-52` — Resource leak when second camera fails to open

If `_init_camera(cam1, ...)` raises (e.g. camera not present), `_cam_left` is already initialised and its underlying file handle is never released. No `__del__` or context manager exists to clean up on partial init failure.

```python
self._cam_left  = self._init_camera(cam0, size, controls)
self._cam_right = self._init_camera(cam1, size, controls)  # throws → cam_left leaks
```

---

### 3. `detection/detector.py:68-71` — `found=True` with `cls="None"` is a semantic trap

When a contour is found but has a near-zero perimeter, the code returns a result with `found=True` but `bbox=None` and `center=None`. `_matches_target` in the scanner only checks `det.found`, so when `target is None` (accept any class) this result passes as a valid match. Any downstream code that unpacks `bbox` or `center` without first checking `is_valid` will raise a `TypeError`.

```python
# detector.py
return DetResult(
    found=True, cls="None", bbox=None, center=None, ...  # is_valid == False!
)

# scanner.py — only checks found, not is_valid
def _matches_target(self, det: DetResult) -> bool:
    if not det.found:       # ← True, passes through
        return False
    if self._cfg.scan.target is None:
        return True         # ← returns True for a bbox-less result
```

---

### 4. `scanner/scanner.py:112` — Indentation inconsistency (2-space vs 4-space)

The `for` loop inside `while True` is indented with 2 spaces while every other block in the file uses 4 spaces. Python accepts it, but it's a latent bug magnet if code is added at the same level.

```python
        while True:
          for ang in self._sweep_angles():   # ← 2 spaces
              ...                            # ← 4 spaces
```

---

## Logic / Correctness Issues

### 5. `stereo/camera.py:80-81` — Frames captured sequentially, not simultaneously

Left and right frames are grabbed one after the other. The right frame is captured approximately one frame period after the left. For any target that moves (or if the servo is still settling), this timing skew produces incorrect disparity and therefore wrong distance estimates.

```python
left  = self._cam_left.capture_array("main")   # t=0
right = self._cam_right.capture_array("main")  # t≈33ms later at 30fps
```

**Fix direction:** Use threading to trigger both captures at as close to the same instant as possible.

---

### 6. `scanner/scanner.py:163-169` — Saves a file for every single partially-matched frame

When `--view` is **not** passed, `_save_per_camera` is called on every frame where either camera has any match — not just confirmed detections. At 30 fps over a 180° sweep this can write hundreds of annotated JPEGs and PNGs per second, filling disk quickly.

```python
if not self._cfg.scan.view:
    self._save_per_camera(...)   # every frame with a partial match
    if confirmed:
        self._save_combined(...) # only on confirmed
```

---

### 7. `_sweep_angles()` — 180° endpoint missed when step doesn't divide 180 evenly

`range(0, 181, step)` only includes 180 when `180 % step == 0`. For example, `--step 7` produces `0, 7, 14, ..., 175` — the 180° position is never scanned.

```python
forward = list(range(0, 181, step))   # step=7 → last value is 175
```

---

## Design / Configuration Issues

### 8. `hparams.SHOW_PREVIEW` + `scanner.py:329` — Double-gated preview breaks the `'c'` quit key

Two separate flags gate the preview window. If `SHOW_PREVIEW = False` in `hparams.py` but `--view` is passed on the CLI, `_show_preview` is entered, no window appears, `cv2.waitKey` is never called, and pressing `'c'` can never stop the scanner — the only exit becomes Ctrl+C or finding the target.

```python
# scanner.py
if self._cfg.scan.view:              # outer gate (CLI)
    quit_requested = self._show_preview(...)

# inside _show_preview
if hparams.SHOW_PREVIEW:             # inner gate (hparams)
    cv2.imshow(...)
    key = cv2.waitKey(1) & 0xFF
    return key == ord("c")
return False                         # ← always returned when SHOW_PREVIEW=False
```

---

### 9. `config.py:58` — Uncalibrated `fx_px` silently produces wrong distances

The focal length default `1893.0 px` has a comment saying "calibrate this!" but there is no runtime warning when the default is used. All distance outputs will be silently wrong on any camera that hasn't been calibrated.

```python
fx_px: float = 1893.0   # horizontal focal length [px] — calibrate this!
```

---

## Minor / Code Quality

### 10. `scanner/scanner.py:201` — Unreachable `return None`

The `while True` loop only exits by `return`-ing a value or propagating an exception through the `finally` block. The `return None` after `finally` can never be reached.

```python
    finally:
        self._cleanup()

    return None   # ← dead code
```

---

## Summary Table

| # | File | Line | Severity | Description |
|---|------|------|----------|-------------|
| 1 | `measure_distance.py` | 53 | **Bug** | `--downscale` arg ignored by detector |
| 2 | `stereo/camera.py` | 51–52 | **Bug** | Resource leak if second camera fails to open |
| 3 | `detection/detector.py` | 68–71 | **Bug** | `found=True` + `bbox=None` can cause `TypeError` |
| 4 | `scanner/scanner.py` | 112 | Style | Mixed 2/4-space indentation |
| 5 | `stereo/camera.py` | 80–81 | Logic | Sequential capture causes stereo timing skew |
| 6 | `scanner/scanner.py` | 163–169 | Logic | Saves every partial-match frame, not just confirmed |
| 7 | `scanner/scanner.py` | 204–208 | Logic | 180° endpoint skipped for non-divisor step values |
| 8 | `hparams.py` + `scanner.py` | 329 | Design | Double-gated preview breaks `'c'` quit key |
| 9 | `config.py` | 58 | Design | Default `fx_px` silently gives wrong distances |
| 10 | `scanner/scanner.py` | 201 | Minor | Unreachable `return None` after `finally` |
