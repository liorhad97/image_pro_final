#!/usr/bin/env python3
"""
Backwards-compatibility shim.

The project has been refactored into sub-packages.
Please use  `python main.py`  going forward.
"""
# Re-export public API so old `import scan` code keeps working.
from main import main                                      # noqa: F401
from config import AppConfig                               # noqa: F401
from detection import DetResult, BlueObjectDetector        # noqa: F401
from stereo import StereoCams, StereoDistanceEstimator     # noqa: F401
from servo import ServoController                          # noqa: F401
from scanner import Scanner, ScanResult                    # noqa: F401
from utils import (                                        # noqa: F401
    hstack_resize,
    save_mask_png,
    bbox_margin_ok,
    normalize_target_name,
)

if __name__ == "__main__":
    main()


import numpy as np
import cv2

from rpi_hardware_pwm import HardwarePWM
from picamera2 import Picamera2
from pathlib import Path


# -----------------------------
# Servo (HW PWM)
# -----------------------------
def angle_to_duty(angle: float, min_us: int = 1000, max_us: int = 2000) -> float:
    """
    Map angle [0..180] to duty cycle (%) for 50Hz servo PWM.
    """
    angle = max(0.0, min(180.0, float(angle)))
    pulse_us = min_us + (max_us - min_us) * (angle / 180.0)
    return (pulse_us / 20000.0) * 100.0  # 50Hz => 20ms => 20000us


# -----------------------------
# Detection / classification
# -----------------------------
@dataclass
class DetResult:
    found: bool
    cls: str  # "Pyramid" / "Cube" / "Cylinder" / "None"
    bbox: Optional[Tuple[int, int, int, int]]   # x,y,w,h (full-res coordinates)
    center: Optional[Tuple[int, int]]           # cx,cy (full-res coordinates)
    mask_raw: Optional[np.ndarray]              # raw inRange mask (full-res)
    mask_clean: Optional[np.ndarray]            # after morphology (full-res)


def detect_blue_and_classify(
    bgr: np.ndarray,
    hsv_lo=(105, 120, 100),
    hsv_hi=(123, 255, 128),
    min_area=50,
    downscale_width=640
) -> DetResult:
    """
    Blue HSV segmentation -> largest contour -> bbox+center -> classify.
    Downscales for speed, then maps bbox back to full res.
    Returns BOTH raw and cleaned masks for debugging.
    """
    H, W = bgr.shape[:2]
    scale = 1.0
    img = bgr

    if W > downscale_width:
        scale = downscale_width / float(W)
        img = cv2.resize(
            bgr, (int(W * scale), int(H * scale)),
            interpolation=cv2.INTER_AREA
        )

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask_raw = cv2.inRange(
        hsv,
        np.array(hsv_lo, dtype=np.uint8),
        np.array(hsv_hi, dtype=np.uint8)
    )

    # Morphological cleanup
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask_clean = cv2.morphologyEx(mask_raw, cv2.MORPH_CLOSE, k, iterations=2)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN,  k, iterations=1)

    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    def _full_masks():
        if scale != 1.0:
            raw_full = cv2.resize(mask_raw, (W, H), interpolation=cv2.INTER_NEAREST)
            clean_full = cv2.resize(mask_clean, (W, H), interpolation=cv2.INTER_NEAREST)
        else:
            raw_full, clean_full = mask_raw, mask_clean
        return raw_full, clean_full

    if not contours:
        raw_full, clean_full = _full_masks()
        return DetResult(False, "None", None, None, raw_full, clean_full)

    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    if area < min_area:
        raw_full, clean_full = _full_masks()
        return DetResult(False, "None", None, None, raw_full, clean_full)

    x, y, w, h = cv2.boundingRect(cnt)
    cx, cy = x + w // 2, y + h // 2

    per = cv2.arcLength(cnt, True)
    if per < 1e-6:
        raw_full, clean_full = _full_masks()
        return DetResult(True, "None", None, None, raw_full, clean_full)

    extent = area / float(w * h + 1e-9)
    circularity = 4.0 * np.pi * area / (per * per + 1e-12)
    approx = cv2.approxPolyDP(cnt, 0.02 * per, True)
    n = len(approx)

    # Simple heuristics (tune on your dataset)
    # NOTE: A front-view cylinder can sometimes look rectangular and be classified as "Cube".
    if extent < 0.58 and n <= 6:
        cls = "Pyramid"
    elif circularity > 0.78 or n >= 8:
        cls = "Cylinder"
    else:
        cls = "Cube"

    # Map bbox/center + masks back to full resolution
    if scale != 1.0:
        inv = 1.0 / scale
        x = int(x * inv)
        y = int(y * inv)
        w = int(w * inv)
        h = int(h * inv)
        cx = int(cx * inv)
        cy = int(cy * inv)

        mask_raw_full = cv2.resize(mask_raw, (W, H), interpolation=cv2.INTER_NEAREST)
        mask_clean_full = cv2.resize(mask_clean, (W, H), interpolation=cv2.INTER_NEAREST)
    else:
        mask_raw_full = mask_raw
        mask_clean_full = mask_clean

    return DetResult(True, cls, (x, y, w, h), (cx, cy), mask_raw_full, mask_clean_full)


def draw_det(bgr: np.ndarray, det: DetResult, label_prefix: str = "") -> np.ndarray:
    out = bgr.copy()

    if not det.found or det.bbox is None or det.center is None:
        cv2.putText(
            out, f"{label_prefix}None", (15, 35),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
        )
        return out

    x, y, w, h = det.bbox
    cx, cy = det.center

    cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 3)
    cv2.circle(out, (cx, cy), 7, (0, 0, 255), -1)

    cv2.putText(
        out, f"{label_prefix}{det.cls}", (x, max(35, y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
    )
    cv2.putText(
        out, f"({cx},{cy})", (x, y + h + 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
    )
    return out


# -----------------------------
# Cameras (2x Picamera2)
# -----------------------------
class StereoCams:
    """
    Per Picamera2 docs:
    - Using format="RGB888" gives a 3-channel uint8 array suitable for OpenCV use.
    - In practice with Picamera2/libcamera this is often already OpenCV-ready BGR ordering.
    - Therefore we DO NOT force cvtColor here.
    """
    def __init__(self, cam0=0, cam1=1, size=(2328, 1748), fps=30):
        self.camL = Picamera2(cam0)
        self.camR = Picamera2(cam1)

        frame_us = int(round(1_000_000 / fps))
        controls = {
            "FrameDurationLimits": (frame_us, frame_us),
            "AeEnable": True,
            "AwbEnable": True,
        }

        confL = self.camL.create_video_configuration(
            main={"size": size, "format": "RGB888"},
            controls=controls
        )
        confR = self.camR.create_video_configuration(
            main={"size": size, "format": "RGB888"},
            controls=controls
        )

        self.camL.configure(confL)
        self.camR.configure(confR)

    def start(self):
        self.camL.start()
        self.camR.start()
        time.sleep(0.3)

        # Warm-up to let AE/AWB settle
        for _ in range(8):
            _ = self.camL.capture_array("main")
            _ = self.camR.capture_array("main")
            time.sleep(0.01)

    def stop(self):
        self.camL.stop()
        self.camR.stop()

    def capture_bgr(self) -> Tuple[np.ndarray, np.ndarray]:
        bgrL = self.camL.capture_array("main")
        bgrR = self.camR.capture_array("main")

        # Safety in case a 4-channel format is returned
        if bgrL.ndim == 3 and bgrL.shape[2] == 4:
            bgrL = bgrL[:, :, :3]
        if bgrR.ndim == 3 and bgrR.shape[2] == 4:
            bgrR = bgrR[:, :, :3]

        return bgrL, bgrR


def hstack_resize(a: np.ndarray, b: np.ndarray, max_width: int = 1920) -> np.ndarray:
    h = min(a.shape[0], b.shape[0])
    a2 = cv2.resize(a, (int(a.shape[1] * (h / a.shape[0])), h), interpolation=cv2.INTER_AREA)
    b2 = cv2.resize(b, (int(b.shape[1] * (h / b.shape[0])), h), interpolation=cv2.INTER_AREA)
    combo = np.hstack([a2, b2])

    if combo.shape[1] > max_width:
        s = max_width / float(combo.shape[1])
        combo = cv2.resize(
            combo,
            (int(combo.shape[1] * s), int(combo.shape[0] * s)),
            interpolation=cv2.INTER_AREA
        )
    return combo


# -----------------------------
# Stereo distance helpers
# -----------------------------
def estimate_distance_from_disparity(
    detL: DetResult,
    detR: DetResult,
    fx_px: float,
    baseline_m: float,
    min_disparity_px: float = 2.0,
    max_vertical_offset_px: float = 60.0,
) -> Tuple[Optional[float], Dict[str, float], Optional[str]]:
    """
    Estimate depth Z (meters) from stereo disparity using center points.

    Z = fx * B / |disparity|

    Returns:
        z_m (or None),
        debug dict,
        error string (or None)
    """
    if not detL.found or not detR.found or detL.center is None or detR.center is None:
        return None, {}, "missing_detection"

    xL, yL = detL.center
    xR, yR = detR.center

    disparity = float(xL - xR)  # sign depends on camera ordering
    d_abs = abs(disparity)
    dy = abs(float(yL - yR))

    dbg = {
        "disparity_px": disparity,
        "disparity_abs_px": d_abs,
        "dy_px": dy,
        "xL": float(xL), "yL": float(yL),
        "xR": float(xR), "yR": float(yR),
    }

    # If images aren't rectified well, large vertical mismatch makes disparity unreliable
    if dy > max_vertical_offset_px:
        return None, dbg, "vertical_mismatch"

    if d_abs < min_disparity_px:
        return None, dbg, "disparity_too_small"

    if fx_px <= 0 or baseline_m <= 0:
        return None, dbg, "invalid_calibration"

    z_m = (fx_px * baseline_m) / d_abs
    return float(z_m), dbg, None


def draw_stereo_metrics_overlay(
    img: np.ndarray,
    z_m: Optional[float],
    dbg: Dict[str, float],
    err: Optional[str],
    wanted_label: str = "",
    extra_status: str = ""
) -> np.ndarray:
    """
    Draw disparity/distance info on combined stereo preview.
    """
    out = img.copy()
    y0 = 35

    if wanted_label:
        cv2.putText(out, wanted_label, (15, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)
        y0 += 32

    if extra_status:
        cv2.putText(out, extra_status, (15, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 200, 0), 2)
        y0 += 30

    if dbg:
        disp = dbg.get("disparity_px", 0.0)
        dy = dbg.get("dy_px", 0.0)
        cv2.putText(
            out,
            f"disp={disp:.1f}px  |disp|={abs(disp):.1f}px  dy={dy:.1f}px",
            (15, y0),
            cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 255), 2
        )
        y0 += 28

    if z_m is not None:
        cv2.putText(
            out,
            f"Distance Z = {z_m:.3f} m ({z_m * 100.0:.1f} cm)",
            (15, y0),
            cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2
        )
    else:
        msg = f"Distance unavailable ({err})" if err else "Distance unavailable"
        cv2.putText(
            out,
            msg,
            (15, y0),
            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2
        )

    return out


# -----------------------------
# Utility helpers
# -----------------------------
def save_mask_png(path: Path, mask: np.ndarray) -> None:
    """
    Save binary/uint8 mask to PNG with visible 0/255 values.
    """
    if mask is None:
        return

    m = mask
    if m.dtype != np.uint8:
        m = m.astype(np.uint8)

    if m.max() <= 1:
        m = (m * 255).astype(np.uint8)

    cv2.imwrite(str(path), m)


def bbox_margin_ok(det: DetResult, img_w: int, img_h: int, margin_px: int) -> bool:
    """
    True only if the bbox is at least margin_px away from all image edges.
    """
    if (not det.found) or (det.bbox is None):
        return False

    x, y, w, h = det.bbox
    if x < margin_px:
        return False
    if y < margin_px:
        return False
    if (x + w) > (img_w - margin_px):
        return False
    if (y + h) > (img_h - margin_px):
        return False
    return True


def normalize_target_name(name: Optional[str]) -> Optional[str]:
    if name is None:
        return None
    s = str(name).strip()
    if s.lower() == "none":
        return None
    s_lower = s.lower()
    mapping = {
        "cube": "Cube",
        "cylinder": "Cylinder",
        "pyramid": "Pyramid",
    }
    if s_lower not in mapping:
        raise ValueError(f"Invalid target '{name}'. Use: Cube, Cylinder, Pyramid, or None")
    return mapping[s_lower]


# -----------------------------
# Scan logic
# -----------------------------
def scan_for_target(
    wanted_target: Optional[str],
    outdir: str = "outputs",
    pwm_channel=0,
    servo_min_us=1000,
    servo_max_us=2000,
    step_deg=5,
    settle_s=0.05,
    view=False,
    edge_margin_px=10,
    width=2328,
    height=1748,
    fps=30,
    baseline_m: float = 0.075,          # camera center-to-center distance [m]
    fx_px: float = 1893.0,             # focal length in pixels (calibrate this!)
    min_disparity_px: float = 2.0,
    max_vertical_offset_px: float = 60.0,
):
    """
    Continuous-like scan: 0 -> 180 -> 0 in small steps.
    Saves ONLY frames where a target is detected.
    Accepts final detection only when:
      1) both cameras detect a target,
      2) both cameras detect the SAME class,
      3) class matches wanted_target (if provided),
      4) both bounding boxes are >= edge_margin_px away from image edges.
    Also computes stereo distance from disparity and overlays it on screen.
    """
    outdir = Path(outdir)
    left_dir = outdir / "left"
    right_dir = outdir / "right"
    both_dir = outdir / "both"
    masks_raw_dir = outdir / "masks_raw"
    masks_clean_dir = outdir / "masks_clean"

    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)
    both_dir.mkdir(parents=True, exist_ok=True)
    masks_raw_dir.mkdir(parents=True, exist_ok=True)
    masks_clean_dir.mkdir(parents=True, exist_ok=True)

    pwm = HardwarePWM(pwm_channel=pwm_channel, hz=50, chip=0)
    cams = StereoCams(cam0=0, cam1=1, size=(width, height), fps=fps)

    def match(det: DetResult) -> bool:
        if not det.found:
            return False
        if wanted_target is None:
            return True
        return det.cls == wanted_target

    try:
        pwm.start(angle_to_duty(90, servo_min_us, servo_max_us))
        print("Servo started (HW PWM channel 0 -> GPIO12). Ctrl+C / ESC / q to stop.")
        print(f"Stereo config: {width}x{height} @ {fps}fps | baseline={baseline_m}m | fx={fx_px}px")
        cams.start()

        forward = list(range(0, 181, step_deg))
        backward = list(range(180, -1, -step_deg))
        sweep = forward + backward[1:-1]

        event_idx = 0

        while True:
            for ang in sweep:
                pwm.change_duty_cycle(angle_to_duty(ang, servo_min_us, servo_max_us))
                time.sleep(settle_s)

                frameL, frameR = cams.capture_bgr()
                detL = detect_blue_and_classify(frameL)
                detR = detect_blue_and_classify(frameR)

                okL = match(detL)
                okR = match(detR)

                same_class = (
                    detL.found and detR.found and
                    detL.cls != "None" and detR.cls != "None" and
                    detL.cls == detR.cls
                )

                edge_okL = bbox_margin_ok(detL, width, height, edge_margin_px)
                edge_okR = bbox_margin_ok(detR, width, height, edge_margin_px)

                both_ok = okL and okR and same_class and edge_okL and edge_okR

                # Stereo distance estimation (only if same class detected)
                z_m = None
                stereo_dbg: Dict[str, float] = {}
                stereo_err: Optional[str] = None
                if same_class:
                    z_m, stereo_dbg, stereo_err = estimate_distance_from_disparity(
                        detL, detR,
                        fx_px=fx_px,
                        baseline_m=baseline_m,
                        min_disparity_px=min_disparity_px,
                        max_vertical_offset_px=max_vertical_offset_px,
                    )

                print(
                    f"pan={ang:3d} | "
                    f"L={detL.cls:8s} {'OK' if okL else '--'} {'EDGE_OK' if edge_okL else 'EDGE_BAD'} | "
                    f"R={detR.cls:8s} {'OK' if okR else '--'} {'EDGE_OK' if edge_okR else 'EDGE_BAD'} | "
                    f"SAME={'Y' if same_class else 'N'} | "
                    f"Z={'%.3fm' % z_m if z_m is not None else stereo_err or '--'}"
                )

                annL = draw_det(frameL, detL, label_prefix="L: ")
                annR = draw_det(frameR, detR, label_prefix="R: ")

                ts = int(time.time() * 1000)

                # Save masks + per-camera annotated frames whenever that camera matched target criteria
                if okL and detL.mask_raw is not None and detL.mask_clean is not None:
                    save_mask_png(
                        masks_raw_dir / f"ev_{event_idx:04d}_t{ts}_pan{ang:03d}_L_RAW_{detL.cls}.png",
                        detL.mask_raw
                    )
                    save_mask_png(
                        masks_clean_dir / f"ev_{event_idx:04d}_t{ts}_pan{ang:03d}_L_CLEAN_{detL.cls}.png",
                        detL.mask_clean
                    )
                    cv2.imwrite(str(left_dir / f"ev_{event_idx:04d}_t{ts}_pan{ang:03d}_L_{detL.cls}.jpg"), annL)

                if okR and detR.mask_raw is not None and detR.mask_clean is not None:
                    save_mask_png(
                        masks_raw_dir / f"ev_{event_idx:04d}_t{ts}_pan{ang:03d}_R_RAW_{detR.cls}.png",
                        detR.mask_raw
                    )
                    save_mask_png(
                        masks_clean_dir / f"ev_{event_idx:04d}_t{ts}_pan{ang:03d}_R_CLEAN_{detR.cls}.png",
                        detR.mask_clean
                    )
                    cv2.imwrite(str(right_dir / f"ev_{event_idx:04d}_t{ts}_pan{ang:03d}_R_{detR.cls}.jpg"), annR)

                # Combined view (save only when final acceptance condition is met)
                if both_ok:
                    combo = hstack_resize(annL, annR, max_width=1920)
                    status = (
                        f"SAME class={detL.cls} | edge_margin={edge_margin_px}px "
                        f"| z={'%.3fm' % z_m if z_m is not None else 'N/A'}"
                    )
                    combo = draw_stereo_metrics_overlay(
                        combo,
                        z_m=z_m,
                        dbg=stereo_dbg,
                        err=stereo_err,
                        wanted_label=f"Target: {wanted_target if wanted_target is not None else 'Any'}",
                        extra_status=status
                    )
                    cv2.imwrite(str(both_dir / f"ev_{event_idx:04d}_t{ts}_pan{ang:03d}_BOTH_{detL.cls}.jpg"), combo)

                # Live preview
                if view:
                    combo_view = hstack_resize(annL, annR, max_width=1920)
                    extra_status = (
                        f"SAME={'Y' if same_class else 'N'}  "
                        f"EDGE_L={'OK' if edge_okL else 'BAD'}  "
                        f"EDGE_R={'OK' if edge_okR else 'BAD'}"
                    )
                    combo_view = draw_stereo_metrics_overlay(
                        combo_view,
                        z_m=z_m,
                        dbg=stereo_dbg,
                        err=stereo_err,
                        wanted_label=f"Target: {wanted_target if wanted_target is not None else 'Any'} | pan={ang} deg",
                        extra_status=extra_status
                    )
                    cv2.imshow("Stereo Scan (L | R)", combo_view)

                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord('q')):  # ESC or q
                        print("User exit (ESC/q).")
                        return None

                if okL or okR:
                    event_idx += 1

                if both_ok:
                    print("\nFOUND target in BOTH cameras (confirmed same class + edge margin check)!")
                    print(f"Angle: {ang} deg")
                    print(f"Left  bbox={detL.bbox}, center={detL.center}, class={detL.cls}")
                    print(f"Right bbox={detR.bbox}, center={detR.center}, class={detR.cls}")
                    if z_m is not None:
                        print(f"Estimated distance: {z_m:.3f} m ({z_m * 100.0:.1f} cm)")
                    else:
                        print(f"Distance unavailable: {stereo_err}, dbg={stereo_dbg}")
                    print(f"Saved detected frames + masks to: {outdir.resolve()}")

                    return {
                        "angle_deg": ang,
                        "left": {
                            "bbox": detL.bbox,
                            "center": detL.center,
                            "class": detL.cls,
                        },
                        "right": {
                            "bbox": detR.bbox,
                            "center": detR.center,
                            "class": detR.cls,
                        },
                        "stereo": {
                            "distance_m": z_m,
                            "distance_cm": (z_m * 100.0) if z_m is not None else None,
                            "debug": stereo_dbg,
                            "error": stereo_err,
                            "baseline_m": baseline_m,
                            "fx_px": fx_px,
                        }
                    }

    finally:
        try:
            cams.stop()
        except Exception:
            pass
        try:
            pwm.stop()
        except Exception:
            pass
        if view:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser()

    ap.add_argument(
        "--target", type=str, default=None,
        help="Cube | Cylinder | Pyramid | None (default: accept any blue target)"
    )

    ap.add_argument("--step", type=int, default=5, help="servo step degrees")
    ap.add_argument("--settle", type=float, default=0.05, help="seconds to wait after each servo move")
    ap.add_argument("--view", action="store_true", help="show OpenCV live preview")
    ap.add_argument("--outdir", type=str, default="outputs", help="save detections here")

    ap.add_argument("--min_us", type=int, default=1000, help="servo min pulse width (us)")
    ap.add_argument("--max_us", type=int, default=2000, help="servo max pulse width (us)")

    # Image/frame settings (requested)
    ap.add_argument("--width", type=int, default=2328, help="camera width")
    ap.add_argument("--height", type=int, default=1748, help="camera height")
    ap.add_argument("--fps", type=int, default=30, help="camera FPS")

    # Acceptance constraints
    ap.add_argument("--edge-margin", type=int, default=10,
                    help="require bbox to be at least this many pixels from image edge in BOTH cameras")

    # Stereo distance parameters
    ap.add_argument("--baseline-m", type=float, default=0.075,
                    help="camera baseline (center-to-center) in meters")
    ap.add_argument("--fx-px", type=float, default=1900.0,
                    help="focal length in pixels (calibrated)")
    ap.add_argument("--min-disp", type=float, default=2.0,
                    help="minimum |disparity| in pixels to compute distance")
    ap.add_argument("--max-dy", type=float, default=60.0,
                    help="maximum allowed |yL-yR| (px) for stereo distance")

    args = ap.parse_args()

    try:
        wanted = normalize_target_name(args.target)
    except ValueError as e:
        print(e)
        return

    res = scan_for_target(
        wanted_target=wanted,
        outdir=args.outdir,
        step_deg=args.step,
        settle_s=args.settle,
        view=args.view,
        servo_min_us=args.min_us,
        servo_max_us=args.max_us,
        edge_margin_px=args.edge_margin,
        width=args.width,
        height=args.height,
        fps=args.fps,
        baseline_m=args.baseline_m,
        fx_px=args.fx_px,
        min_disparity_px=args.min_disp,
        max_vertical_offset_px=args.max_dy,
    )
    print("Result:", res)


if __name__ == "__main__":
    main()
