#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys

import hparams as HP
from config import (
    AppConfig,
    CameraConfig,
    DetectionConfig,
    ScanConfig,
    ServoConfig,
    StereoConfig,
)
from scanner import Scanner
from utils import normalize_target_name

# parses CLI arguments, builds AppConfig, and starts the stereo scanner

def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="Stereo-vision pan-scan for blue 3-D targets.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    ap.add_argument(
        "--target",
        type=str,
        default=None,
        metavar="CLASS",
        help="Cube | Cylinder | Pyramid | None  (None = any blue target)",
    )

    g_scan = ap.add_argument_group("Scan behaviour")
    g_scan.add_argument("--step",   type=int,   default=HP.SCAN_STEP_DEG,    help="Servo step size (degrees)")
    g_scan.add_argument("--settle", type=float, default=HP.SCAN_SETTLE_S,   help="Settle time after each servo move (s)")
    g_scan.add_argument("--view",   action="store_true",                     help="Show live OpenCV preview window")
    g_scan.add_argument("--outdir", type=str,   default=HP.SCAN_OUTDIR,     help="Directory for saved detections")
    g_scan.add_argument("--edge-margin", type=int, default=HP.SCAN_EDGE_MARGIN_PX, help="Min pixels from edge for valid bbox")
    g_scan.add_argument(
        "--center-left-frac",
        type=float,
        default=HP.SCAN_CENTER_LEFT_FRAC,
        help="Left vertical gate as frame-width fraction (0..1)",
    )
    g_scan.add_argument(
        "--center-right-frac",
        type=float,
        default=HP.SCAN_CENTER_RIGHT_FRAC,
        help="Right vertical gate as frame-width fraction (0..1)",
    )

    g_cam = ap.add_argument_group("Camera")
    g_cam.add_argument("--width",  type=int, default=HP.CAMERA_WIDTH,  help="Capture width (px)")
    g_cam.add_argument("--height", type=int, default=HP.CAMERA_HEIGHT, help="Capture height (px)")
    g_cam.add_argument("--fps",    type=int, default=HP.CAMERA_FPS,    help="Target frame rate")

    g_servo = ap.add_argument_group("Servo")
    g_servo.add_argument("--min-us", type=int, default=HP.SERVO_MIN_US, help="Servo min pulse (µs) = 0°")
    g_servo.add_argument("--max-us", type=int, default=HP.SERVO_MAX_US, help="Servo max pulse (µs) = 180°")

    g_stereo = ap.add_argument_group("Stereo distance")
    g_stereo.add_argument("--baseline-m", type=float, default=HP.STEREO_BASELINE_M,           help="Camera baseline (m)")
    g_stereo.add_argument("--fx-px",      type=float, default=HP.STEREO_FX_PX,               help="Focal length (px)")
    g_stereo.add_argument("--min-disp",   type=float, default=HP.STEREO_MIN_DISPARITY_PX,    help="Min |disparity| to compute Z (px)")
    g_stereo.add_argument("--max-dy",     type=float, default=HP.STEREO_MAX_VERTICAL_OFFSET_PX, help="Max |yL-yR| for stereo validity (px)")

    return ap


def build_config(args: argparse.Namespace) -> AppConfig:
    try:
        target = normalize_target_name(args.target)
    except ValueError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)

    return AppConfig(
        camera=CameraConfig(
            width=args.width,
            height=args.height,
            fps=args.fps,
        ),
        servo=ServoConfig(
            min_us=args.min_us,
            max_us=args.max_us,
        ),
        detection=DetectionConfig(),
        stereo=StereoConfig(
            baseline_m=args.baseline_m,
            fx_px=args.fx_px,
            min_disparity_px=args.min_disp,
            max_vertical_offset_px=args.max_dy,
        ),
        scan=ScanConfig(
            step_deg=args.step,
            settle_s=args.settle,
            edge_margin_px=args.edge_margin,
            center_left_frac=args.center_left_frac,
            center_right_frac=args.center_right_frac,
            outdir=args.outdir,
            view=args.view,
            target=target,
        ),
    )


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    config = build_config(args)

    if config.scan.view:
        print("[INFO] Press 'c' in the preview window to stop.")
    scanner = Scanner(config)
    result = scanner.run()
    if result is not None:
        print("Result:", result)


if __name__ == "__main__":
    main()
