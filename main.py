#!/usr/bin/env python3
"""
Entry point for the stereo-vision object scanner.

Run:
    python main.py --target Cube --view
    python main.py --target Cylinder --step 3 --settle 0.03
    python main.py                         # accept any blue object
"""
from __future__ import annotations

import argparse
import sys

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


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="Stereo-vision pan-scan for blue 3-D targets.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Target
    ap.add_argument(
        "--target",
        type=str,
        default=None,
        metavar="CLASS",
        help="Cube | Cylinder | Pyramid | None  (None = any blue target)",
    )

    # Scan behaviour
    g_scan = ap.add_argument_group("Scan behaviour")
    g_scan.add_argument("--step",   type=int,   default=5,       help="Servo step size (degrees)")
    g_scan.add_argument("--settle", type=float, default=0.05,    help="Settle time after each servo move (s)")
    g_scan.add_argument("--view",   action="store_true",          help="Show live OpenCV preview window")
    g_scan.add_argument("--outdir", type=str,   default="outputs", help="Directory for saved detections")
    g_scan.add_argument("--edge-margin", type=int, default=10,   help="Min pixels from edge for valid bbox")

    # Camera
    g_cam = ap.add_argument_group("Camera")
    g_cam.add_argument("--width",  type=int, default=2328, help="Capture width (px)")
    g_cam.add_argument("--height", type=int, default=1748, help="Capture height (px)")
    g_cam.add_argument("--fps",    type=int, default=30,   help="Target frame rate")

    # Servo
    g_servo = ap.add_argument_group("Servo")
    g_servo.add_argument("--min-us", type=int, default=1000, help="Servo min pulse (µs) = 0°")
    g_servo.add_argument("--max-us", type=int, default=2000, help="Servo max pulse (µs) = 180°")

    # Stereo distance
    g_stereo = ap.add_argument_group("Stereo distance")
    g_stereo.add_argument("--baseline-m", type=float, default=0.075,  help="Camera baseline (m)")
    g_stereo.add_argument("--fx-px",      type=float, default=1893.0, help="Focal length (px)")
    g_stereo.add_argument("--min-disp",   type=float, default=2.0,    help="Min |disparity| to compute Z (px)")
    g_stereo.add_argument("--max-dy",     type=float, default=60.0,   help="Max |yL-yR| for stereo validity (px)")

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
        detection=DetectionConfig(),   # HSV defaults from config.py
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
            outdir=args.outdir,
            view=args.view,
            target=target,
        ),
    )


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    config = build_config(args)

    scanner = Scanner(config)
    result = scanner.run()
    print("Result:", result)


if __name__ == "__main__":
    main()
