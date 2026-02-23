#!/usr/bin/env python3
"""
Live stereo distance measurement.

Captures frames from both cameras, detects the blue object in each,
and shows a side-by-side window with the measured distance.

Usage
-----
    python measure_distance.py
    python measure_distance.py --baseline-m 0.075 --fx-px 1893

Press 'q' or Esc to quit.
"""
from __future__ import annotations

import argparse
import sys

import cv2

import hparams as HP
from utils.image_utils import hstack_resize


def _positive_int(value: str) -> int:
    n = int(value)
    if n <= 0:
        raise argparse.ArgumentTypeError("must be > 0")
    return n


def _non_negative_float(value: str) -> float:
    x = float(value)
    if x < 0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return x


def _positive_float(value: str) -> float:
    x = float(value)
    if x <= 0:
        raise argparse.ArgumentTypeError("must be > 0")
    return x


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Live stereo distance measurement for a blue object.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--baseline-m", type=_positive_float, default=0.075,
                    help="Camera baseline — centre-to-centre separation (m)")
    ap.add_argument("--fx-px",      type=_positive_float, default=1893.0,
                    help="Horizontal focal length from calibration (px)")
    ap.add_argument("--min-disp",   type=_positive_float, default=2.0,
                    help="Min |disparity| to trust a distance reading (px)")
    ap.add_argument("--max-dy",     type=_non_negative_float, default=60.0,
                    help="Max vertical offset |yL-yR| before rejecting (px)")
    ap.add_argument("--width",  type=_positive_int, default=2328)
    ap.add_argument("--height", type=_positive_int, default=1748)
    ap.add_argument("--fps",    type=_positive_int, default=30)
    ap.add_argument("--downscale", type=_positive_int, default=HP.DOWNSCALE_WIDTH,
                    help="Resize frames to this width for display (px)")
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    try:
        from detection.detector import BlueObjectDetector
        from stereo.camera import StereoCams
        from stereo.distance import StereoDistanceEstimator
    except ModuleNotFoundError as exc:
        print(
            f"[ERROR] Missing dependency: {exc.name}. Install camera dependencies and retry.",
            file=sys.stderr,
        )
        sys.exit(1)

    detector = BlueObjectDetector(downscale_width=args.downscale)
    estimator = StereoDistanceEstimator(
        fx_px=args.fx_px,
        baseline_m=args.baseline_m,
        min_disparity_px=args.min_disp,
        max_vertical_offset_px=args.max_dy,
    )
    cams = StereoCams(
        cam0=0, cam1=1,
        size=(args.width, args.height),
        fps=args.fps,
    )

    if args.fx_px == HP.STEREO_FX_PX:
        print(
            "[WARN] Using default --fx-px value. Stereo distance needs per-camera calibration for accuracy."
        )

    print("[INFO] Starting cameras…")
    cams.start()
    print("[INFO] Cameras ready. Press 'q' or Esc to quit.")

    try:
        while True:
            left_bgr, right_bgr = cams.capture_bgr()

            det_left  = detector.detect(left_bgr)
            det_right = detector.detect(right_bgr)

            dist = estimator.estimate(det_left, det_right)

            # ── annotate left frame ────────────────────────────────────────
            ann_left  = detector.draw(left_bgr,  det_left,  label_prefix="L: ")
            ann_right = detector.draw(right_bgr, det_right, label_prefix="R: ")

            # ── distance overlay on left frame ─────────────────────────────
            if dist.is_valid:
                label = (
                    f"Distance: {dist.distance_m:.3f} m  "
                    f"({dist.distance_cm:.1f} cm)"
                )
                colour = (0, 255, 0)
            else:
                label = f"Distance: N/A  ({dist.error or 'no detection'})"
                colour = (0, 0, 255)

            cv2.putText(
                ann_left, label,
                (15, ann_left.shape[0] - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, colour, 2,
            )

            # ── disparity debug on right frame ─────────────────────────────
            if dist.debug:
                disp_val = dist.debug.get("disparity_px", 0.0)
                cv2.putText(
                    ann_right,
                    f"disp={disp_val:.1f}px",
                    (15, ann_right.shape[0] - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2,
                )

            # ── display ────────────────────────────────────────────────────
            combined = hstack_resize(ann_left, ann_right,
                                     max_width=args.downscale * 2)
            cv2.imshow("Stereo Distance Measurement  [q / Esc = quit]", combined)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

    finally:
        cams.stop()
        cv2.destroyAllWindows()
        print("[INFO] Done.")


if __name__ == "__main__":
    main()
