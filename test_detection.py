#!/usr/bin/env python3
"""
Simple detection test on static images.

Usage:
    python test_detection.py                        # red, any shape
    python test_detection.py --target Cube          # red cubes only
    python test_detection.py --colour blue          # blue, any shape
    python test_detection.py --target Cube --colour red
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from detection.target_detector import TargetDetector
import hparams as HP

IMAGE_DIR  = Path("images_test")
OUTPUT_DIR = Path("/Users/liorhadad/Documents/GitHub/image_pro_final/images_test_outputs")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=str, default=None,
                    help="Cube | Cylinder | Pyramid | None (any)")
    ap.add_argument("--colour", type=str, default=HP.DETECT_COLOUR,
                    help=f"Colour preset: {', '.join(HP._COLOUR_PRESETS)}")
    args = ap.parse_args()

    detector = TargetDetector(target=args.target, colour=args.colour)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    images = sorted(IMAGE_DIR.glob("*.jpg")) + sorted(IMAGE_DIR.glob("*.png"))
    if not images:
        print(f"No images found in {IMAGE_DIR}")
        return

    print(f"Target: {args.target or 'Any'}  |  Colour: {args.colour}\n")

    for img_path in images:
        bgr = cv2.imread(str(img_path))
        if bgr is None:
            print(f"  [{img_path.name}]  could not read")
            continue

        det = detector.detect(bgr)
        if det.found:
            print(f"  [{img_path.name}]  FOUND  cls={det.cls}  center={det.center}  bbox={det.bbox}")
        else:
            print(f"  [{img_path.name}]  not found")

        annotated = detector.draw(bgr, det)
        out_path = OUTPUT_DIR / img_path.name
        cv2.imwrite(str(out_path), annotated)
        print(f"  saved → {out_path}")

    print("\nDone.")


if __name__ == "__main__":
    main()
