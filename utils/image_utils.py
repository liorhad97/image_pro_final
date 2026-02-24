from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np

import hparams as HP
from detection.models import DetResult

# bbox_margin_ok rejects detections where the bounding box is too close to the frame border
# normalize_target_name converts user input like cube or CUBE to the canonical class name Cube

def hstack_resize(
    left: np.ndarray,
    right: np.ndarray,
    max_width: int = HP.PREVIEW_MAX_WIDTH,
) -> np.ndarray:
    h = min(left.shape[0], right.shape[0])
    left_r = _resize_to_height(left, h)
    right_r = _resize_to_height(right, h)
    combo = np.hstack([left_r, right_r])

    if combo.shape[1] > max_width:
        scale = max_width / float(combo.shape[1])
        new_w = int(combo.shape[1] * scale)
        new_h = int(combo.shape[0] * scale)
        combo = cv2.resize(combo, (new_w, new_h), interpolation=cv2.INTER_AREA)

    return combo


def save_mask_png(path: Path, mask: Optional[np.ndarray]) -> None:
    if mask is None:
        return

    m = mask if mask.dtype == np.uint8 else mask.astype(np.uint8)
    if m.max() <= 1:
        m = (m * 255).astype(np.uint8)

    cv2.imwrite(str(path), m)


def bbox_margin_ok(
    det: DetResult,
    img_w: int,
    img_h: int,
    margin_px: int,
) -> bool:
    if not det.is_valid:
        return False

    x, y, w, h = det.bbox
    return (
        x >= margin_px
        and y >= margin_px
        and (x + w) <= (img_w - margin_px)
        and (y + h) <= (img_h - margin_px)
    )


_TARGET_MAP = {
    "cube": "Cube",
    "cylinder": "Cylinder",
    "pyramid": "Pyramid",
}


def normalize_target_name(name: Optional[str]) -> Optional[str]:
    if name is None:
        return None
    s = str(name).strip()
    if s.lower() == "none":
        return None
    key = s.lower()
    if key not in _TARGET_MAP:
        raise ValueError(
            f"Invalid target '{name}'. Valid options: Cube, Cylinder, Pyramid, None"
        )
    return _TARGET_MAP[key]


def _resize_to_height(img: np.ndarray, h: int) -> np.ndarray:
    new_w = int(img.shape[1] * (h / img.shape[0]))
    return cv2.resize(img, (new_w, h), interpolation=cv2.INTER_AREA)
