from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np

from .models import DetResult


class BlueObjectDetector:
    """
    Detects the largest blue object in a BGR frame using HSV thresholding,
    morphological cleanup, and contour analysis.  Classifies the shape as
    Pyramid, Cube, or Cylinder using simple geometric heuristics.

    Usage
    -----
    detector = BlueObjectDetector()
    result: DetResult = detector.detect(bgr_frame)
    annotated = detector.draw(bgr_frame, result)
    """

    # Default HSV range tuned for a matte-blue object under indoor lighting.
    DEFAULT_HSV_LO: Tuple[int, int, int] = (105, 120, 50)
    DEFAULT_HSV_HI: Tuple[int, int, int] = (123, 255, 255)

    def __init__(
        self,
        hsv_lo: Tuple[int, int, int] = DEFAULT_HSV_LO,
        hsv_hi: Tuple[int, int, int] = DEFAULT_HSV_HI,
        min_area: int = 50,
        downscale_width: int = 640,
    ) -> None:
        self._hsv_lo = np.array(hsv_lo, dtype=np.uint8)
        self._hsv_hi = np.array(hsv_hi, dtype=np.uint8)
        self._min_area = min_area
        self._downscale_width = downscale_width

        # Morphological structuring element (shared across calls)
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

    # ------------------------------------------------------------------ public
    def detect(self, bgr: np.ndarray) -> DetResult:
        """
        Run blue-segmentation + shape classification on *bgr*.

        The image is optionally downscaled for speed; all coordinates and
        masks are mapped back to the original resolution before returning.
        """
        H, W = bgr.shape[:2]
        scale, small = self._maybe_downscale(bgr, W)

        mask_raw = self._build_raw_mask(small)
        mask_clean = self._apply_morphology(mask_raw)

        raw_full, clean_full = self._upscale_masks(mask_raw, mask_clean, scale, W, H)

        contours, _ = cv2.findContours(
            mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return DetResult.empty(raw_full, clean_full)

        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < self._min_area:
            return DetResult.empty(raw_full, clean_full)

        perimeter = cv2.arcLength(cnt, True)
        if perimeter < 1e-6:
            return DetResult(
                found=True, cls="None", bbox=None, center=None,
                mask_raw=raw_full, mask_clean=clean_full,
            )

        x, y, w, h = cv2.boundingRect(cnt)
        cx, cy = x + w // 2, y + h // 2
        cls = self._classify(cnt, cv2.contourArea(cnt), perimeter, w, h)

        if scale != 1.0:
            x, y, w, h, cx, cy = self._scale_coords(x, y, w, h, cx, cy, 1.0 / scale)

        return DetResult(
            found=True,
            cls=cls,
            bbox=(x, y, w, h),
            center=(cx, cy),
            mask_raw=raw_full,
            mask_clean=clean_full,
        )

    def draw(
        self,
        bgr: np.ndarray,
        det: DetResult,
        label_prefix: str = "",
    ) -> np.ndarray:
        """Return a copy of *bgr* annotated with the detection result."""
        out = bgr.copy()

        if not det.is_valid:
            cv2.putText(
                out, f"{label_prefix}None",
                (15, 35), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 0, 255), 2,
            )
            return out

        x, y, w, h = det.bbox   # type: ignore[misc]
        cx, cy = det.center      # type: ignore[misc]

        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.circle(out, (cx, cy), 7, (0, 0, 255), -1)
        cv2.putText(
            out, f"{label_prefix}{det.cls}",
            (x, max(35, y - 10)), cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 255, 0), 2,
        )
        cv2.putText(
            out, f"({cx},{cy})",
            (x, y + h + 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (255, 255, 255), 2,
        )
        return out

    # ----------------------------------------------------------------- private
    def _maybe_downscale(
        self, bgr: np.ndarray, W: int
    ) -> Tuple[float, np.ndarray]:
        if W > self._downscale_width:
            scale = self._downscale_width / float(W)
            H = bgr.shape[0]
            small = cv2.resize(
                bgr,
                (int(W * scale), int(H * scale)),
                interpolation=cv2.INTER_AREA,
            )
            return scale, small
        return 1.0, bgr

    def _build_raw_mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv, self._hsv_lo, self._hsv_hi)

    def _apply_morphology(self, mask: np.ndarray) -> np.ndarray:
        closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel, iterations=2)
        opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, self._kernel, iterations=1)
        return opened

    @staticmethod
    def _upscale_masks(
        raw: np.ndarray,
        clean: np.ndarray,
        scale: float,
        W: int,
        H: int,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if scale != 1.0:
            raw_full = cv2.resize(raw, (W, H), interpolation=cv2.INTER_NEAREST)
            clean_full = cv2.resize(clean, (W, H), interpolation=cv2.INTER_NEAREST)
            return raw_full, clean_full
        return raw, clean

    @staticmethod
    def _classify(
        cnt: np.ndarray,
        area: float,
        perimeter: float,
        w: int,
        h: int,
    ) -> str:
        """
        Geometric heuristics to label the contour as Pyramid, Cylinder, or Cube.

        Notes
        -----
        - *extent*      = area / bounding-box area  (how tightly the blob fills its box)
        - *circularity* = 4π·area / perimeter²       (1.0 = perfect circle)
        - *n*           = vertex count after DP approximation
        """
        extent = area / float(w * h + 1e-9)
        circularity = 4.0 * np.pi * area / (perimeter * perimeter + 1e-12)
        approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
        n = len(approx)

        if extent < 0.58 and n <= 6:
            return "Pyramid"
        if circularity > 0.78 or n >= 8:
            return "Cylinder"
        return "Cube"

    @staticmethod
    def _scale_coords(
        x: int, y: int, w: int, h: int, cx: int, cy: int, factor: float
    ) -> Tuple[int, int, int, int, int, int]:
        return (
            int(x * factor), int(y * factor),
            int(w * factor), int(h * factor),
            int(cx * factor), int(cy * factor),
        )
