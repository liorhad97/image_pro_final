"""
Colour-based object detector using HSV thresholding and contour analysis.

Public classes
--------------
ColourObjectDetector
    Generic detector that accepts one or two HSV ranges and combines them
    into a single mask.  Classifies the largest blob as Pyramid, Cube, or
    Cylinder using geometric heuristics.

RedObjectDetector
    Concrete subclass pre-configured for red objects, which span two
    disjoint HSV hue ranges (0–12° and 165–180°).
"""
from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np

import hparams as HP
from .models import DetResult


class ColourObjectDetector:
    """
    Detects the largest coloured object in a BGR frame using dual-range
    HSV thresholding, morphological cleanup, and contour analysis.
    Classifies the detected shape as Pyramid, Cube, or Cylinder using
    simple geometric heuristics.

    Supports colours that span two hue ranges (e.g. red wraps around
    the 0°/180° boundary) by OR-ing two inRange masks together.

    Usage
    -----
    detector = ColourObjectDetector(hsv_lo1=..., hsv_hi1=...,
                                    hsv_lo2=..., hsv_hi2=...)
    result: DetResult = detector.detect(bgr_frame)
    annotated = detector.draw(bgr_frame, result)
    """

    def __init__(
        self,
        hsv_lo1: Tuple[int, int, int] = HP.HSV_LO1,
        hsv_hi1: Tuple[int, int, int] = HP.HSV_HI1,
        hsv_lo2: Tuple[int, int, int] = HP.HSV_LO2,
        hsv_hi2: Tuple[int, int, int] = HP.HSV_HI2,
        min_area: int = HP.MIN_AREA,
        downscale_width: int = HP.DOWNSCALE_WIDTH,
    ) -> None:
        # Store HSV bounds as uint8 arrays for cv2.inRange
        self._hsv_lo1 = np.array(hsv_lo1, dtype=np.uint8)
        self._hsv_hi1 = np.array(hsv_hi1, dtype=np.uint8)
        self._hsv_lo2 = np.array(hsv_lo2, dtype=np.uint8)
        self._hsv_hi2 = np.array(hsv_hi2, dtype=np.uint8)
        self._min_area = min_area
        self._downscale_width = downscale_width

        # Morphological structuring element — reused across every call
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, HP.MORPH_KERNEL_SIZE)

    # ------------------------------------------------------------------ public

    def detect(self, bgr: np.ndarray) -> DetResult:
        """
        Run colour-segmentation + shape classification on *bgr*.

        The image is optionally downscaled for speed; all coordinates and
        masks are mapped back to the original resolution before returning.

        Parameters
        ----------
        bgr : np.ndarray
            Full-resolution BGR frame from the camera.

        Returns
        -------
        DetResult
            Detection result with bounding box, centre, class label, and masks.
        """
        H, W = bgr.shape[:2]
        scale, small = self._maybe_downscale(bgr, W)

        mask_raw   = self._build_raw_mask(small)
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
        """
        Return a copy of *bgr* annotated with the detection result.

        Parameters
        ----------
        bgr : np.ndarray
            Original frame to annotate.
        det : DetResult
            Detection result to visualise.
        label_prefix : str
            Optional prefix for the class label (e.g. ``"L: "``).
        """
        out = bgr.copy()

        if not det.is_valid:
            cv2.putText(
                out, f"{label_prefix}None",
                (15, 35), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 0, 255), 2,
            )
            return out

        x, y, w, h = det.bbox    # type: ignore[misc]
        cx, cy     = det.center  # type: ignore[misc]

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
        """Resize *bgr* to at most *_downscale_width* px wide, return (scale, img)."""
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
        """Create a binary mask by OR-ing both HSV inRange results."""
        hsv   = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self._hsv_lo1, self._hsv_hi1)
        mask2 = cv2.inRange(hsv, self._hsv_lo2, self._hsv_hi2)
        return cv2.bitwise_or(mask1, mask2)

    def _apply_morphology(self, mask: np.ndarray) -> np.ndarray:
        """Close then open the mask to fill holes and remove noise."""
        closed = cv2.morphologyEx(mask,   cv2.MORPH_CLOSE, self._kernel, iterations=HP.MORPH_CLOSE_ITER)
        opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN,  self._kernel, iterations=HP.MORPH_OPEN_ITER)
        return opened

    @staticmethod
    def _upscale_masks(
        raw: np.ndarray,
        clean: np.ndarray,
        scale: float,
        W: int,
        H: int,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Restore downscaled masks to the original frame resolution."""
        if scale != 1.0:
            raw_full   = cv2.resize(raw,   (W, H), interpolation=cv2.INTER_NEAREST)
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
        Label the contour as ``"Pyramid"``, ``"Cylinder"``, or ``"Cube"``
        using geometric heuristics.

        Notes
        -----
        - *extent*      = area / bounding-box area  (1.0 = perfectly fills box)
        - *circularity* = 4π·area / perimeter²       (1.0 = perfect circle)
        - *n*           = vertex count after Douglas-Peucker approximation
        """
        extent       = area / float(w * h + 1e-9)
        circularity  = 4.0 * np.pi * area / (perimeter * perimeter + 1e-12)
        approx       = cv2.approxPolyDP(cnt, HP.CLASSIFY_DP_EPSILON * perimeter, True)
        n            = len(approx)

        if extent < HP.CLASSIFY_PYRAMID_MAX_EXTENT and n <= HP.CLASSIFY_PYRAMID_MAX_VERTICES:
            return "Pyramid"
        if circularity > HP.CLASSIFY_CYLINDER_MIN_CIRCULARITY or n >= HP.CLASSIFY_CYLINDER_MIN_VERTICES:
            return "Cylinder"
        return "Cube"

    @staticmethod
    def _scale_coords(
        x: int, y: int, w: int, h: int, cx: int, cy: int, factor: float
    ) -> Tuple[int, int, int, int, int, int]:
        """Scale all bounding-box and centre coordinates by *factor*."""
        return (
            int(x * factor), int(y * factor),
            int(w * factor), int(h * factor),
            int(cx * factor), int(cy * factor),
        )


class RedObjectDetector(ColourObjectDetector):
    """
    Detects red objects using two HSV hue ranges.

    Red wraps around the HSV hue circle, so two overlapping ranges are
    needed:
    - Range 1 (upper red) : hue 0–12°
    - Range 2 (deep red)  : hue 165–180°

    Inherits all detection and drawing logic from :class:`ColourObjectDetector`.
    """

    def __init__(
        self,
        hsv_lo1: Tuple[int, int, int] = HP.HSV_LO1,
        hsv_hi1: Tuple[int, int, int] = HP.HSV_HI1,
        hsv_lo2: Tuple[int, int, int] = HP.HSV_LO2,
        hsv_hi2: Tuple[int, int, int] = HP.HSV_HI2,
        min_area: int = HP.MIN_AREA,
        downscale_width: int = HP.DOWNSCALE_WIDTH,
    ) -> None:
        super().__init__(
            hsv_lo1=hsv_lo1, hsv_hi1=hsv_hi1,
            hsv_lo2=hsv_lo2, hsv_hi2=hsv_hi2,
            min_area=min_area, downscale_width=downscale_width,
        )
