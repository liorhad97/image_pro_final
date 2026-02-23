from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import cv2
import numpy as np

import hparams as HP
from detection.models import DetResult


@dataclass
class StereoDistanceResult:
    """Holds the outcome of a stereo disparity-based distance estimate."""

    distance_m: Optional[float]
    distance_cm: Optional[float]
    disparity_px: Optional[float]
    dy_px: Optional[float]
    debug: Dict[str, float] = field(default_factory=dict)
    error: Optional[str] = None

    @property
    def is_valid(self) -> bool:
        return self.distance_m is not None


class StereoDistanceEstimator:
    """
    Estimates the depth (Z, in metres) of a detected object using the
    standard stereo disparity formula:

        Z = (f_x * B) / |disparity|

    where *f_x* is the horizontal focal length (pixels) and *B* is the
    camera baseline (metres).

    Parameters
    ----------
    fx_px : float
        Horizontal focal length in pixels (from camera calibration).
    baseline_m : float
        Physical distance between the two camera optical centres (metres).
    min_disparity_px : float
        Minimum absolute disparity below which the result is considered
        unreliable (avoids division-by-near-zero).
    max_vertical_offset_px : float
        Maximum allowed |y_left - y_right| before rejecting the estimate.
        A large vertical offset usually indicates poor rectification.
    """

    def __init__(
        self,
        fx_px: float = HP.STEREO_FX_PX,
        baseline_m: float = HP.STEREO_BASELINE_M,
        min_disparity_px: float = HP.STEREO_MIN_DISPARITY_PX,
        max_vertical_offset_px: float = HP.STEREO_MAX_VERTICAL_OFFSET_PX,
    ) -> None:
        if fx_px <= 0:
            raise ValueError(f"fx_px must be positive, got {fx_px}")
        if baseline_m <= 0:
            raise ValueError(f"baseline_m must be positive, got {baseline_m}")

        self.fx_px = fx_px
        self.baseline_m = baseline_m
        self.min_disparity_px = min_disparity_px
        self.max_vertical_offset_px = max_vertical_offset_px

    # ------------------------------------------------------------------ public
    def estimate(self, det_left: DetResult, det_right: DetResult) -> StereoDistanceResult:
        """
        Compute depth from *det_left* and *det_right* centre points.

        Returns a :class:`StereoDistanceResult` whose ``error`` attribute
        is ``None`` on success and contains a short reason string on failure.
        """
        if not (det_left.is_valid and det_right.is_valid):
            return StereoDistanceResult(
                distance_m=None, distance_cm=None,
                disparity_px=None, dy_px=None,
                error="missing_detection",
            )

        xL, yL = det_left.center   # type: ignore[misc]
        xR, yR = det_right.center  # type: ignore[misc]

        disparity = float(xL - xR)
        d_abs = abs(disparity)
        dy = abs(float(yL - yR))

        debug: Dict[str, float] = {
            "disparity_px": disparity,
            "disparity_abs_px": d_abs,
            "dy_px": dy,
            "xL": float(xL), "yL": float(yL),
            "xR": float(xR), "yR": float(yR),
        }

        if dy > self.max_vertical_offset_px:
            return StereoDistanceResult(
                distance_m=None, distance_cm=None,
                disparity_px=disparity, dy_px=dy,
                debug=debug, error="vertical_mismatch",
            )

        if d_abs < self.min_disparity_px:
            return StereoDistanceResult(
                distance_m=None, distance_cm=None,
                disparity_px=disparity, dy_px=dy,
                debug=debug, error="disparity_too_small",
            )

        z_m = (self.fx_px * self.baseline_m) / d_abs

        return StereoDistanceResult(
            distance_m=z_m,
            distance_cm=z_m * 100.0,
            disparity_px=disparity,
            dy_px=dy,
            debug=debug,
            error=None,
        )

    # ------------------------------------------------------------------ overlay
    @staticmethod
    def draw_overlay(
        img: np.ndarray,
        result: StereoDistanceResult,
        title: str = "",
        extra_status: str = "",
    ) -> np.ndarray:
        """
        Render disparity / distance diagnostics onto *img*.

        Returns a copy; the original array is not mutated.
        """
        out = img.copy()
        y0 = 35

        def _put(text: str, colour: tuple, size: float = 0.75) -> None:
            nonlocal y0
            cv2.putText(
                out, text, (15, y0),
                cv2.FONT_HERSHEY_SIMPLEX, size, colour, 2,
            )
            y0 += int(size * 40)

        if title:
            _put(title, (255, 255, 255), 0.85)
        if extra_status:
            _put(extra_status, (255, 200, 0), 0.75)

        if result.debug:
            disp = result.debug.get("disparity_px", 0.0)
            dy = result.debug.get("dy_px", 0.0)
            _put(
                f"disp={disp:.1f}px  |disp|={abs(disp):.1f}px  dy={dy:.1f}px",
                (0, 255, 255), 0.72,
            )

        if result.is_valid:
            _put(
                f"Distance Z = {result.distance_m:.3f} m "   # type: ignore[str-format]
                f"({result.distance_cm:.1f} cm)",             # type: ignore[str-format]
                (0, 255, 0), 0.85,
            )
        else:
            msg = (
                f"Distance unavailable ({result.error})"
                if result.error else "Distance unavailable"
            )
            _put(msg, (0, 0, 255), 0.75)

        return out
