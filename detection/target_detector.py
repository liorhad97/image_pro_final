"""
Target-aware detector wrapper.

Wraps :class:`ColourObjectDetector` with a colour preset and a target-class
filter so callers only need to specify a colour name and an optional target
shape — no raw HSV values required.

Colour names are resolved via ``hparams._COLOUR_PRESETS``.
Passing ``target=None`` accepts any detected class.

Usage
-----
detector = TargetDetector(target="Cube", colour="red")
result: DetResult = detector.detect(bgr_frame)
annotated = detector.draw(bgr_frame, result)
"""
from __future__ import annotations

from typing import Optional

import numpy as np

import hparams as HP
from .detector import ColourObjectDetector
from .models import DetResult


class TargetDetector:
    """
    Colour detector that resolves HSV ranges from a named colour preset
    and filters results to a specific target class.

    Parameters
    ----------
    target : str | None
        ``"Cube"``, ``"Cylinder"``, ``"Pyramid"``, or ``None`` (any class).
    colour : str
        One of the keys in ``hparams._COLOUR_PRESETS``
        (``"red"``, ``"blue"``, ``"green"``, ``"yellow"``).
        Defaults to ``hparams.DETECT_COLOUR`` (currently ``"red"``).
    """

    def __init__(
        self,
        target: Optional[str] = None,
        colour: str = HP.DETECT_COLOUR,
        min_area: int = HP.MIN_AREA,
        downscale_width: int = HP.DOWNSCALE_WIDTH,
    ) -> None:
        if colour not in HP._COLOUR_PRESETS:
            raise ValueError(
                f"Unknown colour '{colour}'. "
                f"Valid options: {', '.join(HP._COLOUR_PRESETS)}"
            )

        preset = HP._COLOUR_PRESETS[colour]
        self._target = target
        self._inner = ColourObjectDetector(
            hsv_lo1=preset["hsv_lo1"],
            hsv_hi1=preset["hsv_hi1"],
            hsv_lo2=preset["hsv_lo2"],
            hsv_hi2=preset["hsv_hi2"],
            min_area=min_area,
            downscale_width=downscale_width,
        )

    # ------------------------------------------------------------------ public

    def detect(self, bgr: np.ndarray) -> DetResult:
        """
        Run detection on *bgr* and return the result only when it matches
        the configured target.  Returns ``DetResult.empty`` on a miss or when
        the class does not match.
        """
        result = self._inner.detect(bgr)

        if self._target is not None and result.cls != self._target:
            return DetResult.empty(result.mask_raw, result.mask_clean)

        return result

    def draw(
        self,
        bgr: np.ndarray,
        det: DetResult,
        label_prefix: str = "",
    ) -> np.ndarray:
        """Delegate annotation drawing to the inner detector."""
        return self._inner.draw(bgr, det, label_prefix)
