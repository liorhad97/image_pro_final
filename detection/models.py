from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class DetResult:
    """Holds the output of a single-frame object detection pass."""

    found: bool
    cls: str                                     # "Pyramid" | "Cube" | "Cylinder" | "None"
    bbox: Optional[Tuple[int, int, int, int]]    # (x, y, w, h) in full-resolution pixels
    center: Optional[Tuple[int, int]]            # (cx, cy) in full-resolution pixels
    mask_raw: Optional[np.ndarray]               # raw inRange mask (full-res, uint8)
    mask_clean: Optional[np.ndarray]             # after morphological cleanup (full-res, uint8)

    # ------------------------------------------------------------------ helpers
    @property
    def is_valid(self) -> bool:
        """True when the detection has a usable bbox and center."""
        return self.found and self.bbox is not None and self.center is not None

    @classmethod
    def empty(
        cls,
        mask_raw: Optional[np.ndarray] = None,
        mask_clean: Optional[np.ndarray] = None,
    ) -> "DetResult":
        """Return a no-detection result (convenience factory)."""
        return cls(
            found=False,
            cls="None",
            bbox=None,
            center=None,
            mask_raw=mask_raw,
            mask_clean=mask_clean,
        )
