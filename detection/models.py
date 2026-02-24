from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# holds the output of one detection pass, bbox and center are None when nothing was found
@dataclass
class DetResult:
    found: bool
    cls: str
    bbox: Optional[Tuple[int, int, int, int]]
    center: Optional[Tuple[int, int]]
    mask_raw: Optional[np.ndarray]
    mask_clean: Optional[np.ndarray]

    @property
    def is_valid(self) -> bool:
        return self.found and self.bbox is not None and self.center is not None

    @classmethod
    def empty(
        cls,
        mask_raw: Optional[np.ndarray] = None,
        mask_clean: Optional[np.ndarray] = None,
    ) -> "DetResult":
        return cls(
            found=False,
            cls="None",
            bbox=None,
            center=None,
            mask_raw=mask_raw,
            mask_clean=mask_clean,
        )
