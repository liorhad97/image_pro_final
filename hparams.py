"""
Hyperparameters for all vision / detection components.

All tunable numbers live here so they can be changed in one place without
touching business logic in detector.py or config.py.
"""
from typing import Tuple

# ──────────────────────────────────────────────────────────────
# HSV colour thresholds  (hue 0-179, sat 0-255, val 0-255)
# ──────────────────────────────────────────────────────────────
HSV_LO: Tuple[int, int, int] = (105, 120, 50)
HSV_HI: Tuple[int, int, int] = (123, 255, 255)

# ──────────────────────────────────────────────────────────────
# Detection filtering
# ──────────────────────────────────────────────────────────────
MIN_AREA: int = 50            # minimum contour area [px²] to consider a hit
DOWNSCALE_WIDTH: int = 640    # resize frames wider than this before processing

# ──────────────────────────────────────────────────────────────
# Morphological cleanup
# ──────────────────────────────────────────────────────────────
MORPH_KERNEL_SIZE: Tuple[int, int] = (7, 7)
MORPH_CLOSE_ITER: int = 2
MORPH_OPEN_ITER: int = 1

# ──────────────────────────────────────────────────────────────
# Display / preview
# ──────────────────────────────────────────────────────────────
SHOW_PREVIEW: bool = True   # set True only when a display is available

# ──────────────────────────────────────────────────────────────
# Shape-classification thresholds
# ──────────────────────────────────────────────────────────────
# Extent  = blob area / bounding-box area  (1.0 = perfectly fills its box)
# Circularity = 4π·area / perimeter²        (1.0 = perfect circle)
CLASSIFY_DP_EPSILON: float = 0.02          # Douglas-Peucker approximation factor
CLASSIFY_PYRAMID_MAX_EXTENT: float = 0.58  # extent below this → Pyramid
CLASSIFY_PYRAMID_MAX_VERTICES: int = 6     # AND vertex count ≤ this → Pyramid
CLASSIFY_CYLINDER_MIN_CIRCULARITY: float = 0.78  # circularity above this → Cylinder
CLASSIFY_CYLINDER_MIN_VERTICES: int = 8    # OR vertex count ≥ this → Cylinder
