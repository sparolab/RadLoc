"""Loading and layout normalisation for scanning-radar polar images.

Two on-disk layouts are in circulation for the same data:

* Oxford form, ``(num_azimuth, 11 + num_range)``, the first eleven columns
  carrying timestamp and encoder metadata
* preprocessed form, ``(num_range, num_azimuth)``, metadata already removed

Both are normalised here to rows = range bins, cols = azimuths, matching
``radloc/polar_image.hpp``.
"""

from __future__ import annotations

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - cv2 is the only loader dependency
    cv2 = None

#: Metadata columns prepended to each azimuth in the Oxford format.
OXFORD_HEADER_COLS = 11


def to_polar_scan(raw: np.ndarray, strip_header: bool = False) -> np.ndarray:
    """Normalise an arbitrary polar image to ``(range, azimuth)``, float64.

    ``strip_header`` is off by default because the descriptors archived under
    ``/storage/Datasets/ReFeree`` were generated with the eleven metadata
    columns left in place.
    """
    if raw.size == 0:
        raise ValueError("empty radar image")

    # The longer axis is range.
    scan = raw.T if raw.shape[0] < raw.shape[1] else raw

    if strip_header:
        if scan.shape[0] <= OXFORD_HEADER_COLS:
            raise ValueError("image too small to strip header")
        scan = scan[OXFORD_HEADER_COLS:]

    return np.ascontiguousarray(scan, dtype=np.float64)


def load_polar_image(path: str, strip_header: bool = False) -> np.ndarray:
    if cv2 is None:
        raise ImportError("opencv-python is required to load radar images")
    raw = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if raw is None:
        raise FileNotFoundError(f"cannot read {path}")
    return to_polar_scan(raw, strip_header)


def to_cartesian_points(
    scan: np.ndarray,
    range_resolution: float,
    threshold: float,
    range_offset: int = 0,
) -> np.ndarray:
    """Detections above ``threshold`` as an ``(N, 2)`` array of metres."""
    rows, cols = scan.shape
    r_idx, a_idx = np.nonzero(scan[range_offset:] > threshold)
    ranges = r_idx * range_resolution
    angles = 2.0 * np.pi * a_idx / cols
    return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
