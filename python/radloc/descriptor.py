"""The RadLoc place descriptor.

A scan is reduced to one value per range band: the mean return over every
azimuth in that band. Azimuths whose mean return stands out against their
neighbours are rejected first by a 1-D cell-averaging CFAR, which removes
receiver saturation streaks.

Earlier versions stored ``255 - intensity``. Since the band value is a mean,
that is only an affine flip of what is stored here, and both retrieval stages
are invariant to it: the fine stage compares differences and the coarse stage
is a Euclidean nearest-neighbour search, so the shared offset cancels in each.
:func:`from_inverted` converts archived descriptors.

The descriptor is unweighted. Range weighting - counting distant bands for more
- belongs to how two places are compared, not to the place itself, so it lives
in :mod:`radloc.retrieval` and is applied once when the coarse key is built.

Mirrors ``radloc/descriptor.hpp``; the two agree to floating-point round-off.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass

import numpy as np

from .polar_image import load_polar_image


@dataclass(frozen=True)
class DescriptorParams:
    patch_range: int = 84      # range bins per band
    patch_azimuth: int = 4     # azimuths per cell
    cfar_guard: int = 2        # guard cells either side of the cell under test
    cfar_reference: int = 84   # reference cells either side
    cfar_pfa: float = 0.2      # probability of false alarm


def ca_cfar_1d(signal: np.ndarray, guard: int, reference: int, pfa: float) -> np.ndarray:
    """Cell-averaging CFAR: one flag per sample, true where it exceeds the
    scaled average of its two reference windows."""
    if reference <= 0:
        raise ValueError("reference cells must be > 0")
    n = len(signal)
    if n == 0:
        return np.zeros(0, dtype=bool)

    window = guard + reference
    threshold_factor = reference * (pfa ** (-1.0 / reference) - 1.0)

    padded = np.pad(signal, window, mode="edge")
    kernel_left = np.zeros(2 * window + 1)
    kernel_left[:reference] = 1.0 / reference
    kernel_right = np.zeros(2 * window + 1)
    kernel_right[-reference:] = 1.0 / reference

    left = np.convolve(padded, kernel_left, mode="valid")
    right = np.convolve(padded, kernel_right, mode="valid")
    return signal > 0.5 * (left + right) * threshold_factor


def range_weighted(bands: np.ndarray, dims: int | None = None) -> np.ndarray:
    """Band ``b`` scaled by ``b + 1``, optionally truncated to the leading
    ``dims``. This is the coarse search key."""
    n = len(bands) if dims is None else min(dims, len(bands))
    return bands[:n] * np.arange(1, n + 1)


def from_inverted(inverted: np.ndarray) -> np.ndarray:
    """Convert a descriptor archived as ``(255 - mean) * (b + 1)`` into the
    unweighted bands used here. The two rank candidates identically."""
    inverted = np.asarray(inverted, dtype=np.float64).ravel()
    b = np.arange(1, len(inverted) + 1)
    return 255.0 - inverted / b


def compute_descriptor(scan, params: DescriptorParams = DescriptorParams()) -> np.ndarray:
    """Mean return per range band. Length is ``num_range // patch_range``."""
    if isinstance(scan, (str, bytes)):
        scan = load_polar_image(scan)
    scan = np.asarray(scan, dtype=np.float64)

    n_range, n_azimuth = scan.shape
    n_bands = n_range // params.patch_range
    n_cells = n_azimuth // params.patch_azimuth
    if n_bands <= 0 or n_cells <= 0:
        raise ValueError("scan smaller than one patch")

    masked = ca_cfar_1d(
        scan.mean(axis=0), params.cfar_guard, params.cfar_reference, params.cfar_pfa
    )

    work = scan.copy()
    work[:, masked] = np.nan
    cropped = work[: n_bands * params.patch_range, : n_cells * params.patch_azimuth]
    patches = cropped.reshape(
        n_bands, params.patch_range, n_cells, params.patch_azimuth
    ).transpose(0, 2, 1, 3)

    # A cell whose azimuths were all rejected averages to nan, and a band whose
    # cells all did likewise; both are folded to zero below, so the warning
    # numpy raises for them is noise.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        cells = np.nanmean(patches.reshape(n_bands, n_cells, -1), axis=2)
        bands = np.nanmean(cells, axis=1)
    return np.nan_to_num(bands, nan=0.0)


def write_descriptor(path: str, bands: np.ndarray) -> None:
    """Plain text, whitespace separated on one line."""
    with open(path, "w") as handle:
        handle.write(" ".join(f"{v:.9g}" for v in np.asarray(bands).ravel()) + "\n")


def read_descriptor(path: str) -> np.ndarray:
    bands = np.loadtxt(path, dtype=np.float64).ravel()
    if bands.size == 0:
        raise ValueError(f"empty descriptor {path}")
    return bands
