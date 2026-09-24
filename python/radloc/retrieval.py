"""Two-stage place retrieval.

coarse
    Euclidean nearest neighbours over the leading ``coarse_dims`` of the
    range-weighted descriptor, via a k-d tree. Weighting the bands makes
    distant structure dominate the shortlist.
fine
    the shortlist is re-ranked on the full, unweighted descriptor, so near and
    far bands count equally once candidates are few.

Weighting is therefore applied exactly once, when the coarse key is built.
Mirrors ``radloc/retrieval.hpp``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np
from scipy.spatial import cKDTree

from .descriptor import range_weighted


class FineMetric(Enum):
    #: Mean absolute difference over all bands. This is what the original
    #: implementation computed: its ``huber_loss`` overwrote the delta it was
    #: passed and tested ``abs_diff <= 0``, so every band took the L1 branch.
    MEAN_ABSOLUTE = "mean_absolute"
    #: Huber as it was presumably intended. Changes the ranking, so results are
    #: not comparable with archived runs.
    HUBER = "huber"


@dataclass
class RetrievalParams:
    #: Bands used for the coarse k-d tree. The original used 20 between
    #: sessions and 8 within one session.
    coarse_dims: int = 20
    top_k: int = 10
    fine_metric: FineMetric = FineMetric.MEAN_ABSOLUTE
    huber_delta: float = 50.0  # only read when fine_metric is HUBER


def fine_distance(a: np.ndarray, b: np.ndarray, params: RetrievalParams) -> float:
    diff = np.abs(np.asarray(a) - np.asarray(b))
    if params.fine_metric is FineMetric.HUBER:
        d = params.huber_delta
        loss = np.where(diff <= d, 0.5 * diff**2, d * (diff - 0.5 * d))
    else:
        loss = diff
    return float(np.mean(loss))


class PlaceDatabase:
    """A searchable set of places. Descriptors are stored unweighted; the
    coarse keys are derived once on :meth:`build`."""

    def __init__(self, params: RetrievalParams | None = None) -> None:
        self.params = params or RetrievalParams()
        self._descriptors: list[np.ndarray] = []
        self._tree: cKDTree | None = None

    def add(self, bands: np.ndarray) -> int:
        bands = np.asarray(bands, dtype=np.float64).ravel()
        if len(bands) < self.params.coarse_dims:
            raise ValueError("descriptor shorter than coarse_dims")
        self._descriptors.append(bands)
        self._tree = None
        return len(self._descriptors) - 1

    def build(self) -> None:
        if not self._descriptors:
            self._tree = None
            return
        keys = np.vstack(
            [range_weighted(d, self.params.coarse_dims) for d in self._descriptors]
        )
        self._tree = cKDTree(keys)

    def __len__(self) -> int:
        return len(self._descriptors)

    def descriptor(self, index: int) -> np.ndarray:
        return self._descriptors[index]

    def query(self, bands: np.ndarray) -> list[tuple[int, float]]:
        """Nearest places as ``(index, fine distance)``, best first."""
        if self._tree is None:
            raise RuntimeError("call build() before query()")
        bands = np.asarray(bands, dtype=np.float64).ravel()

        k = min(max(self.params.top_k, 1), len(self._descriptors))
        _, candidates = self._tree.query(
            range_weighted(bands, self.params.coarse_dims), k=k
        )
        candidates = np.atleast_1d(candidates)

        scored = [
            (int(c), fine_distance(bands, self._descriptors[c], self.params))
            for c in candidates
        ]
        scored.sort(key=lambda pair: pair[1])
        return scored
