"""Checks that the package, the C++ core and the archived descriptors agree.

    python3 -m pytest tests/ -v
    RADLOC_MULRAN=/data/referee/Mulran python3 -m pytest tests/ -v
"""

from __future__ import annotations

import glob
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import radloc  # noqa: E402

MULRAN = Path(os.environ.get("RADLOC_MULRAN", "/storage/Datasets/ReFeree/Mulran"))
SEQUENCES = ["KAIST_03", "Riverside_03"]


def sequence_paths(name):
    polar = sorted(glob.glob(str(MULRAN / name / "polar" / "*.png")))
    npy = sorted(glob.glob(str(MULRAN / name / "refereepp_84x4" / "*.npy")))
    if not polar or not npy:
        pytest.skip(f"{name} not available under {MULRAN}")
    return polar, npy


@pytest.mark.parametrize("name", SEQUENCES)
def test_descriptor_matches_archive(name):
    """The descriptor reproduces the archived .npy, allowing for the affine
    flip that from_inverted() undoes."""
    polar, npy = sequence_paths(name)
    for i in (0, 10, 500):
        if i >= len(polar) or i >= len(npy):
            continue
        actual = radloc.compute_descriptor(polar[i])
        expected = radloc.from_inverted(np.load(npy[i]))
        assert actual.shape == expected.shape
        assert np.max(np.abs(actual - expected)) / np.max(np.abs(expected)) < 1e-12


@pytest.mark.parametrize("name", SEQUENCES)
def test_retrieval_matches_legacy(name):
    """The two-stage search returns the shortlists the original produced."""
    _, npy = sequence_paths(name)
    n = min(400, len(npy))
    archived = np.array([np.load(f).ravel() for f in npy[:n]])
    bands = np.array([radloc.from_inverted(a) for a in archived])

    split = n // 2
    params = radloc.RetrievalParams(coarse_dims=20, top_k=10)
    db = radloc.PlaceDatabase(params)
    for b in bands[:split]:
        db.add(b)
    db.build()

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from legacy_reference import huber_loss
    from scipy.spatial import cKDTree

    legacy_tree = cKDTree(archived[:split, :20])
    for q in range(split, n):
        ours = db.query(bands[q])

        _, cand = legacy_tree.query(archived[q, :20], k=min(10, split))
        inverse = 1 / np.arange(1, archived.shape[1] + 1)
        theirs = sorted(
            ((int(c), huber_loss(archived[q] * inverse, archived[c] * inverse))
             for c in np.atleast_1d(cand)),
            key=lambda pair: pair[1],
        )
        assert [i for i, _ in ours] == [i for i, _ in theirs]
        for (_, a), (_, b) in zip(ours, theirs):
            assert abs(a - b) < 1e-9


@pytest.mark.parametrize("name", SEQUENCES)
def test_matches_cpp_core(name):
    """The package and the C++ core produce the same descriptor."""
    binary = Path(__file__).resolve().parents[2] / "cpp/radloc/build/validate_descriptor"
    if not binary.exists():
        pytest.skip("C++ core not built; see cpp/radloc/README.md")
    polar, npy = sequence_paths(name)
    result = subprocess.run(
        [str(binary), str(Path(polar[0]).parent), str(Path(npy[0]).parent), "10"],
        capture_output=True, text=True, check=False,
    )
    assert "PASS" in result.stdout, result.stdout + result.stderr
