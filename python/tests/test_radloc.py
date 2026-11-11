"""Data-free checks of the package, and of the package against the C++ core (cpp/build/describe)."""
import os
import subprocess
from pathlib import Path

import numpy as np
import pytest

import radloc
from radloc import PlaceDatabase, RetrievalParams, compute_descriptor
from radloc.cli import main as cli

DESCRIBE = Path(os.environ.get("RADLOC_CPP_DESCRIBE", Path(__file__).resolve().parents[2] / "cpp/build/describe"))


def scene(seed, n_range=3360, n_azimuth=400):
    rng = np.random.default_rng(seed)
    r = np.arange(n_range)[:, None]
    img = 20 * rng.random((n_range, n_azimuth)) + 80 * np.exp(-r / 800)
    for _ in range(60):
        a, b = rng.integers(0, n_range - 40), rng.integers(0, n_azimuth - 6)
        img[a:a + 40, b:b + 6] += 120 * rng.random()
    return img


def test_version():
    assert radloc.__version__ == "2.0.0"


def test_rotation_invariant():
    s = scene(1)
    d0, d1 = compute_descriptor(s), compute_descriptor(np.roll(s, 100, axis=1))
    assert d0.shape == (40,)
    assert np.allclose(d0, d1, atol=1e-9)


def test_retrieval_and_limit():
    db = PlaceDatabase()
    for i in range(8):
        db.add(compute_descriptor(scene(10 + i)))
    q = compute_descriptor(np.roll(scene(13), 37, axis=1))
    assert db.query(q)[0][0] == 3
    assert all(i < 3 for i, _ in db.query(q, limit=3))
    assert db.query(q, limit=0) == []


def test_cli(tmp_path):
    import cv2
    polar = tmp_path / "polar"
    polar.mkdir()
    for i in range(6):
        cv2.imwrite(str(polar / f"{i:03d}.png"), np.clip(scene(i % 3), 0, 255).astype(np.uint8))
    cli(["describe", str(polar), "--out", str(tmp_path / "d.npy")])
    assert np.load(tmp_path / "d.npy").shape == (6, 40)
    cli(["loops", str(tmp_path / "d.npy"), "--out", str(tmp_path / "loops.csv"), "--exclude", "2"])
    rows = (tmp_path / "loops.csv").read_text().splitlines()[1:]
    assert rows and all(int(r.split(",")[0]) % 3 == int(r.split(",")[1]) % 3 for r in rows)


@pytest.mark.skipif(not DESCRIBE.exists(), reason="C++ core not built (cmake -S cpp -B cpp/build)")
def test_cpp_matches_python(tmp_path):
    import cv2
    paths = []
    for i in range(3):
        p = tmp_path / f"{i}.png"
        img = np.clip(scene(20 + i), 0, 255).astype(np.uint8)
        cv2.imwrite(str(p), img if i != 1 else img.T)   # one stored azimuth-major: both normalise the layout
        paths.append(p)
    out = subprocess.run([str(DESCRIBE), *map(str, paths)], check=True, capture_output=True, text=True).stdout
    for p, line in zip(paths, out.strip().splitlines()):
        cpp = np.array(line.split(), dtype=float)
        assert np.allclose(cpp, compute_descriptor(str(p)), atol=1e-9)
