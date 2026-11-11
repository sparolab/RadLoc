"""Command line: describe a radar sequence, then find its loops.

    radloc describe <polar_dir> --out descriptors.npy
    radloc loops descriptors.npy --out loops.csv [--exclude 100] [--threshold 3.0]
"""
from __future__ import annotations

import argparse
from glob import glob
from pathlib import Path

import numpy as np

from . import __version__
from .descriptor import compute_descriptor
from .retrieval import PlaceDatabase, RetrievalParams


def _describe(a) -> None:
    images = sorted(glob(str(Path(a.polar_dir) / "*.png")))
    if not images:
        raise SystemExit(f"no .png scans under {a.polar_dir}")
    images = images[:: a.every]
    D = np.stack([compute_descriptor(p) for p in images])
    np.save(a.out, D)
    Path(a.out).with_suffix(".txt").write_text("\n".join(Path(p).name for p in images) + "\n")
    print(f"{a.out}: {D.shape[0]} scans x {D.shape[1]} range bands")


def _loops(a) -> None:
    D = np.load(a.descriptors)
    db = PlaceDatabase(RetrievalParams(coarse_dims=a.coarse_dims, top_k=a.top_k))
    rows = []
    for i, d in enumerate(D):
        if i > a.exclude:
            m = db.query(d, limit=i - a.exclude)
            if m and m[0][1] < a.threshold:
                rows.append((i, m[0][0], m[0][1]))
        db.add(d)
    with open(a.out, "w") as f:
        f.write("query,match,distance\n")
        f.writelines(f"{q},{m},{d:.6f}\n" for q, m, d in rows)
    print(f"{a.out}: {len(rows)} loops in {len(D)} scans")


def main(argv=None) -> None:
    p = argparse.ArgumentParser(prog="radloc", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--version", action="version", version=f"radloc {__version__}")
    sub = p.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("describe", help="RadLoc descriptors of a folder of polar scans")
    d.add_argument("polar_dir")
    d.add_argument("--out", required=True, help=".npy (N x bands); the scan names go to a .txt beside it")
    d.add_argument("--every", type=int, default=1)
    d.set_defaults(fn=_describe)
    l = sub.add_parser("loops", help="loop candidates within one sequence")
    l.add_argument("descriptors")
    l.add_argument("--out", required=True)
    l.add_argument("--exclude", type=int, default=100, help="recent scans not searched")
    l.add_argument("--threshold", type=float, default=3.0, help="largest fine distance accepted")
    l.add_argument("--coarse-dims", type=int, default=20)
    l.add_argument("--top-k", type=int, default=10)
    l.set_defaults(fn=_loops)
    a = p.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()
