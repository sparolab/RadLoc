#!/usr/bin/env python3
"""Compute RadLoc descriptors for a sequence.

    describe_sequence.py <polar_dir> <out_dir>
"""
import sys
from glob import glob
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import radloc


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    polar_dir, out_dir = sys.argv[1], Path(sys.argv[2])
    out_dir.mkdir(parents=True, exist_ok=True)

    images = sorted(glob(f"{polar_dir}/*.png"))
    for i, image in enumerate(images):
        bands = radloc.compute_descriptor(image)
        radloc.write_descriptor(out_dir / f"{i},{Path(image).stem}.rld", bands)
        if i % 100 == 0:
            print(f"  {i} / {len(images)}", end="\r", flush=True)
    print(f"wrote {len(images)} descriptors to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
