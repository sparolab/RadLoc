# radloc (Python reference)

The research implementation. It mirrors the header-only C++ core under
`cpp/radloc` module for module, and the two are checked against each other and
against the archived descriptors by `tests/`. The C++ side is what the ROS 2
nodes use; this side is for reproducing the paper and for trying things out.

```
radloc/
  polar_image.py   polar image loading, layout normalisation, point extraction
  descriptor.py    CA-CFAR azimuth rejection + per-range-band descriptor
  retrieval.py     two-stage place search (coarse k-d tree -> fine re-rank)
scripts/
  describe_sequence.py   write descriptors for a sequence
tests/
  legacy_reference.py    the original implementation, kept verbatim as a fixture
  test_equivalence.py    package vs archive, package vs original, package vs C++
```

## Install

```bash
pip install -e .[test]
```

## Use

```python
import radloc

db = radloc.PlaceDatabase()                 # defaults: coarse 20, top-k 10
for path in scans:
    db.add(radloc.compute_descriptor(path))
db.build()

for index, distance in db.query(radloc.compute_descriptor(query_path)):
    ...
```

## Test

```bash
python3 -m pytest tests/ -v
RADLOC_MULRAN=/data/referee/Mulran python3 -m pytest tests/ -v   # in Docker
```

Everything also runs in the project image, which already carries numpy, scipy
and OpenCV:

```bash
docker compose -f ../docker/docker-compose.yml run --rm radloc \
  bash -lc "cd /radloc/python && python3 -m pytest tests/ -q"
```

## Differences from the original

Only ReFeree++ was carried over; the nine competing descriptors and the
analysis scripts in the source tree were left behind, along with the benchmark
plumbing they shared.

* **The descriptor is unweighted.** Range weighting is applied once, when the
  coarse search key is built (`range_weighted`). The original multiplied by
  `arange(1, 41)` at descriptor time and divided it straight back out at
  re-rank time; the result is identical and the round trip is gone.
* **Band values are mean return, not `255 - mean`.** Because the band value is
  a mean, the two differ by an affine flip, and both retrieval stages are
  invariant to it. `from_inverted()` converts archived descriptors.
* **The fine metric is named for what it computes.** The original called it a
  Huber loss, but overwrote the delta it was passed and tested `abs_diff <= 0`,
  so every band took the L1 branch. `FineMetric.HUBER` offers the intended
  behaviour; it changes the ranking, so results are not comparable with
  archived runs.

`tests/legacy_reference.py` keeps the original verbatim, so these claims are
checked rather than asserted.
