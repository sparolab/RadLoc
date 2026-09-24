# radloc (C++ core)

Header-only C++17 library for radar place recognition and 3-DoF registration.
Depends on OpenCV only — no ROS, no PCL, no FFTW, no Python. It is meant to be
dropped into a ROS 2 node as a plain CMake subdirectory.

```
include/radloc/
  polar_image.hpp   polar image loading, layout normalisation, point extraction
  descriptor.hpp    CA-CFAR azimuth rejection + per-range-band descriptor
  retrieval.hpp     two-stage place search (coarse k-d tree -> fine re-rank)
  phase_corr.hpp    3-DoF registration by phase correlation
  third_party/      nanoflann + k-d tree adaptor (BSD)
```

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Use

```cpp
#include "radloc/descriptor.hpp"
#include "radloc/retrieval.hpp"
#include "radloc/phase_corr.hpp"

radloc::PlaceDatabase db;                       // defaults: coarse 20, top-k 10
for (const auto& path : scans) db.add(radloc::computeDescriptor(path));
db.build();

const auto matches = db.query(radloc::computeDescriptor(query_path));
const auto rel = radloc::registerScans(target_points, source_points);
// rel.x, rel.y [m], rel.theta [rad], rel.response
```

## Design notes

**The descriptor is unweighted.** Range weighting — counting distant bands for
more — is applied once, when the coarse search key is built
(`Descriptor::rangeWeighted`). The Python original multiplied by `arange(1,41)`
at descriptor time and divided it straight back out at re-rank time; the result
is identical and the round trip is gone.

**Band values are mean return, not `255 - mean`.** Because the band value is a
mean, the two differ by an affine flip, and both retrieval stages are invariant
to it: the fine stage compares differences, the coarse stage is a Euclidean
nearest-neighbour search. `fromInverted()` converts archived descriptors.

**The fine metric is a mean absolute difference.** The reference called this a
Huber loss, but its implementation overwrote the delta it was passed and tested
`abs_diff <= 0`, so every band took the L1 branch. `FineMetric::Huber` offers
the presumably intended behaviour; it changes the ranking, so results are not
comparable with archived runs.

**Rotation is disambiguated.** The FFT magnitude is centrally symmetric, so the
log-polar stage recovers rotation only modulo 180 degrees. `PhaseCorrParams::
resolve_180_ambiguity` (on by default) de-rotates both candidates and keeps the
stronger translation peak. Without it, 5 of 8 rotations beyond +-90 degrees come
back flipped by pi — see `tools/test_phase_corr.cpp`.

**Translation is reported in the target frame.** Phase correlation measures the
shift against the de-rotated source, so the raw peak lives in the rotated frame
and is rotated back before being returned.

## Validation

`validate_descriptor` and `validate_retrieval` check the C++ output against the
archived Python descriptors and a line-for-line Python port of the reference
search (`tools/reference_retrieval.py`).

```bash
M=/storage/Datasets/ReFeree/Mulran
./build/validate_descriptor $M/KAIST_03/polar $M/KAIST_03/refereepp_84x4 50
./build/validate_retrieval  $M/KAIST_03/refereepp_84x4 400 20 10 > /tmp/cpp.txt
python3 tools/reference_retrieval.py $M/KAIST_03/refereepp_84x4 400 20 10 > /tmp/py.txt
diff /tmp/cpp.txt /tmp/py.txt

./build/test_phase_corr $M/KAIST_03/polar/1567410201812840928.png
```

| check | sequence | result |
| --- | --- | --- |
| descriptor vs archived `.npy` | KAIST_03, Riverside_03 | max relative error ~1e-14 |
| retrieval vs Python reference | KAIST_03, Riverside_03 | byte-identical, coarse 20/8 x top-k 10/5 |
| registration, synthetic ground truth | KAIST_03 | 14/14 within 0.4 m, 0.4 deg |

## Licence

BSD 3-Clause, matching the parent repository. `third_party/` is BSD-licensed
nanoflann by Jose Luis Blanco et al.
