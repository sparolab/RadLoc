<div align="center">
  <h1>RadLoc</h1>
  <a href="https://pypi.org/project/radloc/"><img src="https://img.shields.io/pypi/v/radloc?logo=pypi&logoColor=white&label=PyPI" alt="PyPI" /></a>
  <a href="python"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" alt="Python" /></a>
  <a href="cpp"><img src="https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white" alt="C++17" /></a>
  <a href="applications/slam/ros1"><img src="https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white" alt="ROS 1 Noetic" /></a>
  <a href="applications/slam/ros2"><img src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white" alt="ROS 2 Humble" /></a>
  <a href="#whale-docker"><img src="https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white" alt="Docker" /></a>
  <a href="https://github.com/sparolab/RadLoc/actions/workflows/ci.yml"><img src="https://github.com/sparolab/RadLoc/actions/workflows/ci.yml/badge.svg?branch=main" alt="CI" /></a>
  <a href="https://sparolab.github.io/research/radloc/"><img src="https://img.shields.io/badge/Project-Page-6f42c1" alt="Project page" /></a>
  <a href="https://arxiv.org/abs/2607.08115"><img src="https://img.shields.io/badge/arXiv-2607.08115-b33737?logo=arXiv" alt="arXiv" /></a>
  <a href="https://arxiv.org/abs/2607.08115"><img src="https://img.shields.io/badge/IROS-2026-1f6feb" alt="IROS 2026" /></a>
  <a href="https://www.youtube.com/watch?v=q8iaEpMSANU"><img src="https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white" alt="YouTube" /></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-BSD--3--Clause-green" alt="BSD-3-Clause" /></a>
  <br />
  <a href="https://pypi.org/project/radloc/"><img src="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=14&duration=2500&pause=800&center=true&vCenter=true&width=200&height=24&lines=%24+pip+install+radloc" alt="pip install radloc" /></a>
  <br />
  <img src="assets/radloc.gif" alt="RadLoc demo" width="95%" />
  <p>
    <strong>[IROS 2026] RadLoc: Radar-based 3-DoF Global Localization via Fast, Robust,<br />
    and Lightweight Spatial Descriptor Across Diverse Environmental Scenarios</strong>
  </p>
  <p>
    Hogyun Kim, Jiwon Choi, Jungwoo Lee, Younggun Cho<br />
    <a href="https://sites.google.com/view/sparo"><strong>Spatial AI and Robotics Lab (SPARO)</strong></a>, Inha University
  </p>
</div>

______________________________________________________________________

RadLoc is a radar global descriptor and the SLAM that runs on it. A scan is reduced to one value per range band, averaged over azimuth:

- :compass: **Rotation invariant and tiny**: forty floats describe a place, whatever the heading.
- :mag: **Two-stage search**: a k-d tree over the leading range-weighted bands proposes candidates, the full descriptor re-ranks them.
- :dart: **3-DoF registration without an initial guess**: loops are registered by phase correlation, so there are no correspondences to find and nothing to initialize.
- :world_map: **One library, three applications**: single-session [SLAM](#world_map-slam), [multi-session SLAM](#link-multi-session-slam) and [multi-robot map merging with Commerge](#handshake-multi-robot-slam-commerge), on ROS 1 and ROS 2.

```
cpp/                          C++17 core (header-only, OpenCV): descriptor, place search, phase-correlation registration
python/radloc/                the same in Python (pip install radloc), checked against the C++
applications/
  slam/
    common/                   ROS-free radar odometry (from Yeti) and pose-graph pieces, shared by both ROS versions
    ros1/radloc_ros           ROS 1 Noetic: odometry_node + pose_graph_node
    ros2/radloc_{odometry,slam,interfaces}   ROS 2 Humble: the same nodes
  multi-session-slam/         align sessions written by the SLAM (ROS-free tool, colcon or plain CMake)
  multi-robot-slam/           Commerge: merge two robots' maps by exchanging a few keyframes
docker/                       ROS 1 and ROS 2 images
```

______________________________________________________________________

## :package: Prerequisites

- **Library**: Python 3.9+ (NumPy, SciPy, OpenCV) or a C++17 compiler with OpenCV. Nothing else.
- **SLAM**: ROS 1 Noetic or ROS 2 Humble, PCL, GTSAM 4.2 — or just Docker, which brings all of it.
- **Data**: radar scans as polar PNGs (one row per azimuth, Oxford/MulRan layout), e.g. [MulRan](https://sites.google.com/view/mulran-pr) `KAIST_03/polar`.

## :hammer_and_wrench: Installation

```bash
pip install radloc
# or from a clone
git clone https://github.com/sparolab/RadLoc.git && cd RadLoc && pip install .
```

### C++

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release && cmake --build cpp/build -j8
ctest --test-dir cpp/build                  # synthetic checks of descriptor, search and registration
```

In your project, `add_subdirectory(RadLoc/cpp)` and link `radloc::radloc` (header-only).

## :rocket: Quick start

```bash
radloc describe <MulRan>/KAIST_03/polar --out kaist03.npy     # one descriptor per scan
radloc loops kaist03.npy --out loops.csv --exclude 100       # loop candidates within the run
```

```python
import radloc

db = radloc.PlaceDatabase()                       # coarse k-d tree on 20 bands, top 10 re-ranked
for scan in scans:                                # polar PNG paths or (range x azimuth) arrays
    db.add(radloc.compute_descriptor(scan))
matches = db.query(radloc.compute_descriptor(query), limit=len(db) - 100)   # [(index, distance), ...]
```

```cpp
#include "radloc/descriptor.hpp"
#include "radloc/phase_corr.hpp"
#include "radloc/retrieval.hpp"

radloc::PlaceDatabase db;
for (const auto& path : scans) db.add(radloc::computeDescriptor(path));
const auto matches = db.query(radloc::computeDescriptor(query_path));
const auto rel = radloc::registerScans(target_points, source_points);   // rel.x, rel.y [m], rel.theta [rad]
```

______________________________________________________________________

## :world_map: SLAM

Radar odometry (Yeti: Cen features, ORB matching, motion-distorted RANSAC with Doppler correction) publishes the motion, the feature cloud and the RadLoc descriptor of every scan; the pose-graph node closes loops with RadLoc, registers them by phase correlation and optimizes with iSAM2. The run is written as a **session** (pose graph, descriptors, scans) that the multi-session tool reads back.

MulRan `KAIST_03`, 3398 keyframes, 1723 loops: odometry alone drifts to 98.3 m against ground truth; with loop closure the error stays near 6 m for the whole 6.2 km.

<p align="center"><img src="assets/kaist03_trajectory.png" alt="KAIST_03 trajectory: odometry drifts, RadLoc SLAM stays on the ground truth" width="90%" /></p>

```bash
# ROS 2 Humble
ros2 launch radloc_slam radloc_slam.launch.py sequence_dir:=<MulRan>/KAIST_03 save_directory:=/data/output/KAIST_03/
# ROS 1 Noetic
roslaunch radloc_ros radloc_slam.launch sequence_dir:=<MulRan>/KAIST_03 save_directory:=/data/output/KAIST_03/
```

Both read `polar/` from the sequence folder in the Oxford layout (MulRan scans: convert once with [`applications/slam/tools/mulran_to_oxford.py`](applications/slam/tools/mulran_to_oxford.py)); the session goes to `save_directory` (`optimized_poses_kitti.txt`, `odom_poses_kitti.txt`, `keyframe_times.txt`, `singlesession_posegraph.g2o`, `RadLocDescriptors/`, `Scans/`). See [applications/slam](applications/slam).

## :link: Multi-session SLAM

Two halves of that run, each in its own frame — the second retraces the first over about 78 % of its length. 997 inter-session loops in 18 s. Measured as the distance from each keyframe of the second session to the nearest of the first, which needs no external ground truth:

| | p50 | within 5 m |
| --- | --- | --- |
| before | 50.64 m | 8.9 % |
| after | **0.59 m** | **77.3 %** |

The ceiling matches the overlap: what has a counterpart is aligned, the rest has nothing to align to.

```bash
multi_session_node applications/multi-session-slam/config/kaist03_split.yaml     # sessions_dir/{01,02} -> save_directory
```

See [applications/multi-session-slam](applications/multi-session-slam).

## :handshake: Multi-robot SLAM (Commerge)

Two robots, each mapping on its own, merge their maps by exchanging only a few keyframes: [Commerge](https://github.com/sparolab/Commerge) (IJRR 2026) takes the RadLoc affinity between the robots and picks the keyframes worth sending. On MulRan `KAIST_03` split into two robots (robot B in a frame turned by 60°): 27 of 2,344 keyframes exchanged (1.2 %, 2.6 KB of poses), B's frame recovered within 0.05°.

<p align="center"><img src="https://raw.githubusercontent.com/sparolab/Commerge/main/tools/demo/radar/demo.gif" alt="Commerge on MulRan KAIST_03 split into two radar robots, with the RadLoc affinity" width="90%" /></p>

See [applications/multi-robot-slam](applications/multi-robot-slam).

______________________________________________________________________

## :whale: Docker

```bash
docker compose -f docker/docker-compose.yml build                # ROS 2 Humble image (radloc:humble)
docker compose -f docker/docker-compose.yml run --rm slam        # SLAM on SEQUENCE_DIR -> SAVE_DIR
docker compose -f docker/docker-compose.yml run --rm multi_session
docker build -f docker/Dockerfile.ros1 -t radloc:noetic .        # ROS 1 Noetic image
```

Datasets are mounted read-only; see [docker/README.md](docker/README.md) for the paths.

## :bulb: Design notes

**The descriptor is unweighted.** Range weighting — counting distant bands for more — is applied once, when the coarse search key is built. It belongs to how two places are compared, not to the place itself.

**Search is two-stage.** A k-d tree over the leading range-weighted bands proposes candidates; the shortlist is re-ranked on the full unweighted descriptor, where the shared offset cancels.

**Rotation is disambiguated.** The FFT magnitude is centrally symmetric, so the log-polar stage recovers rotation only modulo 180°. Both candidates are de-rotated and the stronger translation peak wins; without this, five rotations in eight beyond ±90° come back flipped by π.

See [cpp/README.md](cpp/README.md) and [python/README.md](python/README.md) for the rest.

## Citation

If you use RadLoc in your research, please cite our [paper](https://arxiv.org/abs/2607.08115).

<details>
  <summary><strong>See bibtex</strong></summary>

```bibtex
@inproceedings{kim2026radloc,
  author    = {Kim, Hogyun and Choi, Jiwon and Lee, Jungwoo and Cho, Younggun},
  title     = {RadLoc: Radar-based 3-DoF Global Localization via Fast, Robust, and Lightweight Spatial Descriptor Across Diverse Environmental Scenarios},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2026}
}
```

</details>

## Contributing

Issues and pull requests are welcome.

<a href="https://github.com/sparolab/RadLoc/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=sparolab/RadLoc" />
</a>

## Acknowledgements

The radar odometry is [Yeti](https://github.com/keenan-burnett/yeti_radar_odometry) by Keenan Burnett, with the feature detector of Sarah H. Cen and Paul Newman. RadLoc builds on [ReFeree](https://github.com/sparolab/ReFeree). Related: [SOLiD](https://github.com/sparolab/SOLiD), [Commerge](https://github.com/sparolab/Commerge), [SKiD-SLAM](https://github.com/sparolab/SKiD-SLAM).

## License

BSD 3-Clause. See [LICENSE](LICENSE).
