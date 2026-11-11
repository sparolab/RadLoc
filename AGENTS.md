# RadLoc — setup guide for coding agents

You are setting up and running RadLoc (a radar global descriptor: one value per range band, rotation invariant, with two-stage place search and phase-correlation registration) for a user. Take them from a fresh machine to RadLoc SLAM on MulRan `KAIST_03`, then optionally multi-session SLAM and multi-robot merging. Follow the steps in order. Run every command yourself, check its output before moving on, and stop to ask the user only where a step says **Ask**.

## 0. Ask the user first

**Ask**, in a single message:

1. Where their radar data is: a MulRan sequence folder with `polar/` (one PNG per scan, one row per azimuth) and `global_pose.csv`, e.g. `.../Mulran/KAIST_03`. If they have none, point them to https://sites.google.com/view/mulran-pr (registration required).
2. Library only (**pip**), or the SLAM too — then **Docker** (recommended; ROS 2 Humble image, or ROS 1 Noetic) or their own ROS 1 / ROS 2 install (needs PCL and GTSAM 4.2).
3. Where to save results (default `~/radloc_results`).

## 1. Get the code

```bash
git clone https://github.com/sparolab/RadLoc.git && cd RadLoc
```

Layout: `cpp/` header-only C++17 core (OpenCV only: `descriptor.hpp`, `retrieval.hpp`, `phase_corr.hpp`, `polar_image.hpp`; tests in `cpp/tests`, tools in `cpp/tools`). `python/radloc/` the same in Python, plus the `radloc` command (`cli.py`). `applications/slam/common/` ROS-free odometry (Yeti) and pose-graph pieces; `applications/slam/ros1/radloc_ros` and `applications/slam/ros2/radloc_{interfaces,odometry,slam}` the nodes; `applications/multi-session-slam/` the ROS-free multi-session tool; `applications/multi-robot-slam/` how to use Commerge. `docker/` the images.

## 2. Library

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -U pip && pip install ".[test]"
radloc --version                                   # radloc 2.0.0
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release && cmake --build cpp/build -j8 && ctest --test-dir cpp/build
pytest -q python/tests                             # includes the C++/Python parity test once cpp/build exists
radloc describe <SEQ>/polar --out <OUT>/kaist03.npy
radloc loops <OUT>/kaist03.npy --out <OUT>/loops.csv --exclude 100
```

Check: `ctest` passes, pytest passes (the archive tests skip without `RADLOC_MULRAN`), `kaist03.npy` has 40 columns.

## 3. SLAM

The odometry needs the Oxford layout (one row per azimuth with an 11-byte header). If `polar/*.png` are 3360 x 400 (range x azimuth, as downloaded from MulRan), convert them first, outside the dataset folder:

```bash
python3 applications/slam/tools/mulran_to_oxford.py <SEQ>/polar <DATA>/Mulran/KAIST_03/polar
```

Docker, ROS 2:

```bash
docker compose -f docker/docker-compose.yml build
RADLOC_DATASETS_HOST=<parent of Mulran> RADLOC_OUTPUT_HOST=<OUT> SEQUENCE_DIR=/data/datasets/Mulran/KAIST_03 \
  SAVE_DIR=/data/output/KAIST_03/ docker compose -f docker/docker-compose.yml run --rm slam
```

Docker, ROS 1: `docker build -f docker/Dockerfile.ros1 -t radloc:noetic .`, then the `slam_ros1` service with the same variables. Run one ROS 2 SLAM per Docker network (or give each a different `ROS_DOMAIN_ID`): two ROS 2 containers on the default bridge hear each other's topics and mix their runs.

The odometry replays every scan (100 ms apart by default, `scan_period_ms`); `KAIST_03` (3,461 scans) takes 15–20 minutes. MulRan needs `dataset:=mulran` (the default: 0.0596 m per range bin); Oxford Radar RobotCar `dataset:=oxford`. Check the log for `loop <i> <- <j>` lines and the session in `SAVE_DIR`: `optimized_poses_kitti.txt`, `odom_poses_kitti.txt`, `keyframe_times.txt`, `singlesession_posegraph.g2o`, `RadLocDescriptors/`, `Scans/`. To score against ground truth, interpolate the MulRan ground truth at `keyframe_times.txt` and align with a rigid SE(2) transform (the README result: 3459 keyframes, 1916 loops, ATE 71.7 m for the odometry, 5.4 m with loop closure, the same on ROS 1 and ROS 2). A few `the motion is supported by N of M matches; keeping the previous motion` warnings are expected (4 on KAIST_03); many of them mean a wrong `dataset` or a wrong input layout.

## 4. Multi-session SLAM (optional)

Two sessions in `<sessions_dir>/01` and `<sessions_dir>/02` (e.g. the SLAM run on the two halves of a sequence, or on two sequences of the same place). Copy `applications/multi-session-slam/config/kaist03_split.yaml`, set `sessions_dir` and `save_directory` (**it is deleted and recreated**: never point it at anything the user keeps), and run `multi_session_node <config.yaml>` (ROS 2 image: `ros2 run radloc_multi_session multi_session_node <config.yaml>`).

## 5. Multi-robot SLAM (optional)

Follow `applications/multi-robot-slam/README.md`: `pip install commerge`, then Commerge's `tools/demo/radar` recipe on the same MulRan sequence.

## 6. Report

Tell the user: the versions installed, test results, the number of keyframes and loops of the SLAM run, where the session is, and (if run) the multi-session or Commerge result line.
