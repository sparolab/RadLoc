# SLAM

Radar odometry + pose-graph SLAM with RadLoc loop closing, on ROS 1 Noetic and ROS 2 Humble. Both use the same
ROS-free code in [`common/`](common):

| | |
| --- | --- |
| `common/src/{radar_utils,features,association,key_operation}.cpp` | radar odometry from [Yeti](https://github.com/keenan-burnett/yeti_radar_odometry): Cen features, ORB matching, motion-distorted RANSAC with Doppler correction |
| `common/include/radloc_slam/keyframe_store.hpp` | keyframes, their clouds and the RadLoc place database behind one lock |
| `common/src/loop_registration.cpp` | a loop proposal to a relative pose by phase correlation (`radloc::registerScans`) |
| `common/src/graph_io.cpp`, `reoptimize.cpp` | the session files; re-solving a saved pose graph with other loop noise |

| | ROS 1 Noetic ([`ros1/radloc_ros`](ros1/radloc_ros)) | ROS 2 Humble ([`ros2/`](ros2)) |
| --- | --- | --- |
| odometry | `rosrun radloc_ros odometry_node` | `ros2 run radloc_odometry odometry_node` |
| SLAM | `rosrun radloc_ros pose_graph_node` | `ros2 run radloc_slam pose_graph_node` |
| both | `roslaunch radloc_ros radloc_slam.launch sequence_dir:=... save_directory:=...` | `ros2 launch radloc_slam radloc_slam.launch.py sequence_dir:=... save_directory:=...` |

The odometry node replays `<sequence_dir>/polar/*.png` (MulRan / Oxford layout) and publishes `odom`,
`cloud_local`, `cloud_global` and `descriptor` (`RadLocDescriptor`: the range bands); the SLAM node keeps a
keyframe every `keyframe_gap_m` (0: every scan), proposes a loop for each new keyframe among those at least
`loop_min_travel_m` (10 m) back along the path, registers it by phase correlation (`loop_min_response` 0.3) and
optimizes with iSAM2. Parameters and topics are the same on both ROS versions.

The session written to `save_directory` (`singlesession_posegraph.g2o`, `odom_poses_kitti.txt`,
`optimized_poses_kitti.txt`, `keyframe_times.txt`, `RadLocDescriptors/`, `Scans/`) is what
[multi-session SLAM](../multi-session-slam) reads.

Build without Docker: put `ros1/radloc_ros` (catkin) or `ros2/*` (colcon) in a workspace, as a symlink or a
copy, and pass `-DRADLOC_ROOT=<this repository>` if the package no longer sits at
`applications/slam/ros*/` in it.
