# Multi-session SLAM

Aligns a query session to a central one, both written by the [SLAM](../slam): inter-session loops from RadLoc
(the place database of the central session queried with each keyframe of the query session), registered by
phase correlation and rejected above `loopFitnessScoreThreshold`, then one pose graph with an anchor node per
session (the LT-SLAM formulation) solved with iSAM2. ROS-free: file in, file out.

```bash
# plain CMake (any machine with GTSAM, PCL, OpenCV)
cmake -S applications/multi-session-slam -B build/ms -DCMAKE_BUILD_TYPE=Release && cmake --build build/ms -j8
build/ms/multi_session_node applications/multi-session-slam/config/kaist03_split.yaml
# ROS 2 image: ros2 run radloc_multi_session multi_session_node <config.yaml>
# ROS 1 image: multi_session_node <config.yaml>
```

`config/*.yaml`: `sessions_dir` holds one folder per session (`central_sess_name`, `query_sess_name`), and the
result goes to `save_directory` (removed and recreated on each run): the optimized poses of both sessions in the
central frame and the inter-session loops.

MulRan `KAIST_03` split in two halves, each in its own frame (`config/kaist03_split.yaml`): 997 inter-session
loops in 18 s; the distance from each keyframe of the second session to the nearest of the first goes from
50.64 m (p50) to **0.59 m**, and 77.3 % of them land within 5 m (8.9 % before), which matches the overlap of the
two halves (about 78 %).
