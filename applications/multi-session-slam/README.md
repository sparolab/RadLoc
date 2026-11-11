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

MulRan `KAIST_03` cut in two halves (1730 and 1731 scans), each run through the SLAM in its own frame
(`dataset:=mulran`), then `config/kaist03_split.yaml`: 896 inter-session loops in about 30 s. The distance from
each keyframe of the second session to the nearest of the first goes from 29.31 m (p50) to **0.75 m**, and
77.4 % of them land within 5 m (12.9 % before); against ground truth, with the first session aligned to it
(SE(2)), the second session's error goes from 405.2 m to **5.7 m** mean (5.6 m median).
