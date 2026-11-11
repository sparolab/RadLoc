# Multi-robot SLAM: Commerge

Robots that map on their own merge their maps by exchanging only a few keyframes:
[**Commerge**](https://github.com/sparolab/Commerge) (IJRR 2026) takes the RadLoc affinity between two robots,
finds the regions where both saw the same places in sequence, and keeps the few consistent pairs worth sending.

<p align="center"><img src="https://raw.githubusercontent.com/sparolab/Commerge/main/tools/demo/radar/demo.gif" alt="Commerge on MulRan KAIST_03 split into two radar robots: maps from the radar's own detections, RadLoc affinity, Commerge's steps (A)-(C) and the merge" width="95%" /></p>

MulRan `KAIST_03` split into two robots (robot B in its own frame, turned by 60°), one RadLoc descriptor per
keyframe, affinity `1 - D / max D` of the fine distances:

```
[commerge] registered B to A from 14 selected pairs: RMS 1.25 m; vs ground truth: 0.21 m, 0.05 deg (B keyframes 0.45 m off on average)
[commerge] exchanged 12 A + 15 B keyframes (1.2 % of 2344, 2.6 KB of poses) from region(s) [0, 1, 3, 5, 6, 7, 8, 9] of 11
```

```bash
pip install commerge radloc
git clone https://github.com/sparolab/Commerge.git && cd Commerge
python tools/demo/radar/make_sessions.py --seq <MulRan>/KAIST_03 --gt <MulRan>/KAIST_03/global_pose.csv --out runs/radar
commerge runs/radar/robot_a runs/radar/robot_b --affinity runs/radar/affinity.npy --threshold 0.93 --min-area 50 \
    --registration pairs --policy all --out runs/radar/merged
```

Full recipe: [Commerge `tools/demo/radar`](https://github.com/sparolab/Commerge/tree/main/tools/demo/radar).
Any affinity works: with your own RadLoc sessions, `radloc describe` each robot's scans and pass
`1 - D / max D` of the pairwise fine distances (`radloc.fine_distance`) as `--affinity`.
