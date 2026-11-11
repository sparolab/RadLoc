"""Radar odometry and pose-graph SLAM over a recorded sequence.

    ros2 launch radloc_slam radloc_slam.launch.py sequence_dir:=/data/radar/Mulran/KAIST_03 dataset:=mulran

``dataset`` picks the radar's range resolution (radloc_odometry/config/<dataset>.yaml): mulran 0.0596 m per
bin (also Boreas), oxford 0.0432 m per bin (Oxford Radar RobotCar). A wrong value scales the whole trajectory.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sequence_dir = LaunchConfiguration("sequence_dir")
    save_directory = LaunchConfiguration("save_directory")

    return LaunchDescription([
        DeclareLaunchArgument("sequence_dir",
                              description="Sequence directory holding polar/"),
        DeclareLaunchArgument("save_directory", default_value="/data/output/",
                              description="Where the pose graph is written; end with /"),
        DeclareLaunchArgument("keyframe_gap_m", default_value="0.0"),
        DeclareLaunchArgument("dataset", default_value="mulran",
                              description="mulran (0.0596 m/bin) or oxford (0.0432 m/bin, Oxford Radar RobotCar); Boreas is 0.0596 like MulRan"),
        DeclareLaunchArgument("scan_period_ms", default_value="100"),

        Node(
            package="radloc_odometry",
            executable="odometry_node",
            name="radloc_odometry",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("radloc_odometry"), "config",
                                      [LaunchConfiguration("dataset"), ".yaml"]]),
                {"sequence_dir": sequence_dir, "scan_period_ms": LaunchConfiguration("scan_period_ms")},
            ],
        ),
        Node(
            package="radloc_slam",
            executable="pose_graph_node",
            name="radloc_slam",
            output="screen",
            parameters=[{
                "save_directory": save_directory,
                "keyframe_gap_m": LaunchConfiguration("keyframe_gap_m"),
            }],
        ),
    ])
