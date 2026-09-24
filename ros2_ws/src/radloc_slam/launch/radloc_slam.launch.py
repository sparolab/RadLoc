"""Radar odometry and pose-graph SLAM over a recorded sequence.

    ros2 launch radloc_slam radloc_slam.launch.py sequence_dir:=/data/radar/Mulran/KAIST_03
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

        Node(
            package="radloc_odometry",
            executable="odometry_node",
            name="radloc_odometry",
            output="screen",
            parameters=[{"sequence_dir": sequence_dir}],
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
