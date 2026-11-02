"""Fabrika dunyasi (bos, robotsuz). Robotlar spawn_robot.launch.py ile eklenir.

  ros2 launch yektemak_gazebo factory.launch.py gui:=true
"""

import os

import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gazebo_guard import check_no_running_gazebo  # noqa: E402


def generate_launch_description():
    gazebo_ros = FindPackageShare("gazebo_ros")
    sim = FindPackageShare("yektemak_gazebo")
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    return LaunchDescription([
        OpaqueFunction(function=check_no_running_gazebo),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution(
            [sim, "worlds", "factory.world"])),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", [PathJoinSubstitution([sim, "models"]),
                                                     os.pathsep, os.environ.get("GAZEBO_MODEL_PATH", "")]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([gazebo_ros, "launch", "gzserver.launch.py"])),
            launch_arguments={"world": world, "verbose": "false"}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([gazebo_ros, "launch", "gzclient.launch.py"])),
            condition=IfCondition(gui)),
    ])
