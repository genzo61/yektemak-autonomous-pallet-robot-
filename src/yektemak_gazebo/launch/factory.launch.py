"""Fabrika dunyasi (bos, robotsuz). Robotlar spawn_robot.launch.py ile eklenir.

  ros2 launch yektemak_gazebo factory.launch.py gui:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_ros = FindPackageShare("gazebo_ros")
    sim = FindPackageShare("yektemak_gazebo")
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    return LaunchDescription([
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
