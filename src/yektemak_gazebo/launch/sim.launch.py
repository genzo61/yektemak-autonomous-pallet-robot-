"""Gazebo depo dünyası + Yektemak robotu + lift controller'ları.

Örnek:
  ros2 launch yektemak_gazebo sim.launch.py
  ros2 launch yektemak_gazebo sim.launch.py gui:=false   # başsız (headless)
"""

import os

import sys

from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gazebo_guard import check_no_running_gazebo  # noqa: E402


def generate_launch_description():
    gazebo_ros_share = FindPackageShare("gazebo_ros")
    description_share = FindPackageShare("yektemak_description")
    sim_share = FindPackageShare("yektemak_gazebo")

    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")

    declare_args = [
        DeclareLaunchArgument("gui", default_value="true",
                              description="Gazebo arayüzü (gzclient) açılsın mı"),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([sim_share, "worlds", "warehouse.world"]),
            description="Gazebo dünya dosyası"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
    ]

    model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[PathJoinSubstitution([sim_share, "models"]), os.pathsep,
               os.environ.get("GAZEBO_MODEL_PATH", "")],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gzserver.launch.py"])),
        launch_arguments={"world": world, "verbose": "false"}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gzclient.launch.py"])),
        condition=IfCondition(gui),
    )

    robot_description = ParameterValue(
        Command(["xacro ",
                 PathJoinSubstitution([description_share, "urdf", "yektemak_robot.urdf.xacro"]),
                 " use_sim:=true"]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "yektemak_robot",
                   "-x", x, "-y", y, "-z", "0.01", "-Y", yaw],
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
        )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawner("joint_state_broadcaster"), spawner("lift_controller")],
        )
    )

    return LaunchDescription(
        declare_args
        + [OpaqueFunction(function=check_no_running_gazebo), model_path, gzserver, gzclient, robot_state_publisher, spawn_robot,
           start_controllers]
    )
