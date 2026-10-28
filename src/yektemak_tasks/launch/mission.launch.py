"""Gorev yoneticisi (Nav2 calisiyor olmali)."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="yektemak_tasks",
            executable="mission_manager",
            name="mission_manager",
            output="screen",
            parameters=[PathJoinSubstitution(
                [FindPackageShare("yektemak_tasks"), "config", "mission.yaml"])],
        ),
    ])
