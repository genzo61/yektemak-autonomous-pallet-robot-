from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("yektemak_gazebo"),
                    "launch",
                    "simulation.launch.py",
                ]
            )
        )
    )

    return LaunchDescription(
        [
            gazebo_simulation,
        ]
    )

