from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

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

    velocity_smoother_config = PathJoinSubstitution(
        [
            FindPackageShare("yektemak_bringup"),
            "config",
            "velocity_smoother.yaml",
        ]
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[velocity_smoother_config],
        remappings=[
            ("cmd_vel", "cmd_vel_raw"),
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )

    velocity_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_velocity",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["velocity_smoother"],
            }
        ],
    )

    return LaunchDescription(
        [
            gazebo_simulation,
            velocity_smoother,
            velocity_lifecycle_manager,
        ]
    )

