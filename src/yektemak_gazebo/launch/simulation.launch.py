from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_package = FindPackageShare("gazebo_ros")
    description_package = FindPackageShare("yektemak_description")
    simulation_package = FindPackageShare("yektemak_gazebo")

    gazebo_launch_file = PathJoinSubstitution(
        [gazebo_package, "launch", "gazebo.launch.py"]
    )

    world_file = PathJoinSubstitution(
        [simulation_package, "worlds", "empty.world"]
    )

    robot_xacro_file = PathJoinSubstitution(
        [
            description_package,
            "urdf",
            "yektemak_robot.urdf.xacro",
        ]
    )

    robot_description = ParameterValue(
        Command(["xacro ", robot_xacro_file]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "world": world_file,
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_yektemak_robot",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "yektemak_robot",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.01",
        ],
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_robot,
        ]
    )
