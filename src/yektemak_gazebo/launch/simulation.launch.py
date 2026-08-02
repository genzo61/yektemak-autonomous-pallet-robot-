from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

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
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    lift_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="lift_position_controller_spawner",
        output="screen",
        arguments=[
            "lift_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                lift_controller_spawner,
            ],
        )
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_robot,
            start_controllers_after_spawn,
        ]
    )
