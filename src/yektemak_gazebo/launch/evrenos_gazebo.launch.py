import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # -----------------------------------------------------
    # Packages
    # -----------------------------------------------------

    gazebo_share = get_package_share_directory("gazebo_ros")
    description_share = get_package_share_directory("yektemak_description")
    simulation_share = get_package_share_directory("yektemak_gazebo")


    # -----------------------------------------------------
    # Files
    # -----------------------------------------------------

    xacro_file = os.path.join(
        description_share,
        "urdf",
        "evrenos_robot.urdf.xacro",
    )

    world_file = os.path.join(
        simulation_share,
        "worlds",
        "evrenos_test.world",
    )

    ekf_config = os.path.join(
        simulation_share,
        "config",
        "evrenos_ekf.yaml",
    )

    # -----------------------------------------------------
    # Robot description
    # -----------------------------------------------------

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
            ]
        ),
        value_type=str,
    )


    # -----------------------------------------------------
    # Gazebo
    # -----------------------------------------------------

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share,
                "launch",
                "gazebo.launch.py",
            )
        ),

        launch_arguments={
            "world": world_file,
            "verbose": "false",
        }.items(),
    )


    # -----------------------------------------------------
    # Robot State Publisher
    # -----------------------------------------------------

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


    # -----------------------------------------------------
    # Spawn EVRENOS
    # -----------------------------------------------------

    spawn_evrenos = Node(

        package="gazebo_ros",
        executable="spawn_entity.py",

        name="spawn_evrenos",

        output="screen",

        arguments=[
            "-entity",
            "evrenos_evr03",

            "-topic",
            "robot_description",

            "-x",
            "0.0",

            "-y",
            "0.0",

            "-z",
            "0.05",

            "-Y",
            "0.0",
        ],
    )

        # -----------------------------------------------------
    # EKF Localization
    # -----------------------------------------------------

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config,
            {
                "use_sim_time": True,
            },
        ],
    )

    # -----------------------------------------------------
    # Launch
    # -----------------------------------------------------

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_evrenos,
            ekf_node,
        ]
    )