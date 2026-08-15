import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

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

    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="true",
    )

    slam_enabled = LaunchConfiguration("slam")

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
    )

    rviz_enabled = LaunchConfiguration("rviz")

    xacro_file = os.path.join(
        description_share,
        "urdf",
        "evrenos_robot.urdf.xacro",
    )

    world_file = os.path.join(
        simulation_share,
        "worlds",
        "evrenos_competition.world",
    )

    ekf_config = os.path.join(
        simulation_share,
        "config",
        "evrenos_ekf.yaml",
    )

    pointcloud_to_scan_config = os.path.join(
        simulation_share,
        "config",
        "evrenos_pointcloud_to_scan.yaml",
    )

    slam_config = os.path.join(
        simulation_share,
        "config",
        "evrenos_slam.yaml",
    )

    rviz_config = os.path.join(
        simulation_share,
        "rviz",
        "evrenos_slam.rviz",
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

            # TEKNOFEST ana parkur başlangıcı
            "-x",
            "9.2",

            "-y",
            "-8.0",

            # Biraz yukarıda doğsun, fizik ile zemine otursun
            "-z",
            "0.03",

            # Robotun +X önü parkur boyunca sola (-X) baksın
            "-Y",
            "3.14159265",
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
    # PointCloud2 -> LaserScan
    # -----------------------------------------------------

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",

        remappings=[
            ("cloud_in", "/points"),
            ("scan", "/scan"),
        ],

        parameters=[
            pointcloud_to_scan_config,
            {
                "use_sim_time": True,
            },
        ],
    )

        # -----------------------------------------------------
    # SLAM TOOLBOX
    # -----------------------------------------------------

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(slam_enabled),
        parameters=[
            slam_config,
            {
                "use_sim_time": True,
            },
        ],
    )


    # -----------------------------------------------------
    # RVIZ
    # -----------------------------------------------------

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="evrenos_rviz",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz_enabled),
    )
    
    moving_obstacle_node = Node(
        package="yektemak_gazebo",
        executable="evrenos_moving_obstacle.py",
        name="evrenos_sliding_obstacle",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
    )   
    
    # -----------------------------------------------------
    # Launch
    # -----------------------------------------------------

    return LaunchDescription(
        [
            slam_arg,
            gazebo,
            robot_state_publisher,
            spawn_evrenos,
            ekf_node,
            pointcloud_to_laserscan,
            slam_node,
            rviz_node,
            rviz_arg,
            #moving_obstacle_node,
        ]
    )