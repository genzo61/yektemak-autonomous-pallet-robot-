#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from launch.conditions import IfCondition

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # Package paths
    # ============================================================

    bringup_share = get_package_share_directory(
        'yektemak_bringup'
    )

    navigation_share = get_package_share_directory(
        'yektemak_navigation'
    )

    nav2_share = get_package_share_directory(
        'nav2_bringup'
    )


    # ============================================================
    # Default files
    #
    # Existing competition map is only a prototype default.
    # On the real vehicle, replace it with the map produced by
    # evrenos_real_mapping.launch.py.
    # ============================================================

    default_map = os.path.join(
        navigation_share,
        'maps',
        'evrenos_competition_map.yaml'
    )

    real_nav2_params = os.path.join(
        bringup_share,
        'config',
        'evrenos_nav2_real.yaml'
    )

    nav2_rviz_config = os.path.join(
        navigation_share,
        'rviz',
        'evrenos_nav2.rviz'
    )


    # ============================================================
    # Launch arguments
    # ============================================================

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='STM32 USB CDC serial port'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='STM32 serial baudrate'
    )

    track_width_arg = DeclareLaunchArgument(
        'track_width',
        default_value='1.065',
        description=(
            'Prototype effective skid-steer track width. '
            'Measure/tune on real EVRENOS.'
        )
    )

    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/points',
        description=(
            'Real Unitree PointCloud2 topic. '
            'Change to actual driver topic on Jetson.'
        )
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScan topic used by AMCL/Nav2'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to saved map YAML'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Real vehicle uses system time'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2'
    )


    # ============================================================
    # Configurations
    # ============================================================

    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    track_width = LaunchConfiguration('track_width')

    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    map_file = LaunchConfiguration('map')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')


    # ============================================================
    # REAL BASE
    #
    # STM32
    #   -> /wheel/odom
    #
    # EKF
    #   -> /odometry/filtered
    #   -> odom -> base_footprint
    #
    # robot_state_publisher
    #   -> base_footprint -> sensor frames
    # ============================================================

    real_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                'launch',
                'evrenos_real_base.launch.py'
            )
        ),

        launch_arguments={
            'serial_port': serial_port,
            'baudrate': baudrate,
            'track_width': track_width,
            'use_sim_time': use_sim_time,
        }.items()
    )


    # ============================================================
    # REAL LIDAR PIPELINE
    #
    # Unitree PointCloud2
    #        |
    #        v
    # pointcloud_to_laserscan
    #        |
    #        v
    #       /scan
    # ============================================================

    real_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_share,
                'launch',
                'evrenos_real_sensors.launch.py'
            )
        ),

        launch_arguments={
            'cloud_topic': cloud_topic,
            'scan_topic': scan_topic,
            'use_sim_time': use_sim_time,
        }.items()
    )


    # ============================================================
    # NAV2
    #
    # SLAM DISABLED.
    #
    # map_server + AMCL:
    #       map -> odom
    #
    # EKF:
    #       odom -> base_footprint
    #
    # Nav2:
    #       goal -> /cmd_vel
    #
    # STM32 bridge:
    #       /cmd_vel -> real vehicle
    # ============================================================

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_share,
                'launch',
                'bringup_launch.py'
            )
        ),

        launch_arguments={
            'slam': 'False',
            'map': map_file,

            'use_sim_time': use_sim_time,

            'params_file': real_nav2_params,

            'autostart': 'True',

            # Keep composition disabled during integration.
            # Easier to inspect individual ROS2 nodes.
            'use_composition': 'False',
        }.items()
    )


    # ============================================================
    # RViz
    # ============================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='evrenos_real_nav2_rviz',

        output='screen',

        arguments=[
            '-d',
            nav2_rviz_config
        ],

        parameters=[
            {
                'use_sim_time': use_sim_time
            }
        ],

        condition=IfCondition(use_rviz)
    )


    # ============================================================

    return LaunchDescription(
        [
            serial_port_arg,
            baudrate_arg,
            track_width_arg,

            cloud_topic_arg,
            scan_topic_arg,

            map_arg,

            use_sim_time_arg,
            use_rviz_arg,

            real_base,
            real_sensors,

            nav2,

            rviz,
        ]
    )