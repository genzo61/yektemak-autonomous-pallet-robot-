#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ============================================================
    # Package paths
    # ============================================================

    bringup_share = get_package_share_directory(
        'yektemak_bringup'
    )

    slam_toolbox_share = get_package_share_directory(
        'slam_toolbox'
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
            'Prototype EVRENOS effective track width. '
            'Measure/tune on real vehicle.'
        )
    )

    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/points',
        description=(
            'Real Unitree PointCloud2 topic. '
            'Replace with actual driver topic on Jetson.'
        )
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScan topic used by SLAM'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Real vehicle uses system time'
    )


    # ============================================================
    # Configurations
    # ============================================================

    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    track_width = LaunchConfiguration('track_width')

    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    use_sim_time = LaunchConfiguration('use_sim_time')


    # ============================================================
    # REAL BASE
    #
    # robot_state_publisher
    # STM32 bridge
    # wheel odometry
    # EKF
    #
    # Output:
    #   /odometry/filtered
    #   odom -> base_footprint
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
    # REAL SENSOR PIPELINE
    #
    # Unitree PointCloud2
    #        ↓
    # pointcloud_to_laserscan
    #        ↓
    # /scan
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
    # SLAM TOOLBOX
    #
    # /scan
    #    +
    # odom -> base_footprint
    #    ↓
    # SLAM Toolbox
    #    ↓
    # /map
    # map -> odom
    # ============================================================

    slam_config = os.path.join(
        bringup_share,
        'config',
        'evrenos_slam_real.yaml'
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_share,
                'launch',
                'online_async_launch.py'
            )
        ),

        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': use_sim_time,
        }.items()
    )


    # ============================================================

    return LaunchDescription(
        [
            serial_port_arg,
            baudrate_arg,
            track_width_arg,

            cloud_topic_arg,
            scan_topic_arg,

            use_sim_time_arg,

            real_base,
            real_sensors,
            slam_toolbox,
        ]
    )

