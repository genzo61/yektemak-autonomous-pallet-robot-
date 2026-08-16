#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import LaunchConfiguration,PathJoinSubstitution


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
        default_value='/unilidar/cloud',
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

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='BNO055 IMU topic'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Real vehicle uses system time'
    )

    start_lidar_driver_arg = DeclareLaunchArgument(
        'start_lidar_driver',
        default_value='false',
        description='Start Unitree L1 UDP driver'
    )

    lidar_ip_arg = DeclareLaunchArgument(
        'lidar_ip',
        default_value='192.168.1.62'
    )

    local_ip_arg = DeclareLaunchArgument(
        'local_ip',
        default_value='192.168.1.2'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='6101'
    )

    local_port_arg = DeclareLaunchArgument(
        'local_port',
        default_value='6201'
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
    imu_topic = LaunchConfiguration('imu_topic')

    start_lidar_driver = LaunchConfiguration('start_lidar_driver')

    lidar_ip = LaunchConfiguration('lidar_ip')
    local_ip = LaunchConfiguration('local_ip')

    lidar_port = LaunchConfiguration('lidar_port')
    local_port = LaunchConfiguration('local_port')

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
            'start_lidar_driver': start_lidar_driver,
            'imu_topic': imu_topic,

            'cloud_topic': cloud_topic,
            'scan_topic': scan_topic,

            'lidar_ip': lidar_ip,
            'local_ip': local_ip,

            'lidar_port': lidar_port,
            'local_port': local_port,

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

            imu_topic_arg,
            use_sim_time_arg,

            start_lidar_driver_arg,
            lidar_ip_arg,
            local_ip_arg,
            lidar_port_arg,
            local_port_arg,

            real_base,
            real_sensors,
            slam_toolbox,
        ]
    )
