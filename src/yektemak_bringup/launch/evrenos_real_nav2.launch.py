#!/usr/bin/env python3

import os
from launch_ros.substitutions import FindPackageShare
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

from launch.substitutions import LaunchConfiguration,PathJoinSubstitution

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

    motion_enabled_arg = DeclareLaunchArgument(
        'motion_enabled', default_value='false'
    )
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed', default_value='0.10'
    )
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed', default_value='0.20'
    )
    watchdog_timeout_arg = DeclareLaunchArgument(
        'watchdog_timeout', default_value='0.5'
    )
    startup_zero_duration_arg = DeclareLaunchArgument(
        'startup_zero_duration', default_value='1.0'
    )

    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/unilidar/cloud',
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

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='Selected real IMU topic consumed by the EKF'
    )

    start_bno055_driver_arg = DeclareLaunchArgument(
        'start_bno055_driver',
        default_value='false',
        description='Start BNO055 driver'
    )

    bno055_config_arg = DeclareLaunchArgument(
        'bno055_config',
        default_value=os.path.join(
            bringup_share,
            'config',
            'evrenos_bno055_real.yaml'
        ),
        description='BNO055 real-vehicle configuration file'
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
    motion_enabled = LaunchConfiguration('motion_enabled')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    watchdog_timeout = LaunchConfiguration('watchdog_timeout')
    startup_zero_duration = LaunchConfiguration('startup_zero_duration')

    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    map_file = LaunchConfiguration('map')
    imu_topic = LaunchConfiguration('imu_topic')
    start_bno055_driver = LaunchConfiguration('start_bno055_driver')
    bno055_config = LaunchConfiguration('bno055_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')

    start_lidar_driver = LaunchConfiguration('start_lidar_driver')

    lidar_ip = LaunchConfiguration('lidar_ip')
    local_ip = LaunchConfiguration('local_ip')

    lidar_port = LaunchConfiguration('lidar_port')
    local_port = LaunchConfiguration('local_port')


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
            'motion_enabled': motion_enabled,
            'max_linear_speed': max_linear_speed,
            'max_angular_speed': max_angular_speed,
            'watchdog_timeout': watchdog_timeout,
            'startup_zero_duration': startup_zero_duration,
            'imu_topic': imu_topic,
            'start_bno055_driver': start_bno055_driver,
            'bno055_config': bno055_config,
            'start_lidar_driver': start_lidar_driver,

            'cloud_topic': cloud_topic,
            'scan_topic': scan_topic,
            'imu_topic': imu_topic,
            'imu_frame': 'unilidar_imu',
            'start_bno055_driver': start_bno055_driver,

            'lidar_ip': lidar_ip,
            'local_ip': local_ip,

            'lidar_port': lidar_port,
            'local_port': local_port,

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
        PathJoinSubstitution([
            FindPackageShare('yektemak_bringup'), # pyright: ignore[reportUndefinedVariable]
            'launch',
            'evrenos_real_sensors.launch.py',
        ])
    ),
        launch_arguments={
            'start_lidar_driver': start_lidar_driver,

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
            start_lidar_driver_arg,
            lidar_ip_arg,
            local_ip_arg,
            lidar_port_arg,
            local_port_arg,

            serial_port_arg,
            baudrate_arg,
            track_width_arg,
            motion_enabled_arg,
            max_linear_speed_arg,
            max_angular_speed_arg,
            watchdog_timeout_arg,
            startup_zero_duration_arg,

            cloud_topic_arg,
            scan_topic_arg,

            map_arg,
            imu_topic_arg,
            start_bno055_driver_arg,
            bno055_config_arg,
            use_sim_time_arg,
            use_rviz_arg,

            real_base,
            real_sensors,

            nav2,

            rviz,
        ]
    )
