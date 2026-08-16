#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import (
    Command,
    LaunchConfiguration,
)

from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ============================================================
    # Package paths
    # ============================================================

    bringup_share = get_package_share_directory(
        'yektemak_bringup'
    )

    description_share = get_package_share_directory(
        'yektemak_description'
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
            'Effective skid-steer track width in meters. '
            'Prototype value; measure/tune on real EVRENOS.'
        )
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description=(
            'BNO055 sensor_msgs/Imu topic. '
            'Replace with actual driver topic on the real vehicle.'
        )
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )


    # ============================================================
    # Launch configurations
    # ============================================================

    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    track_width = LaunchConfiguration('track_width')
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic = LaunchConfiguration('imu_topic')


    # ============================================================
    # Files
    # ============================================================

    robot_xacro = os.path.join(
        description_share,
        'urdf',
        'evrenos_robot.urdf.xacro'
    )

    ekf_config = os.path.join(
        bringup_share,
        'config',
        'evrenos_ekf_real.yaml'
    )


    # ============================================================
    # Robot description
    # ============================================================

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                robot_xacro,
            ]
        ),
        value_type=str
    )


    # ============================================================
    # Robot State Publisher
    #
    # Publishes:
    #
    # base_footprint
    #   └── base_link
    #       ├── lidar_link
    #       ├── imu_link
    #       ├── gps_link
    #       └── camera_link
    #
    # Dynamic odom -> base_footprint will NOT come from here.
    # EKF owns that transform.
    # ============================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',

        output='screen',

        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }
        ]
    )


    # ============================================================
    # STM32 Bridge
    #
    # /cmd_vel
    #    ↓
    # USB CDC
    #    ↓
    # STM32
    #
    # STM32 encoder feedback
    #    ↓
    # /wheel/odom
    # ============================================================

    stm32_bridge = Node(
        package='yektemak_bringup',
        executable='evrenos_stm32_bridge.py',
        name='evrenos_stm32_bridge',

        output='screen',

        parameters=[
            {
                'serial_port': serial_port,

                'baudrate': ParameterValue(
                    baudrate,
                    value_type=int
                ),

                'track_width': ParameterValue(
                    track_width,
                    value_type=float
                ),

                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/wheel/odom',

                'odom_frame': 'odom',
                'base_frame': 'base_footprint',

                'publish_rate': 20.0,
                'cmd_vel_timeout': 0.5,
                'reconnect_interval': 2.0,

                'right_rear_sign': 1.0,
                'right_front_sign': 1.0,
                'left_rear_sign': 1.0,
                'left_front_sign': 1.0,
            }
        ]
    )


    # ============================================================
    # Robot Localization EKF
    #
    # /wheel/odom
    #      ↓
    # EKF
    #      ↓
    # /odometry/filtered
    #
    # EKF owns:
    #
    # odom -> base_footprint
    #
    # Later BNO055 /imu/data will also be fused here.
    # ============================================================

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',

        output='screen',

        parameters=[
            ekf_config,
            {
                'use_sim_time': use_sim_time,
            }
        ],

        remappings=[
            (
                '/imu/data',
                imu_topic
            ),
        ]
    )


    # ============================================================
    # Launch
    # ============================================================

    return LaunchDescription(
        [
            serial_port_arg,
            baudrate_arg,
            track_width_arg,
            use_sim_time_arg,

            robot_state_publisher,
            stm32_bridge,
            ekf_node,
            imu_topic_arg,
        ]
    )