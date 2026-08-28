#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

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
        default_value='1.11',
        description=(
            'Effective skid-steer track width in meters. '
            'Synced with firmware EKSEN_ACIKLIGI_METRE; '
            're-verify with a real 360-degree turn test.'
        )
    )

    motion_enabled_arg = DeclareLaunchArgument(
        'motion_enabled',
        default_value='false',
        description='Allow non-zero velocity commands to reach STM32'
    )

    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.10',
        description='Maximum commanded linear speed in m/s'
    )

    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='0.20',
        description='Maximum commanded angular speed in rad/s'
    )

    watchdog_timeout_arg = DeclareLaunchArgument(
        'watchdog_timeout',
        default_value='0.5',
        description='cmd_vel watchdog timeout in seconds'
    )

    startup_zero_duration_arg = DeclareLaunchArgument(
        'startup_zero_duration',
        default_value='1.0',
        description='Zero-command interlock after connect/reconnect in seconds'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description=(
            'Filtered BNO055 sensor_msgs/Imu topic consumed by the EKF.'
        )
    )

    start_bno055_driver_arg = DeclareLaunchArgument(
        'start_bno055_driver',
        default_value='false',
        description=(
            'Start the BNO055 driver. Keep false until the device '
            'connection and mounting orientation are verified.'
        )
    )

    bno055_config_arg = DeclareLaunchArgument(
        'bno055_config',
        default_value=os.path.join(
            bringup_share,
            'config',
            'evrenos_bno055_real.yaml'
        ),
        description='BNO055 transport, covariance, and axis-remap config'
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
    motion_enabled = LaunchConfiguration('motion_enabled')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    watchdog_timeout = LaunchConfiguration('watchdog_timeout')
    startup_zero_duration = LaunchConfiguration('startup_zero_duration')
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic = LaunchConfiguration('imu_topic')
    start_bno055_driver = LaunchConfiguration('start_bno055_driver')
    bno055_config = LaunchConfiguration('bno055_config')


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
    # BNO055 IMU
    #
    # Driver-native filtered output: bno055/imu
    # EVRENOS public interface:     /imu/data, frame_id=imu_link
    # ============================================================

    bno055_driver = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        condition=IfCondition(start_bno055_driver),
        parameters=[
            bno055_config,
            {
                'ros_topic_prefix': 'bno055/',
                'frame_id': 'imu_link',
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            (
                'bno055/imu',
                imu_topic
            ),
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

                'motion_enabled': ParameterValue(
                    motion_enabled,
                    value_type=bool
                ),

                'max_linear_speed': ParameterValue(
                    max_linear_speed,
                    value_type=float
                ),

                'max_angular_speed': ParameterValue(
                    max_angular_speed,
                    value_type=float
                ),

                'watchdog_timeout': ParameterValue(
                    watchdog_timeout,
                    value_type=float
                ),

                'startup_zero_duration': ParameterValue(
                    startup_zero_duration,
                    value_type=float
                ),

                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/wheel/odom',

                'odom_frame': 'odom',
                'base_frame': 'base_footprint',

                'publish_rate': 20.0,
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
    # Optional BNO055 /imu/data is fused here when available.
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
            motion_enabled_arg,
            max_linear_speed_arg,
            max_angular_speed_arg,
            watchdog_timeout_arg,
            startup_zero_duration_arg,
            use_sim_time_arg,
            imu_topic_arg,
            start_bno055_driver_arg,
            bno055_config_arg,

            robot_state_publisher,
            bno055_driver,
            stm32_bridge,
            ekf_node,
        ]
    )
