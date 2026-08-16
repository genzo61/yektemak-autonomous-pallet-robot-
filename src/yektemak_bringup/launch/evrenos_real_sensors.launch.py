from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from launch.conditions import IfCondition

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # Launch arguments
    # ============================================================

    use_sim_time = LaunchConfiguration('use_sim_time')

    start_lidar_driver = LaunchConfiguration(
        'start_lidar_driver'
    )

    cloud_topic = LaunchConfiguration(
        'cloud_topic'
    )

    scan_topic = LaunchConfiguration(
        'scan_topic'
    )

    lidar_ip = LaunchConfiguration(
        'lidar_ip'
    )

    local_ip = LaunchConfiguration(
        'local_ip'
    )

    lidar_port = LaunchConfiguration(
        'lidar_port'
    )

    local_port = LaunchConfiguration(
        'local_port'
    )


    # ============================================================
    # Unitree L1 UDP driver
    #
    # Disabled by default so development/mock tests can run
    # without physical LiDAR hardware.
    # ============================================================

    unitree_l1_driver = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            PathJoinSubstitution([
                FindPackageShare(
                    'yektemak_unitree_lidar'
                ),

                'launch',

                'evrenos_unitree_l1_udp.launch.py',
            ])
        ),

        condition=IfCondition(
            start_lidar_driver
        ),

        launch_arguments={

            'lidar_ip':
                lidar_ip,

            'local_ip':
                local_ip,

            'lidar_port':
                lidar_port,

            'local_port':
                local_port,

            'cloud_topic':
                cloud_topic,

            'cloud_frame':
                'lidar_link',

        }.items(),
    )


    # ============================================================
    # PointCloud2 -> LaserScan
    # ============================================================

    pointcloud_to_scan_config = PathJoinSubstitution([

        FindPackageShare(
            'yektemak_bringup'
        ),

        'config',

        'evrenos_pointcloud_to_scan_real.yaml',
    ])


    pointcloud_to_scan = Node(

        package='pointcloud_to_laserscan',

        executable='pointcloud_to_laserscan_node',

        name='pointcloud_to_laserscan',

        output='screen',

        parameters=[

            pointcloud_to_scan_config,

            {
                'use_sim_time':
                    use_sim_time,
            },

        ],

        remappings=[

            (
                'cloud_in',
                cloud_topic
            ),

            (
                'scan',
                scan_topic
            ),

        ],
    )


    # ============================================================
    # Launch description
    # ============================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            'start_lidar_driver',

            default_value='false',

            description=(
                'Start EVRENOS Unitree L1 UDP driver. '
                'Keep false during laptop/mock testing.'
            )
        ),

        DeclareLaunchArgument(
            'cloud_topic',

            default_value='/unilidar/cloud',

            description=(
                'Unitree L1 PointCloud2 topic'
            )
        ),

        DeclareLaunchArgument(
            'scan_topic',

            default_value='/scan',

            description=(
                'LaserScan output topic'
            )
        ),

        DeclareLaunchArgument(
            'lidar_ip',

            default_value='192.168.1.62',

            description=(
                'Unitree L1 IP address'
            )
        ),

        DeclareLaunchArgument(
            'local_ip',

            default_value='192.168.1.2',

            description=(
                'Navigation computer Ethernet IP'
            )
        ),

        DeclareLaunchArgument(
            'lidar_port',

            default_value='6101',

            description=(
                'Unitree L1 UDP port'
            )
        ),

        DeclareLaunchArgument(
            'local_port',

            default_value='6201',

            description=(
                'Navigation computer UDP receive port'
            )
        ),

        DeclareLaunchArgument(
            'use_sim_time',

            default_value='false',

            description=(
                'Real sensors use system time'
            )
        ),

        unitree_l1_driver,

        pointcloud_to_scan,
    ])