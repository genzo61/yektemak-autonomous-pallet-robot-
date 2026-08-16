from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    lidar_ip = LaunchConfiguration('lidar_ip')
    local_ip = LaunchConfiguration('local_ip')

    lidar_port = LaunchConfiguration('lidar_port')
    local_port = LaunchConfiguration('local_port')

    cloud_topic = LaunchConfiguration('cloud_topic')
    cloud_frame = LaunchConfiguration('cloud_frame')

    imu_topic = LaunchConfiguration('imu_topic')
    imu_frame = LaunchConfiguration('imu_frame')

    cloud_scan_num = LaunchConfiguration('cloud_scan_num')

    rotate_yaw_bias = LaunchConfiguration('rotate_yaw_bias')

    range_scale = LaunchConfiguration('range_scale')
    range_bias = LaunchConfiguration('range_bias')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')


    unitree_node = Node(
        package='yektemak_unitree_lidar',
        executable='evrenos_unitree_l1_udp_node',
        name='evrenos_unitree_l1_udp',
        output='screen',

        parameters=[{
            'lidar_ip': lidar_ip,
            'local_ip': local_ip,

            'lidar_port': lidar_port,
            'local_port': local_port,

            'cloud_topic': cloud_topic,
            'cloud_frame': cloud_frame,

            'imu_topic': imu_topic,
            'imu_frame': imu_frame,

            'cloud_scan_num': cloud_scan_num,

            'rotate_yaw_bias': rotate_yaw_bias,

            'range_scale': range_scale,
            'range_bias': range_bias,
            'range_min': range_min,
            'range_max': range_max,
        }]
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            'lidar_ip',
            default_value='192.168.1.62',
            description='Unitree L1 IP address'
        ),

        DeclareLaunchArgument(
            'local_ip',
            default_value='192.168.1.2',
            description='EVRENOS navigation computer Ethernet IP'
        ),

        DeclareLaunchArgument(
            'lidar_port',
            default_value='6101',
            description='Unitree L1 UDP port'
        ),

        DeclareLaunchArgument(
            'local_port',
            default_value='6201',
            description='Local UDP receive port'
        ),

        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/unilidar/cloud',
            description='Unitree L1 PointCloud2 output topic'
        ),

        DeclareLaunchArgument(
            'cloud_frame',
            default_value='lidar_link',
            description='PointCloud2 frame'
        ),

        DeclareLaunchArgument(
            'imu_topic',
            default_value='/unilidar/imu',
            description='Unitree L1 internal IMU topic'
        ),

        DeclareLaunchArgument(
            'imu_frame',
            default_value='unilidar_imu',
            description='Unitree L1 internal IMU frame'
        ),

        DeclareLaunchArgument(
            'cloud_scan_num',
            default_value='18',
            description='Number of L1 scans accumulated per cloud'
        ),

        DeclareLaunchArgument(
            'rotate_yaw_bias',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'range_scale',
            default_value='0.001'
        ),

        DeclareLaunchArgument(
            'range_bias',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'range_min',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'range_max',
            default_value='50.0'
        ),

        unitree_node,
    ])