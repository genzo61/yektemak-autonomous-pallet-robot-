"""slam_toolbox ile harita cikarma (online async).

Harita kaydetmek icin (ayri terminalde):
  ros2 run nav2_map_server map_saver_cli -f ~/yektemak_ws/src/yektemak_navigation/maps/warehouse
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration("slam_params_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("yektemak_navigation"), "config", "slam_mapping.yaml"]),
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[params, {"use_sim_time": True}],
        ),
    ])
