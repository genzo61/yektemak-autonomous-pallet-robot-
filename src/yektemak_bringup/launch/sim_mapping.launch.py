"""ADIM 1 - Haritalama: Gazebo + LiDAR filtresi + slam_toolbox + RViz

  ros2 launch yektemak_bringup sim_mapping.launch.py
  (baska terminal) ros2 run teleop_twist_keyboard teleop_twist_keyboard
  (bitince)        ros2 run nav2_map_server map_saver_cli -f warehouse
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(pkg, name, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), "launch", name])),
        launch_arguments=(args or {}).items(),
    )


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        include("yektemak_gazebo", "sim.launch.py", {"gui": gui}),
        include("yektemak_navigation", "scan_filter.launch.py"),
        include("yektemak_navigation", "slam.launch.py"),
        Node(
            condition=IfCondition(rviz),
            package="rviz2", executable="rviz2", output="screen",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("yektemak_navigation"), "rviz", "yektemak_nav.rviz"])],
            parameters=[{"use_sim_time": True}],
        ),
    ])
