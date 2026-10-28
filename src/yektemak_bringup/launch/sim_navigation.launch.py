"""ADIM 2 - Navigasyon: Gazebo + LiDAR filtresi + AMCL + Nav2 + RViz

  ros2 launch yektemak_bringup sim_navigation.launch.py
  RViz'de "Nav2 Goal" ile hedef verin.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
        # Gazebo ve controller'lar ayaga kalksin, sonra Nav2
        TimerAction(period=8.0, actions=[include("yektemak_navigation", "navigation.launch.py")]),
        Node(
            condition=IfCondition(rviz),
            package="rviz2", executable="rviz2", output="screen",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("yektemak_navigation"), "rviz", "yektemak_nav.rviz"])],
            parameters=[{"use_sim_time": True}],
        ),
    ])
