"""ADIM 3 - Tam otonom demo: navigasyon + gorev yoneticisi

  ros2 launch yektemak_bringup sim_mission.launch.py

Robot sirayla pick_1 -> drop_1 ve pick_2 -> drop_2 gorevlerini yapar,
sonra park alanina doner. Durumu izlemek icin:
  ros2 topic echo /mission/status
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        include("yektemak_bringup", "sim_navigation.launch.py", {"gui": gui, "rviz": rviz}),
        TimerAction(period=15.0, actions=[include("yektemak_tasks", "mission.launch.py")]),
    ])
