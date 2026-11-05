"""LIMAN DEMOSU: acik liman sahasi + 3 robot (4 kose LiDAR, RTK-GNSS) + filo + panel.

  killall -9 gzserver gzclient          # once eski Gazebo kalmasin
  ros2 launch yektemak_bringup port_demo.launch.py
  Tarayicida: http://localhost:8080

  auto:=true    otomatik akis acilista acik (giris -> saha -> konteyner -> rihtim)
  gui:=false    Gazebo penceresi kapali (cok daha hafif)
  robots:=2     robot sayisi (1-3), bilgisayar zorlanirsa
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# ad, renk, sarj istasyonu (port_stations.yaml ile ayni)
ROBOTS = [("robot1", "Blue", 0.0, 10.0), ("robot2", "Orange", 0.0, 13.0), ("robot3", "Green", 0.0, 16.0)]


def include(pkg, name, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(pkg), "launch", name])),
        launch_arguments=args.items())


def setup(context):
    n = max(1, min(3, int(LaunchConfiguration("robots").perform(context))))
    gui = LaunchConfiguration("gui").perform(context)
    auto = LaunchConfiguration("auto").perform(context)
    nav = get_package_share_directory("yektemak_navigation")
    world = os.path.join(get_package_share_directory("yektemak_gazebo"), "worlds", "port.world")
    stations = os.path.join(get_package_share_directory("yektemak_fleet"), "config", "port_stations.yaml")
    actions = [include("yektemak_gazebo", "factory.launch.py", {"gui": gui, "world": world})]
    for i, (ns, color, x, y) in enumerate(ROBOTS[:n]):
        pose = {"x": str(x), "y": str(y), "yaw": "0.0"}
        actions.append(TimerAction(period=8.0 + 12.0 * i, actions=[include(
            "yektemak_gazebo", "spawn_robot.launch.py",
            {"ns": ns, "color": color, "lidar_layout": "corners", **pose})]))
        actions.append(TimerAction(period=45.0 + 5.0 * i, actions=[include(
            "yektemak_navigation", "robot_navigation.launch.py",
            {"ns": ns, **pose, "localization": "gnss", "lidar_layout": "corners",
             "map": os.path.join(nav, "maps", "port.yaml"), "stations_file": stations})]))
    actions.append(TimerAction(period=70.0, actions=[include(
        "yektemak_fleet", "fleet.launch.py", {"scenario": "port", "auto": auto, "robots": str(n)})]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("auto", default_value="false"),
        DeclareLaunchArgument("robots", default_value="3"),
        OpaqueFunction(function=setup),
    ])
