"""TAM DEMO: fabrika + 2 robot + Nav2 + filo yoneticisi + operator paneli.

  ros2 launch yektemak_bringup fleet_demo.launch.py
  Tarayicida: http://localhost:8080

  auto:=true    otomatik uretim acilista acik
  gui:=false    Gazebo penceresi kapali (daha hafif)
  rviz:=false   RViz kapali (panel zaten haritayi gosteriyor)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(pkg, name, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(pkg), "launch", name])),
        launch_arguments=args.items())


def generate_launch_description():
    gui, rviz, auto = (LaunchConfiguration(k) for k in ("gui", "rviz", "auto"))
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("auto", default_value="false"),
        include("yektemak_bringup", "fleet_sim.launch.py", {"gui": gui, "rviz": rviz}),
        # Nav2 yiginlari ayaga kalksin, sonra filo (ajanlar Nav2'yi kendileri de bekler)
        TimerAction(period=50.0, actions=[include("yektemak_fleet", "fleet.launch.py", {"auto": auto})]),
    ])
