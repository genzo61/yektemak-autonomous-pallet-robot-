"""FILO: fabrika + 2 robot + her robot icin Nav2.

  ros2 launch yektemak_bringup fleet_sim.launch.py
  ros2 launch yektemak_bringup fleet_sim.launch.py gui:=false rviz:=false

Robot sayisini/konumlarini degistirmek icin asagidaki ROBOTS listesini duzenleyin.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ad, renk, sarj istasyonu konumu (x, y, yaw)
ROBOTS = [
    ("robot1", "Blue", 0.5, 1.5, 0.0),
    ("robot2", "Orange", 0.5, -1.5, 0.0),
]


def include(pkg, name, args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(pkg), "launch", name])),
        launch_arguments=args.items())


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    actions = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        include("yektemak_gazebo", "factory.launch.py", {"gui": gui}),
    ]
    for i, (ns, color, x, y, yaw) in enumerate(ROBOTS):
        pose = {"x": str(x), "y": str(y), "yaw": str(yaw)}
        # Robotlari sirayla ekle (Gazebo ayni anda iki spawn'da zorlanabiliyor)
        actions.append(TimerAction(period=6.0 + 12.0 * i, actions=[
            include("yektemak_gazebo", "spawn_robot.launch.py", {"ns": ns, "color": color, **pose})]))
        actions.append(TimerAction(period=40.0 + 5.0 * i, actions=[
            include("yektemak_navigation", "robot_navigation.launch.py", {"ns": ns, **pose})]))
    actions.append(Node(
        condition=IfCondition(rviz), package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("yektemak_navigation"), "rviz",
                                               "yektemak_fleet.rviz"])],
        parameters=[{"use_sim_time": True}]))
    return LaunchDescription(actions)
