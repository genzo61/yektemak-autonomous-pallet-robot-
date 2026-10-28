"""Kayitli harita + AMCL + Nav2 (nav2_bringup uzerinden)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = FindPackageShare("yektemak_navigation")
    # Not - arguman adlari benzersiz; gzserver.launch.py de "params_file" tanimliyor
    # ve launch argumanlari include'lar arasinda paylasiliyor.
    map_file = LaunchConfiguration("map")
    params = LaunchConfiguration("nav2_params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value=PathJoinSubstitution([nav_share, "maps", "warehouse.yaml"])),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution([nav_share, "config", "nav2_params.yaml"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])),
            launch_arguments={
                "map": map_file,
                "params_file": params,
                "use_sim_time": "true",
                "autostart": "true",
                "use_composition": "False",
            }.items(),
        ),
    ])
