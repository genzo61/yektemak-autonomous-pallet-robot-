"""/scan_raw -> /scan (on) ve /scan_rear_raw -> /scan_rear (arka)"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain",
            output="screen",
            parameters=[PathJoinSubstitution(
                [FindPackageShare("yektemak_navigation"), "config", "laser_filter.yaml"])],
            remappings=[("scan", "scan_raw"), ("scan_filtered", "scan")],
        ),
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="rear_scan_filter",
            output="screen",
            parameters=[PathJoinSubstitution(
                [FindPackageShare("yektemak_navigation"), "config", "laser_filter.yaml"])],
            remappings=[("scan", "scan_rear_raw"), ("scan_filtered", "scan_rear")],
        ),
    ])
