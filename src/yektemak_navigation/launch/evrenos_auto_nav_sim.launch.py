import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory('yektemak_navigation')

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Restart the waypoint mission after reaching the finish',
    )

    nav2_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_share,
                'launch',
                'evrenos_nav2_sim.launch.py',
            )
        )
    )

    auto_mission = Node(
        package='yektemak_navigation',
        executable='evrenos_auto_mission.py',
        name='evrenos_auto_mission',
        output='screen',
        parameters=[
            os.path.join(
                navigation_share,
                'config',
                'evrenos_auto_mission.yaml',
            ),
            {'loop': LaunchConfiguration('loop')},
        ],
    )

    return LaunchDescription([
        loop_arg,
        nav2_sim,
        auto_mission,
    ])
