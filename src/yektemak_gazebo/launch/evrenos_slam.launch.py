import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    gazebo_share = get_package_share_directory("yektemak_gazebo")

    slam_config = os.path.join(
        gazebo_share,
        "config",
        "evrenos_slam.yaml",
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {
                "use_sim_time": True,
            },
        ],
    )

    return LaunchDescription(
        [
            slam_toolbox,
        ]
    )

