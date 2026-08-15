import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    gazebo_share = get_package_share_directory(
        "yektemak_gazebo"
    )

    navigation_share = get_package_share_directory(
        "yektemak_navigation"
    )

    nav2_share = get_package_share_directory(
        "nav2_bringup"
    )


    # -----------------------------------------------------
    # Files
    # -----------------------------------------------------

    map_file = os.path.join(
        navigation_share,
        "maps",
        "evrenos_competition_map.yaml",
    )

    nav2_rviz_config = os.path.join(
        navigation_share,
        "rviz",
        "evrenos_nav2.rviz",
    )

    params_file = os.path.join(
        navigation_share,
        "config",
        "evrenos_nav2.yaml",
    )


    # -----------------------------------------------------
    # Gazebo + robot + sensors + EKF
    # SLAM disabled because AMCL owns map -> odom
    # -----------------------------------------------------

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share,
                "launch",
                "evrenos_gazebo.launch.py",
            )
        ),

        launch_arguments={
            "slam": "false",
            "rviz": "false",
        }.items(),
    )


    # -----------------------------------------------------
    # Nav2
    # -----------------------------------------------------

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_share,
                "launch",
                "bringup_launch.py",
            )
        ),

        launch_arguments={
            "slam": "False",
            "map": map_file,
            "use_sim_time": "True",
            "params_file": params_file,
            "autostart": "True",

            # Humble'da ilk test için composition kapalı:
            # hata ayıklaması daha kolay.
            "use_composition": "False",
        }.items(),
    )

    nav2_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="evrenos_nav2_rviz",
        output="screen",
        arguments=["-d", nav2_rviz_config],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            simulation,
            nav2,
            nav2_rviz,
        ]
    )

