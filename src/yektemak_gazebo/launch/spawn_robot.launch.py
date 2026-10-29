"""Tek bir robotu namespace ile Gazebo'ya ekler (Gazebo zaten calisiyor olmali).

  ros2 launch yektemak_gazebo spawn_robot.launch.py ns:=robot1 x:=0.5 y:=1.5 color:=Blue

Frame'ler ns/ oneki alir (robot1/odom, robot1/base_footprint ...) ve hepsi ortak
/tf agacinda yayinlanir; boylece tek RViz / tek panel iki robotu birden gorur.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution


def setup(context):
    ns = LaunchConfiguration("ns").perform(context)
    color = LaunchConfiguration("color").perform(context)
    x, y, yaw = (LaunchConfiguration(k).perform(context) for k in ("x", "y", "yaw"))

    description = ParameterValue(
        Command(["xacro ",
                 PathJoinSubstitution([FindPackageShare("yektemak_description"), "urdf",
                                       "yektemak_robot.urdf.xacro"]),
                 f" use_sim:=true ns:={ns} body_color:={color}"]),
        value_type=str)

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace=ns, output="screen",
        parameters=[{"robot_description": description, "use_sim_time": True,
                     "frame_prefix": f"{ns}/"}],
    )
    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", output="screen",
        arguments=["-topic", f"/{ns}/robot_description", "-entity", ns,
                   "-x", x, "-y", y, "-z", "0.01", "-Y", yaw],
    )

    def spawner(name):
        return Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=[name, "--controller-manager", f"/{ns}/controller_manager",
                               "--controller-manager-timeout", "90"])

    controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[spawner("joint_state_broadcaster"), spawner("lift_controller")]))
    return [rsp, spawn, controllers]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="robot1"),
        DeclareLaunchArgument("color", default_value="Blue"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=setup),
    ])
