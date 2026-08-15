from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")

    package_share = FindPackageShare("yektemak_description")

    xacro_file = PathJoinSubstitution(
        [
            package_share,
            "urdf",
            "evrenos_robot.urdf.xacro",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            package_share,
            "rviz",
            "yektemak_robot.rviz",
        ]
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Joint State Publisher GUI kullanılsın mı?",
            ),

            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                    }
                ],
            ),

            Node(
                condition=IfCondition(use_gui),
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),

            Node(
                condition=UnlessCondition(use_gui),
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
            ),
        ]
    )

