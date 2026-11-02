"""Filo yonetimi: filo yoneticisi + her robot icin ajan + rosbridge + operator paneli.

Simulasyon (fleet_sim.launch.py) calisiyor olmali. Hepsi bir arada icin:
  ros2 launch yektemak_bringup fleet_demo.launch.py
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory("yektemak_fleet")
    stations_file = os.path.join(share, "config", "stations.yaml")
    stations = yaml.safe_load(open(stations_file))["stations"]
    fleet_cfg = os.path.join(share, "config", "fleet.yaml")
    params = yaml.safe_load(open(fleet_cfg))["fleet_manager"]["ros__parameters"]
    auto = LaunchConfiguration("auto").perform(context).lower() == "true"

    nodes = [Node(package="yektemak_fleet", executable="fleet_manager", name="fleet_manager",
                  output="screen",
                  parameters=[fleet_cfg, {"stations_file": stations_file, "auto_start": auto}])]
    for robot, home in zip(params["robots"], params["homes"]):
        h = stations[home]
        nodes.append(Node(
            package="yektemak_tasks", executable="robot_agent", name="robot_agent",
            namespace=robot, output="screen",
            parameters=[os.path.join(share, "config", "agent.yaml"),
                        {"robot_name": robot, "home": [h["x"], h["y"], h["yaw"]]}]))
    nodes.append(Node(package="rosbridge_server", executable="rosbridge_websocket",
                      name="rosbridge_websocket", output="screen",
                      parameters=[{"port": 9090, "use_sim_time": True}]))
    nodes.append(Node(package="yektemak_fleet", executable="dashboard_server",
                      name="dashboard_server", output="screen", parameters=[{"port": 8080}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("auto", default_value="false",
                              description="Otomatik uretim modu acilista acik mi"),
        OpaqueFunction(function=setup),
    ])
