"""Tek bir robot icin (namespace'li) LiDAR filtresi + AMCL + Nav2.

  ros2 launch yektemak_navigation robot_navigation.launch.py ns:=robot1 x:=0.5 y:=1.5

Neden nav2_bringup kullanmiyoruz?
  nav2_bringup her robot icin /tf'yi /robotX/tf'ye yeniden adlandiriyor; yani
  her robotun ayri bir TF agaci oluyor. Biz tum robotlari TEK /tf agacinda,
  onekli frame'lerle (robot1/base_footprint, robot2/base_footprint) tutuyoruz.
  Boylece tek RViz ve tek operator paneli butun filoyu ayni haritada gorur.

Konum bulma iki sekilde olabilir:
  localization:=amcl   Kapali alan (fabrika). LiDAR + kayitli harita + AMCL.
  localization:=gnss   Acik alan (liman). RTK-GNSS + IMU + odometri ->
                       navsat_transform_node + ekf_node (robot_localization).
LiDAR duzeni:
  lidar_layout:=front_rear   On + arka LiDAR, laser_filters ile filtrelenir.
  lidar_layout:=corners      4 kose LiDAR, scan_merger ile tek taramada birlesir.

Tek robot icin yazilmis config/nav2_params.yaml burada sablon olarak okunur,
frame ve topic adlari robotun onekiyle yeniden yazilir ve gecici bir dosyaya
kaydedilir.
"""

import copy
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FRAME_KEYS = ("frame", "frame_id", "base_frame", "robot_base_frame", "global_frame",
              "odom_frame_id", "base_frame_id", "box_frame")
ROBOT_FRAMES = ("odom", "base_footprint", "base_link")

NAV_NODES = ["controller_server", "smoother_server", "planner_server", "behavior_server",
             "bt_navigator", "waypoint_follower", "velocity_smoother"]


def prefix_frames(obj, ns):
    """Robot frame'lerini ve /scan topic'ini ns ile onekler (ozyinelemeli)."""
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if isinstance(v, str) and (k in FRAME_KEYS or k.endswith("_frame")) and v in ROBOT_FRAMES:
                out[k] = f"{ns}/{v}"
            elif k == "topic" and v in ("/scan", "/scan_rear"):
                out[k] = f"/{ns}{v}"
            else:
                out[k] = prefix_frames(v, ns)
        return out
    if isinstance(obj, list):
        return [prefix_frames(v, ns) for v in obj]
    return obj


def namespace_keys(params, ns):
    """'amcl:' -> '/robot1/amcl:', 'local_costmap: local_costmap:' -> '/robot1/local_costmap/local_costmap:'"""
    out = {}
    for k, v in params.items():
        if isinstance(v, dict) and "ros__parameters" in v:
            out[f"/{ns}/{k}"] = v
        elif isinstance(v, dict):
            for k2, v2 in v.items():
                out[f"/{ns}/{k}/{k2}"] = v2
    return out


def write_tmp(data, name):
    fd, path = tempfile.mkstemp(prefix=f"{name}_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return path


def setup(context):
    ns = LaunchConfiguration("ns").perform(context)
    x = float(LaunchConfiguration("x").perform(context))
    y = float(LaunchConfiguration("y").perform(context))
    yaw = float(LaunchConfiguration("yaw").perform(context))
    map_yaml = LaunchConfiguration("map").perform(context)
    template = LaunchConfiguration("nav2_params_file").perform(context)
    localization = LaunchConfiguration("localization").perform(context)
    layout = LaunchConfiguration("lidar_layout").perform(context)
    stations_file = LaunchConfiguration("stations_file").perform(context)
    share = get_package_share_directory("yektemak_navigation")

    params = yaml.safe_load(open(template))
    params.pop("map_saver", None)
    params = prefix_frames(params, ns)
    p = params["amcl"]["ros__parameters"]
    p["initial_pose"] = {"x": x, "y": y, "z": 0.0, "yaw": yaw}
    params["map_server"]["ros__parameters"]["yaml_filename"] = map_yaml
    if layout == "corners":
        # 4 kose LiDAR tek /scan'de birlesiyor; arka LiDAR'li gozlem kaynagi yok.
        # Arka koseler de LiDAR tasidigi icin footprint arkada da uzar.
        for cm in ("local_costmap", "global_costmap"):
            cp = params[cm][cm]["ros__parameters"]
            cp["obstacle_layer"]["observation_sources"] = "scan"
            cp["obstacle_layer"].pop("scan_rear", None)
            cp["footprint"] = "[[0.53, 0.25], [0.53, -0.25], [-0.53, -0.25], [-0.53, 0.25]]"
    if localization == "gnss":
        params.pop("amcl", None)
    nav_file = write_tmp(namespace_keys(params, ns), f"nav2_{ns}")

    filt = yaml.safe_load(open(os.path.join(share, "config", "laser_filter.yaml")))
    filt = prefix_frames(copy.deepcopy(filt), ns)
    filt_file = write_tmp(namespace_keys(filt, ns), f"laser_filter_{ns}")

    common = {"use_sim_time": True}
    nodes = []
    if layout == "corners":
        nodes.append(Node(
            package="yektemak_tasks", executable="scan_merger", name="scan_merger", namespace=ns,
            output="screen", parameters=[common, {"base_frame": f"{ns}/base_footprint"}]))
    else:
        nodes += lidar_filter_nodes(ns, filt_file)
    loc_nodes = ["map_server"]
    nodes.append(Node(package="nav2_map_server", executable="map_server", name="map_server",
                      namespace=ns, output="screen", parameters=[nav_file, common]))
    if localization == "gnss":
        nodes += gnss_nodes(ns, x, y, yaw, stations_file)
    else:
        loc_nodes.append("amcl")
        nodes.append(Node(package="nav2_amcl", executable="amcl", name="amcl",
                          namespace=ns, output="screen", parameters=[nav_file, common]))
    nodes.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                      name="lifecycle_manager_localization", namespace=ns, output="screen",
                      parameters=[common, {"autostart": True, "node_names": loc_nodes,
                                           "bond_timeout": 30.0}]))
    for name in NAV_NODES:
        pkg = {"controller_server": "nav2_controller", "smoother_server": "nav2_smoother",
               "planner_server": "nav2_planner", "behavior_server": "nav2_behaviors",
               "bt_navigator": "nav2_bt_navigator", "waypoint_follower": "nav2_waypoint_follower",
               "velocity_smoother": "nav2_velocity_smoother"}[name]
        remap = []
        if name == "controller_server":
            remap = [("cmd_vel", "cmd_vel_nav")]
        if name == "velocity_smoother":
            remap = [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]
        nodes.append(Node(package=pkg, executable=name, name=name, namespace=ns, output="screen",
                          parameters=[nav_file, common], remappings=remap))
    nodes.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                      name="lifecycle_manager_navigation", namespace=ns, output="screen",
                      parameters=[common, {"autostart": True, "node_names": NAV_NODES,
                                   # Yogun CPU altinda acilista 4 s yetmiyor
                                   "bond_timeout": 30.0}]))
    return nodes


def lidar_filter_nodes(ns, filt_file):
    return [
        Node(package="laser_filters", executable="scan_to_scan_filter_chain",
             name="scan_to_scan_filter_chain", namespace=ns, output="screen",
             parameters=[filt_file], remappings=[("scan", "scan_raw"), ("scan_filtered", "scan")]),
        Node(package="laser_filters", executable="scan_to_scan_filter_chain",
             name="rear_scan_filter", namespace=ns, output="screen",
             parameters=[filt_file], remappings=[("scan", "scan_rear_raw"), ("scan_filtered", "scan_rear")]),
    ]


def gnss_nodes(ns, x, y, yaw, stations_file):
    """RTK simulatoru + navsat_transform + EKF (map cercevesinde)."""
    datum = yaml.safe_load(open(stations_file))["datum"]
    common = {"use_sim_time": True}
    seed = sum(ord(c) for c in ns)
    ekf = {
        "frequency": 20.0, "two_d_mode": True, "publish_tf": True, "print_diagnostics": False,
        "map_frame": "map", "odom_frame": f"{ns}/odom", "base_link_frame": f"{ns}/base_footprint",
        "world_frame": "map", "transform_timeout": 0.1,
        # Tekerlek odometrisi: hizlar
        "odom0": f"/{ns}/odom",
        "odom0_config": [False, False, False, False, False, False,
                         True, True, False, False, False, True, False, False, False],
        # IMU: mutlak yon + donus hizi
        "imu0": f"/{ns}/imu/data",
        "imu0_config": [False, False, False, False, False, True,
                        False, False, False, False, False, True, False, False, False],
        "imu0_remove_gravitational_acceleration": True,
        # RTK konumu (navsat_transform_node ciktisi)
        "odom1": f"/{ns}/odometry/gps",
        "odom1_config": [True, True, False, False, False, False,
                         False, False, False, False, False, False, False, False, False],
        "initial_state": [x, y, 0.0, 0.0, 0.0, yaw, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
    navsat = {
        "frequency": 10.0, "delay": 1.0, "magnetic_declination_radians": 0.0, "yaw_offset": 0.0,
        "zero_altitude": True, "broadcast_cartesian_transform": False, "publish_filtered_gps": False,
        "use_odometry_yaw": False, "wait_for_datum": True,
        "datum": [float(datum["lat"]), float(datum["lon"]), 0.0],
    }
    return [
        Node(package="yektemak_gnss", executable="rtk_simulator", name="rtk_simulator",
             namespace=ns, output="screen",
             parameters=[common, {"stations_file": stations_file, "frame_id": f"{ns}/base_footprint",
                                  "seed": seed}]),
        Node(package="robot_localization", executable="navsat_transform_node", name="navsat_transform",
             namespace=ns, output="screen", parameters=[common, navsat],
             remappings=[("imu", "imu/data"), ("gps/fix", "gnss/fix")]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_map",
             namespace=ns, output="screen", parameters=[common, ekf]),
    ]


def generate_launch_description():
    share = get_package_share_directory("yektemak_navigation")
    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="robot1"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "factory.yaml")),
        DeclareLaunchArgument("nav2_params_file",
                              default_value=os.path.join(share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("localization", default_value="amcl", description="amcl | gnss"),
        DeclareLaunchArgument("lidar_layout", default_value="front_rear", description="front_rear | corners"),
        DeclareLaunchArgument("stations_file", default_value=os.path.join(
            get_package_share_directory("yektemak_fleet"), "config", "port_stations.yaml")),
        OpaqueFunction(function=setup),
    ])
