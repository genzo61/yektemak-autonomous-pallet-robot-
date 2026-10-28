"""Haritalama gezgini.

slam_toolbox calisirken robotu depoda onceden tanimli bir rotada gezdirir,
boylece haritayi klavyeyle surmeden cikarabilirsiniz. Rota odometriye gore
izlenir (simulasyonda odometri cok iyidir).

  ros2 launch yektemak_bringup sim_mapping.launch.py
  ros2 run yektemak_tasks map_explorer
  ros2 run nav2_map_server map_saver_cli -f warehouse
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from yektemak_tasks.geometry import clamp, normalize_angle, yaw_from_quaternion

# Depo dunyasina gore rota (odom = baslangic pozu = park alani)
DEFAULT_ROUTE = [
    0.0, 0.0,
    6.6, 0.0,
    7.3, 1.0,
    9.0, 1.0,
    16.4, 0.6,
    16.4, 4.2,
    0.6, 4.2,
    0.6, -4.2,
    16.4, -4.2,
    16.4, -0.8,
    9.0, -1.0,
    7.3, -1.0,
    6.6, 0.0,
    0.5, 0.0,
]


class MapExplorer(Node):

    def __init__(self):
        super().__init__("map_explorer")
        self.declare_parameter("route", DEFAULT_ROUTE)
        self.declare_parameter("linear_speed", 0.45)
        self.declare_parameter("angular_speed", 0.6)
        r = list(self.get_parameter("route").value)
        self.route = [(r[i], r[i + 1]) for i in range(0, len(r) - 1, 2)]
        self.v_max = float(self.get_parameter("linear_speed").value)
        self.w_max = float(self.get_parameter("angular_speed").value)

        self.pose = None
        self.index = 1
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.create_timer(0.05, self.step)
        self.get_logger().info(f"{len(self.route)} noktalik haritalama rotasi basliyor")

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def step(self):
        if self.pose is None:
            return
        if self.index >= len(self.route):
            self.cmd_pub.publish(Twist())
            if self.index == len(self.route):
                self.get_logger().info("Rota tamamlandi. Haritayi kaydedebilirsiniz.")
                self.index += 1
            return
        x, y, yaw = self.pose
        gx, gy = self.route[self.index]
        dist = math.hypot(gx - x, gy - y)
        if dist < 0.15:
            self.get_logger().info(f"Nokta {self.index}/{len(self.route) - 1} ({gx:.1f}, {gy:.1f})")
            self.index += 1
            return
        err = normalize_angle(math.atan2(gy - y, gx - x) - yaw)
        cmd = Twist()
        cmd.angular.z = clamp(1.5 * err, -self.w_max, self.w_max)
        # Once hedefe don, sonra ilerle (yavas donusler SLAM icin daha iyi)
        if abs(err) < 0.35:
            cmd.linear.x = clamp(0.8 * dist, 0.08, self.v_max) * (1.0 - abs(err) / 0.35)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = MapExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
