"""RTK-GNSS alici simulatoru (robot basina bir tane).

Gazebo'daki gercek konumu (/<robot>/odom, planar_move dunya konumu) alir, RTK
modelinden gecirip gercek bir alici gibi yayinlar:

  gnss/fix       sensor_msgs/NavSatFix  (5 Hz)  navsat_transform_node bunu okur
  gnss/quality   std_msgs/String JSON   (2 Hz)  {"mode": "FIX", "sigma": .., "age": .., "sats": .., "zone": ..}

  /gnss/base_station_down (std_msgs/Bool, latched)  true: baz istasyonu kapali (demo)

Not: Gazebo'nun kendi GPS eklentisi bina/konteyner golgesini bilmez ve her yerde
ayni kalitede konum verir. Bu dugum sahaya ozgu kaliteyi (port_stations.yaml'daki
gnss_zones) ve RTK durum gecislerini modeller.
"""

import json
import math

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String

from yektemak_gnss.rtk_model import RtkConfig, RtkModel
from yektemak_gnss.utm import LocalFrame

STATUS = {"FIX": NavSatStatus.STATUS_GBAS_FIX, "FLOAT": NavSatStatus.STATUS_SBAS_FIX,
          "SINGLE": NavSatStatus.STATUS_FIX}


class RtkSimulator(Node):

    def __init__(self):
        super().__init__("rtk_simulator")
        p = self.declare_parameter
        p("stations_file", "")
        p("frame_id", "gnss_link")
        p("rate", 5.0)
        p("seed", 0)
        p("link_drop_interval", 200.0)
        p("outage_interval", 900.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        data = yaml.safe_load(open(g("stations_file")))
        d = data["datum"]
        self.frame = LocalFrame(d["lat"], d["lon"])
        cfg = RtkConfig(link_drop_interval=g("link_drop_interval"), outage_interval=g("outage_interval"))
        self.model = RtkModel(data.get("gnss_zones", []), cfg, seed=g("seed") or None)
        self.frame_id = g("frame_id")
        self.truth = None
        self.base_down = False
        self.state = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "odom", self.on_odom, 20)
        self.create_subscription(Bool, "/gnss/base_station_down", self.on_base, latched)
        self.fix_pub = self.create_publisher(NavSatFix, "gnss/fix", 10)
        self.q_pub = self.create_publisher(String, "gnss/quality", 10)
        self.create_timer(1.0 / g("rate"), self.on_timer)
        self.create_timer(0.5, self.on_quality)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.truth = (p.x, p.y)

    def on_base(self, msg):
        self.base_down = msg.data
        self.get_logger().warn("RTK baz istasyonu " + ("KAPALI" if msg.data else "tekrar acik"))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_timer(self):
        if self.truth is None:
            return
        t = self.now()
        if self.base_down:
            self.model.drop_link(1.0)
        st = self.model.step(t, *self.truth)
        self.state = st
        if st.mode == "NONE":
            return
        lat, lon = self.frame.to_geo(st.x, st.y)
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.status.status = STATUS[st.mode]
        m.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GALILEO
        m.latitude, m.longitude, m.altitude = lat, lon, 0.0
        v = st.sigma ** 2
        m.position_covariance = [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, (2 * st.sigma) ** 2]
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(m)

    def on_quality(self):
        st = self.state
        if st is None:
            return
        self.q_pub.publish(String(data=json.dumps({
            "mode": st.mode, "sigma": None if math.isinf(st.sigma) else round(st.sigma, 3),
            "age": st.age, "sats": st.sats, "zone": st.zone, "base_down": self.base_down})))


def main():
    rclpy.init()
    node = RtkSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
