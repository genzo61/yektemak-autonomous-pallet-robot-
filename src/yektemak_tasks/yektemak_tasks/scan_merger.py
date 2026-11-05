"""Dort kose LiDAR'i tek 360 derecelik taramada birlestirir.

Her LiDAR'in olcumleri robot merkezine (base_footprint) tasinir, aciya gore
kutulara ayrilir ve her kutuda en yakin olcum tutulur. Iki cikti uretilir:

  scan       Nav2 / costmap icin. Robotun etrafindaki palet izdusumu temizlenir
             (tasinan paletin raylari engel sanilmasin).
  scan_dock  Docking icin. Sadece robotun kendi govdesi/tekerleri temizlenir;
             yaklasilan paletin raylari gorunur kalir.

  ros2 run yektemak_tasks scan_merger --ros-args -r __ns:=/robot1 -p base_frame:=robot1/base_footprint
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from yektemak_tasks.geometry import yaw_from_quaternion


class ScanMerger(Node):

    def __init__(self):
        super().__init__("scan_merger")
        p = self.declare_parameter
        p("inputs", ["scan_front_left_raw", "scan_front_right_raw", "scan_rear_left_raw", "scan_rear_right_raw"])
        p("base_frame", "base_footprint")
        p("angle_increment", math.radians(0.5))
        p("range_min", 0.10)
        p("range_max", 25.0)
        p("max_age", 0.35)                       # bu kadar eski tarama birlestirmeye girmez (s)
        p("self_box", [-0.56, 0.56, -0.29, 0.29])     # robot govdesi + tekerler [xmin xmax ymin ymax]
        p("pallet_box", [-0.66, 0.66, -0.46, 0.46])   # yuklu palet izdusumu
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base = g("base_frame")
        self.inc = float(g("angle_increment"))
        self.rmin, self.rmax = float(g("range_min")), float(g("range_max"))
        self.max_age = float(g("max_age"))
        self.self_box = list(g("self_box"))
        self.pallet_box = list(g("pallet_box"))
        self.nbins = int(round(2 * math.pi / self.inc))

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.poses = {}     # frame -> (x, y, yaw) base'e gore (sabit)
        self.latest = {}
        for t in g("inputs"):
            self.create_subscription(LaserScan, t, lambda m, t=t: self.latest.__setitem__(t, m),
                                     qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.pub_dock = self.create_publisher(LaserScan, "scan_dock", qos_profile_sensor_data)
        self.create_timer(0.1, self.on_timer)
        self._warned = set()

    def sensor_pose(self, frame):
        if frame not in self.poses:
            try:
                tr = self.tf.lookup_transform(self.base, frame, rclpy.time.Time())
            except Exception as e:  # noqa: BLE001
                if frame not in self._warned:
                    self.get_logger().warn(f"{frame} icin TF yok (henuz): {e}")
                    self._warned.add(frame)
                return None
            t = tr.transform
            self.poses[frame] = (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
        return self.poses[frame]

    @staticmethod
    def inside(x, y, box):
        return (x > box[0]) & (x < box[1]) & (y > box[2]) & (y < box[3])

    def on_timer(self):
        if not self.latest:
            return
        newest = max(self.latest.values(), key=lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        t_new = newest.header.stamp.sec + newest.header.stamp.nanosec * 1e-9
        xs, ys = [], []
        for m in self.latest.values():
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if t_new - t > self.max_age:
                continue
            pose = self.sensor_pose(m.header.frame_id)
            if pose is None:
                continue
            r = np.asarray(m.ranges, dtype=np.float32)
            a = m.angle_min + np.arange(len(r), dtype=np.float32) * m.angle_increment
            ok = np.isfinite(r) & (r >= max(self.rmin, m.range_min)) & (r <= min(self.rmax, m.range_max))
            r, a = r[ok], a[ok]
            sx, sy, syaw = pose
            c, s = math.cos(syaw), math.sin(syaw)
            lx, ly = r * np.cos(a), r * np.sin(a)
            xs.append(sx + c * lx - s * ly)
            ys.append(sy + s * lx + c * ly)
        if not xs:
            return
        x, y = np.concatenate(xs), np.concatenate(ys)
        self.publish(self.pub, newest, x, y, self.pallet_box)
        self.publish(self.pub_dock, newest, x, y, self.self_box)

    def publish(self, pub, ref, x, y, box):
        keep = ~self.inside(x, y, box)
        x, y = x[keep], y[keep]
        rng = np.hypot(x, y)
        ang = np.arctan2(y, x)
        idx = ((ang + math.pi) / self.inc).astype(np.int64) % self.nbins
        out = np.full(self.nbins, np.inf, dtype=np.float32)
        np.minimum.at(out, idx, rng.astype(np.float32))
        msg = LaserScan()
        msg.header.stamp = ref.header.stamp
        msg.header.frame_id = self.base
        msg.angle_min = -math.pi
        msg.angle_max = -math.pi + (self.nbins - 1) * self.inc
        msg.angle_increment = self.inc
        msg.scan_time = 0.1
        msg.range_min = self.rmin
        msg.range_max = self.rmax
        msg.ranges = out.tolist()
        pub.publish(msg)


def main():
    rclpy.init()
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
