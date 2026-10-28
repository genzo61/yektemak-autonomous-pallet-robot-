"""Yektemak gorev yoneticisi: al -> tasi -> birak -> eve don.

Her gorev "alma_istasyonu:birakma_istasyonu" seklindedir (config/mission.yaml).
Bir gorevin adimlari:

  1. NAV_PREDOCK  Nav2 ile paletin onundeki hazirlik noktasina git
  2. DETECT       On LiDAR ile paleti olc (birkac scan ortalamasi)
  3. DOCK         Cizgi takip kontrolcusuyle raylarin arasindan paletin altina gir
                  (hizalama kapisi gecilemezse geri cekil, tekrar olc, tekrar dene)
  4. LIFT         Lift'i kaldir, paleti kilitle (Gazebo'da pallet_attacher)
  5. LOADED       Yuklu footprint + hiz limiti
  6. NAV_DROP     Nav2 ile birakma noktasina git
  7. LOWER        Paleti indir, kilidi birak
  8. UNDOCK       Paletin altindan ileri dogru cik, bos moda don

Durum bilgisi: /mission/status (std_msgs/String)
Ek gorev:      ros2 topic pub --once /mission/request std_msgs/msg/String "{data: 'pick_1:drop_2'}"
"""

import math
import time

import rclpy
from geometry_msgs.msg import Point32, Polygon, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.msg import SpeedLimit
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool

from yektemak_tasks.geometry import (
    clamp, normalize_angle, quaternion_from_yaw, to_local, transform_point,
    yaw_from_quaternion)
from yektemak_tasks.pallet_detection import average_detections, detect_pallet


class MissionError(Exception):
    pass


class MissionManager(BasicNavigator):

    def __init__(self):
        super().__init__(node_name="mission_manager")

        p = self.declare_parameter
        p("home", [0.0, 0.0, 0.0])
        p("station_names", ["pick_1", "pick_2", "drop_1", "drop_2"])
        p("tasks", ["pick_1:drop_1", "pick_2:drop_2"])
        p("return_home", True)
        p("predock_distance", 1.8)
        p("lidar_x", 0.47)
        p("lift_up", 0.10)
        p("lift_release", 0.045)
        p("loaded_speed_percent", 60.0)
        p("footprint_empty", [0.52, 0.25, -0.42, -0.25])
        p("footprint_loaded", [0.62, 0.42, -0.62, -0.42])
        p("dock_speed", 0.12)
        p("dock_gate_lateral", 0.035)
        p("dock_gate_heading", 0.05)
        p("dock_retries", 2)
        p("undock_distance", 1.15)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.home = list(g("home"))
        self.stations = {}
        for name in g("station_names"):
            self.declare_parameter(f"stations.{name}", [0.0, 0.0, 0.0])
            self.stations[name] = list(self.get_parameter(f"stations.{name}").value)
        self.queue = [t for t in g("tasks") if t]
        self.cfg = {n: g(n) for n in (
            "return_home", "predock_distance", "lidar_x", "lift_up", "lift_release",
            "loaded_speed_percent", "footprint_empty", "footprint_loaded", "dock_speed",
            "dock_gate_lateral", "dock_gate_heading", "dock_retries", "undock_distance")}

        self.odom = None
        self.scan = None
        self.amcl_yaw = None
        self.lift_pos = 0.0

        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        self.create_subscription(LaserScan, "scan_raw", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(JointState, "joint_states", self._on_joints, 20)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._on_amcl, 10)
        self.create_subscription(String, "mission/request", self._on_request, 10)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.lift_pub = self.create_publisher(Float64MultiArray, "lift_controller/commands", 10)
        self.status_pub = self.create_publisher(String, "mission/status", 10)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.fp_pubs = [self.create_publisher(Polygon, "local_costmap/footprint", reliable),
                        self.create_publisher(Polygon, "global_costmap/footprint", reliable)]
        self.speed_pub = self.create_publisher(SpeedLimit, "speed_limit", reliable)
        self.attach_cli = self.create_client(SetBool, "pallet_attacher/attach")

    # ------------------------------------------------------------------ abonelikler
    def _on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _on_scan(self, msg):
        self.scan = msg

    def _on_amcl(self, msg):
        self.amcl_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _on_joints(self, msg):
        if "lift_platform_joint" in msg.name:
            self.lift_pos = msg.position[msg.name.index("lift_platform_joint")]

    def _on_request(self, msg):
        task = msg.data.strip()
        if ":" in task:
            self.queue.append(task)
            self.say(f"Yeni gorev kuyruga eklendi: {task}")

    # ------------------------------------------------------------------ yardimcilar
    def say(self, text):
        self.info(text)
        self.status_pub.publish(String(data=text))

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def make_pose(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        _, _, ps.pose.orientation.z, ps.pose.orientation.w = quaternion_from_yaw(yaw)
        return ps

    def navigate(self, x, y, yaw, label):
        self.say(f"Nav2 -> {label} ({x:.2f}, {y:.2f})")
        for attempt in range(3):
            self.goToPose(self.make_pose(x, y, yaw))
            while not self.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.getResult() == TaskResult.SUCCEEDED:
                return
            self.warn(f"Nav2 hedefe ulasamadi ({label}), tekrar {attempt + 1}/3")
            self.clearAllCostmaps()
            self.spin_for(1.0)
        raise MissionError(f"{label} noktasina gidilemedi")

    def set_footprint(self, rect):
        fx, fy, rx, ry = rect
        poly = Polygon(points=[Point32(x=float(fx), y=float(fy)), Point32(x=float(fx), y=float(ry)),
                               Point32(x=float(rx), y=float(ry)), Point32(x=float(rx), y=float(fy))])
        for pub in self.fp_pubs:
            pub.publish(poly)

    def set_speed(self, percent):
        msg = SpeedLimit()
        msg.percentage = True
        msg.speed_limit = float(percent) if percent < 100 else 0.0  # 0 = limitsiz
        self.speed_pub.publish(msg)

    def set_loaded(self, loaded):
        self.set_footprint(self.cfg["footprint_loaded" if loaded else "footprint_empty"])
        self.set_speed(self.cfg["loaded_speed_percent"] if loaded else 100.0)
        self.spin_for(0.5)

    def move_lift(self, height, timeout=8.0):
        msg = Float64MultiArray(data=[float(height)])
        end = time.time() + timeout
        while time.time() < end:
            self.lift_pub.publish(msg)
            self.spin_for(0.1)
            if abs(self.lift_pos - height) < 0.006:
                self.spin_for(0.5)
                return
        raise MissionError(f"Lift {height:.3f} m yuksekligine ulasamadi (su an {self.lift_pos:.3f})")

    def attach(self, on):
        if not self.attach_cli.wait_for_service(timeout_sec=2.0):
            self.warn("pallet_attacher servisi yok (gercek robotta normal)")
            return
        fut = self.attach_cli.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or not res.success:
            raise MissionError("Palet kilitlenemedi: " + (res.message if res else "cevap yok"))
        self.info(res.message)

    def map_yaw_to_odom(self, yaw_map):
        """Haritadaki bir yonu odom cercevesine cevirir (AMCL ile odom arasindaki fark)."""
        if self.amcl_yaw is None or self.odom is None:
            return yaw_map
        return normalize_angle(yaw_map - (self.amcl_yaw - self.odom[2]))

    # ------------------------------------------------------------------ docking
    def detect(self, expected_yaw_odom):
        s = self.scan
        return detect_pallet(list(s.ranges), s.angle_min, s.angle_increment, s.range_min,
                             expected_yaw=normalize_angle(expected_yaw_odom - self.odom[2]))

    def measure_pallet(self, expected_yaw_odom, samples=8):
        """Paleti birkac scan ile olcer; odom cercevesinde (x, y, yaw) dondurur."""
        dets, last = [], None
        end = time.time() + 4.0
        while len(dets) < samples and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.scan is None or self.scan is last or self.odom is None:
                continue
            last = self.scan
            d = self.detect(expected_yaw_odom)
            if d is not None:
                dets.append(d)
        if len(dets) < samples // 2:
            return None
        d = average_detections(dets)
        # LiDAR cercevesi -> base_footprint -> odom
        ox, oy = transform_point(self.odom, d.x + self.cfg["lidar_x"], d.y)
        return ox, oy, normalize_angle(self.odom[2] + d.yaw)

    def drive_straight(self, distance, speed=0.12):
        """Odometriye gore duz git (negatif = geri), baslangic yonunu koruyarak."""
        start = self.odom
        sign = 1.0 if distance >= 0 else -1.0
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            gone, _ = to_local(start, self.odom[0], self.odom[1])
            remaining = abs(distance) - abs(gone)
            if remaining <= 0.005:
                break
            cmd = Twist()
            cmd.linear.x = sign * clamp(0.8 * remaining + 0.03, 0.03, speed)
            cmd.angular.z = clamp(-2.0 * normalize_angle(self.odom[2] - start[2]), -0.3, 0.3)
            self.cmd_pub.publish(cmd)
        self.stop()
        self.spin_for(0.3)

    def dock(self, expected_yaw_odom):
        """Paletin altina gir. Basari durumunda robot palet merkezinde durur."""
        for attempt in range(self.cfg["dock_retries"] + 1):
            target = self.measure_pallet(expected_yaw_odom)
            if target is None:
                raise MissionError("LiDAR ile palet bulunamadi")
            rel = to_local(target, self.odom[0], self.odom[1])
            self.say(f"Palet olculdu (deneme {attempt + 1}): robot palete gore "
                     f"{-rel[0]:.2f} m geride, {rel[1] * 100:+.1f} cm yanda")
            if self._dock_follow(target):
                return
            self.say("Hizalama kapisi gecilemedi, geri cekilip tekrar olculuyor")
            self.drive_straight(-0.6, 0.12)
        raise MissionError("Palete hizalanilamadi")

    def _dock_follow(self, target):
        """Palet ekseni boyunca cizgi takip. False = hizalama kapisinda takildi."""
        tx, ty, tyaw = target
        lidar_x = self.cfg["lidar_x"]
        refine_until = -(lidar_x + 0.55 + 0.30)   # LiDAR ray uclarina 30 cm kalana kadar olcum
        gate = -(lidar_x + 0.55 + 0.05)           # LiDAR ray uclarina 5 cm kala kapi
        last_refine = 0.0
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            rx, ry, ryaw = self.odom
            # Robotun palet cercevesindeki konumu (palet merkezi 0, giris -x tarafindan)
            px, py = to_local((tx, ty, tyaw), rx, ry)
            heading_err = normalize_angle(ryaw - tyaw)

            # Uzaktayken hedefi yeni olcumlerle hafifce guncelle
            if px < refine_until and time.time() - last_refine > 0.4:
                last_refine = time.time()
                d = self.detect(tyaw)
                if d is not None:
                    nx, ny = transform_point(self.odom, d.x + lidar_x, d.y)
                    nyaw = normalize_angle(ryaw + d.yaw)
                    if math.hypot(nx - tx, ny - ty) < 0.15:
                        k = 0.3
                        tx, ty = tx + k * (nx - tx), ty + k * (ny - ty)
                        tyaw = normalize_angle(tyaw + k * normalize_angle(nyaw - tyaw))

            # Hizalama kapisi: raylarin arasina girmeden once yeterince duzgun muyuz?
            if gate - 0.03 < px < gate:
                if (abs(py) > self.cfg["dock_gate_lateral"]
                        or abs(heading_err) > self.cfg["dock_gate_heading"]):
                    self.stop()
                    self.warn(f"Kapi: yanal {py * 100:+.1f} cm, aci {math.degrees(heading_err):+.1f} derece")
                    return False

            if px >= -0.005:
                self.stop()
                self.spin_for(0.5)
                px, py = to_local((tx, ty, tyaw), self.odom[0], self.odom[1])
                self.say(f"Palet altinda: merkez hatasi {px * 100:+.1f} cm ileri, {py * 100:+.1f} cm yan")
                return True

            inside = px > gate
            cmd = Twist()
            cmd.linear.x = clamp(0.6 * (-px) + 0.02, 0.02, 0.06 if inside else self.cfg["dock_speed"])
            # Stanley benzeri kontrol: yanal hatayi kapatan hedef yon
            desired = clamp(-math.atan((2.0 if inside else 4.0) * py), -0.5, 0.5)
            cmd.angular.z = clamp(2.0 * normalize_angle(desired - heading_err), -0.4, 0.4)
            self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ gorev
    def run_task(self, task):
        pick, drop = task.split(":")
        if pick not in self.stations or drop not in self.stations:
            raise MissionError(f"Bilinmeyen istasyon: {task}")
        sx, sy, syaw = self.stations[pick]
        dx, dy, dyaw = self.stations[drop]
        d = self.cfg["predock_distance"]

        self.say(f"=== GOREV {pick} -> {drop} ===")
        self.set_loaded(False)
        self.navigate(sx - d * math.cos(syaw), sy - d * math.sin(syaw), syaw, f"{pick} hazirlik")
        self.spin_for(1.0)
        self.dock(self.map_yaw_to_odom(syaw))

        self.say("Lift kalkiyor")
        self.move_lift(self.cfg["lift_up"])
        self.attach(True)
        self.set_loaded(True)
        self.clearAllCostmaps()

        self.navigate(dx, dy, dyaw, f"{drop} birakma")
        self.say("Palet indiriliyor")
        self.move_lift(self.cfg["lift_release"])
        self.attach(False)
        self.move_lift(0.0)

        self.say("Paletin altindan cikiliyor")
        self.drive_straight(self.cfg["undock_distance"], 0.15)
        self.set_loaded(False)
        self.clearAllCostmaps()
        self.say(f"=== GOREV TAMAM {pick} -> {drop} ===")

    def run(self):
        self.setInitialPose(self.make_pose(*self.home))
        self.waitUntilNav2Active()
        self.set_loaded(False)
        self.say("Nav2 hazir, gorevler basliyor")
        done, failed, went_home = 0, 0, True
        while rclpy.ok():
            if not self.queue:
                if self.cfg["return_home"] and not went_home:
                    self.set_loaded(False)
                    try:
                        self.navigate(*self.home, "park alani")
                    except MissionError as e:
                        self.error(str(e))
                    self.say(f"TUM GOREVLER BITTI: {done} basarili, {failed} basarisiz. "
                             "Yeni gorev bekleniyor.")
                    went_home = True
                self.spin_for(0.5)
                continue
            task = self.queue.pop(0)
            went_home = False
            try:
                self.run_task(task)
                done += 1
            except MissionError as e:
                failed += 1
                self.stop()
                self.error(f"Gorev basarisiz ({task}): {e}")
                self.status_pub.publish(String(data=f"HATA: {e}"))


def main():
    rclpy.init()
    node = MissionManager()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
