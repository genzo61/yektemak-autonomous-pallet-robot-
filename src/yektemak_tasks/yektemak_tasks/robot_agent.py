"""Filo modunda robot ajani.

Her robotta (namespace icinde) bir tane calisir. Gorevleri filo yoneticisinden
alir, MissionManager'in docking / lift / Nav2 yeteneklerini kullanarak yapar.

  ros2 run yektemak_tasks robot_agent --ros-args -r __ns:=/robot1 -p robot_name:=robot1

Konular:
  agent/task            (abone, String JSON)  filodan gelen gorev
  agent/estop           (abone, Bool)         acil durdurma
  /fleet/locks          (abone, String JSON)  bolge kilit tablosu
  /fleet/robot_status   (yayin, 2 Hz)         durum, konum (map), batarya
  /fleet/task_result    (yayin)               gorev sonucu
  /fleet/zone_request   (yayin)               kilit iste / birak
"""

import json
import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from yektemak_tasks.geometry import normalize_angle, transform_point, yaw_from_quaternion
from yektemak_tasks.mission_manager import MissionError, MissionManager


class RobotAgent(MissionManager):

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        p("robot_name", self.get_namespace().strip("/") or "robot")
        p("battery_per_meter", 0.25)      # %/m
        p("battery_idle_per_s", 0.004)    # %/s
        p("battery_charge_per_s", 0.6)    # %/s (sarjdayken)
        p("zone_release_distance", 4.0)   # m
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.name = g("robot_name")
        self.bat_m, self.bat_idle, self.bat_charge = g("battery_per_meter"), g("battery_idle_per_s"), g("battery_charge_per_s")
        self.release_dist = g("zone_release_distance")

        self.battery = 100.0
        self.estop = False
        self.hold = False          # filo trafik yonetimi: yol ver
        self.state = "idle"
        self.step = "Hazir"
        self.task = None
        self.pending = []
        self.loaded = False
        self.pallet = None
        self.locks = {}
        self.held = {}             # zone -> [x, y, sticky]
        self.map_offset = None     # odom -> map (dx, dy, dyaw)
        self.last_odom_xy = None
        self.last_t = time.time()
        self.returning = False
        self.idle_since = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "agent/task", self._on_task, 10)
        self.create_subscription(Bool, "agent/estop", self._on_estop, latched)
        self.create_subscription(Bool, "agent/hold", self._on_hold, latched)
        self.create_subscription(String, "/fleet/locks", self._on_locks, latched)
        self.status_pub = self.create_publisher(String, "/fleet/robot_status", 10)
        self.result_pub = self.create_publisher(String, "/fleet/task_result", 10)
        self.zone_pub = self.create_publisher(String, "/fleet/zone_request", 10)
        self.create_timer(0.5, self._on_timer)

    # ------------------------------------------------------------------ abonelikler
    def _on_task(self, msg):
        try:
            self.pending.append(json.loads(msg.data))
        except ValueError:
            pass

    def _on_estop(self, msg):
        if msg.data != self.estop:
            self.estop = msg.data
            self.stop()
            self.say("ACIL DURDURMA" if self.estop else "Acil durdurma kaldirildi")

    def _on_hold(self, msg):
        self.hold = msg.data

    def _on_locks(self, msg):
        try:
            self.locks = json.loads(msg.data)
        except ValueError:
            pass

    def _on_amcl(self, msg):
        super()._on_amcl(msg)
        if self.odom is None:
            return
        p = msg.pose.pose
        ayaw = yaw_from_quaternion(p.orientation)
        ox, oy, oyaw = self.odom
        dyaw = normalize_angle(ayaw - oyaw)
        c, s = math.cos(dyaw), math.sin(dyaw)
        self.map_offset = (p.position.x - (c * ox - s * oy), p.position.y - (s * ox + c * oy), dyaw)

    def _on_odom(self, msg):
        super()._on_odom(msg)
        xy = self.odom[:2]
        if self.last_odom_xy is not None:
            d = math.hypot(xy[0] - self.last_odom_xy[0], xy[1] - self.last_odom_xy[1])
            if d < 1.0:
                self.battery = max(0.0, self.battery - d * self.bat_m * (1.4 if self.loaded else 1.0))
        self.last_odom_xy = xy

    # ------------------------------------------------------------------ durum
    def map_pose(self):
        if self.odom is None:
            return 0.0, 0.0, 0.0
        if self.map_offset is None:
            return self.odom
        dx, dy, dyaw = self.map_offset
        x, y = transform_point((dx, dy, dyaw), self.odom[0], self.odom[1])
        return x, y, normalize_angle(self.odom[2] + dyaw)

    def _on_timer(self):
        now = time.time()
        dt, self.last_t = now - self.last_t, now
        if self.state == "charging":
            self.battery = min(100.0, self.battery + self.bat_charge * dt)
        else:
            self.battery = max(0.0, self.battery - self.bat_idle * dt)
        self._release_far_zones()
        x, y, yaw = self.map_pose()
        state = "estop" if self.estop else self.state
        self.status_pub.publish(String(data=json.dumps({
            "robot": self.name, "state": state, "step": self.step,
            "battery": round(self.battery, 1), "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3),
            "loaded": self.loaded, "pallet": self.pallet, "estop": self.estop,
            "task_id": self.task["id"] if self.task else None})))

    def say(self, text):
        self.step = text
        super().say(text)

    # ------------------------------------------------------------------ kancalar
    def guard(self):
        if not self.estop:
            return
        self.stop()
        while self.estop and rclpy.ok():
            self.stop()
            rclpy.spin_once(self, timeout_sec=0.1)

    def nav_paused(self):
        return self.estop or self.hold

    def wait_resume(self):
        """Acil durdurma ya da trafik beklemesi bitene kadar dur."""
        prev = self.step
        while (self.estop or self.hold) and rclpy.ok():
            if self.hold and not self.estop:
                self.step = "Trafik: diger robota yol veriliyor"
            self.stop()
            rclpy.spin_once(self, timeout_sec=0.1)
        self.step = prev

    def on_nav_tick(self):
        # Park alanina donerken yeni gorev geldiyse donusu yarida kes
        if self.returning and self.pending:
            self.cancelTask()
            raise MissionError("yeni gorev geldi")

    # ------------------------------------------------------------------ bolge kilitleri
    def zone_msg(self, zone, op):
        self.zone_pub.publish(String(data=json.dumps({"robot": self.name, "zone": zone, "op": op})))

    def acquire_zone(self, zone, station_xy, wait_pose):
        """Bolge kilidini al; doluysa bekleme noktasina git ve sira bekle."""
        if not zone:
            return
        self.zone_msg(zone, "acquire")
        self.spin_for(1.0)
        if self.locks.get(zone) != self.name:
            self.say(f"{zone} bolgesi dolu, bekleme noktasina gidiliyor")
            if wait_pose:
                self.navigate(*wait_pose, f"{zone} bekleme")
            last = 0.0
            while self.locks.get(zone) != self.name and rclpy.ok():
                if time.time() - last > 2.0:
                    self.zone_msg(zone, "acquire")
                    last = time.time()
                self.say(f"{zone} bolgesi icin sira bekleniyor")
                self.spin_for(0.5)
        self.held[zone] = [station_xy[0], station_xy[1], True]

    def finish_zone(self, zone):
        """Bolgedeki is bitti; robot uzaklasinca kilit otomatik birakilir."""
        if zone in self.held:
            self.held[zone][2] = False

    def _release_far_zones(self):
        x, y, _ = self.map_pose()
        for zone, (zx, zy, sticky) in list(self.held.items()):
            if not sticky and math.hypot(x - zx, y - zy) > self.release_dist:
                self.zone_msg(zone, "release")
                del self.held[zone]

    # ------------------------------------------------------------------ gorevler
    def predock(self, pose):
        x, y, yaw = pose
        d = self.cfg["predock_distance"]
        return x - d * math.cos(yaw), y - d * math.sin(yaw), yaw

    def do_transport(self, t):
        picked = False
        pallet = None
        try:
            px, py, pyaw = t["pick_pose"]
            self.set_loaded(False)
            self.acquire_zone(t.get("pick_zone"), (px, py), t.get("pick_wait"))
            self.navigate(*self.predock(t["pick_pose"]), f"{t['pick']} hazirlik")
            self.spin_for(1.0)
            self.say(f"{t['pick']}: palet olculuyor")
            self.dock(self.map_yaw_to_odom(pyaw))
            self.say(f"{t['pick']}: lift kalkiyor")
            self.move_lift(self.cfg["lift_up"])
            pallet = self.attach(True)
            picked, self.loaded, self.pallet = True, True, pallet
            self.set_loaded(True)
            self.clearAllCostmaps()
            self.finish_zone(t.get("pick_zone"))

            dx, dy, dyaw = t["drop_pose"]
            self.acquire_zone(t.get("drop_zone"), (dx, dy), t.get("drop_wait"))
            self.navigate(dx, dy, dyaw, f"{t['drop']} birakma ({pallet or 'palet'})")
            self.say(f"{t['drop']}: palet indiriliyor")
            self.move_lift(self.cfg["lift_release"])
            self.attach(False)
            self.move_lift(0.0)
            self.loaded, self.pallet = False, None
            self.say(f"{t['drop']}: paletin altindan cikiliyor")
            self.drive_straight(self.cfg["undock_distance"], 0.15)
            self.set_loaded(False)
            self.clearAllCostmaps()
            self.finish_zone(t.get("drop_zone"))
            return {"ok": True, "pallet": pallet, "message": "tamam"}
        except MissionError as e:
            self.stop()
            if not picked:
                # Palet kilitlenmeden hata: lift'i indir ki palet yarim kalkik kalmasin
                try:
                    self.move_lift(0.0)
                except MissionError:
                    pass
            for z in (t.get("pick_zone"), t.get("drop_zone")):
                self.finish_zone(z)
            return {"ok": False, "picked": picked, "pallet": pallet, "message": str(e)}

    def do_goto(self, t):
        try:
            self.set_loaded(self.loaded)
            self.navigate(*t["pose"], t.get("target", "hedef"))
            return {"ok": True, "message": "ulasildi"}
        except MissionError as e:
            return {"ok": False, "message": str(e)}

    def run(self):
        x, y, yaw = self.home
        self.setInitialPose(self.make_pose(x, y, yaw))
        self.waitUntilNav2Active()
        self.set_loaded(False)
        self.say("Hazir, filodan gorev bekleniyor")
        while rclpy.ok():
            self.guard()
            if not self.pending:
                at_home = self.map_offset is not None and math.hypot(
                    self.map_pose()[0] - x, self.map_pose()[1] - y) < 0.5
                self.state = "charging" if at_home and self.battery < 99.5 else "idle"
                if self.state == "charging":
                    self.step = f"Sarj oluyor (%{self.battery:.0f})"
                # Is bitti, yeni gorev yok: istasyon sirasini bosaltmak ve sarj olmak
                # icin park alanina don (bolge kilidi robot uzaklasinca birakilir).
                if (not at_home and self.idle_since is not None
                        and self.sim_now() - self.idle_since > 3.0):
                    self.returning, self.idle_since = True, None
                    self.step = "Park alanina donuyor"
                    try:
                        self.navigate(x, y, yaw, "park alani")
                    except MissionError:
                        pass
                    self.returning = False
                self.spin_for(0.2)
                continue
            t = self.pending.pop(0)
            self.task, self.state = t, "busy"
            self.say(f"Gorev {t['id']} basladi")
            res = self.do_transport(t) if t.get("kind") == "transport" else self.do_goto(t)
            res["id"] = t["id"]
            res["robot"] = self.name
            self.result_pub.publish(String(data=json.dumps(res)))
            self.task, self.state = None, "idle"
            self.idle_since = self.sim_now()
            self.say(f"Gorev {t['id']} {'tamamlandi' if res['ok'] else 'basarisiz: ' + res['message']}")


def main():
    rclpy.init()
    node = RobotAgent()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
