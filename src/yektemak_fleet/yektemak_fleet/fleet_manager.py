"""Filo yoneticisi ROS 2 dugumu.

Konular (hepsi std_msgs/String icinde JSON tasir, boylece web paneli rosbridge
ile ek mesaj paketi olmadan konusabilir):

  /fleet/state          (yayin, 2 Hz)   robotlar, gorevler, istasyonlar, kilitler, olaylar
  /fleet/locks          (yayin)         bolge kilit tablosu (ajanlar okur)
  /fleet/command        (abone)         operator komutlari (asagida)
  /fleet/robot_status   (abone)         ajanlarin durum mesajlari
  /fleet/task_result    (abone)         ajanlarin gorev sonuclari
  /fleet/zone_request   (abone)         ajanlarin kilit istekleri
  /<robot>/agent/task   (yayin)         robota atanan gorev
  /<robot>/agent/estop  (yayin, Bool)   acil durdurma

Komut ornekleri:
  {"cmd": "create_task", "pick": "IN_1", "drop": "ST_3", "robot": "auto"}
  {"cmd": "cancel_task", "id": "T4"}
  {"cmd": "goto", "robot": "robot1", "target": "CH_1"}
  {"cmd": "estop", "robot": "all", "on": true}
  {"cmd": "auto", "on": true}
"""

import json
import math

import rclpy
import yaml
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from yektemak_fleet.fleet_core import FleetConfig, FleetCore


class FleetManager(Node):

    def __init__(self):
        super().__init__("fleet_manager")
        self.declare_parameter("stations_file", "")
        self.declare_parameter("robots", ["robot1", "robot2"])
        self.declare_parameter("homes", ["CH_1", "CH_2"])
        self.declare_parameter("auto_start", False)
        self.declare_parameter("battery_low", 30.0)
        self.declare_parameter("battery_ok", 80.0)
        self.declare_parameter("auto_process_time", 45.0)
        self.declare_parameter("auto_ship_time", 30.0)
        for z in ("inbound", "storage", "production", "outbound"):
            self.declare_parameter(f"wait_points.{z}", [0.0, 0.0, 0.0])

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        data = yaml.safe_load(open(g("stations_file")))
        cfg = FleetConfig(battery_low=g("battery_low"), battery_ok=g("battery_ok"),
                          auto_process_time=g("auto_process_time"), auto_ship_time=g("auto_ship_time"))
        waits = {z: list(g(f"wait_points.{z}")) for z in ("inbound", "storage", "production", "outbound")}
        clock = lambda: self.get_clock().now().nanoseconds * 1e-9  # noqa: E731
        self.core = FleetCore(data["stations"], dict(zip(g("robots"), g("homes"))),
                              data.get("initial_pallets", {}), waits, cfg, clock)
        self.core.auto = bool(g("auto_start"))

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "/fleet/state", 10)
        self.locks_pub = self.create_publisher(String, "/fleet/locks", latched)
        self.task_pubs = {r: self.create_publisher(String, f"/{r}/agent/task", 10) for r in self.core.robots}
        self.estop_pubs = {r: self.create_publisher(Bool, f"/{r}/agent/estop", latched)
                           for r in self.core.robots}
        self.hold_pubs = {r: self.create_publisher(Bool, f"/{r}/agent/hold", latched)
                          for r in self.core.robots}
        self.create_subscription(String, "/fleet/command", self.on_command, 20)
        self.create_subscription(String, "/fleet/robot_status", self.on_status, 50)
        self.create_subscription(String, "/fleet/task_result", self.on_result, 20)
        self.create_subscription(String, "/fleet/zone_request", self.on_zone, 50)
        self.gz = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.gz_get = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        self.declare_parameter("expected_world_model", "factory_building")
        self.world_checked = False

        self.create_timer(0.5, self.on_timer)
        self._last_locks = None
        self.core.log("Filo yoneticisi basladi" + (" (otomatik uretim acik)" if self.core.auto else ""))
        self.get_logger().info(f"Filo yoneticisi hazir: {list(self.core.robots)}")

    # ------------------------------------------------------------------ gelenler
    @staticmethod
    def _json(msg):
        try:
            return json.loads(msg.data)
        except (ValueError, TypeError):
            return None

    def on_status(self, msg):
        d = self._json(msg)
        if d:
            self.core.on_robot_status(d)

    def on_result(self, msg):
        d = self._json(msg)
        if d:
            self.core.on_task_result(d)

    def on_zone(self, msg):
        d = self._json(msg)
        if d:
            self.core.zone_request(d.get("robot"), d.get("zone"), d.get("op"))
            self.publish_locks(force=True)

    def on_command(self, msg):
        d = self._json(msg)
        if not d:
            return
        c = self.core
        cmd = d.get("cmd")
        if cmd == "create_task":
            robot = d.get("robot")
            c.create_transport(d.get("pick"), d.get("drop"), None if robot in (None, "", "auto") else robot)
        elif cmd == "cancel_task":
            err = c.cancel(d.get("id"))
            c.log(f"{d.get('id')} iptal edilemedi: {err}" if err else f"{d.get('id')} iptal edildi",
                  "warn" if err else "info")
        elif cmd == "goto":
            c.create_goto(d.get("robot"), d.get("target"))
        elif cmd == "estop":
            on = bool(d.get("on", True))
            targets = list(c.robots) if d.get("robot") in (None, "all") else [d.get("robot")]
            for r in targets:
                if r in self.estop_pubs:
                    self.estop_pubs[r].publish(Bool(data=on))
                    c.robots[r].estop = on
            c.log(f"ACIL DURDURMA {'AKTIF' if on else 'kaldirildi'}: {', '.join(targets)}",
                  "error" if on else "info")
        elif cmd == "auto":
            c.auto = bool(d.get("on"))
            c.log(f"Otomatik uretim modu {'ACIK' if c.auto else 'KAPALI'}")

    # ------------------------------------------------------------------ periyodik
    def check_world(self):
        """Gazebo'da fabrika dunyasi gercekten yuklu mu? (Eski bir Gazebo acik kalinca
        robotlar baska dunyaya eklenir; bu durumu panelde acikca goster.)"""
        if self.world_checked or not self.gz_get.service_is_ready():
            return
        self.world_checked = True
        name = self.get_parameter("expected_world_model").value
        fut = self.gz_get.call_async(GetEntityState.Request(name=name))

        def done(f):
            res = f.result()
            if res is None or not res.success:
                self.core.log(f"UYARI: Gazebo'da '{name}' modeli yok, yanlis dunya acik olabilir. "
                              "Tum ROS/Gazebo sureclerini kapatip (killall -9 gzserver gzclient) "
                              "demoyu yeniden baslatin.", "error")
                self.get_logger().error("Gazebo'da fabrika dunyasi bulunamadi!")
            else:
                self.core.log("Gazebo fabrika dunyasi dogrulandi")
        fut.add_done_callback(done)

    def on_timer(self):
        self.check_world()
        self.core.tick()
        while self.core.outbox:
            robot, task = self.core.outbox.pop(0)
            self.task_pubs[robot].publish(String(data=json.dumps(task)))
        while self.core.hold_changes:
            robot, on = self.core.hold_changes.pop(0)
            self.hold_pubs[robot].publish(Bool(data=on))
        while self.core.sim_actions:
            self.do_sim_action(self.core.sim_actions.pop(0))
        self.publish_locks()
        self.state_pub.publish(String(data=json.dumps(self.core.snapshot())))

    def publish_locks(self, force=False):
        s = json.dumps(self.core.locks, sort_keys=True)
        if force or s != self._last_locks or True:
            self.locks_pub.publish(String(data=s))
            self._last_locks = s

    def do_sim_action(self, a):
        if a.get("op") != "move_pallet" or not self.gz.service_is_ready():
            return
        st = EntityState()
        st.name = a["pallet"]
        st.reference_frame = "world"
        st.pose.position.x, st.pose.position.y, st.pose.position.z = a["x"], a["y"], 0.02
        st.pose.orientation.z = math.sin(a["yaw"] / 2.0)
        st.pose.orientation.w = math.cos(a["yaw"] / 2.0)
        self.gz.call_async(SetEntityState.Request(state=st))


def main():
    rclpy.init()
    node = FleetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
