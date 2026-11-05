"""ros2_control controller'larini guvenilir sekilde baslatir (spawner yerine).

controller_manager/spawner yogun CPU altinda servis cagrisi zaman asimina dusunce
yeniden deniyor, controller "zaten yuklu" oldugu icin hata verip olebiliyor;
controller yuklu kaliyor ama aktif olmuyor (lift hic hareket etmiyor). Bu dugum:
  * controller yoneticisinin mevcut durumunu sorar,
  * eksik adimlari (yukle -> yapilandir -> aktive et) uzun zaman asimlariyla yapar,
  * "zaten yuklu / zaten aktif" durumlarini normal kabul eder, gerekirse tekrar dener.

  ros2 run yektemak_tasks controller_starter --ros-args -p controller_manager:=/robot1/controller_manager \
      -p controllers:="[joint_state_broadcaster, lift_controller]"
"""

import sys
import time

import rclpy
from controller_manager_msgs.srv import (ConfigureController, ListControllers, LoadController,
                                         SwitchController)
from rclpy.node import Node


class ControllerStarter(Node):

    def __init__(self):
        super().__init__("controller_starter")
        self.cm = self.declare_parameter("controller_manager", "/controller_manager").value
        self.names = list(self.declare_parameter("controllers", ["joint_state_broadcaster"]).value)
        self.timeout = float(self.declare_parameter("timeout", 300.0).value)
        mk = lambda t, n: self.create_client(t, f"{self.cm}/{n}")  # noqa: E731
        self.c_list = mk(ListControllers, "list_controllers")
        self.c_load = mk(LoadController, "load_controller")
        self.c_conf = mk(ConfigureController, "configure_controller")
        self.c_switch = mk(SwitchController, "switch_controller")

    def call(self, client, req, wait=30.0):
        if not client.wait_for_service(timeout_sec=wait):
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=wait)
        return fut.result()

    def states(self):
        res = self.call(self.c_list, ListControllers.Request())
        return {c.name: c.state for c in res.controller} if res else None

    def run(self):
        end = time.time() + self.timeout
        while time.time() < end and rclpy.ok():
            st = self.states()
            if st is None:
                self.get_logger().info(f"{self.cm} bekleniyor...")
                continue
            todo = [n for n in self.names if st.get(n) != "active"]
            if not todo:
                self.get_logger().info(f"Controller'lar aktif: {', '.join(self.names)}")
                return True
            for n in todo:
                if n not in st:
                    self.call(self.c_load, LoadController.Request(name=n))
                elif st[n] == "unconfigured":
                    self.call(self.c_conf, ConfigureController.Request(name=n))
                elif st[n] == "inactive":
                    req = SwitchController.Request(activate_controllers=[n], strictness=1)
                    req.timeout.sec = 10
                    self.call(self.c_switch, req)
            time.sleep(0.5)
        self.get_logger().error(f"Controller'lar baslatilamadi: {self.names}")
        return False


def main():
    rclpy.init()
    node = ControllerStarter()
    ok = node.run()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
