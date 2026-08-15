#!/usr/bin/env python3

import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


HELP = """
========================================================
          EVRENOS MANUAL DRIVE
========================================================

                 W
                 ↑
           Q     |     E
             \\  |  /
          A ←  STOP  → D
             /  |  \\
           Z     |     C
                 ↓
                 S

W : İleri
S : Geri

A : Kendi etrafında sola dön
D : Kendi etrafında sağa dön

Q : İleri + sola dön
E : İleri + sağa dön

Z : Geri + sola dön
C : Geri + sağa dön

X / SPACE : DUR

R : Lineer hızı artır
F : Lineer hızı azalt

T : Dönüş hızını artır
G : Dönüş hızını azalt

CTRL+C : Çıkış

========================================================
"""


class EvrenosManualDrive(Node):

    def __init__(self):

        super().__init__("evrenos_manual_drive")

        self.publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        # Yarışma parkuru U dönüşleri için nominal dönüş yarıçapı
        self.turn_radius = 3.0

        # Başlangıç hızları
        self.linear_speed = 1.00
        self.angular_speed = 0.35

        self.max_linear_speed = 3.00
        self.max_angular_speed = 1.20

        self.min_linear_speed = 0.10
        self.min_angular_speed = 0.10

        # Tuş bırakıldığında bu süre sonunda otomatik stop
        self.command_timeout = 0.25

        self.last_key_time = time.monotonic()

        self.current_linear = 0.0
        self.current_angular = 0.0

        # 20 Hz publish
        self.timer = self.create_timer(
            0.05,
            self.publish_command,
        )

        self.get_logger().info("EVRENOS manual drive started.")


    def set_command(self, linear, angular):

        self.current_linear = linear
        self.current_angular = angular

        self.last_key_time = time.monotonic()


    def stop(self):

        self.current_linear = 0.0
        self.current_angular = 0.0

        self.last_key_time = time.monotonic()

        msg = Twist()

        self.publisher.publish(msg)


    def publish_command(self):

        # Tuş bırakılmışsa otomatik dur
        elapsed = time.monotonic() - self.last_key_time

        if elapsed > self.command_timeout:

            self.current_linear = 0.0
            self.current_angular = 0.0

        msg = Twist()

        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular

        self.publisher.publish(msg)


    def process_key(self, key):

        # --------------------------------------------------
        # HAREKET
        # --------------------------------------------------

        if key == "w":

            self.set_command(
                self.linear_speed,
                0.0,
            )

        elif key == "s":

            self.set_command(
                -self.linear_speed,
                0.0,
            )

        elif key == "a":

            self.set_command(
                0.0,
                self.angular_speed,
            )

        elif key == "d":

            self.set_command(
                0.0,
                -self.angular_speed,
            )


        # --------------------------------------------------
        # KAVİSLİ HAREKET
        # --------------------------------------------------

        elif key == "q":

            # Sabit yarıçaplı ileri-sol dönüş
            curve_angular = self.linear_speed / self.turn_radius
            self.set_command(
                self.linear_speed,
                curve_angular,
            )

        elif key == "e":

            # Sabit yarıçaplı ileri-sağ dönüş
            curve_angular = self.linear_speed / self.turn_radius

            self.set_command(
                self.linear_speed,
                -curve_angular,
            )

        elif key == "z":

            curve_angular = self.linear_speed / self.turn_radius

            self.set_command(
                -self.linear_speed,
                -curve_angular,
            )

        elif key == "c":

            curve_angular = self.linear_speed / self.turn_radius

            self.set_command(
                -self.linear_speed,
                curve_angular,
            )


        # --------------------------------------------------
        # STOP
        # --------------------------------------------------

        elif key == "x" or key == " ":

            self.stop()


        # --------------------------------------------------
        # SPEED SETTINGS
        # --------------------------------------------------

        elif key == "r":

            self.linear_speed += 0.05

            self.linear_speed = min(
                self.linear_speed,
                self.max_linear_speed,
            )

            print(
                f"\nLinear speed: {self.linear_speed:.2f} m/s"
            )


        elif key == "f":

            self.linear_speed -= 0.05

            self.linear_speed = max(
                self.linear_speed,
                self.min_linear_speed,
            )

            print(
                f"\nLinear speed: {self.linear_speed:.2f} m/s"
            )


        elif key == "t":

            self.angular_speed += 0.05

            self.angular_speed = min(
                self.angular_speed,
                self.max_angular_speed,
            )

            print(
                f"\nAngular speed: {self.angular_speed:.2f} rad/s"
            )


        elif key == "g":

            self.angular_speed -= 0.05

            self.angular_speed = max(
                self.angular_speed,
                self.min_angular_speed,
            )

            print(
                f"\nAngular speed: {self.angular_speed:.2f} rad/s"
            )


def get_key(timeout=0.05):

    readable, _, _ = select.select(
        [sys.stdin],
        [],
        [],
        timeout,
    )

    if readable:

        return sys.stdin.read(1)

    return None


def main(args=None):

    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)

    node = EvrenosManualDrive()

    print(HELP)

    print(
        f"Başlangıç lineer hız : "
        f"{node.linear_speed:.2f} m/s"
    )

    print(
        f"Başlangıç dönüş hızı : "
        f"{node.angular_speed:.2f} rad/s\n"
    )

    try:

        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():

            key = get_key()

            if key is not None:

                # Ctrl+C
                if key == "\x03":
                    break

                node.process_key(
                    key.lower()
                )

            rclpy.spin_once(
                node,
                timeout_sec=0.01,
            )

    except KeyboardInterrupt:

        pass

    finally:

        # Çıkarken kesinlikle sıfır hız gönder
        node.stop()

        for _ in range(5):

            rclpy.spin_once(
                node,
                timeout_sec=0.02,
            )

            time.sleep(0.02)

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings,
        )

        node.destroy_node()

        rclpy.shutdown()

        print("\nEVRENOS durduruldu.")


if __name__ == "__main__":

    main()

