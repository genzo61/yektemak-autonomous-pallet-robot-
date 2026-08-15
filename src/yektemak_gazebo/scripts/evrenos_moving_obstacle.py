#!/usr/bin/env python3
"""
TEKNOFEST 2026 sliding obstacle controller.

Official requirement:
- obstacle width: 1 m
- lateral speed: 20 cm/s
- moves continuously left/right
- fully exits the 3 m-wide road before reversing

This node moves the Gazebo Classic model named "sliding_obstacle"
using /set_entity_state.
"""

import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState


class SlidingObstacle(Node):
    def __init__(self):
        super().__init__("evrenos_sliding_obstacle")

        self.client = self.create_client(SetEntityState, "/set_entity_state")
        self.get_logger().info("Waiting for /set_entity_state...")
        self.client.wait_for_service()

        self.x = 5.0
        self.center_y = -2.0
        self.z = 0.45

        # 3 m road = +/-1.5 m. A 1 m-wide obstacle is fully outside
        # when its center reaches +/-2.0 m from the road center.
        self.amplitude = 2.10
        self.speed = 0.20  # m/s

        # Triangle-wave period: 4*A / v
        self.period = 4.0 * self.amplitude / self.speed
        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def triangle(self, t):
        p = (t % self.period) / self.period
        if p < 0.25:
            return 4.0 * p
        if p < 0.75:
            return 2.0 - 4.0 * p
        return -4.0 + 4.0 * p

    def tick(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        offset = self.amplitude * self.triangle(t)

        state = EntityState()
        state.name = "sliding_obstacle"
        state.reference_frame = "world"
        state.pose.position.x = self.x
        state.pose.position.y = self.center_y + offset
        state.pose.position.z = self.z
        state.pose.orientation.w = 1.0

        req = SetEntityState.Request()
        req.state = state
        self.client.call_async(req)


def main():
    rclpy.init()
    node = SlidingObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
