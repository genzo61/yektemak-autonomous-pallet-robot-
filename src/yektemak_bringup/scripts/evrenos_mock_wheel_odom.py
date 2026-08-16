#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry


class MockWheelOdom(Node):

    def __init__(self):
        super().__init__('evrenos_mock_wheel_odom')

        self.publisher = self.create_publisher(
            Odometry,
            '/wheel/odom',
            10
        )

        self.timer = self.create_timer(
            0.05,  # 20 Hz
            self.publish_odom
        )

        self.get_logger().warn(
            'MOCK wheel odometry enabled - DEVELOPMENT TEST ONLY'
        )

    def publish_odom(self):

        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        # Vehicle stationary at origin
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.angular.z = 0.0

        # Same channels currently fused by our EKF:
        # vx and yaw rate
        msg.twist.covariance[0] = 0.05
        msg.twist.covariance[35] = 0.10

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = MockWheelOdom()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()