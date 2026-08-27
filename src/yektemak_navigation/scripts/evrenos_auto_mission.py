#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy


class EvrenosAutoMission(Node):
    def __init__(self):
        super().__init__('evrenos_auto_mission')

        self.declare_parameter('initial_pose', [0.0, 0.0, math.pi])
        self.declare_parameter('waypoints', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('localization_wait_sec', 5.0)
        self.declare_parameter('loop', False)

        self.initial_pose = self.get_parameter('initial_pose').value
        waypoint_values = self.get_parameter('waypoints').value
        self.localization_wait_sec = float(
            self.get_parameter('localization_wait_sec').value
        )
        self.loop = bool(self.get_parameter('loop').value)

        if len(self.initial_pose) != 3:
            raise ValueError('initial_pose must contain [x, y, yaw]')
        if not waypoint_values or len(waypoint_values) % 3 != 0:
            raise ValueError('waypoints must contain x, y, yaw triples')

        self.waypoints = [
            self._pose(*waypoint_values[index:index + 3])
            for index in range(0, len(waypoint_values), 3)
        ]

        initial_pose_qos = QoSProfile(depth=1)
        initial_pose_qos.reliability = ReliabilityPolicy.RELIABLE
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            initial_pose_qos,
        )
        self.action_client = ActionClient(
            self,
            FollowWaypoints,
            '/follow_waypoints',
        )

        self.started = False
        self.initial_pose_publish_count = 0
        self.start_timer = self.create_timer(1.0, self._try_start)
        self.get_logger().info('AMCL ve Nav2 waypoint action server bekleniyor...')

    @staticmethod
    def _yaw_to_quaternion(yaw):
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z, pose.pose.orientation.w = (
            self._yaw_to_quaternion(float(yaw))
        )
        return pose

    def _publish_initial_pose(self):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(self.initial_pose[0])
        message.pose.pose.position.y = float(self.initial_pose[1])
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = (
            self._yaw_to_quaternion(float(self.initial_pose[2]))
        )
        message.pose.covariance[0] = 0.05
        message.pose.covariance[7] = 0.05
        message.pose.covariance[35] = 0.02
        self.initial_pose_publisher.publish(message)

    def _try_start(self):
        if self.started:
            return

        # AMCL needs an initial pose before the Nav2 lifecycle can finish.
        # Repeat it while Gazebo, odometry and AMCL are coming online.
        self._publish_initial_pose()
        self.initial_pose_publish_count += 1
        if self.initial_pose_publish_count == 1:
            self.get_logger().info('Başlangıç pozu AMCL için yayınlanıyor.')

        if not self.action_client.server_is_ready():
            return

        self.started = True
        self.start_timer.cancel()
        self._publish_initial_pose()
        self.get_logger().info(
            'Başlangıç pozu yayınlandı; lokalizasyonun oturması bekleniyor.'
        )
        self.mission_timer = self.create_timer(
            self.localization_wait_sec,
            self._send_mission_once,
        )

    def _send_mission_once(self):
        self.mission_timer.cancel()
        now = self.get_clock().now().to_msg()
        for waypoint in self.waypoints:
            waypoint.header.stamp = now

        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints
        self.get_logger().info(
            f'{len(self.waypoints)} waypoint Nav2 görevine gönderiliyor.'
        )
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 waypoint görevini reddetti.')
            return

        self.get_logger().info('Otomatik görev başladı.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        if result.missed_waypoints:
            self.get_logger().warning(
                f'Atlanan waypoint indeksleri: {list(result.missed_waypoints)}'
            )
        else:
            self.get_logger().info('Otomatik görev tamamlandı.')

        if self.loop:
            self.get_logger().info('loop=true: yeni tur başlatılıyor.')
            self.mission_timer = self.create_timer(2.0, self._send_mission_once)


def main(args=None):
    rclpy.init(args=args)
    node = EvrenosAutoMission()
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
