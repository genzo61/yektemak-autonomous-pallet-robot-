#!/usr/bin/env python3

import math
import struct
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import serial


class EvrenosSTM32Bridge(Node):

    CMD_HEADER = 0xCC
    ODOM_HEADER = 0xBB

    CMD_PACKET_SIZE = 14
    ODOM_PACKET_SIZE = 18

    def __init__(self):
        super().__init__('evrenos_stm32_bridge')

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/wheel/odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # Simulation value for now.
        # TODO: Measure / tune on real EVRENOS.
        self.declare_parameter('track_width', 1.065)

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('reconnect_interval', 2.0)

        # Wheel sign correction.
        # Keep +1 initially because forward encoder direction is expected positive.
        self.declare_parameter('right_rear_sign', 1.0)
        self.declare_parameter('right_front_sign', 1.0)
        self.declare_parameter('left_rear_sign', 1.0)
        self.declare_parameter('left_front_sign', 1.0)

        self.serial_port = str(
            self.get_parameter('serial_port').value
        )

        self.baudrate = int(
            self.get_parameter('baudrate').value
        )

        self.cmd_vel_topic = str(
            self.get_parameter('cmd_vel_topic').value
        )

        self.odom_topic = str(
            self.get_parameter('odom_topic').value
        )

        self.odom_frame = str(
            self.get_parameter('odom_frame').value
        )

        self.base_frame = str(
            self.get_parameter('base_frame').value
        )

        self.track_width = float(
            self.get_parameter('track_width').value
        )

        self.publish_rate = float(
            self.get_parameter('publish_rate').value
        )

        self.cmd_vel_timeout = float(
            self.get_parameter('cmd_vel_timeout').value
        )

        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval').value
        )

        self.right_rear_sign = float(
            self.get_parameter('right_rear_sign').value
        )

        self.right_front_sign = float(
            self.get_parameter('right_front_sign').value
        )

        self.left_rear_sign = float(
            self.get_parameter('left_rear_sign').value
        )

        self.left_front_sign = float(
            self.get_parameter('left_front_sign').value
        )

        # ============================================================
        # ROS interfaces
        # ============================================================

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            20
        )

        # ============================================================
        # Internal state
        # ============================================================

        self.serial_connection = None
        self.serial_lock = threading.Lock()

        self.rx_buffer = bytearray()
        self.last_reconnect_attempt = self.get_clock().now()

        self.command_linear_x = 0.0
        self.command_angular_z = 0.0

        # Turret disabled for navigation prototype.
        self.turret_pan_direction = 0
        self.turret_pan_speed = 0
        self.turret_tilt_direction = 0
        self.turret_tilt_speed = 0

        self.last_cmd_time = self.get_clock().now()

        # Integrated wheel odometry pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_odom_time = None

        self.connect_serial()

        period = 1.0 / self.publish_rate

        self.timer = self.create_timer(
            period,
            self.control_loop
        )

        self.get_logger().info(
            'EVRENOS STM32 bridge started'
        )

        self.get_logger().info(
            f'Serial: {self.serial_port} @ {self.baudrate}'
        )

        self.get_logger().info(
            f'cmd_vel: {self.cmd_vel_topic}'
        )

        self.get_logger().info(
            f'wheel odom: {self.odom_topic}'
        )

        self.get_logger().warn(
            f'Using prototype track_width={self.track_width:.3f} m. '
            'Measure/tune this on the real vehicle.'
        )

    # ================================================================
    # Serial
    # ================================================================

    def connect_serial(self):
        try:
            if self.serial_connection is not None:
                try:
                    if self.serial_connection.is_open:
                        self.serial_connection.close()
                except Exception:
                    pass

            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.0,
                write_timeout=0.1
            )

            self.serial_connection.reset_input_buffer()
            self.serial_connection.reset_output_buffer()

            self.rx_buffer.clear()

            self.get_logger().info(
                f'STM32 connected: {self.serial_port}'
            )

            return True

        except Exception as exc:
            self.serial_connection = None

            self.get_logger().warn(
                f'STM32 not available on {self.serial_port}: {exc}'
            )

            return False

    def disconnect_serial(self):

        if self.serial_connection is not None:
            try:
                if self.serial_connection.is_open:
                    self.serial_connection.close()
            except Exception:
                pass

        self.serial_connection = None
        self.rx_buffer.clear()

    # ================================================================
    # ROS cmd_vel
    # ================================================================

    def cmd_vel_callback(self, msg: Twist):
        self.command_linear_x = float(msg.linear.x)
        self.command_angular_z = float(msg.angular.z)

        self.last_cmd_time = self.get_clock().now()

    # ================================================================
    # Jetson -> STM32
    #
    # Packet:
    #
    # 0xCC
    # linear_x       float32
    # angular_z      float32
    # pan_direction  uint8
    # pan_speed      uint8
    # tilt_direction uint8
    # tilt_speed     uint8
    # checksum       uint8
    #
    # Total = 14 bytes
    # ================================================================

    def send_command(self, linear_x, angular_z):

        if (
            self.serial_connection is None
            or not self.serial_connection.is_open
        ):
            return

        try:
            packet_without_checksum = struct.pack(
                '<BffBBBB',
                self.CMD_HEADER,
                float(linear_x),
                float(angular_z),
                int(self.turret_pan_direction),
                int(self.turret_pan_speed),
                int(self.turret_tilt_direction),
                int(self.turret_tilt_speed)
            )

            # Same checksum logic as the supplied STM32 test script:
            # do not include header byte.
            checksum = (
                sum(packet_without_checksum[1:]) & 0xFF
            )

            packet = (
                packet_without_checksum
                + struct.pack('<B', checksum)
            )

            with self.serial_lock:
                self.serial_connection.write(packet)

        except Exception as exc:
            self.get_logger().error(
                f'STM32 write error: {exc}'
            )
            self.disconnect_serial()

    # ================================================================
    # STM32 -> Jetson
    #
    # Packet:
    #
    # 0xBB
    # right_rear   float32
    # right_front  float32
    # left_rear    float32
    # left_front   float32
    # checksum     uint8
    #
    # Total = 18 bytes
    # ================================================================

    def read_serial(self):

        if (
            self.serial_connection is None
            or not self.serial_connection.is_open
        ):
            return

        try:
            available = self.serial_connection.in_waiting

            if available > 0:
                with self.serial_lock:
                    data = self.serial_connection.read(available)

                self.rx_buffer.extend(data)

            self.parse_rx_buffer()

        except Exception as exc:
            self.get_logger().error(
                f'STM32 read error: {exc}'
            )
            self.disconnect_serial()

    def parse_rx_buffer(self):

        while len(self.rx_buffer) >= self.ODOM_PACKET_SIZE:

            # Find 0xBB packet header
            try:
                header_index = self.rx_buffer.index(
                    self.ODOM_HEADER
                )

            except ValueError:
                self.rx_buffer.clear()
                return

            # Remove garbage before header
            if header_index > 0:
                del self.rx_buffer[:header_index]

            if len(self.rx_buffer) < self.ODOM_PACKET_SIZE:
                return

            packet = bytes(
                self.rx_buffer[:self.ODOM_PACKET_SIZE]
            )

            payload = packet[1:17]
            received_checksum = packet[17]

            calculated_checksum = sum(payload) & 0xFF

            if calculated_checksum != received_checksum:
                # Invalid packet. Drop only current header and
                # search for the next valid 0xBB.
                del self.rx_buffer[0]

                self.get_logger().warn(
                    'STM32 odometry checksum mismatch',
                    throttle_duration_sec=2.0
                )

                continue

            try:
                (
                    right_rear,
                    right_front,
                    left_rear,
                    left_front
                ) = struct.unpack(
                    '<ffff',
                    payload
                )

            except struct.error:
                del self.rx_buffer[0]
                continue

            del self.rx_buffer[:self.ODOM_PACKET_SIZE]

            right_rear *= self.right_rear_sign
            right_front *= self.right_front_sign
            left_rear *= self.left_rear_sign
            left_front *= self.left_front_sign

            self.process_wheel_odometry(
                right_rear,
                right_front,
                left_rear,
                left_front
            )

    # ================================================================
    # Four-wheel skid-steer odometry
    # ================================================================

    def process_wheel_odometry(
        self,
        right_rear,
        right_front,
        left_rear,
        left_front
    ):

        now = self.get_clock().now()

        # Average wheels on each side
        right_velocity = (
            right_rear + right_front
        ) / 2.0

        left_velocity = (
            left_rear + left_front
        ) / 2.0

        linear_velocity = (
            right_velocity + left_velocity
        ) / 2.0

        angular_velocity = (
            right_velocity - left_velocity
        ) / self.track_width

        if self.last_odom_time is None:
            self.last_odom_time = now

            self.publish_odometry(
                now,
                linear_velocity,
                angular_velocity
            )

            return

        dt = (
            now - self.last_odom_time
        ).nanoseconds / 1e9

        self.last_odom_time = now

        # Guard against unreasonable timing jumps
        if dt <= 0.0 or dt > 0.5:
            self.publish_odometry(
                now,
                linear_velocity,
                angular_velocity
            )
            return

        # Midpoint integration gives slightly better differential
        # drive odometry during turns.
        delta_yaw = angular_velocity * dt

        heading_mid = self.yaw + (
            delta_yaw * 0.5
        )

        self.x += (
            linear_velocity
            * math.cos(heading_mid)
            * dt
        )

        self.y += (
            linear_velocity
            * math.sin(heading_mid)
            * dt
        )

        self.yaw += delta_yaw

        # Keep yaw in [-pi, pi]
        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw)
        )

        self.publish_odometry(
            now,
            linear_velocity,
            angular_velocity
        )

    # ================================================================
    # /wheel/odom
    #
    # IMPORTANT:
    # This node publishes Odometry only.
    #
    # It DOES NOT publish odom -> base_footprint TF.
    #
    # robot_localization EKF will own that TF.
    # ================================================================

    def publish_odometry(
        self,
        stamp,
        linear_velocity,
        angular_velocity
    ):

        msg = Odometry()

        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame

        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        half_yaw = self.yaw * 0.5

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        msg.twist.twist.linear.x = linear_velocity
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = angular_velocity

        # Prototype covariance.
        # Tune later using real encoder / skid-steer measurements.
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.10

        msg.twist.covariance[0] = 0.05
        msg.twist.covariance[7] = 0.05
        msg.twist.covariance[35] = 0.10

        self.odom_pub.publish(msg)

    # ================================================================
    # Main 20 Hz loop
    # ================================================================

    def control_loop(self):

        if (
        self.serial_connection is None
        or not self.serial_connection.is_open
        ):

            now = self.get_clock().now()

            reconnect_age = (
                now - self.last_reconnect_attempt
            ).nanoseconds / 1e9

            if reconnect_age >= self.reconnect_interval:

                self.last_reconnect_attempt = now

                self.get_logger().info(
                    f'Trying STM32 reconnect on {self.serial_port}...'
                )

                self.connect_serial()

            return

        # Read encoder packets first
        self.read_serial()

        now = self.get_clock().now()

        command_age = (
            now - self.last_cmd_time
        ).nanoseconds / 1e9

        # Safety watchdog:
        # no recent cmd_vel -> STOP
        if command_age > self.cmd_vel_timeout:
            linear_x = 0.0
            angular_z = 0.0

        else:
            linear_x = self.command_linear_x
            angular_z = self.command_angular_z

        self.send_command(
            linear_x,
            angular_z
        )

    # ================================================================

    def shutdown(self):
        self.get_logger().info(
            'Stopping EVRENOS STM32 bridge...'
        )

        # Try to send explicit STOP before closing serial.
        try:
            for _ in range(3):
                self.send_command(0.0, 0.0)

        except Exception:
            pass

        if (
            self.serial_connection is not None
            and self.serial_connection.is_open
        ):
            self.serial_connection.close()


def main(args=None):

    rclpy.init(args=args)

    node = EvrenosSTM32Bridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()