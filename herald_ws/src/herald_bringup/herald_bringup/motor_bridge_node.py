#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import serial


class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation_m', 0.60)  # track width -- measured outer-edge-to-outer-edge; true axle-center-to-center is slightly less, see chat note
        self.declare_parameter('wheel_radius_m', 0.04)      # JGB37-520 wheel, 8cm diameter
        self.declare_parameter('encoder_ppr', 11)            # pulses per MOTOR shaft rev
        self.declare_parameter('gear_ratio', 56.0)             # JGB37-520 178RPM variant, confirmed 56:1
        self.declare_parameter('max_linear_speed_mps', 0.5)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_mux_out')  # twist_mux output, NOT raw /cmd_vel

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self.wheel_sep = self.get_parameter('wheel_separation_m').value
        self.wheel_radius = self.get_parameter('wheel_radius_m').value
        self.encoder_ppr = self.get_parameter('encoder_ppr').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.ticks_per_output_rev = self.encoder_ppr * self.gear_ratio
        self.max_linear = self.get_parameter('max_linear_speed_mps').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.publish_tf = self.get_parameter('publish_tf').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Opened serial port {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise

        self.cmd_sub = self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value, self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self._lock = threading.Lock()

        self._stop_flag = False
        self._read_buffer = ''
        self.read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info('HERALD motor_bridge_node ready.')

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        v_left = linear - (angular * self.wheel_sep / 2.0)
        v_right = linear + (angular * self.wheel_sep / 2.0)

        left_pwm = self._speed_to_pwm(v_left)
        right_pwm = self._speed_to_pwm(v_right)

        cmd = f'M{left_pwm},{right_pwm}\n'
        try:
            self.ser.write(cmd.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write failed: {e}')

    def _speed_to_pwm(self, speed_mps: float) -> int:
        if self.max_linear <= 0:
            return 0
        ratio = speed_mps / self.max_linear
        pwm = int(max(-1.0, min(1.0, ratio)) * self.max_pwm)
        return pwm

    def _serial_read_loop(self):
        while not self._stop_flag and rclpy.ok():
            try:
                chunk = self.ser.read(256).decode('ascii', errors='ignore')
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial read failed: {e}')
                continue

            if not chunk:
                continue

            self._read_buffer += chunk
            while '\n' in self._read_buffer:
                line, self._read_buffer = self._read_buffer.split('\n', 1)
                self._handle_encoder_line(line.strip())

    def _handle_encoder_line(self, line: str):
        if not line.startswith('E'):
            return
        parts = line[1:].split(',')
        if len(parts) != 3:
            return
        try:
            dleft_ticks = int(parts[0])
            dright_ticks = int(parts[1])
            dt_ms = int(parts[2])
        except ValueError:
            return

        if dt_ms <= 0:
            return

        dt_s = dt_ms / 1000.0
        rev_per_tick = 1.0 / self.ticks_per_output_rev
        wheel_circumference = 2.0 * math.pi * self.wheel_radius

        d_left_m = dleft_ticks * rev_per_tick * wheel_circumference
        d_right_m = dright_ticks * rev_per_tick * wheel_circumference

        d_center = (d_left_m + d_right_m) / 2.0
        d_theta = (d_right_m - d_left_m) / self.wheel_sep

        with self._lock:
            mid_theta = self.theta + d_theta / 2.0
            self.x += d_center * math.cos(mid_theta)
            self.y += d_center * math.sin(mid_theta)
            self.theta += d_theta
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            x, y, theta = self.x, self.y, self.theta

        v_linear = d_center / dt_s
        v_angular = d_theta / dt_s

        self._publish_odom(x, y, theta, v_linear, v_angular)

    def _publish_odom(self, x, y, theta, v_linear, v_angular):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = math.sin(theta / 2.0)
            t.transform.rotation.w = math.cos(theta / 2.0)
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self._stop_flag = True
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                self.ser.write(b'M0,0\n')  # stop motors on shutdown
            except serial.SerialException:
                pass
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
