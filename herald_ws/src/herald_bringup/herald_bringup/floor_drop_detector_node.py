#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32


class FloorDropDetectorNode(Node):
    def __init__(self):
        super().__init__('floor_drop_detector_node')

        self.declare_parameter('mount_height_m', 0.21)
        self.declare_parameter('phi_mount_deg', 15.0)  # shared geometry parameter
        self.declare_parameter('delta_margin_m', 0.10)
        self.declare_parameter('exclude_radius_m', 0.0)  # disabled by default

        self.h = self.get_parameter('mount_height_m').value
        self.delta = self.get_parameter('delta_margin_m').value
        self.exclude_radius = self.get_parameter('exclude_radius_m').value
        self.threshold_z = -(self.h + self.delta)

        self.get_logger().info(
            f'Floor-drop rule: flag when z_w < {self.threshold_z:.3f}m '
            f'(h={self.h:.3f}m, delta={self.delta:.3f}m)'
            + (f', excluding points within {self.exclude_radius:.2f}m of sensor'
               if self.exclude_radius > 0 else '')
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/scan_corrected', self._cloud_callback, 10)
        self.alert_pub = self.create_publisher(Bool, '/floor_drop_alert', 10)
        self.range_pub = self.create_publisher(Float32, '/floor_drop_measured_range', 10)

        self.get_logger().info('HERALD floor_drop_detector_node ready.')

    def _cloud_callback(self, msg: PointCloud2):
        flagged = False
        trigger_range = -1.0

        for x, y, z in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            if self.exclude_radius > 0 and math.sqrt(x * x + y * y + z * z) < self.exclude_radius:
                continue
            if z < self.threshold_z:
                flagged = True
                trigger_range = math.hypot(x, y)
                break

        self._publish_result(flagged, trigger_range)

    def _publish_result(self, flagged: bool, trigger_range: float):
        alert_msg = Bool()
        alert_msg.data = flagged
        self.alert_pub.publish(alert_msg)

        range_msg = Float32()
        range_msg.data = float(trigger_range)
        self.range_pub.publish(range_msg)

        if flagged:
            self.get_logger().warn(
                f'Floor drop alert: point below threshold at ~{trigger_range:.2f}m horizontal range '
                f'(z threshold {self.threshold_z:.3f}m)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FloorDropDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
