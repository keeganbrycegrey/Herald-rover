#!/usr/bin/env python3
import csv
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, Bool, Float32

LOG_PATH = os.path.expanduser('~/herald_hazard_log.csv')


class HazardClassificationNode(Node):
    def __init__(self):
        super().__init__('hazard_classification_node')

        self.declare_parameter('cluster_break_distance_m', 0.25)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('wall_min_width_m', 1.0)
        self.declare_parameter('wall_max_line_residual_m', 0.03)
        self.declare_parameter('small_obstacle_max_width_m', 0.5)
        self.declare_parameter('publish_rate_hz', 5.0)

        self.cluster_break = self.get_parameter('cluster_break_distance_m').value
        self.min_cluster_points = self.get_parameter('min_cluster_points').value
        self.wall_min_width = self.get_parameter('wall_min_width_m').value
        self.wall_max_residual = self.get_parameter('wall_max_line_residual_m').value
        self.small_obstacle_max_width = self.get_parameter('small_obstacle_max_width_m').value

        self.latest_scan = None
        self.floor_drop_active = False
        self.floor_drop_range = None

        self.scan_sub = self.create_subscription(
            PointCloud2, '/scan_corrected', self._cloud_callback, 10)
        self.drop_sub = self.create_subscription(
            Bool, '/floor_drop_alert', self._drop_callback, 10)
        self.drop_range_sub = self.create_subscription(
            Float32, '/floor_drop_measured_range', self._drop_range_callback, 10)

        self.hazards_pub = self.create_publisher(String, '/hazards', 10)

        self._init_log()
        self._last_logged_types = set()

        rate = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self._process_and_publish)

        self.get_logger().info('HERALD hazard_classification_node ready (geometric classification).')

    def _init_log(self):
        if not os.path.exists(LOG_PATH):
            with open(LOG_PATH, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['timestamp', 'hazard_type', 'centroid_x', 'centroid_y', 'width_m', 'range_m'])

    def _log_hazard(self, hazard: dict):
        with open(LOG_PATH, 'a', newline='') as f:
            csv.writer(f).writerow([
                time.time(), hazard['type'],
                hazard.get('centroid_x', ''), hazard.get('centroid_y', ''),
                hazard.get('width_m', ''), hazard.get('range_m', ''),
            ])

    def _cloud_callback(self, msg: PointCloud2):
        self.latest_scan = msg

    def _drop_callback(self, msg: Bool):
        self.floor_drop_active = msg.data

    def _drop_range_callback(self, msg: Float32):
        self.floor_drop_range = msg.data

    def _cloud_to_points(self, msg: PointCloud2):
        points = []
        for x, y, z in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points.append((float(x), float(y), float(z)))
        return points

    def _cluster_points(self, points):
        clusters = []
        current = []
        prev_pt = None
        for pt in points:
            if prev_pt is not None:
                d = math.hypot(pt[0] - prev_pt[0], pt[1] - prev_pt[1])
                if d > self.cluster_break:
                    if len(current) >= self.min_cluster_points:
                        clusters.append(current)
                    current = []
            current.append(pt)
            prev_pt = pt
        if len(current) >= self.min_cluster_points:
            clusters.append(current)
        return clusters

    def _classify_cluster(self, cluster):
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        crange = math.hypot(cx, cy)

        x0, y0 = cluster[0][0], cluster[0][1]
        x1, y1 = cluster[-1][0], cluster[-1][1]
        width = math.hypot(x1 - x0, y1 - y0)

        residual = 0.0
        if width > 1e-6:
            dx, dy = (x1 - x0) / width, (y1 - y0) / width
            max_dev = 0.0
            for px, py, _ in cluster:
                t = (px - x0) * dx + (py - y0) * dy
                proj_x, proj_y = x0 + t * dx, y0 + t * dy
                dev = math.hypot(px - proj_x, py - proj_y)
                max_dev = max(max_dev, dev)
            residual = max_dev

        if width >= self.wall_min_width and residual <= self.wall_max_residual:
            htype = 'wall'
        elif width < self.small_obstacle_max_width:
            htype = 'obstacle'
        else:
            htype = 'unclassified_structure'

        return {
            'type': htype,
            'centroid_x': round(cx, 3),
            'centroid_y': round(cy, 3),
            'width_m': round(width, 3),
            'range_m': round(crange, 3),
            'point_count': len(cluster),
        }

    def _process_and_publish(self):
        hazards = []

        if self.latest_scan is not None:
            points = self._cloud_to_points(self.latest_scan)
            clusters = self._cluster_points(points)
            for cluster in clusters:
                hazards.append(self._classify_cluster(cluster))

        if self.floor_drop_active:
            hazards.append({
                'type': 'void',
                'range_m': round(self.floor_drop_range, 3) if self.floor_drop_range is not None else None,
                'centroid_x': None,
                'centroid_y': None,
                'width_m': None,
                'point_count': None,
            })

        current_types = set(h['type'] for h in hazards)
        if current_types != self._last_logged_types:
            for h in hazards:
                self._log_hazard(h)
            self._last_logged_types = current_types

        msg = String()
        msg.data = json.dumps(hazards)
        self.hazards_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HazardClassificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
