#!/usr/bin/env python3
import sys

import matplotlib.pyplot as plt
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def read_bag(bag_path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='sqlite3'),
        ConverterOptions('', '')
    )

    raw_points, corrected_points = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/scan':
            msg = deserialize_message(data, LaserScan)
            for i, r in enumerate(msg.ranges):
                if msg.range_min < r < msg.range_max:
                    theta = msg.angle_min + i * msg.angle_increment
                    raw_points.append((r * np.cos(theta), r * np.sin(theta)))
        elif topic == '/scan_corrected':
            msg = deserialize_message(data, PointCloud2)
            for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                corrected_points.append((p[0], p[2]))  # x vs world-referenced height/z

    return np.array(raw_points), np.array(corrected_points)


def plot_comparison(raw, corrected, out_path):
    if len(raw) == 0 or len(corrected) == 0:
        print(f'WARNING: raw has {len(raw)} points, corrected has {len(corrected)} points. '
              'Check the bag actually contains /scan and /scan_corrected messages '
              '(both nodes running during recording, correct topic names).')
        if len(raw) == 0 and len(corrected) == 0:
            print('Nothing to plot -- exiting without writing a figure.')
            return

    fig, axes = plt.subplots(1, 2, figsize=(10, 5), sharex=True, sharey=True)

    if len(raw) > 0:
        axes[0].scatter(raw[:, 0], raw[:, 1], s=3)
    axes[0].axhline(0, color='gray', linestyle='--', linewidth=0.8)
    axes[0].set_title('Raw (/scan)')

    if len(corrected) > 0:
        axes[1].scatter(corrected[:, 0], corrected[:, 1], s=3)
    axes[1].axhline(0, color='gray', linestyle='--', linewidth=0.8)
    axes[1].set_title('Corrected (/scan_corrected)')

    for ax in axes:
        ax.set_xlabel('X (m)')
        ax.set_aspect('equal')
    axes[0].set_ylabel('Y / Z (m)')

    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    print(f'Saved {out_path} ({len(raw)} raw points, {len(corrected)} corrected points)')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: python3 plot_raw_vs_corrected.py <bag_path> <output_png>')
        sys.exit(1)

    raw, corrected = read_bag(sys.argv[1])
    plot_comparison(raw, corrected, sys.argv[2])
