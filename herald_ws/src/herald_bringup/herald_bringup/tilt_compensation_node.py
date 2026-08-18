#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

import smbus2 as smbus

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

ACCEL_SCALE = 16384.0   # LSB/g at +-2g range
GYRO_SCALE = 131.0      # LSB/(deg/s) at +-250 dps range
GRAVITY_MPS2 = 9.80665


def read_word_2c(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        val -= 0x10000
    return val


class TiltCompensationNode(Node):
    def __init__(self):
        super().__init__('tilt_compensation_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('mpu_address', MPU6050_ADDR)
        self.declare_parameter('imu_update_hz', 100.0)
        self.declare_parameter('complementary_alpha', 0.98)
        self.declare_parameter('phi_mount_deg', 15.0)   # measured, hardware-fixed bracket angle
        self.declare_parameter('mount_height_m', 0.21)  # as-built sensor height, measured to scan origin

        self.declare_parameter('min_obstacle_z_m', -0.05)  # points below this (near-ground) treated as floor, not obstacle
        self.declare_parameter('max_obstacle_z_m', 0.5)    # points above this ignored (out of relevant height band)

        bus_num = self.get_parameter('i2c_bus').value
        self.mpu_addr = self.get_parameter('mpu_address').value
        self.alpha = self.get_parameter('complementary_alpha').value
        self.mount_pitch_offset = math.radians(
            self.get_parameter('phi_mount_deg').value)
        self.mount_height_m = self.get_parameter('mount_height_m').value  # not used in rotation math here; accepted for consistency with the shared extrinsics file
        self.min_obstacle_z = self.get_parameter('min_obstacle_z_m').value
        self.max_obstacle_z = self.get_parameter('max_obstacle_z_m').value

        self.bus = smbus.SMBus(bus_num)
        self._init_mpu()

        self.roll = 0.0
        self.pitch = 0.0
        self.last_imu_time = time.time()

        imu_hz = self.get_parameter('imu_update_hz').value
        self.imu_timer = self.create_timer(1.0 / imu_hz, self._update_filter)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/scan_corrected', 10)
        self.scan_level_pub = self.create_publisher(
            LaserScan, '/scan_level', 10)
        self.imu_raw_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.imu_filtered_pub = self.create_publisher(Vector3, '/imu/filtered_pitch_roll', 10)

        self.get_logger().info('HERALD tilt_compensation_node ready.')

    def _init_mpu(self):
        self.bus.write_byte_data(self.mpu_addr, PWR_MGMT_1, 0)

    def _read_imu_raw(self):
        ax = read_word_2c(self.bus, self.mpu_addr, ACCEL_XOUT_H) / ACCEL_SCALE
        ay = read_word_2c(self.bus, self.mpu_addr, ACCEL_XOUT_H + 2) / ACCEL_SCALE
        az = read_word_2c(self.bus, self.mpu_addr, ACCEL_XOUT_H + 4) / ACCEL_SCALE
        gx = read_word_2c(self.bus, self.mpu_addr, GYRO_XOUT_H) / GYRO_SCALE
        gy = read_word_2c(self.bus, self.mpu_addr, GYRO_XOUT_H + 2) / GYRO_SCALE
        return ax, ay, az, gx, gy

    def _update_filter(self):
        try:
            ax, ay, az, gx, gy = self._read_imu_raw()
        except OSError as e:
            self.get_logger().warn(f'MPU6050 read failed: {e}')
            return

        now = time.time()
        dt = now - self.last_imu_time
        self.last_imu_time = now
        if dt <= 0:
            return

        accel_roll = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        gyro_roll = self.roll + gx * dt
        gyro_pitch = self.pitch + gy * dt

        self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch

        self._publish_imu(ax, ay, az, gx, gy)

    def _publish_imu(self, ax_g, ay_g, az_g, gx_dps, gy_dps):
        stamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.linear_acceleration.x = ax_g * GRAVITY_MPS2
        imu_msg.linear_acceleration.y = ay_g * GRAVITY_MPS2
        imu_msg.linear_acceleration.z = az_g * GRAVITY_MPS2
        imu_msg.angular_velocity.x = math.radians(gx_dps)
        imu_msg.angular_velocity.y = math.radians(gy_dps)
        imu_msg.angular_velocity.z = 0.0  # no yaw-rate axis read/used by this filter
        imu_msg.orientation_covariance[0] = -1.0
        self.imu_raw_pub.publish(imu_msg)

        filtered_msg = Vector3()
        filtered_msg.x = self.roll   # degrees
        filtered_msg.y = self.pitch  # degrees
        filtered_msg.z = 0.0
        self.imu_filtered_pub.publish(filtered_msg)

    def _rotation_matrix(self, roll_rad, pitch_rad):
        cr, sr = math.cos(roll_rad), math.sin(roll_rad)
        cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)

        r00 = cp
        r01 = sp * sr
        r02 = sp * cr
        r10 = 0.0
        r11 = cr
        r12 = -sr
        r20 = -sp
        r21 = cp * sr
        r22 = cp * cr
        return ((r00, r01, r02), (r10, r11, r12), (r20, r21, r22))

    def _scan_callback(self, msg: LaserScan):
        roll_rad = math.radians(self.roll)
        pitch_rad = math.radians(self.pitch) + self.mount_pitch_offset

        rot = self._rotation_matrix(roll_rad, pitch_rad)

        points = []
        level_ranges = [msg.range_max] * len(msg.ranges)

        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                x_l = r * math.cos(angle)
                y_l = r * math.sin(angle)
                z_l = 0.0

                x = rot[0][0] * x_l + rot[0][1] * y_l + rot[0][2] * z_l
                y = rot[1][0] * x_l + rot[1][1] * y_l + rot[1][2] * z_l
                z = rot[2][0] * x_l + rot[2][1] * y_l + rot[2][2] * z_l

                points.append((x, y, z))

                if self.min_obstacle_z <= z <= self.max_obstacle_z:
                    flat_range = math.hypot(x, y)
                    if msg.range_min <= flat_range <= msg.range_max:
                        level_ranges[i] = min(level_ranges[i], flat_range)
            angle += msg.angle_increment

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = point_cloud2.create_cloud(header, fields, points)
        self.cloud_pub.publish(cloud_msg)

        level_scan = LaserScan()
        level_scan.header = header
        level_scan.angle_min = msg.angle_min
        level_scan.angle_max = msg.angle_max
        level_scan.angle_increment = msg.angle_increment
        level_scan.time_increment = msg.time_increment
        level_scan.scan_time = msg.scan_time
        level_scan.range_min = msg.range_min
        level_scan.range_max = msg.range_max
        level_scan.ranges = level_ranges
        self.scan_level_pub.publish(level_scan)


def main(args=None):
    rclpy.init(args=args)
    node = TiltCompensationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
