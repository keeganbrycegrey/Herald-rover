#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    herald_share = get_package_share_directory('herald_bringup')
    extrinsics_yaml = os.path.join(herald_share, 'config', 'sensor_extrinsics.yaml')

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel_mux_out',
        description='Twist topic motor_bridge_node listens to. Override to '
                     '/cmd_vel if running this launch file standalone without twist_mux.'
    )
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    try:
        ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')
        ydlidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ydlidar_share, 'launch', 'ydlidar_launch.py')
            )
        )
        ydlidar_included = True
    except Exception:
        ydlidar_launch = None
        ydlidar_included = False

    tilt_compensation_node = Node(
        package='herald_bringup',
        executable='tilt_compensation_node',
        name='tilt_compensation_node',
        output='screen',
        parameters=[
            extrinsics_yaml,
            {
                'i2c_bus': 1,
                'imu_update_hz': 100.0,
                'complementary_alpha': 0.98,
                'min_obstacle_z_m': -0.05,
                'max_obstacle_z_m': 0.5,
            },
        ],
    )

    floor_drop_node = Node(
        package='herald_bringup',
        executable='floor_drop_detector_node',
        name='floor_drop_detector_node',
        output='screen',
        parameters=[
            extrinsics_yaml,
            {
                'delta_margin_m': 0.10,
                'exclude_radius_m': 0.0,
            },
        ],
    )

    hazard_classification_node = Node(
        package='herald_bringup',
        executable='hazard_classification_node',
        name='hazard_classification_node',
        output='screen',
        parameters=[{
            'cluster_break_distance_m': 0.25,
            'min_cluster_points': 3,
            'wall_min_width_m': 1.0,
            'wall_max_line_residual_m': 0.03,
            'small_obstacle_max_width_m': 0.5,
            'publish_rate_hz': 5.0,
        }],
    )

    motor_bridge_node = Node(
        package='herald_bringup',
        executable='motor_bridge_node',
        name='motor_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyAMA0',
            'baud_rate': 115200,
            'wheel_separation_m': 0.60,
            'wheel_radius_m': 0.04,
            'encoder_ppr': 11,
            'gear_ratio': 56.0,
            'max_linear_speed_mps': 0.5,
            'max_pwm': 255,
            'publish_tf': True,
            'cmd_vel_topic': cmd_vel_topic,
        }],
    )

    actions = [cmd_vel_topic_arg]
    if ydlidar_included:
        actions.append(ydlidar_launch)
    actions.extend([tilt_compensation_node, floor_drop_node, hazard_classification_node, motor_bridge_node])

    return LaunchDescription(actions)
