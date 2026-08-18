#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    herald_share = get_package_share_directory('herald_bringup')
    params_file = os.path.join(herald_share, 'config', 'slam_toolbox_params.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}],
    )

    return LaunchDescription([slam_node])
