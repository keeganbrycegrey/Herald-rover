#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    herald_share = get_package_share_directory('herald_bringup')

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(herald_share, 'launch', 'herald_bringup.launch.py')
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(herald_share, 'launch', 'slam.launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(herald_share, 'launch', 'nav2.launch.py')
        )
    )

    autonomy_manager_node = Node(
        package='herald_bringup',
        executable='autonomy_manager_node',
        name='autonomy_manager_node',
        output='screen',
        parameters=[{
            'explore_timeout_s': 300.0,
            'min_frontier_size_cells': 8,
            'nav_goal_timeout_s': 60.0,
            'default_mode': 'explore_then_navigate',
            'frontier_blacklist_radius_m': 0.5,
            'frontier_blacklist_cooldown_s': 30.0,
        }],
    )

    twist_mux_params = os.path.join(herald_share, 'config', 'twist_mux_params.yaml')
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params],
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}],
    )

    return LaunchDescription([
        base_bringup,
        slam_launch,
        nav2_launch,
        autonomy_manager_node,
        twist_mux_node,
        rosbridge_node,
    ])
