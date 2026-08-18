#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    herald_share = get_package_share_directory('herald_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(herald_share, 'config', 'nav2_params.yaml')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([navigation_launch])
