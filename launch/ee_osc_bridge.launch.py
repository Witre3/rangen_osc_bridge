#!/usr/bin/env python3
"""
Launch the ee_osc_bridge node.

Parameters are loaded from config/ee_osc_bridge.yaml.
Override individual params via --ros-args:
  ros2 launch rangen_osc_bridge ee_osc_bridge.launch.py \
    --ros-args -p osc_target_ip:=192.168.1.42 -p osc_target_port:=9001
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rangen_osc_bridge'),
        'config', 'ee_osc_bridge.yaml',
    )

    return LaunchDescription([
        Node(
            package='rangen_osc_bridge',
            executable='ee_osc_bridge',
            name='ee_osc_bridge',
            output='screen',
            parameters=[config],
        ),
    ])
