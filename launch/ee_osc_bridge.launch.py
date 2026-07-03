#!/usr/bin/env python3
"""
Launch the ee_osc_bridge node.

Parameters are loaded from config/ee_osc_bridge.yaml.
Override osc_target_ip / osc_target_port via launch arguments:
  ros2 launch rangen_osc_bridge ee_osc_bridge.launch.py \
    osc_target_ip:=192.168.1.42 osc_target_port:=9001
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rangen_osc_bridge'),
        'config', 'ee_osc_bridge.yaml',
    )

    osc_target_ip_arg = DeclareLaunchArgument(
        'osc_target_ip',
        default_value='',
        description='Override osc_target_ip from config/ee_osc_bridge.yaml (leave empty to use the config value)',
    )
    osc_target_port_arg = DeclareLaunchArgument(
        'osc_target_port',
        default_value='',
        description='Override osc_target_port from config/ee_osc_bridge.yaml (leave empty to use the config value)',
    )

    def _launch_setup(context, *args, **kwargs):
        overrides = {}
        osc_target_ip = LaunchConfiguration('osc_target_ip').perform(context)
        osc_target_port = LaunchConfiguration('osc_target_port').perform(context)
        if osc_target_ip:
            overrides['osc_target_ip'] = osc_target_ip
        if osc_target_port:
            overrides['osc_target_port'] = int(osc_target_port)

        return [
            Node(
                package='rangen_osc_bridge',
                executable='ee_osc_bridge',
                name='ee_osc_bridge',
                output='screen',
                parameters=[config, overrides],
            ),
        ]

    return LaunchDescription([
        osc_target_ip_arg,
        osc_target_port_arg,
        OpaqueFunction(function=_launch_setup),
    ])
