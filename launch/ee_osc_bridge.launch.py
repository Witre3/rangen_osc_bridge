#!/usr/bin/env python3
"""
Launch the ee_osc_bridge node, plus state_interpreter (which feeds it the
/rangen/elbow_state, /rangen/reach and /rangen/reach/discrete addresses).

Parameters are loaded from config/ee_osc_bridge.yaml and
config/state_interpreter.yaml.
Override osc_target_ip / osc_target_port via launch arguments:
  ros2 launch rangen_osc_bridge ee_osc_bridge.launch.py \
    osc_target_ip:=192.168.1.42 osc_target_port:=9001

Leave the state signals out (e.g. replaying with no TF tree) with:
  ros2 launch rangen_osc_bridge ee_osc_bridge.launch.py state_interpreter:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rangen_osc_bridge')
    config = os.path.join(share, 'config', 'ee_osc_bridge.yaml')
    state_config = os.path.join(share, 'config', 'state_interpreter.yaml')

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

    state_interpreter_arg = DeclareLaunchArgument(
        'state_interpreter',
        default_value='true',
        description='Also launch state_interpreter, which publishes the elbow_state / '
                    'reach / reach_discrete topics the bridge forwards as OSC',
    )

    def _launch_setup(context, *args, **kwargs):
        overrides = {}
        osc_target_ip = LaunchConfiguration('osc_target_ip').perform(context)
        osc_target_port = LaunchConfiguration('osc_target_port').perform(context)
        if osc_target_ip:
            overrides['osc_target_ip'] = osc_target_ip
        if osc_target_port:
            overrides['osc_target_port'] = int(osc_target_port)

        nodes = [
            Node(
                package='rangen_osc_bridge',
                executable='ee_osc_bridge',
                name='ee_osc_bridge',
                output='screen',
                parameters=[config, overrides],
            ),
        ]

        if LaunchConfiguration('state_interpreter').perform(context).lower() \
                not in ('false', '0', 'no'):
            nodes.append(Node(
                package='rangen_osc_bridge',
                executable='state_interpreter',
                name='state_interpreter',
                output='screen',
                parameters=[state_config],
            ))

        return nodes

    return LaunchDescription([
        osc_target_ip_arg,
        osc_target_port_arg,
        state_interpreter_arg,
        OpaqueFunction(function=_launch_setup),
    ])
