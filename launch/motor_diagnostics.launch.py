#!/usr/bin/env python3
"""
Launch diagnostics node for STS motors (standalone or with hardware interface).

This launch file starts the motor_diagnostics_node with the default config.

Example usage:
    ros2 launch sts_hardware_interface motor_diagnostics.launch.py
    ros2 launch sts_hardware_interface motor_diagnostics.launch.py \
        config_file:=/path/to/custom.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('sts_hardware_interface'),
                'config', 'motor_diagnostics_config.yaml'
            ]),
            description='Path to diagnostics config YAML'
        )
    ]

    config_file = LaunchConfiguration('config_file')

    diagnostics_node = Node(
        package='sts_hardware_interface',
        executable='motor_diagnostics_node',
        name='motor_diagnostics_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription(declared_arguments + [diagnostics_node])
