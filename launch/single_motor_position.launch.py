#!/usr/bin/env python3
"""
Launch file for single STS motor in position mode.

This launch file demonstrates basic usage of the STS Hardware Interface with a single motor
configured in position (servo) mode. It sets up the complete ros2_control stack including:
- Robot state publisher for TF transforms
- Controller manager for hardware interface lifecycle
- Joint state broadcaster for publishing joint states
- Joint trajectory controller for position commands

The launch file accepts command-line arguments for hardware configuration and supports
both real hardware and mock/simulation mode for testing without motors.

Example usage:
    ros2 launch sts_hardware_interface single_motor_position.launch.py serial_port:=/dev/ttyACM0
    ros2 launch sts_hardware_interface single_motor_position.launch.py use_mock:=true gui:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for single motor position mode demonstration.

    Returns
    -------
    LaunchDescription
        Complete launch configuration with all nodes and parameters

    """
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for STS motor communication'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'motor_id',
            default_value='1',
            description='Motor ID (1-253)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'baud_rate',
            default_value='1000000',
            description='Serial baud rate'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description='Use mock/simulation mode (no hardware required)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start joint_state_publisher_gui for manual control'
        )
    )

    # Initialize Arguments
    serial_port = LaunchConfiguration('serial_port')
    motor_id = LaunchConfiguration('motor_id')
    baud_rate = LaunchConfiguration('baud_rate')
    use_mock = LaunchConfiguration('use_mock')
    gui = LaunchConfiguration('gui')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('sts_hardware_interface'), 'config',
                 'single_motor_position.urdf.xacro']
            ),
            ' ',
            'serial_port:=', serial_port,
            ' ',
            'motor_id:=', motor_id,
            ' ',
            'baud_rate:=', baud_rate,
            ' ',
            'use_mock:=', use_mock,
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Controller configuration
    controller_config = PathJoinSubstitution(
        [FindPackageShare('sts_hardware_interface'), 'config', 'single_motor_position_controllers.yaml']
    )

    # Controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[robot_description, controller_config],
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Arm controller (joint trajectory controller for position mode)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    # Joint state publisher GUI (optional)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(gui),
    )

    nodes = [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        joint_state_publisher_gui_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
