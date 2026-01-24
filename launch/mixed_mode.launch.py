#!/usr/bin/env python3

"""Launch file for mixed-mode motor chain (wheel + arm + gripper).

This launch file demonstrates advanced usage of the STS Hardware Interface with multiple
motors operating in different control modes on the same serial bus:
- Wheel joint in velocity (Mode 1) for continuous rotation
- Arm joint in position/servo (Mode 0) for precise positioning
- Gripper joint in PWM/effort (Mode 2) for force control

The configuration showcases:
- Mixed-mode operation: Different motors in different operating modes
- Multi-motor coordination: SyncWrite for efficient bus communication
- Multiple controller types: Position, velocity, and effort controllers
- Complex robot configurations: Combining different actuator types

Example usage:
    ros2 launch sts_hardware_interface mixed_mode.launch.py serial_port:=/dev/ttyACM0
    ros2 launch sts_hardware_interface mixed_mode.launch.py use_mock:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for mixed-mode motor chain demonstration.

    Returns:
        LaunchDescription: Complete launch configuration with all nodes and parameters
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

    # Initialize Arguments
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_mock = LaunchConfiguration('use_mock')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('sts_hardware_interface'), 'config', 'mixed_mode.urdf.xacro']
            ),
            ' ',
            'serial_port:=', serial_port,
            ' ',
            'baud_rate:=', baud_rate,
            ' ',
            'use_mock:=', use_mock,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Controller configuration
    controller_config = PathJoinSubstitution(
        [FindPackageShare('sts_hardware_interface'), 'config', 'mixed_mode_controllers.yaml']
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

    # Joint trajectory controller (for arm in servo mode)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    # Velocity controller (for wheel in velocity mode)
    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_controller', '--controller-manager', '/controller_manager'],
    )

    # Effort controller (for gripper in PWM mode)
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
    )

    # Delay controllers start after joint state broadcaster
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_wheel_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[wheel_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    nodes = [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller,
        delay_wheel_controller,
        delay_gripper_controller,
    ]

    return LaunchDescription(declared_arguments + nodes)
