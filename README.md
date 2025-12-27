# STS Hardware Interface

ROS2 hardware interface package for Feetech STS series servo motors.

## Overview

This package provides a `ros2_control` hardware interface for individual Feetech STS servo motors. It was extracted from the `lekiwi_ros2` package to create a reusable, standalone interface that can be used in any ROS2 robot system.

## Features

- **Complete STS Servo Support**: Works with all Feetech STS series servo motors
- **Three Operating Modes**:
  - Mode 0 (Servo): Position control with speed and acceleration limits
  - Mode 1 (Velocity): Closed-loop velocity control (default)
  - Mode 2 (PWM): Open-loop PWM/effort control
- **Full Diagnostics**: Position, velocity, load, voltage, temperature, current, and motion status
- **Shared Serial Bus**: Multiple motors can share a single serial connection
- **ros2_control Integration**: Seamless integration with the ros2_control framework

## Dependencies

- `rclcpp` and `rclcpp_lifecycle`
- `hardware_interface` and `pluginlib`
- `lekiwi_ros2` (for SCServo library)

## Usage

### URDF Configuration

```xml
<ros2_control name="my_motor" type="actuator">
  <hardware>
    <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="motor_id">1</param>
    <param name="baud_rate">1000000</param>
    <param name="operating_mode">1</param>  <!-- 0=servo, 1=velocity, 2=PWM -->
  </hardware>
  <joint name="my_joint">
    <!-- Command interfaces (mode-dependent) -->
    <command_interface name="velocity"/>      <!-- Mode 1, 0 -->
    <command_interface name="acceleration"/>  <!-- Mode 1, 0 -->
    <!-- <command_interface name="position"/>  Mode 0 only -->
    <!-- <command_interface name="effort"/>    Mode 2 only -->
    
    <!-- State interfaces (all modes) -->
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="load"/>
    <state_interface name="voltage"/>
    <state_interface name="temperature"/>
    <state_interface name="current"/>
    <state_interface name="is_moving"/>
  </joint>
</ros2_control>
```

### Parameters

- **serial_port** (required): Serial port path (e.g., `/dev/ttyACM0`)
- **motor_id** (required): Motor ID on the serial bus (1-253)
- **baud_rate** (optional): Communication baud rate (default: 1000000)
- **operating_mode** (optional): Motor operating mode (default: 1)
  - 0 = Servo (position control)
  - 1 = Velocity (closed-loop speed control)
  - 2 = PWM (open-loop effort control)

## State Interfaces (All Modes)

- **position**: Current position in radians
- **velocity**: Current velocity in rad/s
- **load**: Motor load/torque as percentage (-100.0 to +100.0)
- **voltage**: Supply voltage in volts
- **temperature**: Internal temperature in degrees Celsius
- **current**: Motor current draw in amperes
- **is_moving**: Motion status (1.0=moving, 0.0=stopped)

## Command Interfaces (Mode-Dependent)

### Mode 0 (Servo/Position Control)
- **position**: Target position in radians
- **velocity**: Maximum speed in rad/s
- **acceleration**: Acceleration value (0-254, 0=no limit)

### Mode 1 (Velocity Control)
- **velocity**: Target velocity in rad/s
- **acceleration**: Acceleration value (0-254, 0=no limit)

### Mode 2 (PWM/Effort Control)
- **effort**: PWM duty cycle (-1.0 to +1.0)

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
source install/setup.bash
```

## Migration from lekiwi_ros2

If you were previously using `lekiwi_ros2/STSActuatorInterface`, update your URDF:

**Before:**
```xml
<plugin>lekiwi_ros2/STSActuatorInterface</plugin>
```

**After:**
```xml
<plugin>sts_hardware_interface/STSHardwareInterface</plugin>
```

## License

Apache-2.0

## Author

Aditya Kamath (adityakamath@live.com)
