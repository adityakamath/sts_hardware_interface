# Quick Start Guide

Complete setup and basic usage instructions for the STS Hardware Interface.

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone <repository-url> sts_hardware_interface
cd sts_hardware_interface
git submodule update --init --recursive
```

### 2. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
source install/setup.bash
```

## Hardware Setup

### 1. Connect Motors

- Connect Feetech STS servos to USB-to-serial adapter
- Default baud rate: 1000000
- Ensure each motor has a unique ID (1-253)

### 2. Configure Motor IDs

Use the Feetech debugging software or vendor tools to:
- Assign unique IDs to each motor
- Set appropriate baud rate
- Verify motors respond to commands

### 3. Find Serial Port

```bash
# List available serial ports
ls /dev/tty*

# Common ports:
# - Linux: /dev/ttyUSB0, /dev/ttyACM0
# - macOS: /dev/tty.usbserial-*
```

## Basic Configuration

### Single Motor Example

Create a URDF file with ros2_control configuration:

```xml
<?xml version="1.0"?>
<robot name="single_motor_robot">
  <ros2_control name="sts_system" type="system">
    <hardware>
      <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
      <param name="serial_port">/dev/ttyUSB0</param>
      <param name="baud_rate">1000000</param>
    </hardware>

    <joint name="joint1">
      <param name="motor_id">1</param>
      <param name="operating_mode">1</param>  <!-- Velocity mode -->
      <command_interface name="velocity"/>
      <command_interface name="acceleration"/>
      <command_interface name="emergency_stop"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="load"/>
      <state_interface name="voltage"/>
      <state_interface name="temperature"/>
      <state_interface name="current"/>
    </joint>
  </ros2_control>
</robot>
```

### Controller Configuration

Create `controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

joint_state_broadcaster:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster

forward_command_controller:
  ros__parameters:
    type: forward_command_controller/ForwardCommandController
    joints:
      - joint1
    interface_name: velocity
```

## Launching the System

### 1. Load Robot Description

```bash
ros2 param set /robot_state_publisher robot_description \
  "$(xacro /path/to/your/robot.urdf.xacro)"
```

### 2. Start Controller Manager

```bash
ros2 run controller_manager ros2_control_node \
  --ros-args --params-file /path/to/controllers.yaml
```

### 3. Spawn Controllers

```bash
# Load and start joint state broadcaster
ros2 control load_controller --set-state active joint_state_broadcaster

# Load and start your command controller
ros2 control load_controller --set-state active forward_command_controller
```

## Testing

### 1. Verify Hardware Connection

```bash
ros2 control list_hardware_interfaces
```

Expected output shows available command and state interfaces for each joint.

### 2. Monitor Joint States

```bash
ros2 topic echo /joint_states
```

Should show real-time position, velocity, and effort for all joints.

### 3. Send Commands

**Velocity Mode**:

```bash
ros2 topic pub /forward_command_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [2.0]"  # 2.0 rad/s
```

**Position Mode** (operating_mode=0):

```bash
ros2 topic pub /forward_command_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [1.57]"  # π/2 radians
```

**PWM Mode** (operating_mode=2):

```bash
ros2 topic pub /forward_command_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [500.0]"  # PWM value
```

## Operating Modes

### Mode 0: Position (Servo)

- **Use Case**: Arm joints, precise positioning
- **Interfaces**: position, velocity, acceleration commands
- **Range**: ±π radians (with multi-turn: unlimited)

```xml
<joint name="arm_joint">
  <param name="motor_id">1</param>
  <param name="operating_mode">0</param>
  <param name="min_position">-3.14</param>
  <param name="max_position">3.14</param>
  <command_interface name="position"/>
  <command_interface name="velocity"/>  <!-- optional profile velocity -->
  <command_interface name="acceleration"/>  <!-- optional profile accel -->
</joint>
```

### Mode 1: Velocity (Wheel)

- **Use Case**: Continuous rotation (wheels, spinners)
- **Interfaces**: velocity, acceleration commands
- **Range**: Unlimited rotation

```xml
<joint name="wheel_joint">
  <param name="motor_id">2</param>
  <param name="operating_mode">1</param>
  <param name="max_velocity">15.0</param>  <!-- rad/s -->
  <command_interface name="velocity"/>
  <command_interface name="acceleration"/>
</joint>
```

### Mode 2: PWM (Effort)

- **Use Case**: Force/torque control, custom behaviors
- **Interfaces**: effort command
- **Range**: Motor-specific PWM values

```xml
<joint name="gripper_joint">
  <param name="motor_id">3</param>
  <param name="operating_mode">2</param>
  <command_interface name="effort"/>
</joint>
```

## Advanced Features

### Multi-Turn Tracking

Enable unlimited rotation in position mode:

```xml
<hardware>
  <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
  <param name="enable_multi_turn">true</param>
</hardware>
```

### Mock Mode (Simulation)

Test without hardware:

```xml
<hardware>
  <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
  <param name="enable_mock_mode">true</param>
</hardware>
```

### SyncWrite Optimization

For multiple motors, enable efficient batch writes:

```xml
<hardware>
  <param name="use_sync_write">true</param>  <!-- default -->
</hardware>
```

## Common Issues

### Motor Not Responding

1. **Check serial port**:
   ```bash
   ls -l /dev/ttyUSB0
   # Should show readable/writable permissions
   ```

2. **Add user to dialout group** (Linux):
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

3. **Verify motor ID**:
   - Use vendor software to confirm motor ID matches URDF
   - Check baud rate is consistent

### Position Jumps or Drift

- **Enable multi-turn tracking** if joints rotate beyond ±π
- **Check mechanical connection**: loose coupling can cause position errors
- **Verify position feedback**: echo `/joint_states` to see raw readings

### Communication Errors

- **Lower baud rate**: Try 115200 instead of 1000000
- **Check cable quality**: Use shielded USB cable
- **Reduce bus load**: Decrease update_rate in controller config

### Emergency Stop Not Working

- Emergency stop is broadcast (stops all motors), send to system:
  ```bash
  ros2 topic pub /motor_system/emergency_stop std_msgs/msg/Bool "data: true"
  ```
  (Replace `/motor_system` with your ros2_control system name)

## Next Steps

- See [architecture.md](architecture.md) for system design details
- Review CI/CD documentation for testing and quality assurance
- Integrate with your robot's controller stack
- Tune velocity/acceleration limits for your application

## Additional Resources

- [ros2_control documentation](https://control.ros.org/)
- [Feetech STS3215 manual](http://www.feetechrc.com)
- [SCServo protocol documentation](https://github.com/feetech-rc/SCServo_Linux)
