# Quick Start Guide

Complete setup and usage instructions for the STS Hardware Interface.

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/sts_hardware_interface.git
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

# Common ports: /dev/ttyUSB0, /dev/ttyACM0
```

## Running the Examples

This package provides ready-to-use example launch files that demonstrate different use cases.

### Example 1: Single Motor (Velocity Mode)

The simplest example with one motor in velocity mode.

**What it includes:**
- URDF: [config/single_motor.urdf.xacro](../config/single_motor.urdf.xacro)
- Controllers: [config/velocity_controller.yaml](../config/velocity_controller.yaml)
- Launch file: [launch/single_motor.launch.py](../launch/single_motor.launch.py)

**How to run:**

```bash
# With real hardware
ros2 launch sts_hardware_interface single_motor.launch.py serial_port:=/dev/ttyACM0

# With custom motor ID (default is 1)
ros2 launch sts_hardware_interface single_motor.launch.py serial_port:=/dev/ttyACM0 motor_id:=5

# In mock mode (no hardware required)
ros2 launch sts_hardware_interface single_motor.launch.py use_mock:=true
```

**What it does:**
1. Loads robot description with one continuous joint (`wheel_joint`)
2. Starts `ros2_control` node with velocity controller
3. Spawns `joint_state_broadcaster` and `velocity_controller`

**Test the motor:**

```bash
# Command velocity (rad/s)
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [2.0]"

# Monitor joint state
ros2 topic echo /joint_states
```

### Example 2: Mixed Mode (Multiple Motors)

Demonstrates three motors in different operating modes on the same serial bus.

**What it includes:**
- URDF: [config/mixed_mode.urdf.xacro](../config/mixed_mode.urdf.xacro)
- Controllers: [config/mixed_mode_controllers.yaml](../config/mixed_mode_controllers.yaml)
- Launch file: [launch/mixed_mode.launch.py](../launch/mixed_mode.launch.py)

**Motors configured:**
- Motor 1 (`wheel_joint`): **Mode 1** - Velocity control
- Motor 2 (`arm_joint`): **Mode 0** - Position/servo control
- Motor 3 (`gripper_joint`): **Mode 2** - PWM/effort control

**How to run:**

```bash
# With real hardware (requires 3 motors with IDs 1, 2, 3)
ros2 launch sts_hardware_interface mixed_mode.launch.py serial_port:=/dev/ttyACM0

# In mock mode
ros2 launch sts_hardware_interface mixed_mode.launch.py use_mock:=true
```

**What it does:**
1. Loads robot description with three joints in different modes
2. Starts `ros2_control` node with SyncWrite enabled for efficiency
3. Spawns multiple controllers:
   - `joint_state_broadcaster` - publishes joint states
   - `wheel_controller` - velocity control for wheel
   - `arm_controller` - trajectory control for arm
   - `gripper_controller` - effort control for gripper

**Test each motor:**

```bash
# Wheel (velocity command)
ros2 topic pub /wheel_controller/commands std_msgs/msg/Float64MultiArray "data: [3.0]"

# Arm (position trajectory)
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: ['arm_joint'],
      points: [{positions: [1.0], time_from_start: {sec: 2}}]
    }
  }"

# Gripper (effort command)
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]"
```

### Launch File Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `serial_port` | string | `/dev/ttyACM0` | Serial port path |
| `baud_rate` | int | `1000000` | Communication baud rate |
| `use_mock` | bool | `false` | Enable mock mode (no hardware) |
| `motor_id` | int | `1` | Motor ID on serial bus (single_motor.launch.py only) |

## Testing

### 1. Verify Hardware Connection

```bash
ros2 control list_hardware_interfaces
```

Expected output shows available command and state interfaces for each joint.

### 2. Monitor Joint States

**Standard state** (`/joint_states`):

```bash
ros2 topic echo /joint_states  # position, velocity, effort
```

**Additional state** (`/dynamic_joint_states`):

```bash
ros2 topic echo /dynamic_joint_states  # voltage, temperature, current, is_moving
```

To enable `/dynamic_joint_states`, add to your controller YAML:

```yaml
joint_state_broadcaster:
  ros__parameters:
    extra_joints: [wheel_joint, arm_joint]
```

See [config/mixed_mode_controllers.yaml](../config/mixed_mode_controllers.yaml) for complete example.

## Configuration

For detailed configuration options, operating modes, parameters, and advanced features, see the [Architecture Guide](ARCHITECTURE.md).

## Additional Resources

- [ros2_control documentation](https://control.ros.org/)
- [Feetech STS3215 documentation](https://www.feetechrc.com/2020-05-13_56655.html)
- [Original FTServo_Linux repository](https://github.com/ftservo/FTServo_Linux)
- [My SCServo_Linux fork](https://github.com/adityakamath/SCServo_Linux)
