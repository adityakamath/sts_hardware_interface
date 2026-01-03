# STS Hardware Interface Architecture

System design and configuration guide for the Feetech STS servo motor hardware interface.

## Overview

The STS Hardware Interface is a `ros2_control` SystemInterface plugin that connects ROS 2 controllers to Feetech STS series servo motors. It provides position, velocity, and effort control modes with configurable parameters.

## System Architecture

```mermaid
flowchart TB
    subgraph ROS2["ROS 2 Controllers"]
        C1[Position Controller]
        C2[Velocity Controller]
        C3[Effort Controller]
    end

    subgraph HWI["STS Hardware Interface"]
        SI[SystemInterface Plugin]
    end

    subgraph Hardware["Physical Motors"]
        M1[Motor 1 - Position Mode]
        M2[Motor 2 - Velocity Mode]
        M3[Motor 3 - PWM Mode]
    end

    C1 --> SI
    C2 --> SI
    C3 --> SI
    SI <--> M1
    SI <--> M2
    SI <--> M3
```

## Operating Modes

| Mode | Use Case | Command Interfaces | Position Limits |
|------|----------|-------------------|-----------------|
| **0: Position** | Arm joints, precise positioning | position, velocity†, acceleration† | 0 to 2π radians (configurable) |
| **1: Velocity** | Wheels, continuous rotation | velocity, acceleration† | Unlimited |
| **2: PWM/Effort** | Force control, grippers | effort (-1.0 to +1.0) | N/A |

† Optional interfaces

**Example configuration:**

```xml
<!-- Mode 0: Position -->
<joint name="arm_joint">
  <param name="motor_id">1</param>
  <param name="operating_mode">0</param>
  <param name="min_position">0.0</param>
  <param name="max_position">6.283</param>
</joint>

<!-- Mode 1: Velocity -->
<joint name="wheel_joint">
  <param name="motor_id">2</param>
  <param name="operating_mode">1</param>
  <param name="max_velocity">15.0</param>
</joint>

<!-- Mode 2: PWM -->
<joint name="gripper_joint">
  <param name="motor_id">3</param>
  <param name="operating_mode">2</param>
</joint>
```

## State Interfaces

All modes provide the following optional state interfaces. Configure only what you need:

| Interface | Unit | Description |
|-----------|------|-------------|
| `position` | radians | Current joint angle |
| `velocity` | rad/s | Current angular velocity |
| `load` | -100 to +100% | Motor load/torque |
| `voltage` | volts | Supply voltage |
| `temperature` | °C | Motor temperature |
| `current` | amperes | Motor current draw |
| `is_moving` | 0.0 or 1.0 | Motion status |

### Accessing State Interfaces

The `joint_state_broadcaster` publishes motor state to two topics:

**Standard state (`/joint_states`):**

- Published as `sensor_msgs/JointState`
- Contains `position`, `velocity`, and `effort` fields
- Always available

**Additional state (`/dynamic_joint_states`):**

- Published as `control_msgs/DynamicJointState`
- Contains all other state interfaces (load, voltage, temperature, current, is_moving)
- Requires configuration in controller YAML:

```yaml
joint_state_broadcaster:
  ros__parameters:
    extra_joints:
      - wheel_joint
      - arm_joint
```

See [config/mixed_mode_controllers.yaml](../config/mixed_mode_controllers.yaml) for a complete example.

## Hardware Parameters

Configure these at the system level in your URDF:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | *required* | Serial port path (e.g., `/dev/ttyACM0`) |
| `baud_rate` | int | 1000000 | Communication baud rate (9600-1000000) |
| `communication_timeout_ms` | int | 100 | Serial timeout (1-1000 ms) |
| `use_sync_write` | bool | true | Batch commands for multiple motors |
| `enable_mock_mode` | bool | false | Simulation mode (no hardware required) |

## Joint Parameters

Configure these per joint:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_id` | int | *required* | Motor ID on serial bus (1-253) |
| `operating_mode` | int | 1 | 0=Position, 1=Velocity, 2=PWM |
| `min_position` | double | 0.0 | Min position limit (Mode 0 only) |
| `max_position` | double | 6.283 | Max position limit (2π radians, Mode 0 only) |
| `max_velocity` | double | 5.216 | Max velocity (3400 steps/s × 2π/4096 ≈ 5.216 rad/s) |
| `max_effort` | double | 1.0 | Max PWM duty cycle (Mode 2 only) |

## Unit Conversions

The STS motors use step-based units internally (4096 steps per revolution). The hardware interface converts between motor units and ROS 2 standard units (radians, rad/s):

- **Position:** 4096 steps = 2π radians (one full revolution)
  - Conversion: `radians = steps × (2π / 4096)`
- **Velocity:** Maximum 3400 steps/s = 5.216 rad/s
  - Conversion: `rad/s = steps/s × (2π / 4096)`
- **Default max_velocity:** 3400 steps/s × (2π / 4096) ≈ 5.216 rad/s

## Communication

- **Protocol:** Feetech STS/SCServo packet format
- **Bus type:** Half-duplex RS485 or TTL serial
- **Topology:** Daisy-chain (all motors on one bus, unique IDs 1-253)
- **SyncWrite:** Batches commands to multiple motors (enabled by default, improves timing)

## Additional Features

### Emergency Stop

Broadcast command that stops all motors simultaneously:

```bash
ros2 topic pub /sts_system/emergency_stop std_msgs/msg/Bool "data: true"   # activate
ros2 topic pub /sts_system/emergency_stop std_msgs/msg/Bool "data: false"  # release
```

### Mock Mode

Test without hardware by setting `enable_mock_mode: true` in hardware configuration.

## ROS 2 Controller Compatibility

| Controller | Use Case | Compatible Modes |
|------------|----------|------------------|
| `JointTrajectoryController` | Arm manipulation | Mode 0 |
| `JointGroupVelocityController` | Wheels, continuous motion | Mode 1 |
| `JointGroupEffortController` | Force control, grippers | Mode 2 |
| `DiffDriveController` | Differential drive | Mode 1 |
| `ForwardCommandController` | Direct control, testing | All modes |

## Example Configurations

See [config/single_motor.urdf.xacro](../config/single_motor.urdf.xacro) and [config/mixed_mode.urdf.xacro](../config/mixed_mode.urdf.xacro) for complete examples.

## Troubleshooting

| Issue | Solutions |
|-------|-----------|
| **Motors not responding** | • Verify `serial_port` path and permissions<br>• Confirm `motor_id` matches physical motor<br>• Check `baud_rate` matches motor config<br>• Test with `enable_mock_mode: true` |
| **Position drift/jumps** | • Verify `min_position`/`max_position` range (default: 0 to 2π)<br>• Check position limits match mechanism |
| **Communication errors** | • Decrease controller `update_rate`<br>• Reduce number of state interfaces<br>• Test with single motor first<br>• Check cable quality and bus termination |

## Further Reading

- [Quick Start Guide](quick-start.md) - Setup and usage examples
- [ros2_control Documentation](https://control.ros.org/)
- [Feetech STS3215 Documentation](https://www.feetechrc.com/2020-05-13_56655.html)
