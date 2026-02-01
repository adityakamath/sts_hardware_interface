# STS Hardware Interface

![Project Status](https://img.shields.io/badge/Status-WIP-yellow)
![ROS 2](https://img.shields.io/badge/ROS%202-Kilted%20(Ubuntu%2024.04)-blue?style=flat&logo=ros&logoSize=auto)
![ROS 2 Control](https://img.shields.io/badge/ros2__control-SystemInterface-blue?style=flat&logo=ros&logoSize=auto)
![Website](https://img.shields.io/badge/Website-kamathrobotics.com-darkorange?style=flat&logo=hashnode&logoSize=auto&link=https%3A%2F%2Fkamathrobotics.com&link=https%3A%2F%2Fkamathrobotics.com)
[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/sts_hardware_interface)
![License](https://img.shields.io/github/license/adityakamath/sts_hardware_interface?label=License)

> `ros2_control` `SystemInterface` for Feetech STS series servo motors (STS3215 and compatible).

**⚠️ Status:** Only **Mode 1 (Velocity)** has been tested. Modes 0 (Position) and 2 (PWM) are experimental.

## Features

- **Scalable Multi-Motor Support**: Control 1 to 253 motors on a single serial bus
- **Three Operating Modes**: Position (servo, closed-loop), Velocity (wheel, closed-loop), and PWM (effort, open-loop) control per motor
- **Mixed-Mode Operation**: Different motors in different modes on the same serial bus
- **Multi-Motor Coordination**: Efficient SyncWrite for chains of motors
- **Safety Features**: Broadcast emergency stop, hardware limits, automatic error recovery
- **Full State Feedback**: Position, velocity, load, voltage, temperature, current, motion status
- **Mock Mode**: Hardware-free simulation for development and testing

## Command Interfaces

**Mode 0 (Position/Servo):**
- `position` - Target position (radians)
- `velocity` - Maximum speed (rad/s)
- `acceleration` - Acceleration value in Mode 0 (0-254)

**Mode 1 (Velocity):**
- `velocity` - Target velocity (rad/s)
- `acceleration` - Acceleration value in Mode 1 (0-254)

**Mode 2 (PWM/Effort):**
- `effort` - PWM duty cycle (-1.0 to +1.0, open-loop control)

**Broadcast (all modes):**
- `emergency_stop` - Stops all motors immediately

## State Interfaces

The hardware always exports all 7 state interfaces for every joint:

- `position` - Current position (radians)
- `velocity` - Current velocity (rad/s)
- `effort` - Motor load percentage (-100.0 to +100.0%)
- `voltage` - Supply voltage (volts)
- `temperature` - Internal temperature (°C)
- `current` - Motor current draw (amperes)
- `is_moving` - Motion status (1.0 = moving, 0.0 = stopped)

Note: All state interfaces are always exported regardless of URDF configuration. URDF state interface declarations are optional but recommended for documentation purposes.

## Quick Start

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/sts_hardware_interface.git
cd sts_hardware_interface
git submodule update --init --recursive
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
```

See the **[Quick Start guide](docs/quick-start.md)** for detailed instructions on running the example launch files and configuring your hardware.

## Configuration Example

```xml
<ros2_control name="sts_system" type="system">
  <hardware>
    <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">1000000</param>
    <param name="use_sync_write">true</param>
  </hardware>

  <joint name="wheel_joint">
    <param name="motor_id">1</param>
    <param name="operating_mode">1</param>

    <command_interface name="velocity"/>
    <command_interface name="acceleration"/>

    <!-- State interfaces (optional declarations for documentation) -->
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <state_interface name="temperature"/>
    <!-- Optional: All 7 state interfaces can be enabled if needed -->
  </joint>
</ros2_control>
```

## Documentation

- **[Quick Start guide](docs/quick-start.md)** - Detailed setup and usage instructions
- **[Design documentation](docs/design.md)** - Implementation details and design decisions

## Hardware Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | *required* | Serial port path (e.g., `/dev/ttyACM0`) |
| `baud_rate` | int | 1000000 | Baud rate: 9600-1000000 |
| `communication_timeout_ms` | int | 100 | Serial timeout: 1-1000 ms |
| `use_sync_write` | bool | true | Enable SyncWrite for multi-motor setups |
| `enable_mock_mode` | bool | false | Simulation mode (no hardware) |
| `max_velocity_steps` | int | 3400 | Max motor velocity in steps/s (STS3215: 3400, STS3032: 2900) |

## Joint Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_id` | int | *required* | Motor ID on serial bus (1-253) |
| `operating_mode` | int | 1 | 0=Position (closed-loop), 1=Velocity (closed-loop), 2=PWM (open-loop) |
| `min_position` | double | 0.0 | Min position limit (radians, Mode 0 only) |
| `max_position` | double | 6.283 | Max position limit (2π radians, Mode 0 only) |
| `max_velocity` | double | 5.22 | Max velocity limit (rad/s, Modes 0 and 1, optional) |
| `max_effort` | double | 1.0 | Max PWM duty cycle (0.0-1.0, Mode 2 only) |

## Dependencies

- **[ROS 2](https://docs.ros.org/en/kilted/)**: Tested with Kilted
- **[ros2_control](https://control.ros.org/)** and **[ros2_controllers](https://control.ros.org/)**
- **[SCServo_Linux](https://github.com/adityakamath/SCServo_Linux)** (included as git submodule)

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.
