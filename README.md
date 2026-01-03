# STS Hardware Interface

[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/sts_hardware_interface)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Iron%20%7C%20Jazzy%20%7C%20Kilted%20%7C%20Rolling-blue)](https://docs.ros.org)
![GitHub License](https://img.shields.io/github/license/adityakamath/sts_hardware_interface)
[![Build & Test](https://github.com/adityakamath/sts_hardware_interface/actions/workflows/build-test.yml/badge.svg)](https://github.com/adityakamath/sts_hardware_interface/actions/workflows/build-test.yml)
[![clang-tidy](https://github.com/adityakamath/sts_hardware_interface/actions/workflows/clang-tidy.yml/badge.svg)](https://github.com/adityakamath/sts_hardware_interface/actions/workflows/clang-tidy.yml)
![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)

> `ros2_control` SystemInterface for Feetech STS series servo motors.

**⚠️ Status:** Only **Mode 1 (Velocity)** has been tested. Modes 0 (Position) and 2 (PWM) are experimental.

## Features

- **Three Operating Modes**: Position (servo), Velocity, and PWM (effort) control per motor
- **Mixed-Mode Operation**: Different motors in different modes on the same serial bus
- **Multi-Motor Coordination**: Efficient SyncWrite for chains of motors
- **Safety Features**: Broadcast emergency stop, hardware limits, automatic error recovery
- **Full State Feedback**: Position, velocity, load, voltage, temperature, current, motion status
- **Mock Mode**: Hardware-free simulation for development and testing

## Command Interfaces

**Mode 0 (Position/Servo):**
- `position` - Target position (radians)
- `velocity` - Maximum speed (rad/s)
- `acceleration` - Acceleration (0-254)

**Mode 1 (Velocity):**
- `velocity` - Target velocity (rad/s)
- `acceleration` - Acceleration (0-254)

**Mode 2 (PWM/Effort):**
- `effort` - PWM duty cycle (-1.0 to +1.0)

**Broadcast (all modes):**
- `emergency_stop` - Stops all motors immediately

## State Interfaces

The hardware can provide up to 7 state interfaces per joint. Configure only the ones you need in your URDF:

- `position` - Current position (radians)
- `velocity` - Current velocity (rad/s)
- `load` - Motor load/torque (-100.0 to +100.0%)
- `voltage` - Supply voltage (volts)
- `temperature` - Internal temperature (°C)
- `current` - Motor current draw (amperes)
- `is_moving` - Motion status (1.0 = moving, 0.0 = stopped)

All state interfaces are optional. Only declare the interfaces your controllers need in the URDF to reduce overhead.

## Quick Start

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/sts_hardware_interface.git
cd sts_hardware_interface
git submodule update --init --recursive
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
```

See the **[Quick Start Guide](docs/quick-start.md)** for detailed instructions on running the example launch files and configuring your hardware.

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
    <param name="max_velocity">15.0</param>

    <command_interface name="velocity"/>
    <command_interface name="acceleration"/>

    <!-- Declare only the state interfaces you need -->
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <!-- Optional: Add temperature monitoring for safety -->
    <state_interface name="temperature"/>
    <!-- Optional: All 7 state interfaces can be enabled if needed -->
  </joint>
</ros2_control>
```

## Documentation

- **[Quick Start Guide](docs/quick-start.md)** - Detailed setup and usage instructions
- **[Architecture](docs/architecture.md)** - Implementation details and design decisions

## Hardware Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | *required* | Serial port path (e.g., `/dev/ttyACM0`) |
| `baud_rate` | int | 1000000 | Baud rate: 9600-1000000 |
| `communication_timeout_ms` | int | 100 | Timeout: 1-1000 ms |
| `use_sync_write` | bool | true | Enable SyncWrite for multi-motor setups |
| `enable_mock_mode` | bool | false | Simulation mode (no hardware) |

## Joint Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_id` | int | *required* | Motor ID on serial bus (1-253) |
| `operating_mode` | int | 1 | 0=Position, 1=Velocity, 2=PWM |
| `min_position` | double | 0.0 | Min position limit (radians, Mode 0 only) |
| `max_position` | double | 6.283 | Max position limit (2π radians, Mode 0 only) |
| `max_velocity` | double | 5.216 | Max velocity (3400 steps/s × 2π/4096 ≈ 5.216 rad/s) |
| `max_effort` | double | 1.0 | Max PWM duty cycle (0.0-1.0, Mode 2 only) |

## Dependencies

- ROS 2: Humble and later
- `ros2_control`
- `ros2_controllers`
- SCServo_Linux (included as git submodule)

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.
