# STS Hardware Interface

[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/sts_hardware_interface)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Iron%20%7C%20Jazzy%20%7C%20Kilted%20%7C%20Rolling-blue)](https://docs.ros.org)
![GitHub License](https://img.shields.io/github/license/adityakamath/sts_hardware_interface)
[![Build & Test](https://github.com/adityakamath/sts_hardware_interface/actions/workflows/build-test.yml/badge.svg)](https://github.com/adityakamath/sts_hardware_interface/actions/workflows/build-test.yml)
[![clang-tidy](https://github.com/adityakamath/sts_hardware_interface/actions/workflows/clang-tidy.yml/badge.svg)](https://github.com/adityakamath/sts_hardware_interface/actions/workflows/clang-tidy.yml)
![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)

> `ros2_control` hardware interface for Feetech STS series servo motors.

**⚠️ Note:** This hardware interface has only been tested with **mode 1 (Velocity Control mode)**. Mode 0 (Servo mode) and mode 2 (Effort/PWM mode) have not been tested and may not work as expected.

## Overview

This package provides a complete hardware interface for controlling Feetech STS3215 (and compatible) servo motors through the ros2_control framework. It supports single motors, motor chains, and mixed-mode operation where different motors can operate in different control modes (untested).

### Key Features

- **Multiple Operating Modes**: Position (servo), Velocity, and PWM (effort) control
- **Mixed-Mode Operation**: Different motors in different modes on the same serial bus
- **Safety**: Emergency stop, hardware limits, automatic error recovery
- **Performance**: Efficient SyncWrite for multi-motor control
- **Diagnostics**: Optional full state feedback (position, velocity, load, voltage, temperature, current)
- **Mock Mode**: Simulation support for testing without hardware

## Quick Start

See [docs/quick-start.md](docs/quick-start.md) for detailed setup instructions.

### Basic Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/sts_hardware_interface.git
cd sts_hardware_interface
git submodule update --init --recursive
cd ..
colcon build --packages-select sts_hardware_interface
```

### Simple Example

Single motor in velocity mode:

```xml
<ros2_control name="my_motor" type="system">
  <hardware>
    <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">1000000</param>
  </hardware>
  <joint name="wheel_joint">
    <param name="motor_id">1</param>
    <param name="operating_mode">1</param>  <!-- Velocity mode -->
    <param name="max_velocity">15.0</param>  <!-- rad/s -->
    <command_interface name="velocity"/>
    <command_interface name="acceleration"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

## Operating Modes

| Mode | Control Type | Use Case | Command Interfaces |
|------|-------------|----------|-------------------|
| 0 (Servo) | Position | Arms, joints | position, velocity, acceleration |
| 1 (Velocity) | Speed | Wheels, continuous | velocity, acceleration |
| 2 (PWM) | PWM | Effort/Force control | PWM duty cycle |

All modes support broadcast emergency_stop command interface and (optional) full state feedback.

## Documentation

- **[Quick Start Guide](docs/quick-start.md)** - Installation, setup, and basic usage
- **[Architecture](docs/architecture.md)** - System design and implementation details

## Configuration

### Hardware Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| serial_port | string | required | Serial port path |
| baud_rate | int | 1000000 | Communication baud rate (9600-1000000) |
| communication_timeout_ms | int | 100 | Communication timeout (1-1000 ms) |
| use_sync_write | bool | true | Enable SyncWrite for multi-motor setups |
| enable_mock_mode | bool | false | Simulation mode |

### Joint Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| motor_id | int | required | Motor ID on serial bus (1-253) |
| operating_mode | int | 1 | Control mode: 0=Servo/Position, 1=Velocity, 2=PWM/Effort |
| min_position | double | 0.0 | Minimum position limit in radians (Mode 0 only) |
| max_position | double | 6.283 | Maximum position limit in radians, 2π (Mode 0 only) |
| max_velocity | double | 5.22 | Maximum velocity limit in rad/s (3400 steps/s) |
| max_effort | double | 1.0 | Maximum PWM duty cycle, 0.0-1.0 (Mode 2 only) |

## Dependencies

- ROS 2 (Humble, Iron, Jazzy, Kilted, or Rolling)
- ros2_control
- SCServo_Linux (included as submodule)

## License

Apache License 2.0 - See [LICENSE](LICENSE) file.

## Author

**Aditya Kamath** - [adityakamath@live.com](mailto:adityakamath@live.com)
