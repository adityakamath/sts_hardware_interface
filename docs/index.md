---
layout: page
title: STS Hardware Interface
subtitle: ros2_control SystemInterface for Feetech STS series servo motors
---

![Project Status](https://img.shields.io/badge/Status-WIP-yellow)
![ROS 2](https://img.shields.io/badge/ROS%202-Kilted%20(Ubuntu%2024.04)-blue?style=flat&logo=ros&logoSize=auto)
![ROS 2 Control](https://img.shields.io/badge/ros2__control-SystemInterface-blue?style=flat&logo=ros&logoSize=auto)
![Repository](https://img.shields.io/badge/Repository-adityakamath%2Fsts__hardware__interface-blue?style=flat&logo=github&logoSize=auto)
![License](https://img.shields.io/github/license/adityakamath/sts_hardware_interface?label=License)

> `ros2_control` SystemInterface for Feetech STS series servo motors (STS3215 and compatible).

**⚠️ Status:** Only **Mode 1 (Velocity)** has been tested. Modes 0 (Position) and 2 (PWM) are experimental.

---

## Key Features

<div style="display: flex; flex-wrap: wrap; gap: 1em; margin: 2em 0;">
  <div style="flex: 1 1 calc(50% - 0.5em); min-width: 280px; background: #f8f9fa; padding: 1.5em; border-radius: 8px; border-left: 4px solid #0366d6;">
    <h3 style="margin-top: 0;">🎛️ Three Operating Modes</h3>
    <p>Position (servo), Velocity, and PWM (effort) control per motor</p>
  </div>
  <div style="flex: 1 1 calc(50% - 0.5em); min-width: 280px; background: #f8f9fa; padding: 1.5em; border-radius: 8px; border-left: 4px solid #28a745;">
    <h3 style="margin-top: 0;">🔀 Mixed-Mode Operation</h3>
    <p>Different motors in different modes on the same serial bus</p>
  </div>
  <div style="flex: 1 1 calc(50% - 0.5em); min-width: 280px; background: #f8f9fa; padding: 1.5em; border-radius: 8px; border-left: 4px solid #6f42c1;">
    <h3 style="margin-top: 0;">⚡ Multi-Motor Coordination</h3>
    <p>Efficient SyncWrite for chains of motors with reduced latency</p>
  </div>
  <div style="flex: 1 1 calc(50% - 0.5em); min-width: 280px; background: #f8f9fa; padding: 1.5em; border-radius: 8px; border-left: 4px solid #dc3545;">
    <h3 style="margin-top: 0;">🛡️ Safety Features</h3>
    <p>Broadcast emergency stop, hardware limits, automatic error recovery</p>
  </div>
  <div style="flex: 1 1 calc(50% - 0.5em); min-width: 280px; background: #f8f9fa; padding: 1.5em; border-radius: 8px; border-left: 4px solid #fd7e14;">
    <h3 style="margin-top: 0;">📊 Full State Feedback</h3>
    <p>Position, velocity, load, voltage, temperature, current, motion status</p>
  </div>
  <div style="flex: 1 1 calc(50% - 0.5em); min-width: 280px; background: #f8f9fa; padding: 1.5em; border-radius: 8px; border-left: 4px solid #20c997;">
    <h3 style="margin-top: 0;">🧪 Mock Mode</h3>
    <p>Hardware-free simulation for development and testing</p>
  </div>
</div>

---

## Command Interfaces

### Mode 0 (Position/Servo)
- `position` - Target position (radians)
- `velocity` - Maximum speed (rad/s)
- `acceleration` - Acceleration (0-254)

### Mode 1 (Velocity)
- `velocity` - Target velocity (rad/s)
- `acceleration` - Acceleration (0-254)

### Mode 2 (PWM/Effort)
- `effort` - PWM duty cycle (-1.0 to +1.0)

### Broadcast (all modes)
- `emergency_stop` - Stops all motors immediately

---

## State Interfaces

The hardware always exports **all 7 state interfaces** for every joint:

- `position` - Current position (radians)
- `velocity` - Current velocity (rad/s)
- `effort` - Motor load percentage (-100.0 to +100.0%)
- `voltage` - Supply voltage (volts)
- `temperature` - Internal temperature (°C)
- `current` - Motor current draw (amperes)
- `is_moving` - Motion status (1.0 = moving, 0.0 = stopped)

**Note:** All state interfaces are always exported regardless of URDF configuration. URDF state interface declarations are optional but recommended for documentation purposes.

---

## Quick Start

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/sts_hardware_interface.git
cd sts_hardware_interface
git submodule update --init --recursive
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
```

### Basic Configuration

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
  </joint>
</ros2_control>
```

---

## Documentation

<div style="display: flex; gap: 1em; margin: 2em 0;">
  <a href="quick-start" style="flex: 1; text-decoration: none;">
    <div style="background: #0366d6; color: white; padding: 1.5em; border-radius: 8px; text-align: center;">
      <h3 style="margin: 0; color: white;">📚 Quick Start Guide</h3>
      <p style="margin: 0.5em 0 0 0; opacity: 0.9;">Setup and usage instructions</p>
    </div>
  </a>
  <a href="architecture" style="flex: 1; text-decoration: none;">
    <div style="background: #28a745; color: white; padding: 1.5em; border-radius: 8px; text-align: center;">
      <h3 style="margin: 0; color: white;">🏗️ Architecture</h3>
      <p style="margin: 0.5em 0 0 0; opacity: 0.9;">Implementation details and design</p>
    </div>
  </a>
</div>

---

## Hardware Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | *required* | Serial port path (e.g., `/dev/ttyACM0`) |
| `baud_rate` | int | 1000000 | Baud rate: 9600-1000000 |
| `communication_timeout_ms` | int | 100 | Serial timeout: 1-1000 ms |
| `use_sync_write` | bool | true | Enable SyncWrite for multi-motor setups |
| `enable_mock_mode` | bool | false | Simulation mode (no hardware) |

## Joint Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_id` | int | *required* | Motor ID on serial bus (1-253) |
| `operating_mode` | int | 1 | 0=Position, 1=Velocity, 2=PWM |
| `min_position` | double | 0.0 | Min position limit (radians, Mode 0 only) |
| `max_position` | double | 6.283 | Max position limit (2π radians, Mode 0 only) |
| `max_effort` | double | 1.0 | Max PWM duty cycle (0.0-1.0, Mode 2 only) |

---

## Dependencies

- **[ROS 2](https://docs.ros.org/en/kilted/)**: Tested with Kilted (Ubuntu 24.04)
- **[ros2_control](https://control.ros.org/)** and **[ros2_controllers](https://control.ros.org/)**
- **[SCServo_Linux](https://github.com/adityakamath/SCServo_Linux)** (included as git submodule)

---

## License

Apache License 2.0 - See [LICENSE](https://github.com/adityakamath/sts_hardware_interface/blob/main/LICENSE) file.

---

## Contact

- **Website:** [kamathrobotics.com](https://kamathrobotics.com)
- **Twitter:** [@kamathsblog](https://twitter.com/kamathsblog)
- **GitHub:** [adityakamath](https://github.com/adityakamath)
