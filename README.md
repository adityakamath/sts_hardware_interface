# STS Hardware Interface

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Iron%20%7C%20Jazzy%20%7C%20Kilted%20%7C%20Rolling-blue)](https://docs.ros.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

[![Build & Test](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/build-test.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/build-test.yml)
[![clang-tidy](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/clang-tidy.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/clang-tidy.yml)
[![Lint](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/lint.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/lint.yml)
[![PR Comments Check](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/pr-comments-check.yml/badge.svg)](https://github.com/YOUR_USERNAME/sts_hardware_interface/actions/workflows/pr-comments-check.yml)

A production-ready `ros2_control` hardware interface for Feetech STS series servo motors with advanced features for mixed-mode operation, safety, and performance.

---

## ✨ Key Features

🎯 **Per-Joint Operating Modes** - Each motor can operate independently in:
- Mode 0 (Servo): Position control with speed/acceleration
- Mode 1 (Velocity): Closed-loop speed control *(default)*
- Mode 2 (PWM): Open-loop effort control

⚡ **Mixed-Mode Support** - Run different motors in different modes on the same bus with automatic SyncWrite optimization

🛑 **Multi-Layer Safety**
- Broadcast emergency stop (all motors at once)
- Per-joint emergency stop
- Hardware limits enforcement
- Automatic error recovery

📊 **Full Diagnostics** - Position, velocity, load, voltage, temperature, current, motion status

🚀 **High Performance** - 200Hz single motor, 150Hz multi-motor with SyncWrite

---

## 🚀 Quick Start

### Installation

```bash
cd ~/ros2_ws/src
git clone <repository-url> sts_hardware_interface
cd sts_hardware_interface
git submodule update --init --recursive
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
source install/setup.bash
```

### Launch Files (Quick Start)

**Single Motor Example**
```bash
# Real hardware
ros2 launch sts_hardware_interface single_motor.launch.py serial_port:=/dev/ttyACM0 motor_id:=1

# Mock/simulation mode (no hardware required)
ros2 launch sts_hardware_interface single_motor.launch.py use_mock:=true
```

**Mixed-Mode Example (Wheel + Arm + Gripper)**
```bash
# Real hardware with 3 motors in different modes
ros2 launch sts_hardware_interface mixed_mode.launch.py serial_port:=/dev/ttyACM0

# Mock/simulation mode
ros2 launch sts_hardware_interface mixed_mode.launch.py use_mock:=true
```

### Manual URDF Configuration

**Single Motor (Velocity Mode)**
```xml
<ros2_control name="my_motor" type="system">
  <hardware>
    <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
  </hardware>
  <joint name="wheel_joint">
    <param name="motor_id">1</param>
    <param name="operating_mode">1</param>  <!-- Velocity -->
    <command_interface name="velocity"/>
    <command_interface name="acceleration"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

**Mixed-Mode Motor Chain**
```xml
<ros2_control name="robot" type="system">
  <hardware>
    <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="use_sync_write">true</param>
  </hardware>

  <!-- Wheel: Velocity mode -->
  <joint name="wheel">
    <param name="motor_id">1</param>
    <param name="operating_mode">1</param>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>

  <!-- Arm: Servo mode -->
  <joint name="arm">
    <param name="motor_id">2</param>
    <param name="operating_mode">0</param>
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>

  <!-- Gripper: PWM mode -->
  <joint name="gripper">
    <param name="motor_id">3</param>
    <param name="operating_mode">2</param>
    <command_interface name="effort"/>
    <state_interface name="load"/>
  </joint>
</ros2_control>
```

**Emergency Stop**
```bash
# Stop ALL motors at once (broadcast)
ros2 topic pub /hardware_name/broadcast_emergency_stop std_msgs/msg/Float64 "{data: 1.0}"

# Stop individual motor
ros2 topic pub /joint_name/emergency_stop std_msgs/msg/Float64 "{data: 1.0}"
```

---

## 📚 Documentation

**→ [Full Documentation Index](docs/INDEX.md)**

| Document | Description |
|----------|-------------|
| **[Quick Start Guide](docs/quick-start.md)** | Get running in 5 minutes |
| **[Architecture](docs/architecture.md)** | System design and flow |
| **[Implementation Guide](docs/implementation-guide.md)** | Per-joint modes, mixed-mode, broadcast stop |
| **[Safety Features](docs/safety-features.md)** | Multi-layer protection and error recovery |
| **[Design Decisions](docs/design-decisions.md)** | Design philosophy and rationale |
| **[Migration Guide](docs/migration-guide.md)** | Migrating from lekiwi_ros2 |

---

## ⚙️ Configuration Reference

### Hardware Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | *required* | Serial port path (e.g., `/dev/ttyACM0`) |
| `baud_rate` | int | `1000000` | Communication baud rate |
| `use_sync_write` | bool | `true` | Enable SyncWrite for multi-motor chains |
| `enable_multi_turn` | bool | `false` | Multi-revolution position tracking (enable for wheels/continuous rotation) |
| `enable_mock_mode` | bool | `false` | Simulation mode (no hardware) |

### Joint Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_id` | int | *required* | Motor ID on serial bus (1-253) |
| `operating_mode` | int | `1` | 0=Servo, 1=Velocity, 2=PWM |
| `min_position` | double | `-∞` | Minimum position limit (radians) |
| `max_position` | double | `+∞` | Maximum position limit (radians) |
| `max_velocity` | double | `+∞` | Maximum velocity limit (rad/s) |
| `max_effort` | double | `1.0` | Maximum effort limit (normalized) |

---

## 🎯 Operating Modes

| Mode | Name | Control | Use Case | Command Interfaces |
|------|------|---------|----------|-------------------|
| **0** | Servo | Position | Arms, grippers, joints | `position`, `velocity`, `acceleration` |
| **1** | Velocity | Speed | Wheels, continuous rotation | `velocity`, `acceleration` |
| **2** | PWM | Effort | Custom control, force feedback | `effort` |

All modes support:
- `emergency_stop` command interface
- Full state interfaces (position, velocity, load, voltage, temperature, current, is_moving)

---

## 📈 Performance

| Configuration | Control Loop Speed |
|--------------|-------------------|
| 1 motor | 200 Hz |
| 3 motors (same mode) | 150 Hz |
| 3 motors (mixed modes) | 120-150 Hz |
| 10 motors (same mode) | 100 Hz |

*Mixed-mode operation automatically groups motors by mode for efficient SyncWrite commands.*

---

## 🔧 Troubleshooting

### Common Issues

**"Failed to open serial port"**
```bash
sudo usermod -a -G dialout $USER  # Add user to dialout group
# Log out and back in
```

**"Failed to ping motor"**
- Verify motor is powered (LED on)
- Check motor ID matches URDF
- Try different baud rates (115200, 1000000)

**Motors not moving**
- Check controller is loaded: `ros2 control list_controllers`
- Verify emergency stop is not active (should be 0.0)
- Check command topics: `ros2 topic list | grep command`

**→ [Full Troubleshooting Guide](docs/quick-start.md#common-issues)**

---

## 🏗️ Architecture

```
ros2_control Framework
         ↓
STSHardwareInterface (Per-Joint Modes + Safety)
         ↓
SCServo_Linux SDK (SyncWrite Optimization)
         ↓
STS Servo Motors (Shared Serial Bus)
```

**Key Components**:
- **Lifecycle Management**: Full ros2_control lifecycle support
- **Thread-Safe Serial Sharing**: Multiple hardware interfaces can share one port
- **Intelligent SyncWrite**: Automatic mode grouping for batch commands
- **Error Recovery**: Automatic ping-based recovery and serial reinitialization

**→ [Detailed Architecture](docs/architecture.md)**

---

## 🛡️ Safety Features

✅ **Multi-Layer Protection**
- Software limits (position, velocity, effort)
- Emergency stops (broadcast + per-joint)
- Lifecycle safety (deactivate, shutdown, error)
- Automatic error recovery

✅ **Fault Tolerance**
- Consecutive error tracking
- Ping-based health checks
- Serial port reinitialization
- Graceful degradation

**→ [Complete Safety Documentation](docs/safety-features.md)**

---

## 🔄 Migration from lekiwi_ros2

**Old:**
```xml
<plugin>lekiwi_ros2/STSActuatorInterface</plugin>
<param name="operating_mode">1</param>  <!-- Global mode -->
```

**New:**
```xml
<plugin>sts_hardware_interface/STSHardwareInterface</plugin>
<!-- No global operating_mode -->

<joint name="my_joint">
  <param name="operating_mode">1</param>  <!-- Per-joint mode -->
</joint>
```

**→ [Full Migration Guide](docs/migration-guide.md)**

---

## 🧪 Testing

### Mock Mode (No Hardware)

```xml
<hardware>
  <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
  <param name="serial_port">/dev/ttyACM0</param>
  <param name="enable_mock_mode">true</param>
</hardware>
```

Simulates motor behavior for testing controllers without physical hardware.

---

## 📦 Dependencies

- **ROS 2**: Humble, Iron, Jazzy, or Kilted (Ubuntu 24.04)
- **ros2_control**: `hardware_interface`, `controller_manager`
- **Standard**: `rclcpp`, `rclcpp_lifecycle`, `pluginlib`
- **SCServo_Linux**: Included as git submodule

---

## 🚧 Future Enhancements

Potential features for future development:

- [ ] **PID Coefficient Tuning** - Add per-joint PID coefficient configuration via URDF parameters
  (P/D/I coefficients at memory addresses 21/22/23) for advanced servo performance fine-tuning
- [ ] **Hardware Tests** - Expand test coverage for hardware communication edge cases
- [ ] **Additional State Interfaces** - Motor internal position/velocity error for advanced diagnostics

---

## 🤝 Contributing

Contributions welcome! Please:
1. Check existing issues/PRs
2. Follow ROS 2 coding standards
3. Add tests for new features
4. Update documentation

---

## 📄 License

MIT License - See [LICENSE](LICENSE) file for details.

---

## 👤 Author

**Aditya Kamath** - [adityakamath@live.com](mailto:adityakamath@live.com)

---

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/your-repo/issues)
- **Documentation**: [docs/INDEX.md](docs/INDEX.md)
- **Quick Start**: [docs/quick-start.md](docs/quick-start.md)

---

**Status**: ✅ Production Ready | **Version**: 1.0.0 | **Last Updated**: 2025-12-29
