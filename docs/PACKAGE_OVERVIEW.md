# STS Hardware Interface - Package Overview

Complete production-ready `ros2_control` hardware interface for Feetech STS series servo motors.

---

## 📦 Package Structure

```
sts_hardware_interface/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # ROS 2 package manifest
├── LICENSE                     # MIT License
├── README.md                   # Main package documentation
├── CHANGELOG.md                # Version history and changes
├── sts_hardware_interface.xml  # ros2_control plugin description
│
├── include/                    # C++ headers
│   └── sts_hardware_interface/
│       └── sts_hardware_interface.hpp
│
├── src/                        # C++ implementation
│   └── sts_hardware_interface.cpp
│
├── launch/                     # Launch files
│   ├── single_motor.launch.py     # Single motor example
│   └── mixed_mode.launch.py       # Mixed-mode example
│
├── config/                     # Configuration files
│   ├── single_motor.urdf.xacro           # Single motor URDF
│   ├── mixed_mode.urdf.xacro             # Mixed-mode URDF
│   ├── velocity_controller.yaml          # Velocity controller config
│   └── mixed_mode_controllers.yaml       # Mixed-mode controllers config
│
├── docs/                       # Documentation
│   ├── INDEX.md                    # Documentation index
│   ├── quick-start.md              # Getting started guide
│   ├── architecture.md             # System architecture
│   ├── implementation-guide.md     # Implementation details
│   ├── design-decisions.md         # Design philosophy
│   ├── safety-features.md          # Safety improvements
│   ├── migration-guide.md          # Migration guide
│   └── PACKAGE_OVERVIEW.md         # This file
│
└── external/                   # External dependencies
    └── SCServo_Linux/          # SCServo SDK (git submodule)
```

---

## 🎯 Key Features

### Per-Joint Operating Modes
Each motor can operate independently in different modes on the same bus:
- **Mode 0 (Servo)**: Position control with velocity/acceleration limits
- **Mode 1 (Velocity)**: Closed-loop speed control
- **Mode 2 (PWM)**: Open-loop effort control

### Mixed-Mode Support
- Run wheel (velocity) + arm (servo) + gripper (PWM) on same serial bus
- Automatic motor grouping by operating mode
- Efficient SyncWrite batch commands per mode group
- 120-150Hz performance with 3 mixed-mode motors

### Multi-Layer Safety
- **Broadcast Emergency Stop**: Stop all motors with single command (ID 0xFE)
- **Per-Joint Emergency Stop**: Individual motor safety control
- **Hardware Limits**: Position, velocity, effort enforcement from URDF
- **Automatic Error Recovery**: Ping-based health checks and serial reinitialization

### Full Diagnostics
- ROS diagnostics integration (`diagnostic_updater`)
- Real-time monitoring: position, velocity, load, voltage, temperature, current
- Performance metrics: read/write cycle times, error counts
- Compatible with `rqt_robot_monitor`

### High Performance
- 200Hz single motor
- 150Hz multi-motor (same mode with SyncWrite)
- 120-150Hz mixed-mode operation
- Thread-safe serial port sharing

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

### Test Without Hardware (Mock Mode)

```bash
ros2 launch sts_hardware_interface single_motor.launch.py use_mock:=true
```

### Run with Real Hardware

```bash
# Single motor
ros2 launch sts_hardware_interface single_motor.launch.py \
  serial_port:=/dev/ttyACM0 \
  motor_id:=1

# Mixed-mode system
ros2 launch sts_hardware_interface mixed_mode.launch.py \
  serial_port:=/dev/ttyACM0
```

---

## 📚 Documentation

Start with [docs/INDEX.md](INDEX.md) for complete documentation navigation.

### Quick Links

- **Getting Started**: [Quick Start Guide](quick-start.md)
- **Architecture**: [System Architecture](architecture.md)
- **Implementation**: [Implementation Guide](implementation-guide.md)
- **Safety**: [Safety Features](safety-features.md)
- **Migration**: [Migration Guide](migration-guide.md)

---

## ⚙️ Configuration

### Hardware Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | *required* | Serial port path (e.g., `/dev/ttyACM0`) |
| `baud_rate` | int | `1000000` | Communication baud rate |
| `use_sync_write` | bool | `true` | Enable SyncWrite for multi-motor |
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

## 🔧 Dependencies

### ROS 2 Packages
- `rclcpp` - ROS 2 C++ client library
- `rclcpp_lifecycle` - Lifecycle node support
- `hardware_interface` - ros2_control hardware interface
- `controller_manager` - Controller management
- `pluginlib` - Plugin loading
- `diagnostic_updater` - Diagnostics support

### External Libraries
- **SCServo_Linux** - Feetech servo SDK (included as git submodule)

### System Requirements
- ROS 2: Humble, Iron, Jazzy, or Kilted (Ubuntu 24.04)
- C++17 compiler
- CMake 3.10+
- Python 3

---

## 🎓 Examples

### Example 1: Single Wheel Motor

```xml
<ros2_control name="wheel_controller" type="system">
  <hardware>
    <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
  </hardware>

  <joint name="wheel_joint">
    <param name="motor_id">1</param>
    <param name="operating_mode">1</param>  <!-- Velocity -->
    <param name="max_velocity">15.0</param>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### Example 2: Mixed-Mode Robot

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
  </joint>

  <!-- Arm: Servo mode -->
  <joint name="arm">
    <param name="motor_id">2</param>
    <param name="operating_mode">0</param>
    <param name="min_position">-1.57</param>
    <param name="max_position">1.57</param>
    <command_interface name="position"/>
  </joint>

  <!-- Gripper: PWM mode -->
  <joint name="gripper">
    <param name="motor_id">3</param>
    <param name="operating_mode">2</param>
    <command_interface name="effort"/>
  </joint>
</ros2_control>
```

---

## 🛡️ Safety

### Emergency Stop

**Broadcast (all motors):**
```bash
ros2 topic pub /hardware_name/broadcast_emergency_stop std_msgs/msg/Float64 "{data: 1.0}"
```

**Per-joint:**
```bash
ros2 topic pub /joint_name/emergency_stop std_msgs/msg/Float64 "{data: 1.0}"
```

### Hardware Limits

Limits are automatically enforced from URDF parameters:
- Position limits prevent mechanical damage
- Velocity limits ensure safe operation
- Effort limits protect motors

---

## 📊 Performance

| Configuration | Control Loop Speed | Notes |
|--------------|-------------------|-------|
| 1 motor | 200 Hz | Single motor operation |
| 3 motors (same mode) | 150 Hz | SyncWrite enabled |
| 3 motors (mixed modes) | 120-150 Hz | Multiple SyncWrite groups |
| 10 motors (same mode) | 100 Hz | Large chain |

---

## 🐛 Troubleshooting

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
- Verify emergency stop is not active
- Check command topics: `ros2 topic list | grep command`

See [docs/quick-start.md#common-issues](quick-start.md#common-issues) for complete troubleshooting guide.

---

## 📄 License

MIT License - See [LICENSE](../LICENSE) file for details.

---

## 👤 Author

**Aditya Kamath**
- Email: adityakamath@live.com
- GitHub: [Link to repository]

---

## 🙏 Acknowledgments

- **ros2_control** - ROS 2 control framework
- **Feetech Robotics** - STS servo motors and SCServo_Linux SDK
- **ROS 2 Community** - Documentation and support

---

## 📞 Support

- **Documentation**: Start with [docs/INDEX.md](INDEX.md)
- **Issues**: Report at [GitHub Issues](https://github.com/your-repo/issues)
- **Questions**: Check [Quick Start - Common Issues](quick-start.md#common-issues)

---

**Package Version**: 1.0.0
**Last Updated**: 2025-12-29
**ROS 2 Distributions**: Humble, Iron, Jazzy, Kilted
**Status**: ✅ Production Ready
