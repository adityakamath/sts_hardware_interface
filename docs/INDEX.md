# STS Hardware Interface Documentation

Complete documentation for the ROS 2 hardware interface for Feetech STS series servo motors.

---

## 📚 Documentation Structure

### Getting Started

- **[Quick Start Guide](quick-start.md)** - Get up and running in 5 minutes
  - Installation steps
  - Basic configuration examples
  - Real hardware setup
  - Common troubleshooting

### Technical Documentation

- **[Architecture](architecture.md)** - System architecture and design overview
  - Lifecycle state machine
  - Communication flow
  - Memory layout
  - Thread safety design

- **[Implementation Guide](implementation-guide.md)** - Implementation details and changes
  - Per-joint operating modes
  - Broadcast emergency stop
  - Mixed-mode SyncWrite support
  - Code modifications summary

- **[Design Decisions](design-decisions.md)** - Design philosophy and rationale
  - Why ros2_control?
  - Interface design choices
  - Performance optimizations
  - Error handling strategy

### Feature Documentation

- **[Safety Features](safety-features.md)** - Comprehensive safety improvements
  - Thread-safe serial port management
  - Lifecycle implementation
  - Hardware limits validation
  - Error recovery mechanism
  - Emergency stop features

### Migration & Compatibility

- **[Migration Guide](migration-guide.md)** - Migrating from previous versions
  - From lekiwi_ros2 to sts_hardware_interface
  - Configuration changes
  - API compatibility notes

---

## 📖 Quick Links by Use Case

### "I want to get started quickly"
→ Start with [Quick Start Guide](quick-start.md)

### "I need to understand the architecture"
→ Read [Architecture](architecture.md) and [Design Decisions](design-decisions.md)

### "I want to use advanced features"
→ Check [Implementation Guide](implementation-guide.md) for:
- Per-joint operating modes
- Mixed-mode motor chains
- Broadcast emergency stop

### "I need to ensure safety"
→ Review [Safety Features](safety-features.md) for:
- Multi-layer protection
- Emergency stop procedures
- Error recovery

### "I'm migrating from lekiwi_ros2"
→ Follow [Migration Guide](migration-guide.md)

---

## 🎯 Common Tasks

| Task | Documentation |
|------|---------------|
| Install and configure | [Quick Start](quick-start.md) |
| Launch single motor | `ros2 launch sts_hardware_interface single_motor.launch.py` |
| Launch mixed-mode system | `ros2 launch sts_hardware_interface mixed_mode.launch.py` |
| Test without hardware | `ros2 launch sts_hardware_interface single_motor.launch.py use_mock:=true` |
| Use mixed operating modes | [Implementation Guide - Mixed Modes](implementation-guide.md#3-mixed-mode-syncwrite-support) |
| Configure emergency stop | [Quick Start - Emergency Stop](quick-start.md#emergency-stop) |
| Troubleshoot issues | [Quick Start - Common Issues](quick-start.md#common-issues) |
| Understand safety features | [Safety Features](safety-features.md) |
| Performance tuning | [Quick Start - Performance Tips](quick-start.md#performance-tips) |

---

## 🚀 Launch Files & Configuration

The package includes ready-to-use launch files and URDF configurations:

### Launch Files

| File | Description | Usage |
|------|-------------|-------|
| `single_motor.launch.py` | Single motor in velocity mode | `ros2 launch sts_hardware_interface single_motor.launch.py` |
| `mixed_mode.launch.py` | 3 motors in different modes (wheel/arm/gripper) | `ros2 launch sts_hardware_interface mixed_mode.launch.py` |

### Configuration Files

| File | Description |
|------|-------------|
| `single_motor.urdf.xacro` | Single motor URDF with configurable parameters |
| `mixed_mode.urdf.xacro` | Mixed-mode system (velocity/servo/PWM modes) |
| `velocity_controller.yaml` | Controller config for velocity mode |
| `mixed_mode_controllers.yaml` | Controller config for mixed-mode system |

### Launch Arguments

**Common Arguments:**
- `serial_port` - Serial port path (default: `/dev/ttyACM0`)
- `baud_rate` - Communication baud rate (default: `1000000`)
- `use_mock` - Enable mock/simulation mode (default: `false`)

**Single Motor Arguments:**
- `motor_id` - Motor ID on bus (default: `1`)
- `gui` - Launch joint state publisher GUI (default: `false`)

---

## 📋 Feature Overview

| Feature | Status | Documentation |
|---------|--------|---------------|
| Per-joint operating modes | ✅ Implemented | [Implementation Guide](implementation-guide.md#1-per-joint-operating-modes) |
| Mixed-mode motor chains | ✅ Implemented | [Implementation Guide](implementation-guide.md#3-mixed-mode-syncwrite-support) |
| Broadcast emergency stop | ✅ Implemented | [Implementation Guide](implementation-guide.md#2-broadcast-emergency-stop) |
| SyncWrite optimization | ✅ Implemented | [Quick Start - Performance](quick-start.md#expected-performance) |
| Thread-safe serial sharing | ✅ Implemented | [Safety Features](safety-features.md#1--thread-safe-serial-port-connection-management) |
| Automatic error recovery | ✅ Implemented | [Safety Features](safety-features.md#5--serial-error-recovery-mechanism) |
| Hardware limits validation | ✅ Implemented | [Safety Features](safety-features.md#3--hardware-limits-validation-from-urdf) |
| Full diagnostics | ✅ Implemented | [Safety Features](safety-features.md#6--ros-diagnostics-integration) |
| Mock mode testing | ✅ Implemented | [Quick Start - Mock Mode](quick-start.md#quick-test-no-hardware) |
| Multi-turn tracking | ✅ Implemented | [Quick Start - Multi-Turn](quick-start.md#multi-turn-tracking) |

---

## 🏗️ Architecture Quick Reference

```
┌─────────────────────────────────────────────────────┐
│           ros2_control Framework                     │
│  (Controller Manager, Controllers, Hardware Mgr)     │
└─────────────────────┬───────────────────────────────┘
                      │ StateInterface / CommandInterface
┌─────────────────────▼───────────────────────────────┐
│        STSHardwareInterface                          │
│  ┌──────────────────────────────────────────────┐   │
│  │  Lifecycle: configure → activate → read/write│   │
│  └──────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────┐   │
│  │  Per-Joint Modes: Servo | Velocity | PWM    │   │
│  └──────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────┐   │
│  │  Safety: E-Stop | Limits | Error Recovery   │   │
│  └──────────────────────────────────────────────┘   │
└─────────────────────┬───────────────────────────────┘
                      │ SCServo_Linux SDK
┌─────────────────────▼───────────────────────────────┐
│              STS Servo Motors                        │
│         (ID 1-253, Shared Serial Bus)                │
└──────────────────────────────────────────────────────┘
```

---

## 📞 Support & Contributing

- **Issues**: Report bugs and request features at [GitHub Issues](https://github.com/your-repo/issues)
- **Questions**: Check [Quick Start - Common Issues](quick-start.md#common-issues) first
- **Contributing**: See CONTRIBUTING.md in the repository root

---

## 📄 License

MIT License - See [LICENSE](../LICENSE) file for details

---

**Last Updated**: 2025-12-29
**Package Version**: 1.0.0
**ROS 2 Distribution**: Humble, Iron, Jazzy, Kilted
