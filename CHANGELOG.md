# Changelog

All notable changes to the STS Hardware Interface package.

## [1.0.0] - 2025-12-29

### Added

#### Launch Files & Configuration
- **`launch/single_motor.launch.py`** - Launch file for single motor in velocity mode
  - Configurable serial port, motor ID, baud rate
  - Mock mode support for testing without hardware
  - Optional joint state publisher GUI
  - Automatic controller spawning

- **`launch/mixed_mode.launch.py`** - Launch file for mixed-mode motor chain
  - 3 motors with different operating modes (wheel/arm/gripper)
  - Automatic controller spawning for all modes
  - Mock mode support

- **`config/single_motor.urdf.xacro`** - URDF template for single motor
  - Parameterized serial port, motor ID, baud rate
  - Mock mode parameter
  - Complete state and command interfaces

- **`config/mixed_mode.urdf.xacro`** - URDF template for mixed-mode system
  - Wheel joint (velocity mode)
  - Arm joint (servo/position mode)
  - Gripper joint (PWM/effort mode)
  - Full joint limit specifications

- **`config/velocity_controller.yaml`** - Controller configuration for velocity mode
  - Joint state broadcaster
  - Velocity controller with 100Hz update rate

- **`config/mixed_mode_controllers.yaml`** - Controller configuration for mixed modes
  - Joint state broadcaster
  - Velocity controller (wheel)
  - Joint trajectory controller (arm)
  - Effort controller (gripper)

#### Core Features
- **Per-Joint Operating Modes** - Each motor can operate in different mode
  - Mode 0: Servo/Position control
  - Mode 1: Velocity control (default)
  - Mode 2: PWM/Effort control

- **Broadcast Emergency Stop** - Stop all motors with single command
  - Uses STS broadcast ID (0xFE)
  - Single command stops all motors regardless of mode
  - Topic: `/hardware_name/broadcast_emergency_stop`

- **Mixed-Mode SyncWrite** - Efficient batch commands for mixed modes
  - Automatic motor grouping by operating mode
  - Separate SyncWrite calls per mode group
  - 120-150Hz performance with 3 mixed-mode motors

#### Safety Features
- Thread-safe serial port connection management
- Hardware limits validation from URDF
- Automatic error recovery mechanism
- Multi-layer emergency stop (broadcast + per-joint)
- Comprehensive diagnostics integration

#### Documentation
- Complete documentation reorganization in `docs/` folder
- **docs/INDEX.md** - Comprehensive documentation index
- **docs/quick-start.md** - 5-minute getting started guide
- **docs/architecture.md** - System architecture overview
- **docs/implementation-guide.md** - Implementation details
- **docs/design-decisions.md** - Design philosophy
- **docs/safety-features.md** - Safety improvements
- **docs/migration-guide.md** - Migration from lekiwi_ros2

### Changed

- **README.md** - Complete rewrite with quick start examples
  - Added launch file examples
  - Added mixed-mode configuration examples
  - Added performance characteristics table
  - Moved detailed docs to `docs/` folder

- **CMakeLists.txt** - Added installation rules
  - Install launch files
  - Install config files
  - Install documentation

- **Documentation Structure** - Reorganized all documentation
  - Moved from root to `docs/` folder
  - Renamed files with semantic names
  - Created comprehensive index

### Removed

- Old backup files and artifacts
  - `fix_write_method.py`
  - `write_logic_replacement.txt`
  - `CODE_REVIEW.md`
  - `DOCUMENTATION_UPDATE_SUMMARY.md`

### Fixed

- Removed duplicate code in `write()` method
- Removed unused variable declarations
- Updated all lifecycle methods to use per-joint operating modes
- Corrected documentation to show `operating_mode` as joint parameter

## Architecture

```
ros2_control Framework
         ↓
STSHardwareInterface (Per-Joint Modes + Safety)
         ↓
SCServo_Linux SDK (SyncWrite Optimization)
         ↓
STS Servo Motors (Shared Serial Bus)
```

## Quick Start

```bash
# Install
cd ~/ros2_ws/src
git clone <repository-url> sts_hardware_interface
cd sts_hardware_interface
git submodule update --init --recursive
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
source install/setup.bash

# Test without hardware
ros2 launch sts_hardware_interface single_motor.launch.py use_mock:=true

# Run with real hardware
ros2 launch sts_hardware_interface single_motor.launch.py serial_port:=/dev/ttyACM0 motor_id:=1

# Run mixed-mode system
ros2 launch sts_hardware_interface mixed_mode.launch.py serial_port:=/dev/ttyACM0
```

## Performance

| Configuration | Control Loop Speed |
|--------------|-------------------|
| 1 motor | 200 Hz |
| 3 motors (same mode) | 150 Hz |
| 3 motors (mixed modes) | 120-150 Hz |
| 10 motors (same mode) | 100 Hz |

## License

MIT License - See LICENSE file for details.

## Author

Aditya Kamath - adityakamath@live.com
