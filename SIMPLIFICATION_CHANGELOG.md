# Hardware Interface Simplification Changelog

This document tracks major simplifications to the STS Hardware Interface to reduce code complexity while maintaining all essential functionality.

## 2025-01-01: Removed Performance Monitoring

### Motivation
- Built-in performance monitoring added runtime overhead
- Duplicated functionality available in ros2_control framework
- Most users don't need custom performance metrics
- Better to use standard ROS 2 tooling for monitoring

### Changes Made

#### Header File (`sts_hardware_interface.hpp`)
**Removed member variables:**
- `std::chrono::steady_clock::time_point last_read_time_`
- `std::chrono::steady_clock::time_point last_write_time_`
- `double read_duration_ms_`
- `double write_duration_ms_`
- `double max_read_duration_ms_`
- `double max_write_duration_ms_`
- `unsigned int cycle_count_`
- `static constexpr unsigned int PERFORMANCE_LOG_INTERVAL`

**Lines removed**: 8 member variables + 1 constant = ~9 lines

#### Implementation File (`sts_hardware_interface.cpp`)
**Removed code sections:**
1. Variable initialization (on_init): ~5 lines
2. Performance timing in read() mock mode: ~4 lines
3. Performance timing in read() real hardware mode: ~10 lines
4. Performance timing in write() mock mode: ~4 lines
5. Performance timing in write() real hardware mode: ~5 lines
6. Diagnostics callback metrics: ~5 lines
7. Removed unused start_time variables: 2 lines

**Total lines removed**: ~35 lines

### New Documentation

Created [docs/PERFORMANCE_MONITORING.md](docs/PERFORMANCE_MONITORING.md) documenting how to use:
- ros2_control Controller Manager statistics
- `ros2 topic hz` and `ros2 topic bw`
- ros2_tracing for advanced profiling
- System-level monitoring (top, htop, chrt)
- Custom diagnostic nodes (optional)

### Migration Guide

**Before** (custom monitoring):
```cpp
// Performance data available via diagnostics
diagnostic_updater_->force_update();  // Includes read/write duration
```

**After** (use ros2_control tools):
```bash
# Monitor controller performance
ros2 topic echo /controller_manager/statistics

# Monitor update rate
ros2 topic hz /joint_states

# Expected: ~50 Hz
```

### Benefits

1. **Reduced complexity**: 44 fewer lines of code
2. **Lower runtime overhead**: No timing calculations every cycle
3. **Standard tooling**: Uses ros2_control's built-in monitoring
4. **Better documentation**: Comprehensive guide to performance monitoring
5. **Maintainability**: One less subsystem to maintain and test

### Performance Impact

**Eliminated overhead per cycle**:
- 2x `std::chrono::steady_clock::now()` calls
- 2x duration calculations
- 2x max value comparisons
- Periodic logging check

**Result**: Slightly faster read/write cycles, especially at high update rates (50+ Hz)

## 2025-01-01: Removed reverse_direction Functionality

### Motivation
- Direction reversal should be handled by URDF joint axis
- Duplicate functionality with URDF `<axis xyz="0 0 -1"/>`
- Risk of double-reversal bugs when both methods used
- Not standard ros2_control practice

### Changes Made

#### Header File
**Removed**:
- `std::vector<bool> reverse_direction_` member variable

#### Implementation File
**Removed**:
- Initialization of reverse_direction_ vector
- Parameter parsing for reverse_direction
- Direction reversal in read() (mock mode)
- Direction reversal in read() (real hardware)
- Direction reversal in write() (SyncWrite path)
- Direction reversal in write() (individual write path)

**Total lines removed**: ~30 lines

### Documentation Created

Created [MOTOR_DIRECTION_HANDLING.md](MOTOR_DIRECTION_HANDLING.md) explaining:
- Why URDF-only approach is better
- How to configure joint axis for upside-down motors
- Migration from old reverse_direction parameter
- Examples and testing procedures

### Benefits

1. **Single source of truth**: URDF axis is authoritative
2. **Standard practice**: Aligns with ros2_control conventions
3. **No double-reversal bugs**: Impossible to misconfigure
4. **Cleaner code**: Removed conditional reversal logic

## 2025-01-01: Removed diagnostic_updater Dependency

### Motivation
- diagnostic_updater adds complexity and dependency for rarely-used feature
- All diagnostic data already available via state interfaces
- Simple logging (RCLCPP_WARN/ERROR) sufficient for critical issues
- Health status determination should be done externally to hardware interface

### Changes Made

#### Header File (`sts_hardware_interface.hpp`)
**Removed includes:**
- `#include "diagnostic_updater/diagnostic_updater.hpp"`
- `#include "diagnostic_updater/publisher.hpp"`

**Removed member variable:**
- `std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_`

**Lines removed**: ~3 lines

#### Implementation File (`sts_hardware_interface.cpp`)
**Removed code sections:**
1. diagnostic_updater initialization in on_configure (mock mode): ~3 lines
2. diagnostic_updater initialization in on_configure (real hardware): ~3 lines
3. diagnostic_updater->force_update() calls in read() (both modes): ~6 lines
4. Unused includes (`<sstream>`, `<iomanip>`): 2 lines

**Lines removed**: ~14 lines

#### Build Configuration
**package.xml**:
- Removed `<depend>diagnostic_updater</depend>`

**CMakeLists.txt**:
- Removed `find_package(diagnostic_updater REQUIRED)`
- Removed `${diagnostic_updater_TARGETS}` from target_link_libraries
- Removed `diagnostic_updater` from ament_export_dependencies

**Lines removed**: ~4 lines

### Design Philosophy

The hardware interface now follows a simpler design:
- **Critical issues**: Logged via RCLCPP_WARN/RCLCPP_ERROR (already in place)
- **Motor telemetry**: Available via state interfaces (position, velocity, load, voltage, temperature, current, is_moving)
- **Health monitoring**: Should be implemented externally using state interface data

### Benefits

1. **Simpler dependencies**: One less ROS package to depend on
2. **Separation of concerns**: Hardware interface focuses on I/O, health logic lives externally
3. **Standard patterns**: Uses ros2_control state interfaces for all monitoring data
4. **Minimal code**: No diagnostic or health status code to maintain

### Migration Guide

**Before** (diagnostic_updater):
```bash
# Diagnostics published automatically
ros2 topic echo /diagnostics
```

**After** (use state interfaces):
```bash
# All motor telemetry available via joint_states
ros2 topic echo /joint_states

# Monitor specific states
ros2 topic echo /joint_states | grep temperature
ros2 topic echo /joint_states | grep voltage
```

For custom health monitoring, create an external node that subscribes to `/joint_states` and applies your health logic.

## 2025-01-01: Simplified Mock Mode Implementation

### Motivation
- Mock mode maintained duplicate state vectors (mock_position_, mock_velocity_, etc.)
- Same data was copied to hw_state_* variables every cycle
- Unnecessary memory usage and code duplication
- hw_state_* variables can be used directly for simulation

### Changes Made

#### Header File (`sts_hardware_interface.hpp`)
**Removed member variables:**
- `std::vector<double> mock_position_`
- `std::vector<double> mock_velocity_`
- `std::vector<double> mock_load_`
- `std::vector<double> mock_temperature_`
- `std::vector<double> mock_voltage_`
- `std::vector<double> mock_current_`

**Lines removed**: ~6 lines

#### Implementation File (`sts_hardware_interface.cpp`)
**Removed code:**
1. Mock variable initialization (on_init): ~6 lines
2. Mock variable assignment in read() mock mode: ~7 lines (copy from mock to hw_state)
3. Mock variable updates in write() emergency stop: ~3 references
4. Mock variable updates in on_deactivate(): ~2 lines

**Modified code:**
1. read() mock mode: Now updates `hw_state_*` directly instead of `mock_*` then copying
2. write() mock mode: Emergency stop sets `hw_state_velocity_` directly
3. on_deactivate() mock mode: Clears `hw_state_velocity_` and `hw_state_load_` directly
4. on_init(): Initialize `hw_state_temperature_` to 25.0 and `hw_state_voltage_` to 12.0

**Net lines removed**: ~17 lines

### Design Rationale

Mock mode now uses the same state variables (`hw_state_*`) that real hardware mode uses:
- **Before**: Simulate in `mock_*` → Copy to `hw_state_*` → Publish
- **After**: Simulate directly in `hw_state_*` → Publish

This is safe because:
- Mock mode and real hardware mode are mutually exclusive (enable_mock_mode flag)
- hw_state_* variables are only read by ros2_control framework, never by hardware
- Eliminates copy operation every control cycle
- Same memory layout regardless of mode

### Benefits

1. **Less memory**: 6 fewer vectors (~48 bytes per motor)
2. **Simpler code**: No duplicate state tracking
3. **Better performance**: No copy operation in read() cycle
4. **Same functionality**: Mock simulation behavior unchanged

## Total Simplification Impact

### Code Reduction
- **Header file**: ~9 lines removed (performance monitoring)
- **Header file**: ~1 line removed (reverse_direction)
- **Header file**: ~3 lines removed (diagnostic_updater)
- **Header file**: ~6 lines removed (mock mode state variables)
- **Header file**: ~5 lines removed (serial port sharing)
- **Implementation file**: ~35 lines removed (performance monitoring)
- **Implementation file**: ~30 lines removed (reverse_direction)
- **Implementation file**: ~14 lines removed (diagnostic_updater)
- **Implementation file**: ~17 lines removed (mock mode duplication)
- **Implementation file**: ~35 lines removed (serial port sharing)
- **Build files**: ~4 lines removed (diagnostic_updater)

**Total**: ~159 lines removed

### Complexity Reduction
- 5 fewer subsystems to maintain (performance monitoring, direction reversal, diagnostics, mock duplication, serial port sharing)
- Fewer dependencies (removed diagnostic_updater package)
- Fewer member variables (20 fewer: 11 from subsystems + 6 from mock mode + 3 from serial sharing)
- Simpler control flow (no reversal conditionals, no diagnostic callbacks, no mock→hw_state copies, no ownership checks)
- No static state (removed static map and mutex)
- Better memory efficiency (mock mode uses same memory as real mode)
- Better alignment with ros2_control standards (single instance per physical system)

### Documentation Improvement
- Added PERFORMANCE_MONITORING.md (comprehensive monitoring guide)
- Added MOTOR_DIRECTION_HANDLING.md (direction handling guide)
- Updated POSITION_ZEROING.md (removed reversal references)

## 2025-01-01: Removed Static Serial Port Sharing

### Motivation
- Package supports single ros2_control instance with multiple joints
- Multiple ros2_control instances sharing one serial port adds unnecessary complexity
- Static variables and mutexes for sharing are over-engineered
- Simplifies lifecycle management (no ownership tracking)

### Changes Made

#### Header File (`sts_hardware_interface.hpp`)
**Removed includes:**
- `#include <mutex>`

**Removed static member variables:**
- `static std::map<std::string, std::shared_ptr<SMS_STS>> serial_port_connections_`
- `static std::mutex serial_port_mutex_`

**Removed member variable:**
- `bool owns_serial_connection_`

**Lines removed**: ~5 lines

#### Implementation File (`sts_hardware_interface.cpp`)
**Removed code sections:**
1. Static member initialization: ~2 lines
2. Serial port sharing check in on_configure(): ~15 lines (replaced with direct creation)
3. Ownership check in on_cleanup(): ~4 lines (now always closes)
4. Ownership check in on_shutdown(): ~4 lines (now always closes)
5. Ownership check in attempt_error_recovery(): ~5 lines (now always attempts recovery)
6. Mutex locks: ~5 references removed

**Modified code:**
1. on_configure(): Direct serial connection creation (no sharing check)
2. on_cleanup(): Always closes serial connection
3. on_shutdown(): Always closes serial connection
4. attempt_error_recovery(): Always attempts serial reconnection

**Net lines removed**: ~35 lines

### Design Rationale

**Before** (with sharing):
- Multiple hardware interfaces could share one serial port
- Required static map, mutex, and ownership tracking
- Complex lifecycle: only owner closes port

**After** (single instance):
- One hardware interface per serial port (standard pattern)
- Direct serial connection ownership
- Simple lifecycle: always open on configure, close on cleanup/shutdown

This is the correct pattern because:
- ros2_control expects one hardware interface instance per physical system
- Multiple motors on same bus are handled as multiple joints in single interface
- No legitimate use case for multiple hardware interfaces sharing one serial port
- Simpler code is more reliable and maintainable

### Benefits

1. **Simpler code**: No ownership tracking, no static state
2. **Fewer dependencies**: Removed `<map>` and `<mutex>` includes
3. **Standard pattern**: Matches ros2_control best practices
4. **Clearer lifecycle**: Direct ownership model
5. **Better error recovery**: Always attempts serial reconnection

## 2025-01-01: Added Comprehensive Parameter Validation

### Motivation
- Invalid parameters can cause cryptic runtime errors
- Early validation provides clear error messages at initialization
- Prevents common configuration mistakes
- Improves user experience and debugging

### Changes Made

#### Implementation File (`sts_hardware_interface.cpp`)
**Added validation for hardware parameters:**
1. **baud_rate**: Must be one of [9600, 19200, 38400, 57600, 115200, 500000, 1000000]
2. **communication_timeout_ms**: Must be between 1 and 1000 ms
3. **motor_id**: Already validated (1-253 range), checked for duplicates
4. **operating_mode**: Already validated (0, 1, or 2)

**Added validation for joint limits:**
1. **Position limits**: min_position must be less than max_position
2. **Velocity limits**: Must be positive, warns if exceeds hardware maximum (~20 rad/s)
3. **Effort limits**: Must be 0-1 for PWM mode

**Lines added**: ~45 lines

### Validation Examples

**Invalid baud rate**:
```
ERROR: Invalid baud_rate: 9800. Supported rates: 9600, 19200, 38400, 57600, 115200, 500000, 1000000
```

**Invalid position limits**:
```
ERROR: Joint 'left_wheel_joint': min_position (-1.000) must be less than max_position (-2.000)
```

**Invalid velocity**:
```
ERROR: Joint 'left_wheel_joint': max_velocity must be positive (got -5.000 rad/s)
```

**Exceeds hardware maximum**:
```
WARN: Joint 'left_wheel_joint': max_velocity (25.000 rad/s) exceeds hardware maximum (~20.944 rad/s)
```

### Benefits

1. **Early error detection**: Catches configuration errors at initialization
2. **Clear error messages**: Specific messages with parameter names and valid ranges
3. **Better user experience**: Users know exactly what's wrong and how to fix it
4. **Prevents runtime failures**: Invalid configurations rejected before hardware access

## 2025-01-01: Implemented Configurable State Interfaces

### Motivation

- Previously exported all 7 state interfaces for every joint unconditionally
- Not all applications need all telemetry data (voltage, temperature, current, load)
- Reading unused state interfaces adds overhead in the read() cycle
- Users should be able to choose which state interfaces they need via URDF

### Changes Made

#### Header File Changes (`sts_hardware_interface.hpp`)

**Added member variables:**

- `std::vector<bool> has_position_state_`
- `std::vector<bool> has_velocity_state_`
- `std::vector<bool> has_load_state_`
- `std::vector<bool> has_voltage_state_`
- `std::vector<bool> has_temperature_state_`
- `std::vector<bool> has_current_state_`
- `std::vector<bool> has_is_moving_state_`

**Lines added**: ~7 lines

#### Implementation File Changes (`sts_hardware_interface.cpp`)

**Added code sections:**

1. State interface tracking initialization in on_init(): ~7 lines
2. State interface detection from URDF in on_init(): ~17 lines
3. Conditional export in export_state_interfaces(): ~28 lines (net change)
4. Conditional reading in read() for real hardware: ~30 lines (net change)
5. Conditional simulation in read() for mock mode: ~20 lines (net change)

**Lines modified/added**: ~102 lines (net change considering refactoring)

### Usage Examples

**Minimal configuration** (only position and velocity for basic control):

```xml
<joint name="wheel_joint">
  <param name="motor_id">1</param>
  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

**Full monitoring** (all telemetry for health monitoring):

```xml
<joint name="wheel_joint">
  <param name="motor_id">1</param>
  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="load"/>
  <state_interface name="voltage"/>
  <state_interface name="temperature"/>
  <state_interface name="current"/>
  <state_interface name="is_moving"/>
</joint>
```

### Implementation Benefits

1. **Flexible configuration**: Users choose exactly which telemetry they need
2. **Reduced overhead**: read() only queries enabled state interfaces from hardware
3. **Backward compatible**: Declaring all 7 interfaces maintains previous behavior
4. **Better performance**: Fewer serial reads per cycle when monitoring is not needed
5. **Standard pattern**: Follows ros2_control convention of respecting URDF declarations

### Performance Impact Analysis

**For minimal configuration (position + velocity only)**:

- **Real hardware**: Eliminates 5 serial reads per motor per cycle (load, voltage, temperature, current, is_moving)
- **Mock mode**: Skips simulation calculations for unused state interfaces
- **Result**: Significantly faster read() cycle, especially with multiple motors

**Example**: With 3 motors and basic control:

- Before: 21 state interface reads per cycle (7 per motor × 3)
- After: 6 state interface reads per cycle (2 per motor × 3)
- **Reduction**: 71% fewer serial operations

## Principles for Future Changes

1. **Prefer standard ros2_control patterns** over custom implementations
2. **Keep hardware interface simple** - it's just hardware I/O
3. **Delegate complex features** to controllers and standard tools
4. **Document external tools** rather than reimplementing them
5. **Validate assumptions** with real use cases before adding features

## Version History

- **v0.3.0** (2025-01-01): Removed diagnostic_updater dependency, added get_health_status()
- **v0.2.0** (2025-01-01): Removed performance monitoring and reverse_direction
- **v0.1.0** (Initial): Full-featured implementation with all monitoring

---

**Summary**: These simplifications removed ~159 lines of code while maintaining all essential functionality. The interface now:
- Delegates performance monitoring to ros2_control tooling
- Handles motor direction via URDF axis definitions
- Uses RCLCPP logging for critical issues only
- Exposes all telemetry via standard state interfaces
- Shares state variables between mock and real modes for efficiency
- Uses single-instance pattern (no serial port sharing between multiple hardware interfaces)
- Has fewer dependencies, no static state, and simpler code following ROS 2 best practices
