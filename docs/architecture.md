# STS Hardware Interface - Technical Explanation

## Table of Contents
1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Key Components](#key-components)
4. [Communication Flow](#communication-flow)
5. [Operating Modes](#operating-modes)
6. [Lifecycle Management](#lifecycle-management)
7. [Error Handling & Recovery](#error-handling--recovery)
8. [Current Implementation Status](#current-implementation-status)
9. [Next Steps](#next-steps)

---

## Overview

The `sts_hardware_interface` package provides a ROS 2 `ros2_control` hardware interface for Feetech STS series servo motors. This package was extracted from the `lekiwi_ros2` package to create a standalone, reusable component that can be integrated into any ROS 2 robot system.

### Purpose
- Enable ros2_control framework integration for STS servo motors
- Provide a clean abstraction layer between ROS 2 controllers and STS hardware
- Support multiple motors on a shared serial bus
- Offer comprehensive diagnostics and safety features

### Key Features
- **Three Operating Modes**: Servo (position), Velocity, and PWM (effort) control
- **Full State Feedback**: Position, velocity, load, voltage, temperature, current, motion status
- **Safety Features**: Emergency stop, hardware limits validation, error recovery
- **Diagnostics**: Integrated diagnostic_updater for health monitoring
- **Mock Mode**: Simulation support for development without hardware
- **Multi-Turn Tracking**: Optional continuous position tracking across multiple revolutions

---

## Architecture

### Component Hierarchy

```
┌─────────────────────────────────────────────────────┐
│           ROS 2 Control Framework                    │
│  (Controller Manager, Controllers, Broadcasters)     │
└────────────────┬────────────────────────────────────┘
                 │
                 │ Standard ros2_control API
                 │
┌────────────────▼────────────────────────────────────┐
│         STSHardwareInterface                         │
│  (Implements hardware_interface::ActuatorInterface)  │
│                                                       │
│  • Lifecycle management (configure/activate/etc.)    │
│  • State/Command interface exports                   │
│  • Unit conversions (rad ↔ motor steps)             │
│  • Error handling & recovery                         │
│  • Performance monitoring                            │
└────────────────┬────────────────────────────────────┘
                 │
                 │ SCServo SDK API
                 │
┌────────────────▼────────────────────────────────────┐
│           SCServo_Linux Library                      │
│  (External dependency, git submodule)                │
│                                                       │
│  • Low-level serial communication                    │
│  • Motor protocol implementation                     │
│  • Register read/write operations                    │
└────────────────┬────────────────────────────────────┘
                 │
                 │ Serial Protocol
                 │
┌────────────────▼────────────────────────────────────┐
│         Feetech STS Servo Motors                     │
│  (Physical Hardware - one or more motors)            │
└─────────────────────────────────────────────────────┘
```

### Directory Structure

```
sts_hardware_interface/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # ROS 2 package manifest
├── sts_hardware_interface.xml  # Plugin descriptor for ros2_control
├── LICENSE                     # MIT license
├── README.md                   # User documentation
├── MIGRATION_SUMMARY.md        # Migration guide from lekiwi_ros2
├── IMPROVEMENTS.md             # Detailed improvement log
├── EXPLANATION.md              # This file
│
├── include/sts_hardware_interface/
│   └── sts_hardware_interface.hpp  # Header with class definition
│
├── src/
│   └── sts_hardware_interface.cpp  # Implementation (INCOMPLETE)
│
└── external/
    └── SCServo_Linux/          # Git submodule with motor SDK
```

---

## Key Components

### 1. STSHardwareInterface Class

The main class inheriting from `hardware_interface::ActuatorInterface`:

```cpp
class STSHardwareInterface : public hardware_interface::ActuatorInterface
```

**Primary Responsibilities:**
- Implement ros2_control lifecycle callbacks
- Export state and command interfaces
- Convert between ROS units (radians, rad/s) and motor units (steps)
- Handle serial communication via SCServo SDK
- Provide error recovery and diagnostics

### 2. Configuration Parameters

Loaded from URDF `<ros2_control>` block:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | *required* | Serial port path (e.g., `/dev/ttyACM0`) |
| `motor_id` | int | *required* | Motor ID on bus (1-253) |
| `operating_mode` | int | 1 | 0=Servo, 1=Velocity, 2=PWM |
| `baud_rate` | int | 1000000 | Serial communication speed |
| `communication_timeout_ms` | int | 100 | Serial timeout in milliseconds |
| `enable_multi_turn` | bool | false | Enable multi-revolution tracking |
| `enable_mock_mode` | bool | false | Enable simulation mode |

### 3. State Interfaces (Read from Hardware)

All modes provide these state interfaces in SI units:

- **position** (rad): Current shaft position
- **velocity** (rad/s): Current rotational velocity
- **load** (percentage): Motor load/torque (-100 to +100)
- **voltage** (V): Supply voltage
- **temperature** (°C): Internal motor temperature
- **current** (A): Motor current draw
- **is_moving** (boolean): Motion status (1.0 = moving, 0.0 = stopped)

### 4. Command Interfaces (Write to Hardware)

Mode-dependent command interfaces:

**Mode 0 (Servo - Position Control):**
- **position** (rad): Target position
- **velocity** (rad/s): Maximum speed limit
- **acceleration** (0-254): Acceleration value (0 = no limit)

**Mode 1 (Velocity Control):**
- **velocity** (rad/s): Target velocity
- **acceleration** (0-254): Acceleration value

**Mode 2 (PWM - Effort Control):**
- **effort** (-1.0 to +1.0): PWM duty cycle

**All Modes:**
- **emergency_stop** (boolean): Emergency stop trigger

### 5. SCServo SDK Integration

The interface uses the `SMS_STS` class from SCServo_Linux:

```cpp
std::shared_ptr<SMS_STS> servo_;
```

**Key SDK Methods Used:**
- `begin()`: Initialize serial port
- `Ping()`: Check motor connectivity
- `InitMotor()`: Configure motor mode and enable torque
- `FeedBack()`: Request all feedback data in one transaction
- `ReadPos()`, `ReadSpeed()`, `ReadLoad()`, etc.: Read cached state
- `WritePosEx()`: Write position + speed + acceleration (Mode 0)
- `WriteSpe()`: Write velocity + acceleration (Mode 1)
- `WritePwm()`: Write PWM value (Mode 2)
- `EnableTorque()`: Enable/disable motor torque

### 6. Thread-Safe Serial Port Sharing

Multiple motors can share a single serial port:

```cpp
// Static shared resources
static std::map<std::string, std::shared_ptr<SMS_STS>> serial_port_connections;
static std::mutex serial_port_mutex;
```

**Mechanism:**
- First motor on a port creates the connection
- Subsequent motors reuse the existing connection
- Mutex protects concurrent access during setup/teardown
- Only the connection owner closes the port on cleanup

---

## Communication Flow

### Read Cycle (State Updates)

```
Controller Manager (100Hz typical)
         │
         ├─> read() called
         │
         ├─> FeedBack(motor_id)  ─────┐
         │                             │
         │   ┌───────────────────────┐ │
         │   │ SCServo SDK           │ │
         │   │ - Send read request   │◄┘
         │   │ - Receive response    │
         │   │ - Cache all values    │
         │   └───────────────────────┘
         │                             │
         ├─> ReadPos(-1)  ────────────┼─> Get cached position
         ├─> ReadSpeed(-1) ───────────┼─> Get cached velocity
         ├─> ReadLoad(-1) ────────────┼─> Get cached load
         ├─> ReadVoltage(-1) ─────────┼─> Get cached voltage
         ├─> ReadTemper(-1) ──────────┼─> Get cached temperature
         ├─> ReadCurrent(-1) ─────────┼─> Get cached current
         ├─> ReadMove(-1) ────────────┼─> Get cached motion status
         │
         ├─> Convert to SI units (rad, rad/s, %, V, °C, A)
         ├─> Update hw_state_* variables
         ├─> Update diagnostics
         └─> return OK
```

**Note:** Using `-1` as motor ID reads from the cache populated by `FeedBack()`, avoiding multiple serial transactions.

### Write Cycle (Command Execution)

```
Controller Manager
         │
         ├─> write() called
         │
         ├─> Check emergency_stop flag
         │      │
         │      ├─ Active? → Stop motor immediately
         │      └─ Not active? → Continue
         │
         ├─> Select based on operating_mode:
         │
         ├─ Mode 0 (Servo):
         │   ├─> Clamp position to limits
         │   ├─> Convert rad → steps
         │   ├─> Convert velocity to max_speed
         │   └─> WritePosEx(motor_id, pos, speed, accel)
         │
         ├─ Mode 1 (Velocity):
         │   ├─> Clamp velocity to limits
         │   ├─> Convert rad/s → steps/s
         │   └─> WriteSpe(motor_id, speed, accel)
         │
         ├─ Mode 2 (PWM):
         │   ├─> Clamp effort to limits
         │   ├─> Convert -1.0 to +1.0 → -1000 to +1000
         │   └─> WritePwm(motor_id, pwm)
         │
         └─> return OK or ERROR
```

---

## Operating Modes

### Mode 0: Servo (Position Control)

**Characteristics:**
- Motor moves to target position
- Speed and acceleration are limits, not targets
- Motor holds position when reached
- Best for precise positioning tasks

**Command Workflow:**
```
Target Position (rad) ─┐
Max Speed (rad/s)      ├─> WritePosEx() ─> Motor
Acceleration (0-254)  ─┘
```

**Use Cases:**
- Joint positioning (robot arms)
- Gripper control
- Any application requiring position hold

### Mode 1: Velocity (Closed-Loop Speed Control)

**Characteristics:**
- Motor maintains target velocity
- Acceleration controls ramp up/down
- No position hold (continuous rotation)
- Best for wheels and continuous motion

**Command Workflow:**
```
Target Velocity (rad/s) ─┐
Acceleration (0-254)      ├─> WriteSpe() ─> Motor
                         ─┘
```

**Use Cases:**
- Wheel motors (mobile robots)
- Conveyor belts
- Fans, pumps

### Mode 2: PWM (Open-Loop Effort Control)

**Characteristics:**
- Direct PWM control (no feedback loop)
- Effort = torque/force (proportional to PWM)
- No speed or position regulation
- Lowest level control

**Command Workflow:**
```
PWM Duty Cycle (-1.0 to +1.0) ─> WritePwm() ─> Motor
```

**Use Cases:**
- Custom control algorithms
- Force control applications
- Testing and calibration

---

## Lifecycle Management

The interface implements the ROS 2 lifecycle state machine:

```
         ┌────────────────┐
         │   Unconfigured │ (on_init called)
         └────────┬───────┘
                  │ configure
         ┌────────▼───────┐
         │   Inactive     │ (on_configure called)
         │                │ - Open serial port
         │                │ - Verify motor connectivity
         └────┬──────┬────┘
              │      │
      activate│      │cleanup
              │      │
         ┌────▼──────▼────┐
         │   Active        │ (on_activate called)
         │                 │ - Initialize motor mode
         │                 │ - Enable torque
         │                 │ - Start control loop
         └────┬──────┬─────┘
              │      │
    deactivate│      │error
              │      │
         ┌────▼──────▼────┐
         │  Finalized      │ (on_deactivate/on_error called)
         │                 │ - Stop motor
         │                 │ - Disable torque
         └─────────────────┘
                  │ shutdown
                  ▼
         ┌─────────────────┐
         │    Shutdown     │ (on_shutdown called)
         │                 │ - Emergency stop
         │                 │ - Close serial port
         └─────────────────┘
```

### Lifecycle Callbacks

**on_init()** - *MISSING IN CURRENT IMPLEMENTATION*
- Parse URDF parameters
- Validate configuration
- Initialize member variables

**on_configure()** - *MISSING IN CURRENT IMPLEMENTATION*
- Open serial port (or reuse existing)
- Ping motor to verify connectivity
- Set up diagnostic updater

**on_activate()** - *MISSING IN CURRENT IMPLEMENTATION*
- Initialize motor with operating mode
- Enable torque
- Ready for control loop

**on_deactivate()** - *IMPLEMENTED*
- Stop motor based on current mode
- Disable torque

**on_cleanup()** - *IMPLEMENTED*
- Close serial port (if owner)
- Release resources

**on_shutdown()** - *IMPLEMENTED*
- Emergency stop motor
- Disable torque
- Close serial port safely

**on_error()** - *IMPLEMENTED*
- Emergency stop with appropriate method per mode
- Disable torque for safety

---

## Error Handling & Recovery

### Error Detection

**Communication Errors:**
- Read failures tracked via `consecutive_read_errors_`
- Write failures tracked via `consecutive_write_errors_`
- SDK error codes logged via `servo_->getErr()`

**Triggering Recovery:**
- After `MAX_CONSECUTIVE_ERRORS` (5) consecutive failures
- `attempt_error_recovery()` is called automatically

### Recovery Process

```
┌─────────────────────────────────────────┐
│   Consecutive errors >= 5 detected      │
└───────────────┬─────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────┐
│  Step 1: Ping motor                     │
│  - servo_->Ping(motor_id)               │
└───────┬──────────────────┬──────────────┘
        │ Success          │ Failure
        │                  │
        ▼                  ▼
  ┌──────────┐     ┌────────────────────┐
  │  RETURN  │     │ Step 2: Reinit     │
  │  SUCCESS │     │ Serial Port        │
  └──────────┘     │ (if we own it)     │
                   └─────────┬──────────┘
                             │
                   ┌─────────▼──────────┐
                   │ servo_->end()      │
                   │ servo_->begin()    │
                   └─────────┬──────────┘
                             │
                   ┌─────────▼──────────┐
                   │ Step 3: Re-ping    │
                   │ motor              │
                   └─────────┬──────────┘
                             │
                   ┌─────────▼──────────┐
                   │ Step 4: Re-init    │
                   │ motor mode         │
                   │ InitMotor()        │
                   └─────────┬──────────┘
                             │
                        Success/Failure
                             │
                             ▼
                   ┌──────────────────┐
                   │  RETURN result   │
                   └──────────────────┘
```

### Emergency Stop

Triggered when `emergency_stop` command interface > 0.5:

1. **Immediate Action** (per mode):
   - Mode 0: Hold current position with zero speed
   - Mode 1: Stop with maximum deceleration
   - Mode 2: Cut PWM to zero

2. **Command Blocking**: Normal commands ignored while active

3. **Release**: When emergency_stop ≤ 0.5, resume normal operation

---

## Current Implementation Status

### ✅ Implemented (Partial - 796 lines)

The `.cpp` file contains implementations for:

1. **Lifecycle Callbacks:**
   - ✅ `on_deactivate()` - Lines 77-75
   - ✅ `on_cleanup()` - Lines 77-97
   - ✅ `on_shutdown()` - Lines 99-147
   - ✅ `on_error()` - Lines 149-196

2. **Control Loop:**
   - ✅ `read()` - Lines 198-366
   - ✅ `write()` - Lines 368-580

3. **Utility Functions:**
   - ✅ Unit conversion helpers (lines 584-654)
   - ✅ `attempt_error_recovery()` (lines 656-724)
   - ✅ `diagnostics_callback()` (lines 726-789)

4. **Plugin Export:**
   - ✅ Pluginlib macro (lines 793-796)

### ❌ Missing (Critical)

The following methods are **declared in the header** but **NOT implemented** in `.cpp`:

1. **on_init()** - *CRITICAL*
   - Parses URDF parameters
   - Validates motor ID, serial port, operating mode
   - Initializes member variables
   - Validates command interfaces vs operating mode
   - Sets up hardware limits

2. **on_configure()** - *CRITICAL*
   - Opens/reuses serial port connection
   - Pings motor to verify connectivity
   - Creates diagnostic updater
   - Essential for hardware initialization

3. **on_activate()** - *CRITICAL*
   - Initializes motor with `InitMotor(motor_id, operating_mode, 1)`
   - Enables torque
   - Required before read/write operations work

4. **export_state_interfaces()** - *CRITICAL*
   - Creates and returns state interfaces for ros2_control
   - Without this, no state data is available to controllers

5. **export_command_interfaces()** - *CRITICAL*
   - Creates and returns command interfaces for ros2_control
   - Without this, controllers cannot send commands

### Impact of Missing Methods

**Current State:** The package will **NOT work** because:
- Motors cannot be initialized (no `on_activate()`)
- Serial port never opens (no `on_configure()`)
- Parameters aren't loaded (no `on_init()`)
- Controllers cannot read state (no `export_state_interfaces()`)
- Controllers cannot send commands (no `export_command_interfaces()`)

**What Works:**
- The code compiles (interfaces are declared)
- Error handling, recovery, and diagnostics logic is solid
- Unit conversions are correct
- Lifecycle cleanup/shutdown works

**What Doesn't:**
- Anything requiring actual motor operation
- Integration with ros2_control framework

---

## Next Steps

### Priority 1: Complete Missing Implementation

**1. Implement `on_init()`**
```cpp
hardware_interface::CallbackReturn STSHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // Parse URDF parameters: serial_port, motor_id, baud_rate, operating_mode, etc.
  // Validate motor_id range (1-253)
  // Validate operating_mode (0, 1, or 2)
  // Load hardware limits from joint params
  // Validate command interfaces match operating mode
  // Initialize all member variables to safe defaults
  // Return SUCCESS or ERROR
}
```

**2. Implement `on_configure()`**
```cpp
hardware_interface::CallbackReturn STSHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Thread-safe serial port management:
  //   - Check if port already open (serial_port_connections map)
  //   - If not, create new SMS_STS and call begin()
  //   - If yes, reuse existing connection
  //   - Store in servo_ member
  // Ping motor to verify connectivity
  // Create diagnostic_updater
  // Return SUCCESS or ERROR
}
```

**3. Implement `on_activate()`**
```cpp
hardware_interface::CallbackReturn STSHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  // Skip if mock mode enabled
  // Initialize motor: InitMotor(motor_id_, operating_mode_, 1)
  // Verify initialization success
  // Read initial state to populate hw_state_* variables
  // Return SUCCESS or ERROR
}
```

**4. Implement `export_state_interfaces()`**
```cpp
std::vector<hardware_interface::StateInterface>
STSHardwareInterface::export_state_interfaces()
{
  // Create StateInterface for each state variable:
  //   - position
  //   - velocity
  //   - load
  //   - voltage
  //   - temperature
  //   - current
  //   - is_moving
  // Return vector of StateInterface objects
}
```

**5. Implement `export_command_interfaces()`**
```cpp
std::vector<hardware_interface::CommandInterface>
STSHardwareInterface::export_command_interfaces()
{
  // Create CommandInterface based on operating_mode_:
  //   Mode 0: position, velocity, acceleration
  //   Mode 1: velocity, acceleration
  //   Mode 2: effort
  //   All: emergency_stop
  // Return vector of CommandInterface objects
}
```

### Priority 2: Testing & Validation

After completing the implementation:

1. **Build Test:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select sts_hardware_interface
   ```

2. **Mock Mode Test:**
   - Test with `enable_mock_mode: true`
   - Verify all three operating modes
   - Check diagnostic output

3. **Hardware Test:**
   - Test with real STS motor
   - Verify communication
   - Test error recovery (disconnect/reconnect)
   - Monitor performance metrics

4. **Multi-Motor Test:**
   - Test multiple motors on same serial port
   - Verify thread-safe port sharing

5. **Integration Test:**
   - Use with ros2_control controller_manager
   - Test with velocity_controller or position_controller
   - Verify emergency stop functionality

### Priority 3: Documentation Updates

1. Update README with complete usage examples
2. Add troubleshooting guide
3. Document SCServo_Linux submodule setup
4. Create example launch files
5. Add parameter reference table

### Priority 4: Future Enhancements

1. **Sync Write Support** (mentioned in header but not implemented)
   - Batch commands to multiple motors
   - Reduce serial bus latency

2. **Dynamic Reconfiguration**
   - Runtime parameter updates
   - Mode switching without reinitialization

3. **Advanced Diagnostics**
   - Error rate tracking
   - Thermal warnings
   - Predictive maintenance

4. **Python Bindings**
   - Expose interface for Python-based testing
   - Scripted motor calibration

---

## Conclusion

The `sts_hardware_interface` package has a **solid architecture** and **comprehensive error handling**, but the implementation is **incomplete**. The missing lifecycle and interface export methods are critical and must be implemented before the package can be used.

Once the missing methods are added, this will be a **production-ready, robust hardware interface** suitable for any ROS 2 robot using Feetech STS servo motors.

### Estimated Effort

- **Implementing missing methods:** 4-6 hours
- **Testing with mock mode:** 1-2 hours
- **Hardware validation:** 2-4 hours
- **Documentation updates:** 2-3 hours

**Total:** 9-15 hours of development work

### Recommendations

1. **Immediate:** Implement the 5 missing critical methods
2. **Short-term:** Test thoroughly in both mock and hardware modes
3. **Medium-term:** Add sync write support and advanced features
4. **Long-term:** Consider contributing back to ROS 2 control tutorials as an example

---

*Document created: December 29, 2025*
*Package: sts_hardware_interface v0.1.0*
*License: MIT*
