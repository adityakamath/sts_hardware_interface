---
layout: page
title: STS Hardware Interface Design
---

System design and implementation guide for the Feetech STS servo motor hardware interface.

## Overview

The STS Hardware Interface is a `ros2_control` SystemInterface plugin that connects ROS 2 controllers to Feetech STS series servo motors (STS3215 and compatible). It provides position, velocity, and effort control modes with full state feedback, safety features, and hardware-free simulation.

**Key Features:**
- Three operating modes per motor (Position/Servo, Velocity, PWM/Effort)
- Mixed-mode operation on the same serial bus
- Efficient multi-motor coordination with SyncWrite
- Full 7-interface state feedback (position, velocity, effort, voltage, temperature, current, motion status)
- Hardware-level emergency stop
- Automatic error recovery
- Mock mode for hardware-free development

---

## System Architecture

<div align="center">
  <img src="assets/img/system_architecture.svg" alt="System Architecture" width="600"/>
</div>

**Architecture Layers:**
1. **ROS 2 Controllers** - Standard ros2_control controllers (JointTrajectoryController, VelocityController, EffortController)
2. **Hardware Interface** - SystemInterface plugin bridging controllers to motor protocol
3. **Communication Layer** - SCServo protocol over RS485/TTL serial (half-duplex daisy-chain)
4. **Physical Motors** - Feetech STS series servo motors with unique IDs (1-253)

---

## Operating Modes

Each motor can be configured independently in one of three modes:

| Mode | Use Case | Command Interfaces | Position Limits | Velocity Semantics |
|------|----------|-------------------|-----------------|-------------------|
| **0: Position** | Arm joints, precise positioning | position, velocity†, acceleration† | 0 to 2π radians (configurable) | Maximum speed during position move |
| **1: Velocity** | Wheels, continuous rotation | velocity, acceleration† | Unlimited | Target velocity for continuous rotation |
| **2: PWM/Effort** | Force control, grippers | effort (-1.0 to +1.0) | N/A | N/A |

† Optional interfaces

**Important:** The `velocity` command interface has different semantics depending on operating mode:
- **Mode 0 (Position)**: Sets the maximum speed when moving to the commanded position
- **Mode 1 (Velocity)**: Sets the target continuous rotation speed

**Configuration Example:**

```xml
<!-- Mode 0: Position/Servo -->
<joint name="arm_joint">
  <param name="motor_id">1</param>
  <param name="operating_mode">0</param>
  <param name="min_position">0.0</param>
  <param name="max_position">6.283</param>
</joint>

<!-- Mode 1: Velocity -->
<joint name="wheel_joint">
  <param name="motor_id">2</param>
  <param name="operating_mode">1</param>
</joint>

<!-- Mode 2: PWM/Effort -->
<joint name="gripper_joint">
  <param name="motor_id">3</param>
  <param name="operating_mode">2</param>
  <param name="max_effort">0.8</param>
</joint>
```

---

## State Interfaces

All modes always export the following state interfaces for every joint:

| Interface | Unit | Description | Scaling Factor |
|-----------|------|-------------|----------------|
| `position` | radians | Current joint angle | 4096 steps = 2π rad |
| `velocity` | rad/s | Current angular velocity | 3400 steps/s max ≈ 5.22 rad/s |
| `effort` | -100 to +100% | Motor load percentage | 0.1% per unit |
| `voltage` | volts | Supply voltage | 0.1V per unit |
| `temperature` | °C | Motor temperature | Direct celsius |
| `current` | amperes | Motor current draw | 6.5mA per unit |
| `is_moving` | 0.0 or 1.0 | Motion status (1.0=moving) | Boolean |

**Note:** All state interfaces are always exported regardless of URDF configuration. URDF state interface declarations are optional but recommended for documentation purposes.

### Accessing State Interfaces

The `joint_state_broadcaster` publishes motor state to two topics:

**Standard state (`/joint_states`):**
- Message type: `sensor_msgs/JointState`
- Contains: `position`, `velocity`, and `effort` fields
- Always available without configuration

**Additional state (`/dynamic_joint_states`):**
- Message type: `control_msgs/DynamicJointState`
- Contains: `voltage`, `temperature`, `current`, `is_moving`
- Requires configuration in controller YAML:

```yaml
joint_state_broadcaster:
  ros__parameters:
    extra_joints:
      - wheel_joint
      - arm_joint
```

See [config/mixed_mode_controllers.yaml](../config/mixed_mode_controllers.yaml) for a complete example.

---

## Configuration Parameters

### Hardware Parameters

Configure these at the `<hardware>` level in your URDF:

| Parameter | Type | Default | Range/Options | Description |
|-----------|------|---------|---------------|-------------|
| `serial_port` | string | *required* | Valid path | Serial port path (e.g., `/dev/ttyACM0`) |
| `baud_rate` | int | 1000000 | 9600, 19200, 38400, 57600, 115200, 500000, 1000000 | Communication baud rate |
| `communication_timeout_ms` | int | 100 | 1-1000 | Serial communication timeout (ms) |
| `use_sync_write` | bool | true | true/false | Batch commands for multiple motors |
| `enable_mock_mode` | bool | false | true/false | Simulation mode (no hardware required) |

### Joint Parameters

Configure these per `<joint>` in your URDF:

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `motor_id` | int | *required* | 1-253 | Motor ID on serial bus |
| `operating_mode` | int | 1 | 0, 1, 2 | 0=Position, 1=Velocity, 2=PWM |
| `min_position` | double | 0.0 | any | Min position limit (radians, Mode 0 only) |
| `max_position` | double | 6.283 | any | Max position limit (radians, Mode 0 only) |
| `max_velocity` | double | 5.22 | any | Max velocity limit (rad/s, currently unused) |
| `max_effort` | double | 1.0 | 0.0-1.0 | Max PWM duty cycle (Mode 2 only) |

**Note:** `max_velocity` parameter is defined in the code but not currently parsed from URDF. The hardware uses a fixed maximum of 3400 steps/s (≈ 5.22 rad/s).

---

## Communication Protocol

### Serial Bus Configuration

- **Protocol:** Feetech STS/SCServo packet format
- **Bus type:** Half-duplex RS485 or TTL serial
- **Topology:** Daisy-chain (all motors on one bus, unique IDs 1-253)
- **Broadcast ID:** 254 (0xFE) - Reserved for emergency stop commands
- **Baud rates:** 9600, 19200, 38400, 57600, 115200, 500000, 1000000 (default: 1000000)

### SyncWrite Benefits and Tradeoffs

**Enabled (`use_sync_write: true`, default):**
- ✅ **Benefit:** All motors receive commands in single packet (~5ms vs ~15ms for 3 motors)
- ✅ **Benefit:** Atomic updates - all motors commanded simultaneously
- ⚠️ **Tradeoff:** No per-motor error reporting (SyncWrite returns void)
- **Best for:** Multi-motor systems where timing synchronization matters

**Disabled (`use_sync_write: false`):**
- ✅ **Benefit:** Individual error detection per motor command
- ⚠️ **Tradeoff:** Higher latency (~5ms per motor)
- **Best for:** Single motor setups, debugging communication failures

---

## Safety Features

### Emergency Stop

Emergency stop is a **hardware-level broadcast command** that stops all motors simultaneously using broadcast ID 254.

**Activation:**
```bash
ros2 topic pub /sts_system/emergency_stop std_msgs/msg/Bool "data: true"
```

**Behavior:**
1. Broadcasts stop command to ALL motors (ID 0xFE)
2. Uses maximum deceleration (acceleration=254)
3. Blocks all subsequent write commands until released
4. Emergency stop state persists until explicit release

**Release:**
```bash
ros2 topic pub /sts_system/emergency_stop std_msgs/msg/Bool "data: false"
```

**Important:** Emergency stop is NOT per-joint - it affects all motors on the bus simultaneously.

### Automatic Error Recovery

The hardware interface automatically recovers from communication failures:

**Recovery Trigger:**
- Activates after 5 consecutive read or write errors

**Recovery Process:**
1. Close serial port
2. Reopen serial connection
3. Ping all motors to verify presence
4. Reinitialize each motor with configured operating mode
5. Re-enable torque on all motors

**Recovery Failure:**
- If recovery fails, hardware interface transitions to ERROR state
- Manual intervention required (restart controller_manager or hardware interface)

---

## Mock Mode (Simulation)

Test controllers without hardware by setting `enable_mock_mode: true` in hardware configuration.

### Simulation Behavior

**Mode 0 (Position):**
- First-order position control with velocity limiting
- Simulates smooth approach to target position
- Respects commanded maximum velocity

**Mode 1 (Velocity):**
- Direct velocity integration to position
- Immediate velocity response

**Mode 2 (PWM/Effort):**
- PWM scaled to velocity (effort × 10.0 rad/s)
- Simplified torque-to-velocity model

**Additional Simulations:**
- **Load:** Based on velocity percentage (higher speed = higher load percentage)
- **Motion detection:** `is_moving` threshold at 0.01 rad/s
- **State variables:** Voltage, temperature, current remain at zero (not simulated)

Mock mode provides realistic command/state behavior for controller development without hardware.

---

## Unit Conversions

The STS motors use step-based units internally. The hardware interface converts between motor units and ROS 2 standard units:

### Position Conversion
- **Motor units:** 0-4095 steps (12-bit resolution)
- **ROS 2 units:** 0-2π radians (one full revolution)
- **Conversion:** `radians = steps × (2π / 4096)`
- **Note:** Position wraps at 2π (4096 steps)

### Velocity Conversion
- **Motor units:** ±3400 steps/s maximum
- **ROS 2 units:** ±5.22 rad/s maximum
- **Conversion:** `rad/s = steps/s × (2π / 4096)`

### Effort/Load Conversion
- **Motor units:** -1000 to +1000 (load percentage × 10)
- **ROS 2 units:** -100.0 to +100.0%
- **Conversion:** `percentage = load_raw × 0.1`

### Other State Conversions
- **Voltage:** `volts = raw × 0.1` (raw 100 = 10.0V)
- **Current:** `amperes = raw × 0.0065` (raw 100 = 0.65A)
- **Temperature:** Direct celsius value

---

## ROS 2 Controller Compatibility

| Controller | Package | Use Case | Compatible Modes |
|------------|---------|----------|------------------|
| `JointTrajectoryController` | joint_trajectory_controller | Arm manipulation, precise positioning | Mode 0 |
| `JointGroupVelocityController` | velocity_controllers | Wheels, continuous motion | Mode 1 |
| `JointGroupEffortController` | effort_controllers | Force control, grippers | Mode 2 |
| `DiffDriveController` | diff_drive_controller | Differential drive robots | Mode 1 |
| `ForwardCommandController` | forward_command_controller | Direct control, testing | All modes |

**Controller Configuration Requirements:**
- All controllers require `joint_state_broadcaster` to be running
- Position controllers (Mode 0) need `position` and `velocity` command interfaces
- Velocity controllers (Mode 1) need `velocity` command interface
- Effort controllers (Mode 2) need `effort` command interface

---

## Example Configurations

### Single Motor (Velocity Mode)

See [config/single_motor.urdf.xacro](../config/single_motor.urdf.xacro):
- One motor in velocity mode
- Disabled SyncWrite (single motor)
- Configurable motor ID via launch argument

### Mixed Mode (Multi-Motor)

See [config/mixed_mode.urdf.xacro](../config/mixed_mode.urdf.xacro):
- Three motors in different modes
- Enabled SyncWrite for coordination
- Demonstrates position, velocity, and effort control

---

## Troubleshooting

| Issue | Possible Causes | Solutions |
|-------|----------------|-----------|
| **Motors not responding** | • Serial port permissions<br>• Wrong motor ID<br>• Baud rate mismatch<br>• Communication wiring | • `sudo chmod 666 /dev/ttyACM0`<br>• Verify motor IDs with vendor tools<br>• Check `baud_rate` matches motor config<br>• Test with `enable_mock_mode: true` |
| **Position drift/jumps** | • Incorrect position limits<br>• Position wrapping at 2π<br>• Encoder issues | • Verify `min_position`/`max_position` range<br>• Check position limits match mechanism<br>• Monitor raw encoder values |
| **Communication errors** | • Controller update rate too high<br>• Too many motors on bus<br>• Cable quality issues | • Decrease controller `update_rate`<br>• Enable `use_sync_write: true`<br>• Reduce number of state interfaces<br>• Test with single motor first |
| **Emergency stop stuck** | • Emergency stop not released<br>• Hardware error state | • Publish `data: false` to emergency_stop topic<br>• Restart controller_manager<br>• Check motor error states |
| **Consecutive errors** | • Loose connections<br>• Power supply issues<br>• Motor firmware errors | • Check serial cable connections<br>• Verify motor power supply (6-12V)<br>• Monitor error recovery attempts |

---

## Further Reading

- [Quick Start Guide](quick-start.md) - Setup and usage examples
- [ros2_control Documentation](https://control.ros.org/)
- [Feetech STS3215 Documentation](https://www.feetechrc.com/2020-05-13_56655.html)
- [SCServo_Linux Library](https://github.com/adityakamath/SCServo_Linux)
