# Quick Start Guide - STS Hardware Interface

Get your STS servo motors running with ros2_control in 5 minutes!

## Prerequisites

- ROS 2 (Humble, Iron, or newer)
- STS servo motors (Feetech STS series)
- USB-to-Serial adapter (or built-in serial port)
- motors configured with unique IDs (1-253)

## Installation

### 1. Clone the repository

```bash
cd ~/ros2_ws/src
git clone <your-repo-url> sts_hardware_interface
cd sts_hardware_interface
```

### 2. Initialize submodule

```bash
git submodule update --init --recursive
```

### 3. Build

```bash
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
source install/setup.bash
```

## Quick Test (No Hardware)

Test the interface without motors using mock mode:

```bash
# Launch single motor in simulation mode
ros2 launch sts_hardware_interface single_motor.launch.py use_mock:=true

# In another terminal, check hardware status
ros2 control list_hardware_interfaces

# Send velocity command
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [2.0]}"

# Monitor joint states
ros2 topic echo /joint_states
```

## Real Hardware Setup

### 1. Connect your motor

- Connect STS motor to USB-to-Serial adapter
- Power the motor (6-12V)
- Note the serial port: `/dev/ttyACM0` or `/dev/ttyUSB0`

### 2. Verify motor ID

Use the Feetech Debug tool or SCServo library to verify/set motor ID:

```bash
# Example: Motor should respond to ping at ID 1
# (Requires SCServo tools - see SCServo_Linux documentation)
```

### 3. Launch with real hardware

**Single Motor (Velocity Mode):**

```bash
# Launch with your serial port and motor ID
ros2 launch sts_hardware_interface single_motor.launch.py \
  serial_port:=/dev/ttyACM0 \
  motor_id:=1 \
  baud_rate:=1000000

# Send velocity command
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [5.0]}"

# Monitor diagnostics
ros2 topic echo /diagnostics
```

**Mixed-Mode (Wheel + Arm + Gripper):**

```bash
# Launch with 3 motors in different operating modes
ros2 launch sts_hardware_interface mixed_mode.launch.py \
  serial_port:=/dev/ttyACM0

# Send commands to different controllers
ros2 topic pub /wheel_controller/commands std_msgs/msg/Float64MultiArray "{data: [5.0]}"
ros2 topic pub /arm_controller/joint_trajectory ...
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5]}"
```

### 4. Alternative: Manual URDF Configuration

If you prefer to create your own URDF/launch files:

**Single Motor Example:**

```xml
<ros2_control name="wheel_controller" type="system">
  <hardware>
    <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">1000000</param>
  </hardware>

  <joint name="wheel_joint">
    <param name="motor_id">1</param>
    <param name="operating_mode">1</param>  <!-- Velocity mode (per-joint) -->
    <param name="max_velocity">15.0</param>  <!-- rad/s -->
    <command_interface name="velocity"/>
    <command_interface name="acceleration"/>
    <command_interface name="emergency_stop"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="load"/>
    <state_interface name="voltage"/>
    <state_interface name="temperature"/>
    <state_interface name="current"/>
    <state_interface name="is_moving"/>
  </joint>
</ros2_control>
```

**Three-Motor Chain Example (Same Mode):**

```xml
<ros2_control name="mobile_base" type="system">
  <hardware>
    <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">1000000</param>
    <param name="use_sync_write">true</param>  <!-- Enable SyncWrite -->
  </hardware>

  <joint name="wheel_left">
    <param name="motor_id">1</param>
    <param name="operating_mode">1</param>  <!-- Velocity mode -->
    <param name="max_velocity">15.0</param>
    <command_interface name="velocity"/>
    <command_interface name="acceleration"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="wheel_right">
    <param name="motor_id">2</param>
    <param name="operating_mode">1</param>  <!-- Velocity mode -->
    <param name="max_velocity">15.0</param>
    <command_interface name="velocity"/>
    <command_interface name="acceleration"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="wheel_back">
    <param name="motor_id">3</param>
    <param name="operating_mode">1</param>  <!-- Velocity mode -->
    <param name="max_velocity">15.0</param>
    <command_interface name="velocity"/>
    <command_interface name="acceleration"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

**Mixed-Mode Chain Example (NEW!):**

```xml
<ros2_control name="robot_arm" type="system">
  <hardware>
    <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">1000000</param>
    <param name="use_sync_write">true</param>  <!-- Automatic mode grouping -->
  </hardware>

  <joint name="wheel_joint">
    <param name="motor_id">1</param>
    <param name="operating_mode">1</param>  <!-- Velocity control -->
    <param name="max_velocity">15.0</param>
    <command_interface name="velocity"/>
    <command_interface name="acceleration"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="arm_joint">
    <param name="motor_id">2</param>
    <param name="operating_mode">0</param>  <!-- Position/servo control -->
    <param name="min_position">-1.57</param>
    <param name="max_position">1.57</param>
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="acceleration"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="gripper_joint">
    <param name="motor_id">3</param>
    <param name="operating_mode">2</param>  <!-- PWM/effort control -->
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="load"/>
  </joint>
</ros2_control>
```

See the complete URDF examples in `config/single_motor.urdf.xacro` and `config/mixed_mode.urdf.xacro`.

## Monitoring

### Check diagnostics

```bash
ros2 topic echo /diagnostics
```

Look for:
- Motor status (OK/WARN/ERROR)
- Temperature, voltage, current
- Load percentage
- Performance metrics (read/write duration)

### Monitor state

```bash
ros2 topic echo /joint_states
```

## Common Issues

### "Failed to open serial port"

**Cause:** Port doesn't exist or no permissions

**Fix:**
```bash
# Check available ports
ls /dev/ttyACM* /dev/ttyUSB*

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in

# Or use sudo (not recommended)
sudo chown $USER /dev/ttyACM0
```

### "Failed to ping motor"

**Cause:** Wrong motor ID, wrong baud rate, or motor not powered

**Fix:**
- Verify motor is powered (LED on)
- Check motor ID matches URDF
- Try different baud rates (115200, 1000000)
- Use Feetech Debug tool to verify motor responds

### "Communication timeout"

**Cause:** Baud rate mismatch or cable issue

**Fix:**
- Try `<param name="baud_rate">115200</param>`
- Check USB cable quality
- Reduce cable length (<3m recommended)

### Motors not moving

**Cause:** Torque not enabled or safety limits

**Fix:**
- Check controller is loaded and active
- Verify commands are being sent
- Check `emergency_stop` is not active (should be 0.0)
- Increase `max_velocity` limit if clamping

## Operating Modes

### Mode 0: Position Control (Servo)

**Use for:** Arms, grippers, joints that need to hold position

```xml
<param name="operating_mode">0</param>
```

```cpp
// Commands
position: 1.57      // Target: π/2 radians
velocity: 5.0       // Max speed limit
acceleration: 100   // Acceleration (0-254)
```

### Mode 1: Velocity Control (Default)

**Use for:** Wheels, continuous rotation

```xml
<param name="operating_mode">1</param>
```

```cpp
// Commands
velocity: 10.0      // Target: 10 rad/s
acceleration: 50    // Acceleration (0-254)
```

### Mode 2: PWM/Effort Control

**Use for:** Custom control, force control

```xml
<param name="operating_mode">2</param>
```

```cpp
// Commands
effort: 0.5         // 50% PWM (+/- 1.0)
```

## Performance Tips

### For Motor Chains (Multiple Motors)

1. **Enable SyncWrite** (default: enabled)
   ```xml
   <param name="use_sync_write">true</param>
   ```

2. **Use high baud rate**
   ```xml
   <param name="baud_rate">1000000</param>
   ```

3. **Group motors on same serial port**
   - All motors on same chain use same `serial_port`
   - Motors automatically use SyncWrite

### Expected Performance

| Motors | Configuration | Control Loop Speed |
|--------|--------------|-------------------|
| 1 | Single mode | 200 Hz |
| 3 | Same mode, SyncWrite | 150 Hz |
| 3 | Mixed modes, SyncWrite | 120-150 Hz |
| 3 | SyncWrite disabled | 50 Hz |
| 10 | Same mode, SyncWrite | 100 Hz |
| 10 | Mixed modes, SyncWrite | 80-100 Hz |

**Mixed-Mode Note**: When using motors with different operating modes, the interface automatically groups motors by mode and sends separate SyncWrite commands for each group. This is slightly slower than single-mode operation but still much faster than individual writes.

## Advanced Features

### Multi-Turn Tracking

**What It Does:** Tracks continuous position beyond ±180° for applications requiring multi-revolution tracking (e.g., mobile robot wheels, continuous rotation joints).

**The Problem:** STS servo motors internally report position as 0-4095 steps, which wraps around at ±π radians (±180°). Without multi-turn tracking, a motor rotating 3 full revolutions forward would report the same position as 1 revolution forward.

**The Solution:** When enabled, the hardware interface detects position wrap-around and maintains a revolution counter, providing true continuous position tracking.

**Enable Multi-Turn Tracking:**

```xml
<param name="enable_multi_turn">true</param>
```

**Use Cases:**

- **Mobile Robot Wheels** - Accurate odometry requires tracking total wheel rotation beyond ±180°
- **Multi-Revolution Joints** - Arms or gimbals that rotate more than one full revolution
- **Continuous Rotation** - Applications where position accumulates over time

**Impact:**

- **Code Cost:** 37 lines, negligible (~10 CPU instructions per read cycle)
- **Memory:** 16 bytes per joint (revolution counter + last position)
- **Performance:** No measurable impact on control loop frequency

**When to Disable:** Only disable if all joints are limited to ±180° range and never need position accumulation (saves ~16 bytes per joint).

### Emergency Stop

**Broadcast Emergency Stop (NEW!)** - Stop ALL motors at once:

```bash
# Stop ALL motors immediately
ros2 topic pub /hardware_name/broadcast_emergency_stop std_msgs/msg/Float64 "{data: 1.0}"

# Release (allow motion again)
ros2 topic pub /hardware_name/broadcast_emergency_stop std_msgs/msg/Float64 "{data: 0.0}"
```

This uses the STS broadcast ID (254/0xFE) to send a single stop command to all motors on the bus, regardless of their operating mode.

**Per-motor emergency stop:**

```bash
# Stop individual motor/joint
ros2 topic pub /joint_name/emergency_stop std_msgs/msg/Float64 "{data: 1.0}"

# Release
ros2 topic pub /joint_name/emergency_stop std_msgs/msg/Float64 "{data: 0.0}"
```

### Hardware Limits

Protect motors from exceeding safe ranges:

```xml
<joint name="arm_joint">
  <param name="motor_id">1</param>
  <param name="min_position">-1.57</param>  <!-- -π/2 rad -->
  <param name="max_position">1.57</param>   <!-- +π/2 rad -->
  <param name="max_velocity">10.0</param>   <!-- 10 rad/s -->
  <param name="max_effort">0.8</param>      <!-- 80% max -->
</joint>
```

Commands will be automatically clamped to these limits.

## Next Steps

1. **Read the full README** for detailed parameter descriptions
2. **Check IMPLEMENTATION_SUMMARY.md** for architecture details
3. **Review IMPROVEMENTS.md** for safety features
4. **Join the community** (if applicable - add your discussion forum/discord)

## Support

**Issues:** Report bugs at [GitHub Issues](your-repo-url/issues)

**Documentation:** See [README.md](README.md) for complete reference

**Examples:** Check the `examples/` directory (if you create one)

---

**Happy controlling! 🤖**
