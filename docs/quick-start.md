# Quick Start Guide

Complete setup and usage instructions for the STS Hardware Interface.

---

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/sts_hardware_interface.git
cd sts_hardware_interface
git submodule update --init --recursive
```

### 2. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
source install/setup.bash
```

---

## Hardware Setup

### 1. Connect Motors

- Connect Feetech STS servos to USB-to-serial adapter (RS485 or TTL)
- Use daisy-chain topology to connect multiple motors on a single bus
- This hardware interface supports **1 to 253 motors** on a single serial bus
- Default baud rate: 1000000
- Ensure each motor has a unique ID (1-253)

### 2. Configure Motor IDs

Use the Feetech debugging software or vendor tools to:

- Assign unique IDs to each motor
- Set appropriate baud rate
- Verify motors respond to commands

### 3. Find Serial Port

```bash
# List available serial ports
ls /dev/tty*

# Common ports: /dev/ttyUSB0, /dev/ttyACM0
```

### 4. Set Serial Port Permissions

```bash
# Give user access to serial port
sudo chmod 666 /dev/ttyACM0

# Or add user to dialout group (permanent solution, requires logout)
sudo usermod -aG dialout $USER
```

---

## Running the Examples

This package provides ready-to-use example launch files that demonstrate different use cases.

### Example 1: Single Motor (Velocity Mode)

The simplest example with one motor in velocity mode.

**What it includes:**

- URDF: [config/single_motor.urdf.xacro](../config/single_motor.urdf.xacro)
- Controllers: [config/velocity_controller.yaml](../config/velocity_controller.yaml)
- Launch file: [launch/single_motor.launch.py](../launch/single_motor.launch.py)

**How to run:**

```bash
# With real hardware
ros2 launch sts_hardware_interface single_motor.launch.py serial_port:=/dev/ttyACM0

# With custom motor ID (default is 1)
ros2 launch sts_hardware_interface single_motor.launch.py serial_port:=/dev/ttyACM0 motor_id:=5

# In mock mode (no hardware required)
ros2 launch sts_hardware_interface single_motor.launch.py use_mock:=true

# With GUI for manual control
ros2 launch sts_hardware_interface single_motor.launch.py use_mock:=true gui:=true
```

**What it does:**

1. Loads robot description with one continuous joint (`wheel_joint`)
2. Starts `ros2_control` node with velocity controller
3. Spawns `joint_state_broadcaster` and `velocity_controller`

**Mock Mode Behavior:**

When running in mock mode (`use_mock:=true`), the hardware interface simulates realistic sensor feedback:

- **Voltage:** ~12V with load-dependent voltage drop (10-12V range)
- **Temperature:** Ambient 25°C + thermal heating from velocity/load (typically 25-40°C)
- **Current:** Proportional to motor effort (up to ~1A at max load)
- **Emergency stop:** Clears all command interfaces to match real hardware behavior

**Test the motor:**

```bash
# Command velocity (rad/s)
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [2.0]"

# Monitor joint state
ros2 topic echo /joint_states

# Monitor additional state (voltage, temperature, current, is_moving)
ros2 topic echo /dynamic_joint_states
```

### Example 2: Mixed Mode (Multiple Motors)

Demonstrates three motors in different operating modes on the same serial bus.

**What it includes:**

- URDF: [config/mixed_mode.urdf.xacro](../config/mixed_mode.urdf.xacro)
- Controllers: [config/mixed_mode_controllers.yaml](../config/mixed_mode_controllers.yaml)
- Launch file: [launch/mixed_mode.launch.py](../launch/mixed_mode.launch.py)

**Motors configured:**

- Motor 1 (`wheel_joint`): **Mode 1** - Velocity control (closed-loop)
- Motor 2 (`arm_joint`): **Mode 0** - Position/servo control (closed-loop)
- Motor 3 (`gripper_joint`): **Mode 2** - PWM/effort control (open-loop)

**How to run:**

```bash
# With real hardware (requires 3 motors with IDs 1, 2, 3)
ros2 launch sts_hardware_interface mixed_mode.launch.py serial_port:=/dev/ttyACM0

# In mock mode
ros2 launch sts_hardware_interface mixed_mode.launch.py use_mock:=true
```

**What it does:**

1. Loads robot description with three joints in different modes
2. Starts `ros2_control` node with SyncWrite enabled for efficiency
3. Spawns multiple controllers:
   - `joint_state_broadcaster` - publishes joint states
   - `wheel_controller` - velocity control for wheel
   - `arm_controller` - trajectory control for arm
   - `gripper_controller` - effort control for gripper

**Test each motor:**

```bash
# Wheel (velocity command)
ros2 topic pub /wheel_controller/commands std_msgs/msg/Float64MultiArray "data: [3.0]"

# Arm (position trajectory)
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: ['arm_joint'],
      points: [{positions: [1.0], time_from_start: {sec: 2}}]
    }
  }"

# Gripper (effort command)
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]"
```

---

## Launch File Arguments

<style>
  .args-table {
    transition: all 0.2s ease;
  }

  .args-table:hover {
    transform: translateY(-2px);
    box-shadow: 0 6px 16px rgba(0,0,0,0.25) !important;
  }
</style>

<table class="args-table" style="width: 100%; border-collapse: separate; border-spacing: 0; margin: 2em auto; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 12px rgba(0,0,0,0.2); border: none;">
  <thead>
    <tr>
      <th colspan="5" style="text-align: center; padding: 0.6em; background: #f8f9fa; border: none;">🚀  Launch File Arguments</th>
    </tr>
    <tr>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Argument</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Type</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Default</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Available In</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>serial_port</code></td>
      <td style="padding: 0.6em; border: none;">string</td>
      <td style="padding: 0.6em; border: none;"><code>/dev/ttyACM0</code></td>
      <td style="padding: 0.6em; border: none;">Both</td>
      <td style="padding: 0.6em; border: none;">Serial port path</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>baud_rate</code></td>
      <td style="padding: 0.6em; border: none;">int</td>
      <td style="padding: 0.6em; border: none;"><code>1000000</code></td>
      <td style="padding: 0.6em; border: none;">Both</td>
      <td style="padding: 0.6em; border: none;">Communication baud rate</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>use_mock</code></td>
      <td style="padding: 0.6em; border: none;">bool</td>
      <td style="padding: 0.6em; border: none;"><code>false</code></td>
      <td style="padding: 0.6em; border: none;">Both</td>
      <td style="padding: 0.6em; border: none;">Enable mock mode (no hardware)</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>motor_id</code></td>
      <td style="padding: 0.6em; border: none;">int</td>
      <td style="padding: 0.6em; border: none;"><code>1</code></td>
      <td style="padding: 0.6em; border: none;">single_motor only</td>
      <td style="padding: 0.6em; border: none;">Motor ID on serial bus</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>gui</code></td>
      <td style="padding: 0.6em; border: none;">bool</td>
      <td style="padding: 0.6em; border: none;"><code>false</code></td>
      <td style="padding: 0.6em; border: none;">single_motor only</td>
      <td style="padding: 0.6em; border: none;">Launch joint_state_publisher_gui for manual control</td>
    </tr>
  </tbody>
</table>

---

## Testing and Monitoring

### 1. Verify Hardware Connection

```bash
ros2 control list_hardware_interfaces
```

Expected output shows available command and state interfaces for each joint.

### 2. Monitor Joint States

**Standard state** (`/joint_states`):

```bash
ros2 topic echo /joint_states  # position, velocity, effort
```

**Additional state** (`/dynamic_joint_states`):

```bash
ros2 topic echo /dynamic_joint_states  # voltage, temperature, current, is_moving
```

To enable `/dynamic_joint_states`, list all desired state interfaces in your controller YAML:

```yaml
joint_state_broadcaster:
  ros__parameters:
    joints:
      - wheel_joint
      - arm_joint
    interfaces:
      - position
      - velocity
      - effort
      - voltage
      - temperature
      - current
      - is_moving
```

See [config/mixed_mode_controllers.yaml](../config/mixed_mode_controllers.yaml) for complete example.

### 3. Check Controller Status

```bash
# List all controllers
ros2 control list_controllers

# Check specific controller state
ros2 control list_hardware_components
```

### 4. Emergency Stop Testing

```bash
# Activate emergency stop (stops ALL motors)
ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true"

# Release emergency stop
ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: false"
```

---

## Troubleshooting

### Motors Not Responding

**Problem:** Motors don't move or hardware interface fails to start.

**Solutions:**

1. **Check serial port permissions:**
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```

2. **Verify motor IDs match configuration:**
   - Use vendor tools to check actual motor IDs
   - Ensure URDF `motor_id` parameters match physical motors

3. **Test baud rate:**
   - Try different baud rates: `baud_rate:=115200`
   - Verify motor baud rate matches configuration

4. **Test in mock mode:**
   ```bash
   ros2 launch sts_hardware_interface single_motor.launch.py use_mock:=true
   ```
   If mock mode works, issue is hardware/communication related.

### Communication Errors

**Problem:** Frequent "Failed to read feedback" or "Failed to write command" errors.

**Solutions:**

1. **Check cable quality and connections:**
   - Ensure solid connections at both ends
   - Try a different USB-to-serial adapter
   - Check for loose daisy-chain connections

2. **Reduce controller update rate:**
   ```yaml
   controller_manager:
     ros__parameters:
       update_rate: 50  # Reduce from 100 Hz
   ```

3. **Enable SyncWrite for multi-motor setups:**
   ```xml
   <param name="use_sync_write">true</param>
   ```

4. **Test with single motor first:**
   - Isolate communication issues by testing one motor
   - Add motors incrementally

### Position Jumps or Drift

**Problem:** Motor position jumps unexpectedly or drifts over time.

**Solutions:**

1. **Verify position and velocity limits:**
   ```xml
   <param name="min_position">0.0</param>
   <param name="max_position">6.283</param>  <!-- 2π radians -->
   <param name="max_velocity">5.22</param>   <!-- rad/s, optional -->
   ```

2. **Check for position wrapping:**
   - STS motors wrap at 2π radians (4096 steps)
   - Ensure mechanism doesn't require >2π range

3. **Monitor encoder values:**
   ```bash
   ros2 topic echo /joint_states
   ```

### Emergency Stop Stuck

**Problem:** Motors won't accept commands after emergency stop.

**Solutions:**

1. **Release emergency stop:**
   ```bash
   ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: false"
   ```

2. **Restart controller manager:**
   ```bash
   ros2 control reload_controller_libraries
   ```

3. **Check motor error states:**
   ```bash
   ros2 topic echo /dynamic_joint_states
   ```

### Mock Mode Not Working

**Problem:** Mock mode fails to start or behave correctly.

**Solutions:**

1. **Verify mock mode is enabled:**
   ```bash
   ros2 launch sts_hardware_interface single_motor.launch.py use_mock:=true
   ```

2. **Check for conflicting hardware:**
   - Ensure real hardware is disconnected
   - Serial port should not be in use

3. **Monitor simulated state:**
   ```bash
   ros2 topic echo /joint_states
   ros2 topic echo /dynamic_joint_states  # Check voltage, temperature, current
   ```
   Mock mode simulates realistic velocity integration and sensor feedback (voltage ~12V, temperature 25-40°C, current proportional to load).

---

## Next Steps

### For Basic Users

- Modify `motor_id` values to match your hardware
- Adjust `baud_rate` if using different motor configuration
- Test emergency stop behavior for safety validation

### For Advanced Users

- Create custom URDF configurations for your robot
- Implement custom controllers using ros2_control framework
- Configure mixed-mode operation for complex mechanisms
- See the [Design document](design.md) for detailed implementation information

### For Developers

- Enable mock mode for hardware-free development
- Study example configurations in `config/` directory
- Review the [Design document](design.md) for system design details
- Contribute improvements via GitHub pull requests

---

## Additional Resources

- [sts_hardware_interface README](https://github.com/adityakamath/sts_hardware_interface/blob/main/README.md)
- [Design documentation](design.md)
- [ros2_control documentation](https://control.ros.org/)
- [Feetech STS3215 documentation](https://www.feetechrc.com/2020-05-13_56655.html)
- [Original FTServo_Linux SDK](https://github.com/ftservo/FTServo_Linux)
- [SCServo_Linux SDK](https://github.com/adityakamath/SCServo_Linux)
