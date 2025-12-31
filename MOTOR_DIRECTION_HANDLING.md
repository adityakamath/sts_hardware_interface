# Motor Direction Handling

## Overview

The STS Hardware Interface handles upside-down motor mounting exclusively through URDF joint axis definitions, with no software-level direction reversal in the hardware interface code.

## Architecture Decision

**Motor direction is controlled by URDF, not by hardware interface parameters.**

This design keeps the hardware interface truthful and simple - it reports exactly what the motors tell it, and sends exactly what the controller commands. All coordinate transformations for physical mounting configurations are handled at the URDF level where they belong.

## URDF Joint Axis Configuration

For upside-down motor mounting (common in mobile robot bases), the joint axis is reversed in the URDF:

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="..." rpy="..."/>
  <axis xyz="0 0 -1"/>  <!-- Reversed Z-axis for upside-down mounting -->
</joint>
```

### Standard vs Reversed Axis

- **Standard mounting** (motor shaft up): `<axis xyz="0 0 1"/>`
- **Upside-down mounting** (motor shaft down): `<axis xyz="0 0 -1"/>`

The URDF axis definition automatically handles:
- Command transformation: Controller commands are negated before reaching hardware
- Feedback transformation: Motor feedback is negated before reaching controller
- Visualization: RViz displays correct motion direction

## Hardware Interface Behavior

The hardware interface operates truthfully:

1. **Commands**: Receives velocity commands from controller and sends them to motors
   - No software reversal applied
   - URDF axis transformation already applied by ros2_control framework

2. **Feedback**: Reads position/velocity from motors and reports to controller
   - No software reversal applied
   - URDF axis transformation automatically applied by ros2_control framework

3. **Position zeroing**: Applied after multi-turn tracking, before any transformations
   - Works seamlessly with any joint axis configuration

## Benefits of URDF-Only Approach

1. **Single source of truth**: URDF is the authoritative definition of robot geometry
2. **Simpler hardware interface**: No need to track per-joint reversal flags
3. **Standard ROS 2 practice**: Aligns with ros2_control conventions
4. **Visualization consistency**: RViz automatically shows correct motion
5. **Less error-prone**: No possibility of double-reversal bugs

## Migration from Previous Implementation

Previous versions of the hardware interface included a `reverse_direction` parameter. This has been removed because:

1. It duplicated functionality already provided by URDF axis definitions
2. It could conflict with URDF axis settings (double reversal)
3. It added unnecessary complexity to the hardware interface
4. It was not standard ros2_control practice

**If upgrading from a version with `reverse_direction`:**

Simply remove the `<param name="reverse_direction">` lines from your URDF. Direction is now controlled exclusively by the joint axis.

## Example Configuration

### LeKiwi Robot (Three Omniwheels)

All motors are mounted upside-down, so all joints use reversed axis:

```xml
<!-- Left wheel joint -->
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel_link"/>
  <origin xyz="${wheel_offset * cos(offset)} ${wheel_offset * sin(offset)} 0"
          rpy="0 ${pi/2} ${offset}"/>
  <axis xyz="0 0 -1"/>  <!-- Reversed for upside-down mounting -->
</joint>
```

No `reverse_direction` parameter needed in ros2_control configuration:

```xml
<joint name="left_wheel_joint">
  <param name="motor_id">7</param>
  <param name="operating_mode">1</param>  <!-- Velocity mode -->
  <param name="max_velocity">15.0</param>
  <!-- No reverse_direction parameter - axis handles it -->

  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

## Testing

To verify correct direction handling:

1. **Visual test**: Launch with RViz and send velocity command
   ```bash
   ros2 launch lekiwi_base_control lekiwi_base.launch.py
   ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
     "{twist: {linear: {x: 0.1}}}" --once
   ```
   Wheels should rotate in the direction that moves robot forward

2. **Odometry test**: Move robot physically and check odometry
   ```bash
   ros2 topic echo /odom
   ```
   Position should increase when robot moves in positive direction

## Related Documentation

- [POSITION_ZEROING.md](POSITION_ZEROING.md) - How position offset works with axis definitions
- [LeKiwi URDF](../lekiwi_base_control/urdf/lekiwi_base.urdf.xacro) - Example configuration
- [ros2_control URDF specification](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html)

## Summary

**Bottom line**: Motor direction is a robot geometry concern, not a hardware interface concern. Use URDF joint axis definitions (`xyz="0 0 -1"` for upside-down motors), and let ros2_control handle the transformations automatically. The hardware interface remains simple and truthful.
