# Position Zeroing Implementation

## Overview

The STS Hardware Interface now implements automatic position zeroing on initialization. When motors are activated, their first position reading is captured as the zero reference point, and all subsequent position feedback is relative to this initial reading.

## Implementation Details

### Changes Made

1. **Added member variable** (`sts_hardware_interface.hpp:227`):
   ```cpp
   std::vector<double> initial_position_offset_;  // Position offset captured on activation
   ```

2. **Initialized vector** (`sts_hardware_interface.cpp:130`):
   ```cpp
   initial_position_offset_.resize(num_joints, 0.0);
   ```

3. **Captured initial position in `on_activate()`** (`sts_hardware_interface.cpp:406-424`):
   - For real hardware: Read first position from motors and store as offset
   - For mock mode: Set offset to 0.0 (mock motors already start at 0)
   - Report zeroed position to logs

4. **Applied offset in `read()`** (`sts_hardware_interface.cpp:684-686`):
   - Subtract offset after position calculation (multi-turn or single-turn)
   - Apply before direction reversal to maintain correct coordinate transformation

## Behavior

### Before
- Motor position reported as absolute encoder value (e.g., 2.5 rad)
- Each motor could start at different position
- Odometry accumulated from arbitrary starting positions

### After
- All motors start at position 0.0 on activation
- Initial encoder readings (e.g., 2.5 rad, -1.3 rad, 0.8 rad) captured as offsets
- Subsequent positions relative to initial reading:
  - If motor was at 2.5 rad initially and moves to 3.0 rad → reported as 0.5 rad
  - If motor was at -1.3 rad initially and moves to -0.8 rad → reported as 0.5 rad

## Order of Operations in `read()`

Position feedback processing follows this exact order:

1. **Read raw position** from motor (0-4095 steps)
2. **Convert to radians** using `raw_position_to_radians()`
3. **Multi-turn tracking** (if enabled): accumulate revolutions
4. **Apply initial offset**: `position -= initial_position_offset_[i]` ← **NEW**
5. Report final position to controller

This order ensures:
- Offset is applied in the motor's natural coordinate frame
- Multi-turn tracking works correctly across zero point
- Coordinate transformations (e.g., for upside-down mounting) are handled via URDF joint axis

## Benefits

1. **Consistent starting conditions**: All joints start at 0.0 regardless of motor orientation
2. **Simplified calibration**: No need to manually align motors to specific positions
3. **Predictable odometry**: Odometry always starts from (0, 0, 0)
4. **Robust operation**: Works with multi-turn tracking and direction reversal

## Example

### Scenario: Three motors at different initial positions

**Initial State (on activation):**
- Left motor encoder: 1.5 rad → offset = 1.5 rad → reported as 0.0 rad
- Back motor encoder: -0.8 rad → offset = -0.8 rad → reported as 0.0 rad
- Right motor encoder: 2.3 rad → offset = 2.3 rad → reported as 0.0 rad

**After robot moves forward 0.5 rad:**
- Left motor encoder: 2.0 rad → 2.0 - 1.5 = **0.5 rad**
- Back motor encoder: -0.3 rad → -0.3 - (-0.8) = **0.5 rad**
- Right motor encoder: 2.8 rad → 2.8 - 2.3 = **0.5 rad**

All motors correctly report 0.5 rad relative motion.

## Compatibility

- **Multi-turn tracking**: Fully compatible - offset is subtracted from continuous position
- **Mock mode**: Fully compatible - offset initialized to 0.0 for simulated motors
- **Existing configurations**: No URDF changes required - feature is automatic
- **Motor orientation**: Works seamlessly with URDF joint axis definitions for upside-down motors

## Testing

To verify position zeroing:

```bash
# Launch robot
ros2 launch lekiwi_base_control lekiwi_base.launch.py

# Check initial positions (should all be 0.0)
ros2 topic echo /joint_states --once

# Send motion command
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1}}}" --once

# Verify positions changed from 0.0
ros2 topic echo /joint_states --once
```

## Implementation Files

- [include/sts_hardware_interface/sts_hardware_interface.hpp](include/sts_hardware_interface/sts_hardware_interface.hpp#L227)
- [src/sts_hardware_interface.cpp](src/sts_hardware_interface.cpp#L130) (initialization)
- [src/sts_hardware_interface.cpp](src/sts_hardware_interface.cpp#L406-424) (capture offset)
- [src/sts_hardware_interface.cpp](src/sts_hardware_interface.cpp#L684-686) (apply offset)
