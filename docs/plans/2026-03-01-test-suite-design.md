# Test Suite Design for `sts_hardware_interface`

**Date:** 2026-03-01
**Status:** Approved
**Approach:** B — GTest unit tests + Python launch integration tests

---

## Context

The package currently has no tests. The `CMakeLists.txt` has a commented-out `TODO` block
pointing at `test_conversions.cpp` and `ament_cmake_gtest`. The goal is maximum coverage
using existing launch files, config files, and URDF xacros, without requiring real hardware.
All hardware interface tests run in mock mode (`enable_mock_mode: true`).

The test suite also validates the service introspection feature added to `/emergency_stop`
(publishes to `/emergency_stop/_service_event`).

---

## File Structure

```
test/
├── test_conversions.cpp           # GTest, no ROS — all conversion functions
├── test_hardware_interface.cpp    # GTest + rclcpp + ResourceManager — lifecycle & behavior
├── test_single_motor.launch.py    # launch_testing — wraps single_motor.launch.py
└── test_mixed_mode.launch.py      # launch_testing — wraps mixed_mode.launch.py
```

---

## Test 1: `test_conversions.cpp`

**Framework:** GTest only (no rclcpp, no ROS overhead)
**Links against:** `include/` directory directly

Tests all 8 pure functions in `sts_hardware_interface/sts_conversions.hpp`.

### Coverage

| Function | Test Cases |
|---|---|
| `raw_position_to_radians` | raw=0→~2π, raw=4095→0.0, raw=2048→~π, direction inversion confirmed |
| `radians_to_raw_position` | 0.0→0, 2π wraps to 0, π→~2048, negative input wraps, clamping at boundaries |
| Position round-trip | `raw_position_to_radians(radians_to_raw_position(x)) ≈ x` within 1-step tolerance |
| `raw_velocity_to_rad_s` | 0→0.0, positive raw→negative rad/s, negative raw→positive rad/s, magnitude |
| `rad_s_to_raw_velocity` | 0.0→0, sign inversion, clamping at ±max_velocity_steps |
| Velocity round-trip | `raw_velocity_to_rad_s(rad_s_to_raw_velocity(v, max)) ≈ v` within clamped range |
| `effort_to_raw_pwm` | 0.0→0, 1.0→1000, −1.0→−1000, 2.0→1000 (clamped), 0.5→500 |
| `clamp_acceleration` | 0.0→0, 127.0→127, −1.0→0 (clamped), 255.0→254 (clamped) |
| `normalize_effort` | no-limit passthrough, clamp to ±max_effort, at-boundary value |
| `apply_limit<double>` | no-limit passthrough, within-range clamp, boundary values |

**~36 test cases total.**

---

## Test 2: `test_hardware_interface.cpp`

**Framework:** GTest + rclcpp
**Links against:** `${PROJECT_NAME}` shared library

Uses `hardware_interface::ResourceManager` with inline URDF strings that mirror the
existing xacro configs. No serial hardware required — all tests use `enable_mock_mode: true`.

### Group 1 — Parameter Validation (`on_init`)

**Valid configurations (expect init SUCCESS):**
- Single velocity joint — mirrors `single_motor.urdf.xacro` defaults
- Single servo joint (mode 0)
- Single PWM joint (mode 2)
- 6-joint mixed mode — mirrors `mixed_mode.urdf.xacro`

**Invalid configurations (expect init ERROR):**
- Missing `serial_port` parameter
- `communication_timeout_ms` = 0 (below minimum)
- `communication_timeout_ms` = 1001 (above maximum)
- `max_velocity_steps` ≤ 0
- No joints defined
- Missing `motor_id` on a joint
- `motor_id` = 0 (below valid range 1–253)
- `motor_id` = 254 (above valid range, 254 is broadcast ID)
- Duplicate `motor_id` values across joints
- `operating_mode` = 3 (invalid, only 0/1/2 supported)
- `min_position` ≥ `max_position`
- `max_velocity` ≤ 0
- `max_effort` > 1.0
- `max_effort` ≤ 0

### Group 2 — Interface Exports

After valid `on_init`, verify correct interface type and count:

| Config | State Interfaces | Command Interfaces |
|---|---|---|
| Single velocity joint | 7 (pos, vel, eff, volt, temp, curr, is_moving) | 2 (velocity, acceleration) |
| Single servo joint | 7 | 3 (position, velocity, acceleration) |
| Single PWM joint | 7 | 1 (effort) |
| 6-joint mixed mode | 42 (7×6) | 12 (3+3+2+2+1+1) |

### Group 3 — Lifecycle in Mock Mode

After configure:
- `/emergency_stop` service exists and is callable
- `/emergency_stop/_service_event` topic is advertised (validates introspection feature)

After activate with `reset_states_on_activate: true`:
- All position, velocity, effort states are zero

After activate with `reset_states_on_activate: false`:
- States retain values from previous read

After deactivate:
- States zeroed

After cleanup:
- Service is destroyed

### Group 4 — Mock Read/Write

- Velocity mode: written velocity propagates to velocity state after `read()`
- Servo mode: written position is reflected in position state after `read()`
- PWM mode: written effort is reflected in state after `read()`

### Group 5 — Emergency Stop

- Activate (`data: true`): `success: true`, message contains "activated", all commands
  zeroed in next `write()`
- Release (`data: false`): `success: true`, message contains "released", commands resume
- Double-activate (call `data: true` twice): returns success, no crash (idempotent)

---

## Test 3: `test_single_motor.launch.py`

**Framework:** `launch_testing`
**Launches:** `single_motor.launch.py use_mock:=true`

Assertions (with per-test timeout):
1. `/emergency_stop` service becomes available
2. `/emergency_stop/_service_event` topic is advertised (end-to-end introspection check)
3. `/joint_states` is published and contains `wheel_joint`
4. `velocity_controller` is in `active` state via `/controller_manager/list_controllers`

---

## Test 4: `test_mixed_mode.launch.py`

**Framework:** `launch_testing`
**Launches:** `mixed_mode.launch.py use_mock:=true`

Assertions:
1. `/emergency_stop` service and `_service_event` topic are available
2. `/joint_states` contains all 6 joints:
   `arm_joint_1`, `arm_joint_2`, `wheel_joint_1`, `wheel_joint_2`,
   `gripper_joint_1`, `gripper_joint_2`
3. `arm_controller`, `wheel_controller`, `gripper_controller` are all in `active` state

---

## Build Changes

### `CMakeLists.txt`

Replace the commented-out `TODO` block inside `if(BUILD_TESTING)` with:

```cmake
find_package(ament_cmake_gtest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

# Test 1: Pure conversion function unit tests (no ROS)
ament_add_gtest(test_conversions test/test_conversions.cpp)
target_include_directories(test_conversions PRIVATE include)

# Test 2: Hardware interface lifecycle and behavior (mock mode)
ament_add_gtest(test_hardware_interface test/test_hardware_interface.cpp)
target_link_libraries(test_hardware_interface ${PROJECT_NAME})

# Test 3: Single motor launch integration test
add_launch_test(test/test_single_motor.launch.py)

# Test 4: Mixed mode launch integration test
add_launch_test(test/test_mixed_mode.launch.py)
```

### `package.xml`

Add inside `<package>`:

```xml
<test_depend>ament_cmake_gtest</test_depend>
<test_depend>launch_testing_ament_cmake</test_depend>
<test_depend>launch_testing_ros</test_depend>
```

---

## Coverage Summary

| Area | Test File | Coverage |
|---|---|---|
| Conversion math | `test_conversions.cpp` | ~100% of `sts_conversions.hpp` |
| Parameter validation | `test_hardware_interface.cpp` | All error paths in `on_init` |
| Interface exports | `test_hardware_interface.cpp` | All 3 modes + mixed |
| Lifecycle | `test_hardware_interface.cpp` | configure/activate/deactivate/cleanup |
| Mock read/write | `test_hardware_interface.cpp` | All 3 operating modes |
| Emergency stop | `test_hardware_interface.cpp` | Activate, release, idempotent |
| Service introspection | Both GTest + launch tests | Topic advertised, end-to-end |
| Single motor stack | `test_single_motor.launch.py` | Full launch → controller active |
| Mixed mode stack | `test_mixed_mode.launch.py` | Full launch → all controllers active |
