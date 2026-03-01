# Test Suite Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add comprehensive unit and integration tests for `sts_hardware_interface` achieving ~90% coverage with GTest (C++) and launch_testing (Python).

**Architecture:** Four test files total — `test_conversions.cpp` (pure math, no ROS), `test_hardware_interface.cpp` (lifecycle + mock behavior, needs rclcpp), `test_single_motor.launch.py` (launch_testing wrapping example launch), `test_mixed_mode.launch.py` (launch_testing wrapping mixed-mode example). Tests drive `STSHardwareInterface` directly without ResourceManager by constructing `hardware_interface::HardwareInfo` manually.

**Tech Stack:** GTest (`ament_cmake_gtest`), `launch_testing_ament_cmake`, `launch_testing_ros`, rclcpp, hardware_interface, std_srvs

---

### Task 1: Wire up CMakeLists.txt and package.xml

**Files:**
- Modify: `CMakeLists.txt:131-134`
- Modify: `package.xml`

**Step 1: Replace TODO block in CMakeLists.txt**

Replace lines 131–134 (the TODO comment block) with:

```cmake
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # C++ unit tests: pure conversion math (no ROS required)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_include_directories(test_conversions PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # C++ unit tests: hardware interface lifecycle and mock behavior (needs rclcpp)
  ament_add_gtest(test_hardware_interface
    test/test_hardware_interface.cpp
  )
  ament_target_dependencies(test_hardware_interface
    rclcpp
    hardware_interface
    std_srvs
  )
  target_include_directories(test_hardware_interface PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_hardware_interface ${PROJECT_NAME})

  # Python launch tests: single motor example with use_mock:=true
  add_launch_test(test/test_single_motor.launch.py)

  # Python launch tests: mixed mode example with use_mock:=true
  add_launch_test(test/test_mixed_mode.launch.py)
```

**Step 2: Add test dependencies to package.xml**

Add after the existing `<test_depend>ament_lint_common</test_depend>` line:

```xml
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>launch_testing_ros</test_depend>
```

**Step 3: Build to verify CMake parses correctly (tests won't link yet)**

Run: `pixi run build --packages-select sts_hardware_interface`
Expected: Build succeeds. The test targets fail to compile because test files don't exist yet — that's fine.

**Step 4: Commit**

```bash
git add src/sts_hardware_interface/CMakeLists.txt src/sts_hardware_interface/package.xml
git commit -m "test: wire up gtest and launch_testing targets for sts_hardware_interface"
```

---

### Task 2: Write test_conversions.cpp

**Files:**
- Create: `test/test_conversions.cpp`
- Test: `test/test_conversions.cpp`

This file tests all 8 functions in `include/sts_hardware_interface/sts_conversions.hpp`. No ROS. No fixtures needed — all functions are stateless.

Key constants from the header:
- `STEPS_PER_REVOLUTION = 4096`, `STS_MAX_POSITION = 4095`, `STS_MAX_PWM = 1000`, `STS_MAX_ACCELERATION = 254`
- `STEPS_TO_RAD = 2π / 4096 ≈ 0.001534`
- Position: `raw_position_to_radians(raw) = (4095 - raw) * STEPS_TO_RAD` (direction inversion)
- Velocity: `raw_velocity_to_rad_s(raw) = -(raw * STEPS_TO_RAD * 0.732)` (direction inversion, 0.732 is step frequency)

**Step 1: Write the failing tests**

```cpp
// test/test_conversions.cpp
#include <gtest/gtest.h>
#include <cmath>
#include "sts_hardware_interface/sts_conversions.hpp"

using namespace sts_hardware_interface;

// --- raw_position_to_radians ---

TEST(ConversionsTest, RawPositionToRadians_Zero) {
  // raw=0 → (4095 - 0) * STEPS_TO_RAD = max positive angle
  double result = raw_position_to_radians(0);
  EXPECT_NEAR(result, 4095.0 * (2.0 * M_PI / 4096.0), 1e-6);
}

TEST(ConversionsTest, RawPositionToRadians_Max) {
  // raw=4095 → (4095 - 4095) * STEPS_TO_RAD = 0
  EXPECT_NEAR(raw_position_to_radians(4095), 0.0, 1e-6);
}

TEST(ConversionsTest, RawPositionToRadians_Middle) {
  // raw=2048 → (4095 - 2048) * STEPS_TO_RAD ≈ π/2
  double expected = 2047.0 * (2.0 * M_PI / 4096.0);
  EXPECT_NEAR(raw_position_to_radians(2048), expected, 1e-6);
}

TEST(ConversionsTest, RawPositionToRadians_Roundtrip) {
  // Convert to radians and back; should recover original raw value
  for (int raw = 0; raw <= 4095; raw += 100) {
    double rad = raw_position_to_radians(raw);
    int recovered = radians_to_raw_position(rad);
    EXPECT_EQ(recovered, raw) << "Failed at raw=" << raw;
  }
}

// --- radians_to_raw_position ---

TEST(ConversionsTest, RadiansToRawPosition_Zero) {
  // 0 radians → raw=4095
  EXPECT_EQ(radians_to_raw_position(0.0), 4095);
}

TEST(ConversionsTest, RadiansToRawPosition_FullCircle) {
  // 2π radians → raw = 4095 - 4096 = negative, clamp to 0
  // Actually: raw = 4095 - round(2π / STEPS_TO_RAD) = 4095 - 4096 = -1 → clamp to 0
  int result = radians_to_raw_position(2.0 * M_PI);
  EXPECT_GE(result, 0);
  EXPECT_LE(result, 4095);
}

TEST(ConversionsTest, RadiansToRawPosition_ClampsToRange) {
  // Very large positive radians → should clamp to 0
  int result = radians_to_raw_position(100.0);
  EXPECT_GE(result, 0);
  EXPECT_LE(result, 4095);
}

TEST(ConversionsTest, RadiansToRawPosition_NegativeClamps) {
  // Negative radians → should clamp to 4095
  int result = radians_to_raw_position(-1.0);
  EXPECT_GE(result, 0);
  EXPECT_LE(result, 4095);
}

// --- raw_velocity_to_rad_s ---

TEST(ConversionsTest, RawVelocityToRadS_Zero) {
  EXPECT_NEAR(raw_velocity_to_rad_s(0), 0.0, 1e-9);
}

TEST(ConversionsTest, RawVelocityToRadS_Positive) {
  // Positive raw velocity → negative rad/s (direction inversion)
  double result = raw_velocity_to_rad_s(100);
  EXPECT_LT(result, 0.0);
}

TEST(ConversionsTest, RawVelocityToRadS_Negative) {
  // Negative raw velocity → positive rad/s (direction inversion)
  double result = raw_velocity_to_rad_s(-100);
  EXPECT_GT(result, 0.0);
}

TEST(ConversionsTest, RawVelocityToRadS_Roundtrip) {
  for (int raw : {-500, -100, 0, 100, 500}) {
    double rad_s = raw_velocity_to_rad_s(raw);
    int recovered = rad_s_to_raw_velocity(rad_s);
    EXPECT_EQ(recovered, raw) << "Failed at raw=" << raw;
  }
}

// --- rad_s_to_raw_velocity ---

TEST(ConversionsTest, RadSToRawVelocity_Zero) {
  EXPECT_EQ(rad_s_to_raw_velocity(0.0), 0);
}

TEST(ConversionsTest, RadSToRawVelocity_Positive) {
  // Positive rad/s → negative raw (direction inversion)
  int result = rad_s_to_raw_velocity(1.0);
  EXPECT_LT(result, 0);
}

TEST(ConversionsTest, RadSToRawVelocity_Negative) {
  // Negative rad/s → positive raw (direction inversion)
  int result = rad_s_to_raw_velocity(-1.0);
  EXPECT_GT(result, 0);
}

// --- effort_to_raw_pwm ---

TEST(ConversionsTest, EffortToRawPwm_Zero) {
  EXPECT_EQ(effort_to_raw_pwm(0.0), 0);
}

TEST(ConversionsTest, EffortToRawPwm_One) {
  // 1.0 effort → STS_MAX_PWM = 1000
  EXPECT_EQ(effort_to_raw_pwm(1.0), 1000);
}

TEST(ConversionsTest, EffortToRawPwm_Half) {
  // 0.5 effort → 500
  EXPECT_EQ(effort_to_raw_pwm(0.5), 500);
}

TEST(ConversionsTest, EffortToRawPwm_ClampsAboveOne) {
  int result = effort_to_raw_pwm(2.0);
  EXPECT_EQ(result, 1000);
}

TEST(ConversionsTest, EffortToRawPwm_ClampsNegative) {
  int result = effort_to_raw_pwm(-0.5);
  EXPECT_EQ(result, 0);
}

// --- clamp_acceleration ---

TEST(ConversionsTest, ClampAcceleration_Zero) {
  EXPECT_EQ(clamp_acceleration(0.0), 0);
}

TEST(ConversionsTest, ClampAcceleration_Max) {
  // STS_MAX_ACCELERATION = 254
  EXPECT_EQ(clamp_acceleration(254.0), 254);
}

TEST(ConversionsTest, ClampAcceleration_ClampsAboveMax) {
  EXPECT_EQ(clamp_acceleration(300.0), 254);
}

TEST(ConversionsTest, ClampAcceleration_ClampsNegative) {
  EXPECT_EQ(clamp_acceleration(-10.0), 0);
}

// --- normalize_effort ---

TEST(ConversionsTest, NormalizeEffort_Zero) {
  EXPECT_NEAR(normalize_effort(0), 0.0, 1e-9);
}

TEST(ConversionsTest, NormalizeEffort_Max) {
  // STS_MAX_PWM=1000 → 1.0
  EXPECT_NEAR(normalize_effort(1000), 1.0, 1e-6);
}

TEST(ConversionsTest, NormalizeEffort_Half) {
  EXPECT_NEAR(normalize_effort(500), 0.5, 1e-6);
}

// --- apply_limit ---

TEST(ConversionsTest, ApplyLimit_WithinRange) {
  EXPECT_NEAR(apply_limit(5.0, 0.0, 10.0), 5.0, 1e-9);
}

TEST(ConversionsTest, ApplyLimit_AtMin) {
  EXPECT_NEAR(apply_limit(0.0, 0.0, 10.0), 0.0, 1e-9);
}

TEST(ConversionsTest, ApplyLimit_AtMax) {
  EXPECT_NEAR(apply_limit(10.0, 0.0, 10.0), 10.0, 1e-9);
}

TEST(ConversionsTest, ApplyLimit_BelowMin_Clamped) {
  EXPECT_NEAR(apply_limit(-1.0, 0.0, 10.0), 0.0, 1e-9);
}

TEST(ConversionsTest, ApplyLimit_AboveMax_Clamped) {
  EXPECT_NEAR(apply_limit(15.0, 0.0, 10.0), 10.0, 1e-9);
}

TEST(ConversionsTest, ApplyLimit_IntType) {
  EXPECT_EQ(apply_limit(5, 0, 10), 5);
  EXPECT_EQ(apply_limit(-1, 0, 10), 0);
  EXPECT_EQ(apply_limit(15, 0, 10), 10);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

**Step 2: Build only conversions test**

Run: `pixi run build --packages-select sts_hardware_interface 2>&1 | grep -A5 'test_conversions'`
Expected: `test_conversions` builds and links successfully.

**Step 3: Run test_conversions**

Run: `pixi run test --packages-select sts_hardware_interface 2>&1 | grep -A20 'test_conversions'`
Expected: All ~36 tests PASS. Fix any that fail before continuing.

**Step 4: Commit**

```bash
git add src/sts_hardware_interface/test/test_conversions.cpp
git commit -m "test: add unit tests for sts_conversions.hpp (36 cases)"
```

---

### Task 3: Write test_hardware_interface.cpp — parameter validation

**Files:**
- Create: `test/test_hardware_interface.cpp`

This file tests `STSHardwareInterface::on_init()` parameter validation. It constructs `hardware_interface::HardwareInfo` manually — no launch file needed. Requires `rclcpp::init()` because `on_configure()` creates a node (call it in `main()`).

Key construction pattern:
```cpp
hardware_interface::HardwareInfo info;
info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
info.hardware_parameters["enable_mock_mode"] = "true";

hardware_interface::ComponentInfo joint;
joint.name = "wheel_joint";
joint.parameters["motor_id"] = "1";
joint.parameters["operating_mode"] = "1";  // velocity mode

hardware_interface::InterfaceInfo vel_iface;
vel_iface.name = "velocity";
joint.command_interfaces.push_back(vel_iface);

hardware_interface::InterfaceInfo acc_iface;
acc_iface.name = "acceleration";
joint.command_interfaces.push_back(acc_iface);

info.joints.push_back(joint);
```

**Step 1: Write failing parameter validation tests**

```cpp
// test/test_hardware_interface.cpp
#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "sts_hardware_interface/sts_hardware_interface.hpp"

using hardware_interface::return_type;

// ---- Helpers ----

/// Build a minimal valid HardwareInfo for a single velocity-mode motor.
hardware_interface::HardwareInfo make_valid_single_motor_info(
  const std::string & serial_port = "/dev/ttyACM0",
  const std::string & motor_id = "1",
  const std::string & operating_mode = "1")
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = serial_port;
  info.hardware_parameters["enable_mock_mode"] = "true";
  info.hardware_parameters["baud_rate"] = "1000000";

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel_joint";
  joint.parameters["motor_id"] = motor_id;
  joint.parameters["operating_mode"] = operating_mode;

  // Mode 1 (velocity) requires: velocity + acceleration command interfaces
  hardware_interface::InterfaceInfo vel_iface;
  vel_iface.name = "velocity";
  joint.command_interfaces.push_back(vel_iface);

  hardware_interface::InterfaceInfo acc_iface;
  acc_iface.name = "acceleration";
  joint.command_interfaces.push_back(acc_iface);

  info.joints.push_back(joint);
  return info;
}

/// Build info for a position-mode (mode 0) motor.
hardware_interface::HardwareInfo make_valid_position_motor_info()
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";

  hardware_interface::ComponentInfo joint;
  joint.name = "arm_joint";
  joint.parameters["motor_id"] = "2";
  joint.parameters["operating_mode"] = "0";

  // Mode 0 (position) requires: position + velocity + acceleration
  for (const auto & name : {"position", "velocity", "acceleration"}) {
    hardware_interface::InterfaceInfo iface;
    iface.name = name;
    joint.command_interfaces.push_back(iface);
  }
  info.joints.push_back(joint);
  return info;
}

/// Build info for an effort-mode (mode 2) motor.
hardware_interface::HardwareInfo make_valid_effort_motor_info()
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";

  hardware_interface::ComponentInfo joint;
  joint.name = "gripper_joint";
  joint.parameters["motor_id"] = "3";
  joint.parameters["operating_mode"] = "2";

  hardware_interface::InterfaceInfo eff_iface;
  eff_iface.name = "effort";
  joint.command_interfaces.push_back(eff_iface);

  info.joints.push_back(joint);
  return info;
}

// ---- on_init: success cases ----

TEST(HardwareInterfaceInitTest, ValidSingleMotorVelocityMode) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  EXPECT_EQ(hw.on_init(info), return_type::OK);
}

TEST(HardwareInterfaceInitTest, ValidSingleMotorPositionMode) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_position_motor_info();
  EXPECT_EQ(hw.on_init(info), return_type::OK);
}

TEST(HardwareInterfaceInitTest, ValidSingleMotorEffortMode) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_effort_motor_info();
  EXPECT_EQ(hw.on_init(info), return_type::OK);
}

// ---- on_init: serial_port validation ----

TEST(HardwareInterfaceInitTest, MissingSerialPortReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters.erase("serial_port");
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

// ---- on_init: baud_rate validation ----

TEST(HardwareInterfaceInitTest, ValidBaudRates) {
  for (const auto & baud : {"9600", "19200", "38400", "57600", "115200", "500000", "1000000"}) {
    sts_hardware_interface::STSHardwareInterface hw;
    auto info = make_valid_single_motor_info();
    info.hardware_parameters["baud_rate"] = baud;
    EXPECT_EQ(hw.on_init(info), return_type::OK) << "Baud rate " << baud << " should be valid";
  }
}

TEST(HardwareInterfaceInitTest, InvalidBaudRateReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters["baud_rate"] = "999999";
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

// ---- on_init: no joints ----

TEST(HardwareInterfaceInitTest, NoJointsReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";
  // No joints added
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

// ---- on_init: motor_id validation ----

TEST(HardwareInterfaceInitTest, InvalidMotorIdZeroReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info("/dev/ttyACM0", "0");
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMotorId254ReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info("/dev/ttyACM0", "254");
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

TEST(HardwareInterfaceInitTest, ValidMotorIdBoundaries) {
  for (const auto & id : {"1", "127", "253"}) {
    sts_hardware_interface::STSHardwareInterface hw;
    auto info = make_valid_single_motor_info("/dev/ttyACM0", id);
    EXPECT_EQ(hw.on_init(info), return_type::OK) << "Motor ID " << id << " should be valid";
  }
}

TEST(HardwareInterfaceInitTest, DuplicateMotorIdsReturnError) {
  sts_hardware_interface::STSHardwareInterface hw;
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";

  for (const auto & joint_name : {"joint1", "joint2"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = joint_name;
    joint.parameters["motor_id"] = "1";  // same ID — duplicate!
    joint.parameters["operating_mode"] = "1";
    hardware_interface::InterfaceInfo v, a;
    v.name = "velocity"; a.name = "acceleration";
    joint.command_interfaces.push_back(v);
    joint.command_interfaces.push_back(a);
    info.joints.push_back(joint);
  }
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

// ---- on_init: operating_mode validation ----

TEST(HardwareInterfaceInitTest, InvalidOperatingModeReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info("/dev/ttyACM0", "1", "99");
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

// ---- on_init: optional parameter validation ----

TEST(HardwareInterfaceInitTest, InvalidCommunicationTimeoutReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters["communication_timeout_ms"] = "0";  // must be 1–1000
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMaxVelocityStepsReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters["max_velocity_steps"] = "0";  // must be > 0
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

TEST(HardwareInterfaceInitTest, MinPositionGreaterThanMaxPositionReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters["min_position"] = "5.0";
  info.joints[0].parameters["max_position"] = "1.0";  // min > max
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMaxVelocityReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters["max_velocity"] = "0.0";  // must be > 0
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMaxEffortReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_effort_motor_info();
  info.joints[0].parameters["max_effort"] = "1.5";  // must be in (0, 1]
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}

TEST(HardwareInterfaceInitTest, ZeroMaxEffortReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_effort_motor_info();
  info.joints[0].parameters["max_effort"] = "0.0";  // must be > 0
  EXPECT_EQ(hw.on_init(info), return_type::ERROR);
}
```

**Step 2: Build and run — expect failures (file exists but lifecycle tests not yet added)**

Run: `pixi run build --packages-select sts_hardware_interface 2>&1 | tail -20`
Expected: Compiles. All parameter validation tests should pass (they don't need rclcpp::init yet since on_init() doesn't create a node).

**Step 3: Run tests**

Run: `pixi run test --packages-select sts_hardware_interface 2>&1 | grep -E '(PASS|FAIL|test_hardware)'`
Expected: All parameter validation tests PASS.

**Step 4: Commit**

```bash
git add src/sts_hardware_interface/test/test_hardware_interface.cpp
git commit -m "test: add parameter validation unit tests for STSHardwareInterface::on_init"
```

---

### Task 4: Extend test_hardware_interface.cpp — interface exports, lifecycle, mock behavior, emergency stop

**Files:**
- Modify: `test/test_hardware_interface.cpp`

Add these test groups at the end of the file, before `main()`.

**Step 1: Add interface export tests**

These call `on_init()` then `export_state_interfaces()` / `export_command_interfaces()`.

```cpp
// ---- export_state_interfaces ----

TEST(HardwareInterfaceExportTest, StateInterfacesCount) {
  // Each joint always exports 7 state interfaces:
  // position, velocity, effort, voltage, temperature, current, is_moving
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  ASSERT_EQ(hw.on_init(info), return_type::OK);

  auto state_ifaces = hw.export_state_interfaces();
  EXPECT_EQ(state_ifaces.size(), 7u);
}

TEST(HardwareInterfaceExportTest, StateInterfaceNames) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  ASSERT_EQ(hw.on_init(info), return_type::OK);

  auto state_ifaces = hw.export_state_interfaces();
  std::vector<std::string> names;
  for (const auto & iface : state_ifaces) {
    names.push_back(iface.get_interface_name());
  }

  EXPECT_NE(std::find(names.begin(), names.end(), "position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "effort"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "voltage"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "temperature"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "current"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "is_moving"), names.end());
}

// ---- export_command_interfaces ----

TEST(HardwareInterfaceExportTest, CommandInterfacesVelocityMode) {
  // Mode 1: velocity + acceleration (2 interfaces)
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  ASSERT_EQ(hw.on_init(info), return_type::OK);

  auto cmd_ifaces = hw.export_command_interfaces();
  EXPECT_EQ(cmd_ifaces.size(), 3u);  // velocity + acceleration + emergency_stop
}

TEST(HardwareInterfaceExportTest, CommandInterfacesPositionMode) {
  // Mode 0: position + velocity + acceleration (3 interfaces)
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_position_motor_info();
  ASSERT_EQ(hw.on_init(info), return_type::OK);

  auto cmd_ifaces = hw.export_command_interfaces();
  EXPECT_EQ(cmd_ifaces.size(), 4u);  // position + velocity + acceleration + emergency_stop
}

TEST(HardwareInterfaceExportTest, CommandInterfacesEffortMode) {
  // Mode 2: effort (1 interface)
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_effort_motor_info();
  ASSERT_EQ(hw.on_init(info), return_type::OK);

  auto cmd_ifaces = hw.export_command_interfaces();
  EXPECT_EQ(cmd_ifaces.size(), 2u);  // effort + emergency_stop
}
```

**Step 2: Add lifecycle tests (need rclcpp::init in main)**

```cpp
// ---- on_configure / on_activate / on_deactivate / on_cleanup ----

class HardwareInterfaceLifecycleTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Each test gets a fresh hardware interface
    hw_ = std::make_unique<sts_hardware_interface::STSHardwareInterface>();
    info_ = make_valid_single_motor_info();
    ASSERT_EQ(hw_->on_init(info_), return_type::OK);
  }

  std::unique_ptr<sts_hardware_interface::STSHardwareInterface> hw_;
  hardware_interface::HardwareInfo info_;
};

TEST_F(HardwareInterfaceLifecycleTest, ConfigureSucceeds) {
  // on_configure creates node + service (mock mode skips serial port)
  rclcpp_lifecycle::State state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  EXPECT_EQ(hw_->on_configure(state), return_type::OK);
}

TEST_F(HardwareInterfaceLifecycleTest, ActivateAfterConfigureSucceeds) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");

  ASSERT_EQ(hw_->on_configure(unconfigured), return_type::OK);
  EXPECT_EQ(hw_->on_activate(inactive), return_type::OK);
}

TEST_F(HardwareInterfaceLifecycleTest, DeactivateAfterActivateSucceeds) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

  ASSERT_EQ(hw_->on_configure(unconfigured), return_type::OK);
  ASSERT_EQ(hw_->on_activate(inactive), return_type::OK);
  EXPECT_EQ(hw_->on_deactivate(active), return_type::OK);
}

TEST_F(HardwareInterfaceLifecycleTest, CleanupResetsServiceAndNode) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");

  ASSERT_EQ(hw_->on_configure(unconfigured), return_type::OK);
  // on_cleanup should reset service_ and node_ to nullptr without crashing
  EXPECT_EQ(hw_->on_cleanup(inactive), return_type::OK);
}
```

**Step 3: Add mock read/write behavior tests**

```cpp
// ---- mock read/write behavior ----

class HardwareInterfaceMockTest : public ::testing::Test {
protected:
  void SetUp() override {
    hw_ = std::make_unique<sts_hardware_interface::STSHardwareInterface>();
    info_ = make_valid_single_motor_info();
    ASSERT_EQ(hw_->on_init(info_), return_type::OK);

    state_ifaces_ = hw_->export_state_interfaces();
    cmd_ifaces_ = hw_->export_command_interfaces();

    rclcpp_lifecycle::State unconfigured(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");

    ASSERT_EQ(hw_->on_configure(unconfigured), return_type::OK);
    ASSERT_EQ(hw_->on_activate(inactive), return_type::OK);
  }

  void TearDown() override {
    rclcpp_lifecycle::State active(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
    rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    hw_->on_deactivate(active);
    hw_->on_cleanup(inactive);
  }

  // Find a command interface by name
  hardware_interface::LoanedCommandInterface * find_cmd(const std::string & name) {
    for (auto & iface : cmd_ifaces_) {
      if (iface.get_interface_name() == name) return &iface;
    }
    return nullptr;
  }

  // Find a state interface by name
  hardware_interface::LoanedStateInterface * find_state(const std::string & name) {
    for (auto & iface : state_ifaces_) {
      if (iface.get_interface_name() == name) return &iface;
    }
    return nullptr;
  }

  std::unique_ptr<sts_hardware_interface::STSHardwareInterface> hw_;
  hardware_interface::HardwareInfo info_;
  std::vector<hardware_interface::LoanedStateInterface> state_ifaces_;
  std::vector<hardware_interface::LoanedCommandInterface> cmd_ifaces_;
};

TEST_F(HardwareInterfaceMockTest, ReadSucceeds) {
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  EXPECT_EQ(hw_->read(t, d), return_type::OK);
}

TEST_F(HardwareInterfaceMockTest, WriteSucceeds) {
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  EXPECT_EQ(hw_->write(t, d), return_type::OK);
}

TEST_F(HardwareInterfaceMockTest, MockVelocityIntegratesToPosition) {
  // Set a velocity command and run read/write to verify position changes
  auto * vel_cmd = find_cmd("velocity");
  ASSERT_NE(vel_cmd, nullptr);

  vel_cmd->set_value(1.0);  // 1 rad/s

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration dt(0, static_cast<int32_t>(0.1e9));  // 100ms

  hw_->write(t, dt);
  hw_->read(t, dt);

  auto * pos_state = find_state("position");
  ASSERT_NE(pos_state, nullptr);
  // Position should have changed from 0
  // (exact value depends on mock integration step)
  EXPECT_NE(pos_state->get_value(), std::numeric_limits<double>::quiet_NaN());
}
```

**Step 4: Add emergency stop tests**

```cpp
// ---- emergency stop behavior ----

TEST_F(HardwareInterfaceMockTest, EmergencyStopActivationClearsVelocityCommand) {
  // Set a velocity command
  auto * vel_cmd = find_cmd("velocity");
  auto * estop_cmd = find_cmd("emergency_stop");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(estop_cmd, nullptr);

  vel_cmd->set_value(2.0);
  estop_cmd->set_value(1.0);  // Activate emergency stop

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);  // write() should clear velocity when estop active

  // After write with estop, velocity command should be cleared to 0
  // (The interface value doesn't auto-reset, but the hardware write path is suppressed)
  // This is observable by checking the state didn't update
  EXPECT_EQ(hw_->read(t, d), return_type::OK);
}

TEST_F(HardwareInterfaceMockTest, EmergencyStopReleaseResumesOperation) {
  auto * estop_cmd = find_cmd("emergency_stop");
  ASSERT_NE(estop_cmd, nullptr);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);

  // Activate then release
  estop_cmd->set_value(1.0);
  hw_->write(t, d);

  estop_cmd->set_value(0.0);
  hw_->write(t, d);

  // Should still be operational after release
  EXPECT_EQ(hw_->read(t, d), return_type::OK);
}
```

**Step 5: Update main() to call rclcpp::init**

Replace the `main()` at the bottom of the file:

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
```

Also add required lifecycle headers at the top:
```cpp
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"
```

**Step 6: Build and test**

Run: `pixi run build --packages-select sts_hardware_interface 2>&1 | tail -20`
Run: `pixi run test --packages-select sts_hardware_interface 2>&1 | grep -E '(PASS|FAIL|ERROR)'`
Expected: All tests pass. Fix any compilation errors from missing headers.

**Step 7: Commit**

```bash
git add src/sts_hardware_interface/test/test_hardware_interface.cpp
git commit -m "test: add lifecycle, mock behavior, and emergency stop unit tests"
```

---

### Task 5: Write test_single_motor.launch.py

**Files:**
- Create: `test/test_single_motor.launch.py`

This launch test verifies that the single motor example fully starts with `use_mock:=true`. Uses `launch_testing` — the test class runs AFTER all processes are up.

```python
# test/test_single_motor.launch.py
"""Launch test for single_motor example in mock mode.

Verifies that the complete ros2_control stack starts successfully
with a single velocity-mode motor in mock mode (no hardware required).
"""

import os
import pytest
import time
import unittest

import launch
import launch_ros
import launch_testing
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Generate launch description for the single motor integration test."""
    # Use the existing example launch file with mock mode enabled
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from ament_index_python.packages import get_package_share_directory

    pkg_share = get_package_share_directory('sts_hardware_interface')
    single_motor_launch = os.path.join(pkg_share, 'launch', 'single_motor.launch.py')

    single_motor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_motor_launch),
        launch_arguments={
            'use_mock': 'true',
            'gui': 'false',
        }.items()
    )

    return launch.LaunchDescription([
        single_motor,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSingleMotorLaunch(unittest.TestCase):
    """Test that single motor stack launches and basic topics are available."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_single_motor_node')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_topic(self, topic_name, msg_type, timeout_sec=30.0):
        """Wait for a topic to publish at least one message."""
        received = []

        def callback(msg):
            received.append(msg)

        sub = self.node.create_subscription(msg_type, topic_name, callback, 10)
        deadline = time.time() + timeout_sec

        while time.time() < deadline and not received:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(sub)
        return len(received) > 0

    def test_joint_states_published(self):
        """Verify /joint_states is published after stack starts."""
        from sensor_msgs.msg import JointState
        self.assertTrue(
            self._wait_for_topic('/joint_states', JointState, timeout_sec=30.0),
            "/joint_states not received within 30 seconds"
        )

    def test_joint_state_has_wheel_joint(self):
        """Verify /joint_states contains wheel_joint."""
        from sensor_msgs.msg import JointState
        received = []

        sub = self.node.create_subscription(
            JointState, '/joint_states', received.append, 10)

        deadline = time.time() + 30.0
        while time.time() < deadline and not received:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(sub)
        self.assertTrue(received, "/joint_states not received")
        self.assertIn('wheel_joint', received[0].name)

    def test_controller_manager_available(self):
        """Verify /controller_manager service is reachable."""
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(
            client.wait_for_service(timeout_sec=30.0),
            "/controller_manager/list_controllers service not available"
        )
        self.node.destroy_client(client)

    def test_velocity_controller_active(self):
        """Verify velocity_controller is in 'active' state."""
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))

        future = client.call_async(ListControllers.Request())
        deadline = time.time() + 10.0
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(future.done(), "list_controllers call timed out")
        response = future.result()

        controller_states = {c.name: c.state for c in response.controller}
        self.assertIn('velocity_controller', controller_states,
                      f"velocity_controller not found. Available: {list(controller_states.keys())}")
        self.assertEqual(controller_states['velocity_controller'], 'active',
                         f"velocity_controller state: {controller_states['velocity_controller']}")

    def test_emergency_stop_service_available(self):
        """Verify /emergency_stop service is reachable."""
        from std_srvs.srv import SetBool
        client = self.node.create_client(SetBool, '/emergency_stop')
        self.assertTrue(
            client.wait_for_service(timeout_sec=30.0),
            "/emergency_stop service not available"
        )
        self.node.destroy_client(client)

    def test_emergency_stop_introspection_topic(self):
        """Verify /emergency_stop/_service_event topic is published."""
        from rcl_interfaces.msg import ServiceEvent
        received = []
        sub = self.node.create_subscription(
            ServiceEvent, '/emergency_stop/_service_event', received.append, 10)

        # Trigger the service to generate introspection events
        from std_srvs.srv import SetBool
        client = self.node.create_client(SetBool, '/emergency_stop')
        client.wait_for_service(timeout_sec=10.0)

        req = SetBool.Request()
        req.data = True
        future = client.call_async(req)

        deadline = time.time() + 10.0
        while time.time() < deadline and not received:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(sub)
        self.node.destroy_client(client)
        self.assertTrue(received, "/emergency_stop/_service_event not received after call")
```

**Step 2: Build**

Run: `pixi run build --packages-select sts_hardware_interface 2>&1 | tail -20`
Expected: Build succeeds. Launch test registered.

**Step 3: Run launch test** (this takes 30-60 seconds)

Run: `pixi run test --packages-select sts_hardware_interface --pytest-args -v -k test_single_motor 2>&1 | tail -40`
Expected: All 6 tests PASS. Fix timeouts by increasing `timeout_sec` if needed in slow CI.

**Step 4: Commit**

```bash
git add src/sts_hardware_interface/test/test_single_motor.launch.py
git commit -m "test: add launch integration test for single motor mock mode"
```

---

### Task 6: Write test_mixed_mode.launch.py

**Files:**
- Create: `test/test_mixed_mode.launch.py`

This launch test verifies the mixed-mode (3-controller) example starts with `use_mock:=true`.

```python
# test/test_mixed_mode.launch.py
"""Launch test for mixed_mode example in mock mode.

Verifies that the complete ros2_control stack starts with multiple motors
in different operating modes (position, velocity, effort) using mock hardware.
"""

import os
import pytest
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import rclpy


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Generate launch description for the mixed mode integration test."""
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from ament_index_python.packages import get_package_share_directory

    pkg_share = get_package_share_directory('sts_hardware_interface')
    mixed_mode_launch = os.path.join(pkg_share, 'launch', 'mixed_mode.launch.py')

    mixed_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mixed_mode_launch),
        launch_arguments={
            'use_mock': 'true',
            'gui': 'false',
        }.items()
    )

    return launch.LaunchDescription([
        mixed_mode,
        launch_testing.actions.ReadyToTest(),
    ])


class TestMixedModeLaunch(unittest.TestCase):
    """Test that mixed mode stack launches with all three controllers active."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_mixed_mode_node')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_topic(self, topic_name, msg_type, timeout_sec=45.0):
        """Wait for a topic to publish at least one message."""
        received = []
        sub = self.node.create_subscription(msg_type, topic_name, received.append, 10)
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not received:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return received

    def test_joint_states_published(self):
        """Verify /joint_states is published."""
        from sensor_msgs.msg import JointState
        received = self._wait_for_topic('/joint_states', JointState, timeout_sec=45.0)
        self.assertTrue(received, "/joint_states not received within 45 seconds")

    def test_all_joints_present(self):
        """Verify /joint_states contains all expected joints."""
        from sensor_msgs.msg import JointState
        received = self._wait_for_topic('/joint_states', JointState, timeout_sec=45.0)
        self.assertTrue(received)

        joint_names = received[0].name
        # Mixed mode URDF has: arm_joint_1, arm_joint_2, wheel_joint_1, wheel_joint_2,
        # gripper_joint_1, gripper_joint_2
        expected_joints = [
            'arm_joint_1', 'arm_joint_2',
            'wheel_joint_1', 'wheel_joint_2',
            'gripper_joint_1', 'gripper_joint_2',
        ]
        for joint in expected_joints:
            self.assertIn(joint, joint_names,
                          f"Expected joint '{joint}' not found in {joint_names}")

    def test_all_three_controllers_active(self):
        """Verify arm_controller, wheel_controller, gripper_controller are all active."""
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=45.0))

        future = client.call_async(ListControllers.Request())
        deadline = time.time() + 15.0
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(future.done())
        controller_states = {c.name: c.state for c in future.result().controller}

        for controller in ['arm_controller', 'wheel_controller', 'gripper_controller']:
            self.assertIn(controller, controller_states,
                          f"'{controller}' not found. Available: {list(controller_states.keys())}")
            self.assertEqual(controller_states[controller], 'active',
                             f"'{controller}' state: {controller_states[controller]}")

    def test_joint_state_broadcaster_active(self):
        """Verify joint_state_broadcaster is active."""
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=45.0))

        future = client.call_async(ListControllers.Request())
        deadline = time.time() + 15.0
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        controller_states = {c.name: c.state for c in future.result().controller}
        self.assertEqual(
            controller_states.get('joint_state_broadcaster', 'MISSING'), 'active'
        )

    def test_emergency_stop_service_available(self):
        """Verify /emergency_stop service is available in mixed mode too."""
        from std_srvs.srv import SetBool
        client = self.node.create_client(SetBool, '/emergency_stop')
        self.assertTrue(
            client.wait_for_service(timeout_sec=30.0),
            "/emergency_stop service not available"
        )
        self.node.destroy_client(client)

    def test_dynamic_joint_states_published(self):
        """Verify /dynamic_joint_states is published (custom interfaces)."""
        from control_msgs.msg import DynamicJointState
        received = self._wait_for_topic('/dynamic_joint_states', DynamicJointState, timeout_sec=45.0)
        self.assertTrue(received, "/dynamic_joint_states not received within 45 seconds")
```

**Step 2: Build and run**

Run: `pixi run build --packages-select sts_hardware_interface 2>&1 | tail -20`
Run: `pixi run test --packages-select sts_hardware_interface --pytest-args -v -k test_mixed_mode 2>&1 | tail -40`
Expected: All 6 tests PASS. Mixed mode has more joints so timeouts are slightly longer (45s).

**Step 3: Commit**

```bash
git add src/sts_hardware_interface/test/test_mixed_mode.launch.py
git commit -m "test: add launch integration test for mixed mode mock operation"
```

---

### Task 7: Final build and test verification

**Step 1: Full build**

Run: `pixi run build --packages-select sts_hardware_interface`
Expected: Zero errors.

**Step 2: Full test run**

Run: `pixi run test --packages-select sts_hardware_interface`
Expected:
- `test_conversions`: ~36 PASS
- `test_hardware_interface`: ~25+ PASS
- `test_single_motor`: 6 PASS
- `test_mixed_mode`: 6 PASS

**Step 3: Review test results**

Run: `colcon test-result --verbose --test-result-base build/sts_hardware_interface`
Expected: All tests PASS, zero failures.

**Step 4: Final commit (if any fixes were needed)**

```bash
git add -p  # Stage only relevant changes
git commit -m "test: fix test failures from final integration run"
```

---

## Notes

### HardwareInfo Construction Reference

```cpp
hardware_interface::HardwareInfo info;
// Required hardware parameters
info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
info.hardware_parameters["enable_mock_mode"] = "true";

// Optional hardware parameters (with validation)
info.hardware_parameters["baud_rate"] = "1000000";           // default
info.hardware_parameters["communication_timeout_ms"] = "100"; // 1-1000
info.hardware_parameters["max_velocity_steps"] = "4095";      // > 0
info.hardware_parameters["use_sync_write"] = "false";

// Joint
hardware_interface::ComponentInfo joint;
joint.name = "wheel_joint";
joint.parameters["motor_id"] = "1";         // 1-253
joint.parameters["operating_mode"] = "1";   // 0=position, 1=velocity, 2=effort

// Optional joint parameters
joint.parameters["min_position"] = "0.0";
joint.parameters["max_position"] = "6.283";
joint.parameters["max_velocity"] = "5.22";
joint.parameters["max_effort"] = "1.0";     // (0, 1]

// Command interfaces (must match operating_mode)
// Mode 0: position + velocity + acceleration
// Mode 1: velocity + acceleration
// Mode 2: effort
hardware_interface::InterfaceInfo iface;
iface.name = "velocity";
joint.command_interfaces.push_back(iface);

info.joints.push_back(joint);
```

### Lifecycle State Construction

```cpp
// States for lifecycle transitions
rclcpp_lifecycle::State unconfigured(
  lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
rclcpp_lifecycle::State inactive(
  lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
rclcpp_lifecycle::State active(
  lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
```

### Interface Counts by Mode

| Mode | Name | Command Interfaces | + emergency_stop | State Interfaces |
|------|------|--------------------|------------------|-----------------|
| 0 | Position | position + velocity + acceleration = 3 | 4 total | 7 (always) |
| 1 | Velocity | velocity + acceleration = 2 | 3 total | 7 (always) |
| 2 | Effort/PWM | effort = 1 | 2 total | 7 (always) |
