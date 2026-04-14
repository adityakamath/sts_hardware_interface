// Copyright 2026 Aditya Kamath
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sts_hardware_interface/sts_hardware_interface.hpp"

using hardware_interface::CallbackReturn;
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

  // Mode 0 (position): only the position interface is required.
  // velocity and acceleration are optional — omitting them makes the hardware
  // use its max speed (raw 0) and default ramp (ACC 0) for position moves.
  hardware_interface::InterfaceInfo pos_iface;
  pos_iface.name = "position";
  joint.command_interfaces.push_back(pos_iface);

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

/// Build info for a servo-mode motor with position + velocity command interfaces.
/// Use this for mock tests that exercise the velocity command path (max-speed control).
hardware_interface::HardwareInfo make_servo_info_with_velocity()
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";

  hardware_interface::ComponentInfo joint;
  joint.name = "arm_joint";
  joint.parameters["motor_id"] = "2";
  joint.parameters["operating_mode"] = "0";

  for (const auto & name : {"position", "velocity"}) {
    hardware_interface::InterfaceInfo iface;
    iface.name = name;
    joint.command_interfaces.push_back(iface);
  }
  info.joints.push_back(joint);
  return info;
}

/// Build info for a servo-mode motor with all three command interfaces.
hardware_interface::HardwareInfo make_servo_info_with_vel_acc()
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";

  hardware_interface::ComponentInfo joint;
  joint.name = "arm_joint";
  joint.parameters["motor_id"] = "2";
  joint.parameters["operating_mode"] = "0";

  for (const auto & name : {"position", "velocity", "acceleration"}) {
    hardware_interface::InterfaceInfo iface;
    iface.name = name;
    joint.command_interfaces.push_back(iface);
  }
  info.joints.push_back(joint);
  return info;
}

/// Build info for a velocity-mode motor WITHOUT the optional acceleration interface.
hardware_interface::HardwareInfo make_velocity_info_no_acc()
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel_joint";
  joint.parameters["motor_id"] = "1";
  joint.parameters["operating_mode"] = "1";

  hardware_interface::InterfaceInfo vel_iface;
  vel_iface.name = "velocity";
  joint.command_interfaces.push_back(vel_iface);
  info.joints.push_back(joint);
  return info;
}

/// Build a HardwareInfo with a velocity-mode motor and a position-mode motor.
hardware_interface::HardwareInfo make_two_motor_info()
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";

  // Motor 1: velocity mode (mode 1)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "wheel_joint";
    joint.parameters["motor_id"] = "1";
    joint.parameters["operating_mode"] = "1";
    hardware_interface::InterfaceInfo v, a;
    v.name = "velocity";
    a.name = "acceleration";
    joint.command_interfaces.push_back(v);
    joint.command_interfaces.push_back(a);
    info.joints.push_back(joint);
  }

  // Motor 2: position mode (mode 0)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "arm_joint";
    joint.parameters["motor_id"] = "2";
    joint.parameters["operating_mode"] = "0";
    // position interface only — velocity and acceleration are optional
    hardware_interface::InterfaceInfo pos_iface;
    pos_iface.name = "position";
    joint.command_interfaces.push_back(pos_iface);
    info.joints.push_back(joint);
  }

  return info;
}

// ---- on_init: success cases ----

TEST(HardwareInterfaceInitTest, ValidOperatingModes) {
  // All three supported operating modes initialise successfully
  {
    sts_hardware_interface::STSHardwareInterface hw;
    EXPECT_EQ(hw.on_init(make_valid_single_motor_info()), CallbackReturn::SUCCESS);
  }
  {
    sts_hardware_interface::STSHardwareInterface hw;
    EXPECT_EQ(hw.on_init(make_valid_position_motor_info()), CallbackReturn::SUCCESS);
  }
  {
    sts_hardware_interface::STSHardwareInterface hw;
    EXPECT_EQ(hw.on_init(make_valid_effort_motor_info()), CallbackReturn::SUCCESS);
  }
}

// ---- on_init: serial_port validation ----

TEST(HardwareInterfaceInitTest, MissingSerialPortReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters.erase("serial_port");
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

// ---- on_init: baud_rate validation ----

TEST(HardwareInterfaceInitTest, ValidBaudRates) {
  for (const auto & baud : {"9600", "19200", "38400", "57600", "115200", "500000", "1000000"}) {
    sts_hardware_interface::STSHardwareInterface hw;
    auto info = make_valid_single_motor_info();
    info.hardware_parameters["baud_rate"] = baud;
    EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS) << "Baud rate " << baud << " should be valid";
  }
}

TEST(HardwareInterfaceInitTest, InvalidBaudRateReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters["baud_rate"] = "999999";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

// ---- on_init: no joints ----

TEST(HardwareInterfaceInitTest, NoJointsReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";
  // No joints added
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

// ---- on_init: motor_id validation ----

TEST(HardwareInterfaceInitTest, InvalidMotorIdBoundariesReturnError) {
  // IDs 0 and 254 are outside the valid range [1, 253]
  for (const char * id : {"0", "254"}) {
    sts_hardware_interface::STSHardwareInterface hw;
    EXPECT_EQ(
      hw.on_init(make_valid_single_motor_info("/dev/ttyACM0", id)),
      CallbackReturn::ERROR) << "Motor ID " << id << " should be invalid";
  }
}

TEST(HardwareInterfaceInitTest, ValidMotorIdBoundaries) {
  for (const auto & id : {"1", "127", "253"}) {
    sts_hardware_interface::STSHardwareInterface hw;
    auto info = make_valid_single_motor_info("/dev/ttyACM0", id);
    EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS) << "Motor ID " << id << " should be valid";
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
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

// ---- on_init: operating_mode validation ----

TEST(HardwareInterfaceInitTest, InvalidOperatingModeReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info("/dev/ttyACM0", "1", "99");
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

// ---- on_init: optional parameter validation ----

TEST(HardwareInterfaceInitTest, InvalidCommunicationTimeoutReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters["communication_timeout_ms"] = "0";  // must be 1–1000
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMaxVelocityStepsReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters["max_velocity_steps"] = "0";  // must be > 0
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, MinPositionGreaterThanMaxPositionReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters["min_position"] = "5.0";
  info.joints[0].parameters["max_position"] = "1.0";  // min > max
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMaxVelocityReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters["max_velocity"] = "0.0";  // must be > 0
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMaxEffortReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_effort_motor_info();
  info.joints[0].parameters["max_effort"] = "1.5";  // must be in (0, 1]
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, ZeroMaxEffortReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_effort_motor_info();
  info.joints[0].parameters["max_effort"] = "0.0";  // must be > 0
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

// ---- on_init: position_center_steps validation ----

TEST(HardwareInterfaceInitTest, PositionCenterStepsValidValues) {
  // center=2048 in servo mode succeeds
  sts_hardware_interface::STSHardwareInterface hw1;
  auto info1 = make_valid_position_motor_info();
  info1.joints[0].parameters["position_center_steps"] = "2048";
  EXPECT_EQ(hw1.on_init(info1), CallbackReturn::SUCCESS);

  // boundary: 0 succeeds
  sts_hardware_interface::STSHardwareInterface hw2;
  auto info2 = make_valid_position_motor_info();
  info2.joints[0].parameters["position_center_steps"] = "0";
  EXPECT_EQ(hw2.on_init(info2), CallbackReturn::SUCCESS);

  // boundary: 4095 succeeds
  sts_hardware_interface::STSHardwareInterface hw3;
  auto info3 = make_valid_position_motor_info();
  info3.joints[0].parameters["position_center_steps"] = "4095";
  EXPECT_EQ(hw3.on_init(info3), CallbackReturn::SUCCESS);
}

TEST(HardwareInterfaceInitTest, PositionCenterStepsInvalidValues) {
  // out of range (> 4095)
  sts_hardware_interface::STSHardwareInterface hw1;
  auto info1 = make_valid_position_motor_info();
  info1.joints[0].parameters["position_center_steps"] = "5000";
  EXPECT_EQ(hw1.on_init(info1), CallbackReturn::ERROR);

  // negative value
  sts_hardware_interface::STSHardwareInterface hw2;
  auto info2 = make_valid_position_motor_info();
  info2.joints[0].parameters["position_center_steps"] = "-1";
  EXPECT_EQ(hw2.on_init(info2), CallbackReturn::ERROR);

  // non-numeric string
  sts_hardware_interface::STSHardwareInterface hw3;
  auto info3 = make_valid_position_motor_info();
  info3.joints[0].parameters["position_center_steps"] = "abc";
  EXPECT_EQ(hw3.on_init(info3), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, PositionCenterStepsIgnoredInNonServoModes) {
  // velocity mode (1): succeeds with warning
  sts_hardware_interface::STSHardwareInterface hw1;
  auto info1 = make_valid_single_motor_info("/dev/ttyACM0", "1", "1");
  info1.joints[0].parameters["position_center_steps"] = "2048";
  EXPECT_EQ(hw1.on_init(info1), CallbackReturn::SUCCESS);

  // PWM mode (2): succeeds with warning
  sts_hardware_interface::STSHardwareInterface hw2;
  auto info2 = make_valid_effort_motor_info();
  info2.joints[0].parameters["position_center_steps"] = "2048";
  EXPECT_EQ(hw2.on_init(info2), CallbackReturn::SUCCESS);
}

// ---- export_state_interfaces ----

TEST(HardwareInterfaceExportTest, StateInterfacesCount) {
  // Each joint always exports 7 state interfaces:
  // position, velocity, effort, voltage, temperature, current, is_moving
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  ASSERT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);

  auto state_ifaces = hw.export_state_interfaces();
  EXPECT_EQ(state_ifaces.size(), 7u);
}

TEST(HardwareInterfaceExportTest, StateInterfaceNames) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  ASSERT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);

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
  // Mode 1: velocity + acceleration = 2 command interfaces
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  ASSERT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);

  auto cmd_ifaces = hw.export_command_interfaces();
  EXPECT_EQ(cmd_ifaces.size(), 2u);
}

TEST(HardwareInterfaceExportTest, CommandInterfacesPositionMode) {
  // Mode 0 with position-only URDF (make_valid_position_motor_info) → 1 command interface
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_position_motor_info();
  ASSERT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);

  auto cmd_ifaces = hw.export_command_interfaces();
  EXPECT_EQ(cmd_ifaces.size(), 1u);
}

TEST(HardwareInterfaceExportTest, CommandInterfacesEffortMode) {
  // Mode 2: effort = 1 command interface
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_effort_motor_info();
  ASSERT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);

  auto cmd_ifaces = hw.export_command_interfaces();
  EXPECT_EQ(cmd_ifaces.size(), 1u);
}

TEST(HardwareInterfaceExportTest, CommandInterfaceNamesVelocityMode) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  ASSERT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);

  auto cmd_ifaces = hw.export_command_interfaces();
  std::vector<std::string> names;
  for (const auto & iface : cmd_ifaces) {
    names.push_back(iface.get_interface_name());
  }
  EXPECT_NE(std::find(names.begin(), names.end(), "velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "acceleration"), names.end());
}

// ---- on_configure / on_activate / on_deactivate / on_cleanup ----

class HardwareInterfaceLifecycleTest : public ::testing::Test {
protected:
  void SetUp() override {
    hw_ = std::make_unique<sts_hardware_interface::STSHardwareInterface>();
    info_ = make_valid_single_motor_info();
    ASSERT_EQ(hw_->on_init(info_), CallbackReturn::SUCCESS);
  }

  std::unique_ptr<sts_hardware_interface::STSHardwareInterface> hw_;
  hardware_interface::HardwareInfo info_;
};

TEST_F(HardwareInterfaceLifecycleTest, ConfigureSucceeds) {
  // on_configure creates node + service (mock mode skips serial port)
  rclcpp_lifecycle::State state(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  EXPECT_EQ(hw_->on_configure(state), CallbackReturn::SUCCESS);
}

TEST_F(HardwareInterfaceLifecycleTest, ActivateAfterConfigureSucceeds) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");

  ASSERT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_activate(inactive), CallbackReturn::SUCCESS);
}

TEST_F(HardwareInterfaceLifecycleTest, DeactivateAfterActivateSucceeds) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

  ASSERT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_deactivate(active), CallbackReturn::SUCCESS);
}

TEST_F(HardwareInterfaceLifecycleTest, CleanupResetsResources) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");

  ASSERT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
  // on_cleanup resets service_ and node_ to nullptr without crashing
  EXPECT_EQ(hw_->on_cleanup(inactive), CallbackReturn::SUCCESS);
}

// ---- mock read/write behavior ----

class HardwareInterfaceMockTest : public ::testing::Test {
protected:
  void SetUp() override {
    hw_ = std::make_unique<sts_hardware_interface::STSHardwareInterface>();
    info_ = make_valid_single_motor_info();
    ASSERT_EQ(hw_->on_init(info_), CallbackReturn::SUCCESS);

    // Export interfaces BEFORE configure (they point into hw_'s internal storage)
    state_ifaces_ = hw_->export_state_interfaces();
    cmd_ifaces_ = hw_->export_command_interfaces();

    rclcpp_lifecycle::State unconfigured(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");

    ASSERT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw_->on_activate(inactive), CallbackReturn::SUCCESS);
  }

  void TearDown() override {
    rclcpp_lifecycle::State active(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
    rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    hw_->on_deactivate(active);
    hw_->on_cleanup(inactive);
  }

  // Find a command interface by interface name
  hardware_interface::CommandInterface * find_cmd(const std::string & name) {
    for (auto & iface : cmd_ifaces_) {
      if (iface.get_interface_name() == name) return &iface;
    }
    return nullptr;
  }

  // Find a state interface by interface name
  hardware_interface::StateInterface * find_state(const std::string & name) {
    for (auto & iface : state_ifaces_) {
      if (iface.get_interface_name() == name) return &iface;
    }
    return nullptr;
  }

  std::unique_ptr<sts_hardware_interface::STSHardwareInterface> hw_;
  hardware_interface::HardwareInfo info_;
  std::vector<hardware_interface::StateInterface> state_ifaces_;
  std::vector<hardware_interface::CommandInterface> cmd_ifaces_;
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

TEST_F(HardwareInterfaceMockTest, InitialPositionStateIsValid) {
  // After activation, position state should be a valid double (not NaN)
  auto * pos_state = find_state("position");
  ASSERT_NE(pos_state, nullptr);
  double pos_value = 0.0;
  (void)pos_state->get_value(pos_value, true);
  EXPECT_FALSE(std::isnan(pos_value));
}

TEST_F(HardwareInterfaceMockTest, MockVelocityCommandChangesPosition) {
  // Set a non-zero velocity command; position should change after read/write cycle
  auto * vel_cmd = find_cmd("velocity");
  auto * pos_state = find_state("position");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(pos_state, nullptr);

  (void)vel_cmd->set_value(1.0);  // 1 rad/s

  double initial_position = 0.0;
  (void)pos_state->get_value(initial_position, true);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration dt(0, static_cast<int32_t>(0.1e9));  // 100 ms

  hw_->write(t, dt);
  hw_->read(t, dt);

  // Position should have changed from the initial value
  double final_position = 0.0;
  (void)pos_state->get_value(final_position, true);
  EXPECT_NE(final_position, initial_position);
}

TEST_F(HardwareInterfaceMockTest, ZeroVelocityCommandHoldsPosition) {
  // With zero velocity, position should remain unchanged
  auto * vel_cmd = find_cmd("velocity");
  auto * pos_state = find_state("position");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(pos_state, nullptr);

  (void)vel_cmd->set_value(0.0);
  double initial_position = 0.0;
  (void)pos_state->get_value(initial_position, true);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration dt(0, static_cast<int32_t>(0.1e9));

  hw_->write(t, dt);
  hw_->read(t, dt);

  double pos_value = 0.0;
  (void)pos_state->get_value(pos_value, true);
  EXPECT_NEAR(pos_value, initial_position, 1e-9);
}

// ============================================================
// Additional lifecycle: on_shutdown (uses HardwareInterfaceLifecycleTest)
// ============================================================

TEST_F(HardwareInterfaceLifecycleTest, ShutdownSucceedsInAnyState) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

  // Without configure
  EXPECT_EQ(hw_->on_shutdown(unconfigured), CallbackReturn::SUCCESS);

  // After configure
  {
    sts_hardware_interface::STSHardwareInterface hw;
    ASSERT_EQ(hw.on_init(info_), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_configure(unconfigured), CallbackReturn::SUCCESS);
    EXPECT_EQ(hw.on_shutdown(inactive), CallbackReturn::SUCCESS);
  }

  // After activate
  {
    sts_hardware_interface::STSHardwareInterface hw;
    ASSERT_EQ(hw.on_init(info_), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_configure(unconfigured), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_activate(inactive), CallbackReturn::SUCCESS);
    EXPECT_EQ(hw.on_shutdown(active), CallbackReturn::SUCCESS);
  }
}

// ============================================================
// Additional lifecycle: on_error (uses HardwareInterfaceLifecycleTest)
// ============================================================

TEST_F(HardwareInterfaceLifecycleTest, OnErrorSucceedsRegardlessOfState) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

  // Before configure
  EXPECT_EQ(hw_->on_error(unconfigured), CallbackReturn::SUCCESS);

  // After activate
  {
    sts_hardware_interface::STSHardwareInterface hw;
    ASSERT_EQ(hw.on_init(info_), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_configure(unconfigured), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_activate(inactive), CallbackReturn::SUCCESS);
    EXPECT_EQ(hw.on_error(active), CallbackReturn::SUCCESS);
  }
}

// ============================================================
// Deactivate state clearing (via HardwareInterfaceMockTest fixture)
// on_deactivate in mock mode: zeros hw_state_velocity_ and hw_state_effort_
// ============================================================

TEST_F(HardwareInterfaceMockTest, DeactivateClearsMotionStates) {
  auto * vel_cmd     = find_cmd("velocity");
  auto * vel_state   = find_state("velocity");
  auto * effort_state = find_state("effort");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(vel_state, nullptr);
  ASSERT_NE(effort_state, nullptr);

  // Drive both velocity and effort states to non-zero
  (void)vel_cmd->set_value(2.0);
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double vel_val = 0.0, eff_val = 0.0;
  (void)vel_state->get_value(vel_val, true);
  (void)effort_state->get_value(eff_val, true);
  ASSERT_NE(vel_val, 0.0) << "Pre-condition: velocity state must be non-zero before deactivate";
  ASSERT_NE(eff_val, 0.0) << "Pre-condition: effort state must be non-zero before deactivate";

  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(hw_->on_deactivate(active), CallbackReturn::SUCCESS);

  (void)vel_state->get_value(vel_val, true);
  (void)effort_state->get_value(eff_val, true);
  EXPECT_NEAR(vel_val, 0.0, 1e-9);
  EXPECT_NEAR(eff_val, 0.0, 1e-9);
}

// ============================================================
// Simulated state values — zero velocity produces baseline readings
// ============================================================

TEST_F(HardwareInterfaceMockTest, IsMovingReflectsVelocityThreshold) {
  auto * vel_cmd  = find_cmd("velocity");
  auto * is_moving = find_state("is_moving");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(is_moving, nullptr);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  double val = -1.0;

  // Zero velocity (default) → not moving
  (void)vel_cmd->set_value(0.0);
  hw_->write(t, d);
  hw_->read(t, d);
  (void)is_moving->get_value(val, true);
  EXPECT_NEAR(val, 0.0, 1e-9) << "Zero velocity should not be moving";

  // Above threshold (1.0 rad/s > 0.01) → moving
  (void)vel_cmd->set_value(1.0);
  hw_->write(t, d);
  hw_->read(t, d);
  (void)is_moving->get_value(val, true);
  EXPECT_NEAR(val, 1.0, 1e-9) << "1.0 rad/s should be moving";

  // Below threshold (0.005 rad/s < 0.01) → not moving
  (void)vel_cmd->set_value(0.005);
  hw_->write(t, d);
  hw_->read(t, d);
  (void)is_moving->get_value(val, true);
  EXPECT_NEAR(val, 0.0, 1e-9) << "0.005 rad/s (below threshold) should not be moving";
}

TEST_F(HardwareInterfaceMockTest, ZeroVelocityProducesBaselineSensors) {
  // Zero velocity → no load → baseline voltage (12.0V), temperature (25°C), zero current
  auto * voltage  = find_state("voltage");
  auto * temp     = find_state("temperature");
  auto * current  = find_state("current");
  ASSERT_NE(voltage, nullptr);
  ASSERT_NE(temp, nullptr);
  ASSERT_NE(current, nullptr);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double v = 0.0, tmp = 0.0, cur = -1.0;
  (void)voltage->get_value(v, true);
  (void)temp->get_value(tmp, true);
  (void)current->get_value(cur, true);
  EXPECT_NEAR(v, 12.0, 1e-6);
  EXPECT_NEAR(tmp, 25.0, 1e-6);
  EXPECT_NEAR(cur, 0.0, 1e-9);
}

TEST_F(HardwareInterfaceMockTest, NonZeroVelocityDrivesAllSensorStates) {
  // Non-zero velocity → simulated load → voltage drops, temperature rises, current and effort > 0
  auto * vel_cmd  = find_cmd("velocity");
  auto * voltage  = find_state("voltage");
  auto * temp     = find_state("temperature");
  auto * current  = find_state("current");
  auto * effort   = find_state("effort");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(voltage, nullptr);
  ASSERT_NE(temp, nullptr);
  ASSERT_NE(current, nullptr);
  ASSERT_NE(effort, nullptr);

  (void)vel_cmd->set_value(1.0);
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double v = 0.0, tmp = 0.0, cur = 0.0, eff = 0.0;
  (void)voltage->get_value(v, true);
  (void)temp->get_value(tmp, true);
  (void)current->get_value(cur, true);
  (void)effort->get_value(eff, true);
  EXPECT_LT(v, 12.0);
  EXPECT_GT(tmp, 25.0);
  EXPECT_GT(cur, 0.0);
  EXPECT_NE(eff, 0.0);
}

// ============================================================
// Emergency stop via service call
//
// read() internally calls rclcpp::spin_some(node_), allowing service
// callbacks on the hardware interface's ROS node to be processed without
// direct access to the private node pointer.
// ============================================================

class HardwareInterfaceEmergencyStopTest : public HardwareInterfaceMockTest
{
protected:
  void SetUp() override
  {
    HardwareInterfaceMockTest::SetUp();
    client_node_ = rclcpp::Node::make_shared("test_estop_client_node");
    estop_client_ = client_node_->create_client<std_srvs::srv::SetBool>("/emergency_stop");
  }

  void TearDown() override
  {
    estop_client_.reset();
    client_node_.reset();
    HardwareInterfaceMockTest::TearDown();
  }

  /// Call /emergency_stop with activate=true/false, wait for service response.
  /// Spins client_node_ and hw_'s internal node (via hw_->read()) in a loop
  /// until the future resolves or ~500 ms elapses.
  bool call_estop(bool activate)
  {
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = activate;
    auto future = estop_client_->async_send_request(req);

    rclcpp::Time t(0, 0, RCL_ROS_TIME);
    rclcpp::Duration d(0, 0);
    for (int i = 0; i < 50; ++i) {
      rclcpp::spin_some(client_node_);
      hw_->read(t, d);  // internally calls rclcpp::spin_some(node_)
      if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        return future.get()->success;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;  // timed out
  }

  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr estop_client_;
};

TEST_F(HardwareInterfaceEmergencyStopTest, EmergencyStopServiceFlow) {
  // Service is available after configure; activate and release both succeed
  ASSERT_TRUE(estop_client_->wait_for_service(std::chrono::seconds(5)));
  EXPECT_TRUE(call_estop(true));   // activate
  EXPECT_TRUE(call_estop(false));  // release
}

TEST_F(HardwareInterfaceEmergencyStopTest, EmergencyStopClearsMotionState) {
  // After emergency stop, write() clears velocity command → is_moving = 0
  ASSERT_TRUE(estop_client_->wait_for_service(std::chrono::seconds(5)));

  auto * vel_cmd   = find_cmd("velocity");
  auto * is_moving = find_state("is_moving");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(is_moving, nullptr);

  (void)vel_cmd->set_value(2.0);
  ASSERT_TRUE(call_estop(true));

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);   // activates stop → clears velocity command
  hw_->read(t, d);    // hw_state_velocity_ = hw_cmd_velocity_ = 0 → is_moving = 0

  double vel_val = 999.0, is_moving_val = -1.0;
  (void)vel_cmd->get_value(vel_val, true);
  (void)is_moving->get_value(is_moving_val, true);
  EXPECT_NEAR(vel_val, 0.0, 1e-9);
  EXPECT_NEAR(is_moving_val, 0.0, 1e-9);
}

TEST_F(HardwareInterfaceEmergencyStopTest, EmergencyStopReleaseAllowsVelocityCommand) {
  // After releasing emergency stop, commands are no longer cleared by write()
  ASSERT_TRUE(estop_client_->wait_for_service(std::chrono::seconds(5)));

  auto * vel_cmd = find_cmd("velocity");
  ASSERT_NE(vel_cmd, nullptr);

  ASSERT_TRUE(call_estop(true));

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);  // activates emergency stop, clears commands

  // Release emergency stop
  ASSERT_TRUE(call_estop(false));

  // Write once to transition emergency_stop_active_ to false
  hw_->write(t, d);

  // Set a velocity command after release; write() must not clear it
  (void)vel_cmd->set_value(3.0);
  hw_->write(t, d);

  double vel_val = 0.0;
  (void)vel_cmd->get_value(vel_val, true);
  EXPECT_NEAR(vel_val, 3.0, 1e-9);
}

// ============================================================
// Multi-joint tests
// ============================================================

class HardwareInterfaceMultiJointTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hw_ = std::make_unique<sts_hardware_interface::STSHardwareInterface>();
    info_ = make_two_motor_info();
    ASSERT_EQ(hw_->on_init(info_), CallbackReturn::SUCCESS);
    state_ifaces_ = hw_->export_state_interfaces();
    cmd_ifaces_ = hw_->export_command_interfaces();
  }

  std::unique_ptr<sts_hardware_interface::STSHardwareInterface> hw_;
  hardware_interface::HardwareInfo info_;
  std::vector<hardware_interface::StateInterface> state_ifaces_;
  std::vector<hardware_interface::CommandInterface> cmd_ifaces_;
};

TEST_F(HardwareInterfaceMultiJointTest, AllInterfacesPresent) {
  // 2 joints × 7 state interfaces = 14; command interfaces: 2 + 1 = 3
  EXPECT_EQ(state_ifaces_.size(), 14u);
  EXPECT_EQ(cmd_ifaces_.size(), 3u);

  const std::vector<std::string> expected_types = {
    "position", "velocity", "effort", "voltage", "temperature", "current", "is_moving"
  };
  const std::vector<std::string> expected_joints = {"wheel_joint", "arm_joint"};

  for (const auto & joint_name : expected_joints) {
    for (const auto & iface_type : expected_types) {
      bool found = false;
      for (const auto & si : state_ifaces_) {
        if (si.get_prefix_name() == joint_name && si.get_interface_name() == iface_type) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Missing: " << joint_name << "/" << iface_type;
    }
  }
}

TEST_F(HardwareInterfaceMultiJointTest, FullLifecycleSucceeds) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

  EXPECT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_activate(inactive), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_deactivate(active), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_cleanup(inactive), CallbackReturn::SUCCESS);
}

TEST_F(HardwareInterfaceMultiJointTest, ReadWriteSucceedForBothJoints) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

  ASSERT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive), CallbackReturn::SUCCESS);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  EXPECT_EQ(hw_->read(t, d), return_type::OK);
  EXPECT_EQ(hw_->write(t, d), return_type::OK);

  hw_->on_deactivate(active);
  hw_->on_cleanup(inactive);
}

TEST_F(HardwareInterfaceMultiJointTest, ShutdownSucceeds) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  ASSERT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  EXPECT_EQ(hw_->on_shutdown(inactive), CallbackReturn::SUCCESS);
}

// ============================================================
// on_init: additional validation gaps — invalid string inputs
// ============================================================

TEST(HardwareInterfaceInitTest, MissingMotorIdReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters.erase("motor_id");
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidHardwareParameterStringReturnsError) {
  // Non-numeric string for a hardware-level parameter → RETURN_ERROR
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters["baud_rate"] = "not_a_number";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidJointParameterStringReturnsError) {
  // Non-numeric string for a joint-level parameter → RETURN_ERROR
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters["min_position"] = "not_a_number";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, ValidMaxVelocitySetsLimitSucceeds) {
  // max_velocity > 0 → sets has_velocity_limits_[i] = true, returns SUCCESS
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters["max_velocity"] = "5.0";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(HardwareInterfaceInitTest, ValidMaxEffortSetsLimitSucceeds) {
  // max_effort in (0, 1] → sets has_effort_limits_[i] = true, returns SUCCESS
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_effort_motor_info();
  info.joints[0].parameters["max_effort"] = "0.5";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(HardwareInterfaceInitTest, DefaultOperatingModeUsesVelocityMode) {
  // No operating_mode parameter → defaults to MODE_VELOCITY via else branch
  sts_hardware_interface::STSHardwareInterface hw;
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";

  hardware_interface::ComponentInfo joint;
  joint.name = "default_mode_joint";
  joint.parameters["motor_id"] = "5";
  // No operating_mode key — triggers the else branch (line 169-171 in source)
  hardware_interface::InterfaceInfo v, a;
  v.name = "velocity";
  a.name = "acceleration";
  joint.command_interfaces.push_back(v);
  joint.command_interfaces.push_back(a);
  info.joints.push_back(joint);

  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

// ============================================================
// on_activate with reset_states_on_activate = false
// ============================================================

TEST(HardwareInterfaceActivateResetTest, ActivateWithResetDisabledSucceeds) {
  // reset_states_on_activate = false → skips zeroing states, logs "preserving" message
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters["reset_states_on_activate"] = "false";
  ASSERT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);

  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  ASSERT_EQ(hw.on_configure(unconfigured), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw.on_activate(inactive), CallbackReturn::SUCCESS);
}

// ============================================================
// PWM mode mock read: effort command drives velocity and position
// ============================================================

class HardwareInterfacePwmMockTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hw_ = std::make_unique<sts_hardware_interface::STSHardwareInterface>();
    info_ = make_valid_effort_motor_info();
    ASSERT_EQ(hw_->on_init(info_), CallbackReturn::SUCCESS);
    state_ifaces_ = hw_->export_state_interfaces();
    cmd_ifaces_ = hw_->export_command_interfaces();

    rclcpp_lifecycle::State unconfigured(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    ASSERT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw_->on_activate(inactive), CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    rclcpp_lifecycle::State active(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
    rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    hw_->on_deactivate(active);
    hw_->on_cleanup(inactive);
  }

  hardware_interface::CommandInterface * find_cmd(const std::string & name)
  {
    for (auto & iface : cmd_ifaces_) {
      if (iface.get_interface_name() == name) { return &iface; }
    }
    return nullptr;
  }

  hardware_interface::StateInterface * find_state(const std::string & name)
  {
    for (auto & iface : state_ifaces_) {
      if (iface.get_interface_name() == name) { return &iface; }
    }
    return nullptr;
  }

  std::unique_ptr<sts_hardware_interface::STSHardwareInterface> hw_;
  hardware_interface::HardwareInfo info_;
  std::vector<hardware_interface::StateInterface> state_ifaces_;
  std::vector<hardware_interface::CommandInterface> cmd_ifaces_;
};

TEST_F(HardwareInterfacePwmMockTest, PwmEffortCommandDrivesPositionChange) {
  // In PWM mock: velocity = effort * 10.0, position += velocity * dt
  // At 0.5 effort, dt=0.1s: Δpos = 0.5 * 10.0 * 0.1 = 0.5 rad
  auto * eff_cmd = find_cmd("effort");
  auto * pos_state = find_state("position");
  ASSERT_NE(eff_cmd, nullptr);
  ASSERT_NE(pos_state, nullptr);

  (void)eff_cmd->set_value(0.5);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration dt(0, static_cast<int32_t>(0.1e9));  // 100 ms
  hw_->write(t, dt);
  hw_->read(t, dt);

  double pos = 0.0;
  (void)pos_state->get_value(pos, true);
  EXPECT_NEAR(pos, 0.5, 1e-6);
}

TEST_F(HardwareInterfacePwmMockTest, PwmZeroEffortHoldsPosition) {
  // Zero effort → zero velocity → position stays at 0
  auto * eff_cmd = find_cmd("effort");
  auto * pos_state = find_state("position");
  ASSERT_NE(eff_cmd, nullptr);
  ASSERT_NE(pos_state, nullptr);

  (void)eff_cmd->set_value(0.0);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration dt(0, static_cast<int32_t>(0.1e9));
  hw_->write(t, dt);
  hw_->read(t, dt);

  double pos = 0.0;
  (void)pos_state->get_value(pos, true);
  EXPECT_NEAR(pos, 0.0, 1e-9);
}

// ============================================================
// Servo mode mock read: position stepping vs. snap-to-target
// ============================================================

class HardwareInterfaceServoMockTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hw_ = std::make_unique<sts_hardware_interface::STSHardwareInterface>();
    info_ = make_servo_info_with_velocity();  // position + velocity for mock stepping tests
    ASSERT_EQ(hw_->on_init(info_), CallbackReturn::SUCCESS);
    state_ifaces_ = hw_->export_state_interfaces();
    cmd_ifaces_ = hw_->export_command_interfaces();

    rclcpp_lifecycle::State unconfigured(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    ASSERT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw_->on_activate(inactive), CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    rclcpp_lifecycle::State active(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
    rclcpp_lifecycle::State inactive(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    hw_->on_deactivate(active);
    hw_->on_cleanup(inactive);
  }

  hardware_interface::CommandInterface * find_cmd(const std::string & name)
  {
    for (auto & iface : cmd_ifaces_) {
      if (iface.get_interface_name() == name) { return &iface; }
    }
    return nullptr;
  }

  hardware_interface::StateInterface * find_state(const std::string & name)
  {
    for (auto & iface : state_ifaces_) {
      if (iface.get_interface_name() == name) { return &iface; }
    }
    return nullptr;
  }

  std::unique_ptr<sts_hardware_interface::STSHardwareInterface> hw_;
  hardware_interface::HardwareInfo info_;
  std::vector<hardware_interface::StateInterface> state_ifaces_;
  std::vector<hardware_interface::CommandInterface> cmd_ifaces_;
};

TEST_F(HardwareInterfaceServoMockTest, PositionStepsWhenTargetFarAndSpeedSmall) {
  // Overshoot branch: |error| > max_step → step by max_step
  // max_step = 0.01 rad/s * 0.01s = 0.0001 rad
  // error = 3.14 rad >> 0.0001 → enters the stepping branch
  auto * pos_cmd = find_cmd("position");
  auto * vel_cmd = find_cmd("velocity");
  auto * pos_state = find_state("position");
  ASSERT_NE(pos_cmd, nullptr);
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(pos_state, nullptr);

  (void)pos_cmd->set_value(3.14);   // far target
  (void)vel_cmd->set_value(0.01);   // very slow max speed

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration dt(0, static_cast<int32_t>(0.01e9));  // 10 ms
  hw_->write(t, dt);
  hw_->read(t, dt);

  double pos = 0.0;
  (void)pos_state->get_value(pos, true);
  // Should have stepped by exactly max_step = 0.01 * 0.01 = 0.0001
  EXPECT_NEAR(pos, 0.0001, 1e-9);
}

TEST_F(HardwareInterfaceServoMockTest, PositionSnapsWhenCloseEnough) {
  // Snap branch: |error| <= max_step → snaps directly to target
  // max_step = 1.0 rad/s * 0.1s = 0.1 rad > error = 0.05 rad
  auto * pos_cmd = find_cmd("position");
  auto * vel_cmd = find_cmd("velocity");
  auto * pos_state = find_state("position");
  ASSERT_NE(pos_cmd, nullptr);
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(pos_state, nullptr);

  (void)pos_cmd->set_value(0.05);
  (void)vel_cmd->set_value(1.0);  // max_step = 0.1 > error = 0.05

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration dt(0, static_cast<int32_t>(0.1e9));  // 100 ms
  hw_->write(t, dt);
  hw_->read(t, dt);

  double pos = 0.0;
  (void)pos_state->get_value(pos, true);
  EXPECT_NEAR(pos, 0.05, 1e-9);
}

TEST_F(HardwareInterfaceServoMockTest, SteppingVelocityDerivedFromDisplacement) {
  // After a step, hw_state_velocity_ = step / dt (not the commanded speed)
  // step = max_step = max_speed * dt_sec, so derived velocity = max_speed
  auto * pos_cmd = find_cmd("position");
  auto * vel_cmd = find_cmd("velocity");
  auto * vel_state = find_state("velocity");
  ASSERT_NE(pos_cmd, nullptr);
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(vel_state, nullptr);

  const double max_speed = 0.01;  // rad/s
  const double dt_sec = 0.01;     // 10 ms
  (void)pos_cmd->set_value(3.14);          // far target
  (void)vel_cmd->set_value(max_speed);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration dt(0, static_cast<int32_t>(dt_sec * 1e9));
  hw_->write(t, dt);
  hw_->read(t, dt);

  double vel = 0.0;
  (void)vel_state->get_value(vel, true);
  // derived velocity = (max_speed * dt_sec) / dt_sec = max_speed
  EXPECT_NEAR(vel, max_speed, 1e-9);
}

TEST_F(HardwareInterfaceServoMockTest, NegativePositionErrorStepsBackward) {
  // Overshoot with negative error: target < current → steps in negative direction
  // First snap to a positive position.
  auto * pos_cmd = find_cmd("position");
  auto * vel_cmd = find_cmd("velocity");
  auto * pos_state = find_state("position");
  ASSERT_NE(pos_cmd, nullptr);
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(pos_state, nullptr);

  // Snap to 1.0 rad first
  (void)pos_cmd->set_value(1.0);
  (void)vel_cmd->set_value(10.0);  // fast: max_step >> error
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration dt_snap(0, static_cast<int32_t>(0.1e9));
  hw_->write(t, dt_snap);
  hw_->read(t, dt_snap);

  double pos_after_snap = 0.0;
  (void)pos_state->get_value(pos_after_snap, true);
  ASSERT_NEAR(pos_after_snap, 1.0, 1e-9);

  // Now command negative direction: target = -3.0, slow speed → steps backward
  (void)pos_cmd->set_value(-3.0);
  (void)vel_cmd->set_value(0.01);  // max_step = 0.01 * 0.01 = 0.0001 rad
  rclcpp::Duration dt_step(0, static_cast<int32_t>(0.01e9));  // 10 ms
  hw_->write(t, dt_step);
  hw_->read(t, dt_step);

  double pos_after_step = 0.0;
  (void)pos_state->get_value(pos_after_step, true);
  // Stepped backward: new pos = 1.0 - 0.0001 = 0.9999
  EXPECT_NEAR(pos_after_step, 1.0 - 0.0001, 1e-9);
}

// ============================================================
// on_init: servo mode (MODE_SERVO) command interface validation
// position is required; velocity and acceleration are optional.
// ============================================================

TEST(HardwareInterfaceInitTest, ServoModeWithPositionAndVelocity) {
  // position + velocity → SUCCESS (velocity is optional in MODE_SERVO)
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_servo_info_with_velocity();
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(HardwareInterfaceInitTest, ServoModeWithAllInterfaces) {
  // position + velocity + acceleration → SUCCESS (both extras are optional)
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_servo_info_with_vel_acc();
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(HardwareInterfaceInitTest, ServoModeWithoutPositionReturnsError) {
  // MODE_SERVO without the mandatory position interface → ERROR
  sts_hardware_interface::STSHardwareInterface hw;
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";

  hardware_interface::ComponentInfo joint;
  joint.name = "arm_joint";
  joint.parameters["motor_id"] = "2";
  joint.parameters["operating_mode"] = "0";

  // Only velocity — position is missing
  hardware_interface::InterfaceInfo vel_iface;
  vel_iface.name = "velocity";
  joint.command_interfaces.push_back(vel_iface);

  info.joints.push_back(joint);
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

// ============================================================
// on_init: velocity mode (MODE_VELOCITY) command interface validation
// velocity is required; acceleration is optional.
// ============================================================

TEST(HardwareInterfaceInitTest, VelocityModeWithVelocityOnly) {
  // velocity only (no acceleration) → SUCCESS
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_velocity_info_no_acc();
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(HardwareInterfaceInitTest, VelocityModeWithoutVelocityReturnsError) {
  // MODE_VELOCITY without the mandatory velocity interface → ERROR
  sts_hardware_interface::STSHardwareInterface hw;
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["serial_port"] = "/dev/ttyACM0";
  info.hardware_parameters["enable_mock_mode"] = "true";

  hardware_interface::ComponentInfo joint;
  joint.name = "wheel_joint";
  joint.parameters["motor_id"] = "1";
  joint.parameters["operating_mode"] = "1";

  // Only acceleration — velocity is missing
  hardware_interface::InterfaceInfo acc_iface;
  acc_iface.name = "acceleration";
  joint.command_interfaces.push_back(acc_iface);

  info.joints.push_back(joint);
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

// ============================================================
// on_init: proportional_acc_deadband parameter validation
// ============================================================

TEST(HardwareInterfaceInitTest, InvalidProportionalAccDeadbandReturnsError) {
  // Negative value and non-numeric string both reject
  for (const char * val : {"-0.1", "not_a_number"}) {
    sts_hardware_interface::STSHardwareInterface hw;
    auto info = make_valid_single_motor_info();
    info.hardware_parameters["proportional_acc_deadband"] = val;
    EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR) << "value=\"" << val << "\" should fail";
  }
}

TEST(HardwareInterfaceInitTest, ValidProportionalAccDeadbandSucceeds) {
  // Zero disables the deadband; positive value enables it — both valid
  for (const char * val : {"0.0", "0.1"}) {
    sts_hardware_interface::STSHardwareInterface hw;
    auto info = make_valid_single_motor_info();
    info.hardware_parameters["proportional_acc_deadband"] = val;
    EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS) << "value=\"" << val << "\" should succeed";
  }
}

// ============================================================
// on_init: proportional_vel_deadband parameter validation
// ============================================================

TEST(HardwareInterfaceInitTest, InvalidProportionalVelDeadbandReturnsError) {
  // Negative value and non-numeric string both reject
  for (const char * val : {"-0.01", "not_a_number"}) {
    sts_hardware_interface::STSHardwareInterface hw;
    auto info = make_valid_single_motor_info();
    info.hardware_parameters["proportional_vel_deadband"] = val;
    EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR) << "value=\"" << val << "\" should fail";
  }
}

TEST(HardwareInterfaceInitTest, ValidProportionalVelDeadbandSucceeds) {
  // Zero disables the deadband; positive value enables it — both valid
  for (const char * val : {"0.0", "0.05"}) {
    sts_hardware_interface::STSHardwareInterface hw;
    auto info = make_valid_single_motor_info();
    info.hardware_parameters["proportional_vel_deadband"] = val;
    EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS) << "value=\"" << val << "\" should succeed";
  }
}

// ============================================================
// on_init: proportional_vel_max parameter validation
// Valid range: [0, max_velocity_steps].  Default max_velocity_steps = 3400.
// ============================================================

TEST(HardwareInterfaceInitTest, InvalidProportionalVelMaxReturnsError) {
  // Negative, exceeding max_velocity_steps (3400), and non-numeric → RETURN_ERROR
  for (const char * val : {"-1", "3401", "not_a_number"}) {
    sts_hardware_interface::STSHardwareInterface hw;
    auto info = make_valid_position_motor_info();
    info.hardware_parameters["proportional_vel_max"] = val;
    EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR) << "value=\"" << val << "\" should fail";
  }
}

TEST(HardwareInterfaceInitTest, ProportionalVelMaxZeroSucceeds) {
  // 0 = disabled; valid regardless of joint count, no warning
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_position_motor_info();
  info.hardware_parameters["proportional_vel_max"] = "0";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(HardwareInterfaceInitTest, ProportionalVelMaxWithSingleServoJointWarnsButSucceeds) {
  // proportional_vel_max > 0 with only 1 servo joint → WARN (SyncWrite needs >1
  // joints) but the configuration is still considered valid → SUCCESS
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_position_motor_info();
  info.hardware_parameters["proportional_vel_max"] = "500";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

// ============================================================
// export_command_interfaces: servo mode optional interface exports
// ============================================================

TEST(HardwareInterfaceExportTest, CommandInterfacesServoModeWithVelocityAndAcc) {
  // Mode 0 with position + velocity + acceleration in URDF → exports 3 command interfaces
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_servo_info_with_vel_acc();
  ASSERT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);

  auto cmd_ifaces = hw.export_command_interfaces();
  EXPECT_EQ(cmd_ifaces.size(), 3u);

  std::vector<std::string> names;
  for (const auto & iface : cmd_ifaces) {
    names.push_back(iface.get_interface_name());
  }
  EXPECT_NE(std::find(names.begin(), names.end(), "position"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "velocity"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "acceleration"), names.end());
}

TEST(HardwareInterfaceExportTest, CommandInterfacesVelocityModeWithoutAcceleration) {
  // Mode 1 with velocity-only URDF → exports 1 command interface (velocity only)
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_velocity_info_no_acc();
  ASSERT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);

  auto cmd_ifaces = hw.export_command_interfaces();
  EXPECT_EQ(cmd_ifaces.size(), 1u);

  std::vector<std::string> names;
  for (const auto & iface : cmd_ifaces) {
    names.push_back(iface.get_interface_name());
  }
  EXPECT_NE(std::find(names.begin(), names.end(), "velocity"), names.end());
  EXPECT_EQ(std::find(names.begin(), names.end(), "acceleration"), names.end());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
