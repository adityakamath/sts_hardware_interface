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
    for (const auto & name : {"position", "velocity", "acceleration"}) {
      hardware_interface::InterfaceInfo iface;
      iface.name = name;
      joint.command_interfaces.push_back(iface);
    }
    info.joints.push_back(joint);
  }

  return info;
}

// ---- on_init: success cases ----

TEST(HardwareInterfaceInitTest, ValidSingleMotorVelocityMode) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(HardwareInterfaceInitTest, ValidSingleMotorPositionMode) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_position_motor_info();
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(HardwareInterfaceInitTest, ValidSingleMotorEffortMode) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_effort_motor_info();
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
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

TEST(HardwareInterfaceInitTest, InvalidMotorIdZeroReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info("/dev/ttyACM0", "0");
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMotorId254ReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info("/dev/ttyACM0", "254");
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
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
  // Mode 0: position + velocity + acceleration = 3 command interfaces
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_position_motor_info();
  ASSERT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);

  auto cmd_ifaces = hw.export_command_interfaces();
  EXPECT_EQ(cmd_ifaces.size(), 3u);
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

TEST_F(HardwareInterfaceLifecycleTest, ShutdownFromConfiguredSucceeds) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  ASSERT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_shutdown(inactive), CallbackReturn::SUCCESS);
}

TEST_F(HardwareInterfaceLifecycleTest, ShutdownFromActiveSucceeds) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_shutdown(active), CallbackReturn::SUCCESS);
}

TEST_F(HardwareInterfaceLifecycleTest, ShutdownWithoutConfigureSucceeds) {
  // Mock mode: on_shutdown should succeed even if on_configure was never called
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  EXPECT_EQ(hw_->on_shutdown(unconfigured), CallbackReturn::SUCCESS);
}

// ============================================================
// Additional lifecycle: on_error (uses HardwareInterfaceLifecycleTest)
// ============================================================

TEST_F(HardwareInterfaceLifecycleTest, OnErrorInMockModeSucceeds) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  // on_error in mock mode returns SUCCESS (simulated emergency stop)
  EXPECT_EQ(hw_->on_error(unconfigured), CallbackReturn::SUCCESS);
}

TEST_F(HardwareInterfaceLifecycleTest, OnErrorAfterActivateSucceeds) {
  rclcpp_lifecycle::State unconfigured(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(hw_->on_configure(unconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_error(active), CallbackReturn::SUCCESS);
}

// ============================================================
// Deactivate state clearing (via HardwareInterfaceMockTest fixture)
// on_deactivate in mock mode: zeros hw_state_velocity_ and hw_state_effort_
// ============================================================

TEST_F(HardwareInterfaceMockTest, DeactivateClearsVelocityState) {
  auto * vel_cmd = find_cmd("velocity");
  auto * vel_state = find_state("velocity");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(vel_state, nullptr);

  // Drive velocity state to non-zero
  (void)vel_cmd->set_value(2.0);
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double vel_val = 0.0;
  (void)vel_state->get_value(vel_val, true);
  ASSERT_NE(vel_val, 0.0) << "Pre-condition: velocity state must be non-zero before deactivate";

  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(hw_->on_deactivate(active), CallbackReturn::SUCCESS);

  (void)vel_state->get_value(vel_val, true);
  EXPECT_NEAR(vel_val, 0.0, 1e-9);
}

TEST_F(HardwareInterfaceMockTest, DeactivateClearsEffortState) {
  auto * vel_cmd = find_cmd("velocity");
  auto * effort_state = find_state("effort");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(effort_state, nullptr);

  // Non-zero velocity produces a non-zero simulated effort (load) state
  (void)vel_cmd->set_value(2.0);
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double eff_val = 0.0;
  (void)effort_state->get_value(eff_val, true);
  ASSERT_NE(eff_val, 0.0) << "Pre-condition: effort state must be non-zero before deactivate";

  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(hw_->on_deactivate(active), CallbackReturn::SUCCESS);

  (void)effort_state->get_value(eff_val, true);
  EXPECT_NEAR(eff_val, 0.0, 1e-9);
}

// ============================================================
// Simulated state values — zero velocity produces baseline readings
// ============================================================

TEST_F(HardwareInterfaceMockTest, ZeroVelocityIsNotMoving) {
  // Default velocity command = 0.0; read() should report is_moving = 0
  auto * is_moving = find_state("is_moving");
  ASSERT_NE(is_moving, nullptr);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double val = -1.0;
  (void)is_moving->get_value(val, true);
  EXPECT_NEAR(val, 0.0, 1e-9);
}

TEST_F(HardwareInterfaceMockTest, NonZeroVelocityIsMoving) {
  auto * vel_cmd = find_cmd("velocity");
  auto * is_moving = find_state("is_moving");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(is_moving, nullptr);

  (void)vel_cmd->set_value(1.0);  // 1.0 rad/s >> 0.01 threshold
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double val = -1.0;
  (void)is_moving->get_value(val, true);
  EXPECT_NEAR(val, 1.0, 1e-9);
}

TEST_F(HardwareInterfaceMockTest, VelocityBelowIsMovingThresholdIsNotMoving) {
  auto * vel_cmd = find_cmd("velocity");
  auto * is_moving = find_state("is_moving");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(is_moving, nullptr);

  (void)vel_cmd->set_value(0.005);  // 0.005 rad/s < 0.01 threshold
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double val = -1.0;
  (void)is_moving->get_value(val, true);
  EXPECT_NEAR(val, 0.0, 1e-9);
}

TEST_F(HardwareInterfaceMockTest, ZeroVelocityVoltageIsBaseline) {
  // Zero velocity → zero simulated effort → no voltage drop → 12.0 V
  auto * voltage = find_state("voltage");
  ASSERT_NE(voltage, nullptr);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double val = 0.0;
  (void)voltage->get_value(val, true);
  EXPECT_NEAR(val, 12.0, 1e-6);
}

TEST_F(HardwareInterfaceMockTest, ZeroVelocityTemperatureIsBaseline) {
  // Zero velocity → zero thermal load → ambient temperature = 25.0°C
  auto * temp = find_state("temperature");
  ASSERT_NE(temp, nullptr);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double val = 0.0;
  (void)temp->get_value(val, true);
  EXPECT_NEAR(val, 25.0, 1e-6);
}

TEST_F(HardwareInterfaceMockTest, ZeroVelocityCurrentIsZero) {
  // Zero velocity → zero simulated effort → zero current draw
  auto * current = find_state("current");
  ASSERT_NE(current, nullptr);

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double val = -1.0;
  (void)current->get_value(val, true);
  EXPECT_NEAR(val, 0.0, 1e-9);
}

TEST_F(HardwareInterfaceMockTest, NonZeroVelocityReducesVoltage) {
  // Non-zero velocity → simulated load → voltage drop below 12 V
  auto * vel_cmd = find_cmd("velocity");
  auto * voltage = find_state("voltage");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(voltage, nullptr);

  (void)vel_cmd->set_value(1.0);
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double val = 0.0;
  (void)voltage->get_value(val, true);
  EXPECT_LT(val, 12.0);
}

TEST_F(HardwareInterfaceMockTest, NonZeroVelocityIncreasesTemperature) {
  // Non-zero velocity → thermal load → temperature above ambient
  auto * vel_cmd = find_cmd("velocity");
  auto * temp = find_state("temperature");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(temp, nullptr);

  (void)vel_cmd->set_value(1.0);
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double val = 0.0;
  (void)temp->get_value(val, true);
  EXPECT_GT(val, 25.0);
}

TEST_F(HardwareInterfaceMockTest, NonZeroVelocityCurrentIsPositive) {
  // Non-zero velocity → simulated effort → current draw > 0
  auto * vel_cmd = find_cmd("velocity");
  auto * current = find_state("current");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(current, nullptr);

  (void)vel_cmd->set_value(1.0);
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double val = 0.0;
  (void)current->get_value(val, true);
  EXPECT_GT(val, 0.0);
}

TEST_F(HardwareInterfaceMockTest, NonZeroVelocityEffortStateIsNonZero) {
  // Non-zero velocity produces a non-zero simulated effort (load percentage / 100)
  auto * vel_cmd = find_cmd("velocity");
  auto * effort = find_state("effort");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(effort, nullptr);

  (void)vel_cmd->set_value(1.0);
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);
  hw_->read(t, d);

  double val = 0.0;
  (void)effort->get_value(val, true);
  EXPECT_NE(val, 0.0);
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

TEST_F(HardwareInterfaceEmergencyStopTest, ServiceIsAvailableAfterConfigure) {
  EXPECT_TRUE(estop_client_->wait_for_service(std::chrono::seconds(5)));
}

TEST_F(HardwareInterfaceEmergencyStopTest, ActivateServiceCallSucceeds) {
  ASSERT_TRUE(estop_client_->wait_for_service(std::chrono::seconds(5)));
  EXPECT_TRUE(call_estop(true));
}

TEST_F(HardwareInterfaceEmergencyStopTest, ReleaseServiceCallSucceeds) {
  ASSERT_TRUE(estop_client_->wait_for_service(std::chrono::seconds(5)));
  ASSERT_TRUE(call_estop(true));
  EXPECT_TRUE(call_estop(false));
}

TEST_F(HardwareInterfaceEmergencyStopTest, EmergencyStopClearsVelocityCommand) {
  // After emergency stop activation, write() must clear velocity command to 0
  ASSERT_TRUE(estop_client_->wait_for_service(std::chrono::seconds(5)));

  auto * vel_cmd = find_cmd("velocity");
  ASSERT_NE(vel_cmd, nullptr);
  (void)vel_cmd->set_value(2.0);

  // call_estop processes the service callback via read()'s spin_some
  ASSERT_TRUE(call_estop(true));

  // write() sees hw_cmd_emergency_stop_ > 0.5 → activates stop → clears commands
  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);

  double vel_val = 999.0;
  (void)vel_cmd->get_value(vel_val, true);
  EXPECT_NEAR(vel_val, 0.0, 1e-9);
}

TEST_F(HardwareInterfaceEmergencyStopTest, EmergencyStopHoldsIsMovingAtZero) {
  // After emergency stop, velocity command is cleared → read() reports not moving
  ASSERT_TRUE(estop_client_->wait_for_service(std::chrono::seconds(5)));

  auto * vel_cmd = find_cmd("velocity");
  auto * is_moving = find_state("is_moving");
  ASSERT_NE(vel_cmd, nullptr);
  ASSERT_NE(is_moving, nullptr);
  (void)vel_cmd->set_value(2.0);

  ASSERT_TRUE(call_estop(true));

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration d(0, 0);
  hw_->write(t, d);   // clears velocity command
  hw_->read(t, d);    // hw_state_velocity_ = hw_cmd_velocity_ = 0 → is_moving = 0

  double val = -1.0;
  (void)is_moving->get_value(val, true);
  EXPECT_NEAR(val, 0.0, 1e-9);
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

TEST_F(HardwareInterfaceMultiJointTest, TotalStateInterfaceCount) {
  // 2 joints × 7 state interfaces each = 14 total
  EXPECT_EQ(state_ifaces_.size(), 14u);
}

TEST_F(HardwareInterfaceMultiJointTest, TotalCommandInterfaceCount) {
  // wheel_joint (vel mode): velocity + acceleration = 2
  // arm_joint   (pos mode): position + velocity + acceleration = 3
  // Total = 5
  EXPECT_EQ(cmd_ifaces_.size(), 5u);
}

TEST_F(HardwareInterfaceMultiJointTest, AllStateInterfaceNamesPresent) {
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

TEST(HardwareInterfaceInitTest, InvalidMotorIdStringReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters["motor_id"] = "not_a_number";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidOperatingModeStringReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters["operating_mode"] = "bad_mode";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidBaudRateStringReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters["baud_rate"] = "not_a_number";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidCommunicationTimeoutStringReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters["communication_timeout_ms"] = "not_a_number";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMaxVelocityStepsStringReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.hardware_parameters["max_velocity_steps"] = "not_a_number";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMinPositionStringReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters["min_position"] = "not_a_number";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMaxPositionStringReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters["max_position"] = "not_a_number";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMaxVelocityStringReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_single_motor_info();
  info.joints[0].parameters["max_velocity"] = "not_a_number";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(HardwareInterfaceInitTest, InvalidMaxEffortStringReturnsError) {
  sts_hardware_interface::STSHardwareInterface hw;
  auto info = make_valid_effort_motor_info();
  info.joints[0].parameters["max_effort"] = "not_a_number";
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
    info_ = make_valid_position_motor_info();
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
