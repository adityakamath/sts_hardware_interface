// Copyright 2025 Aditya Kamath
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

/** @file sts_hardware_interface.cpp
 * @brief ros2_control hardware interface implementation for Feetech STS servo motors */

#include "sts_hardware_interface/sts_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace sts_hardware_interface
{

/** @brief Parse URDF parameters and validate configuration */
hardware_interface::CallbackReturn STSHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // Store hardware info
  info_ = params.hardware_info;

  // Initialize logger
  logger_ = rclcpp::get_logger("STSHardwareInterface");

  RCLCPP_INFO(logger_, "Initializing STS Hardware Interface: %s", info_.name.c_str());

  // ===== Parse hardware-level parameters =====
  try {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  } catch (const std::out_of_range &) {
    RCLCPP_ERROR(
      logger_,
      "Missing required parameter: serial_port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse and validate baud_rate (default: 1000000)
  // Supported baud rates: 9600, 19200, 38400, 57600, 115200, 500000, 1000000
  static const std::vector<int> valid_baud_rates = {9600, 19200, 38400, 57600, 115200, 500000, 1000000};
  baud_rate_ = std::stoi(
    info_.hardware_parameters.count("baud_rate") ?
    info_.hardware_parameters.at("baud_rate") : "1000000");

  if (std::find(valid_baud_rates.begin(), valid_baud_rates.end(), baud_rate_) == valid_baud_rates.end()) {
    RCLCPP_ERROR(
      logger_,
      "Invalid baud_rate: %d. Supported rates: 9600, 19200, 38400, 57600, 115200, 500000, 1000000",
      baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse and validate communication_timeout_ms (default: 100, range: 1-1000)
  communication_timeout_ms_ = std::stoi(
    info_.hardware_parameters.count("communication_timeout_ms") ?
    info_.hardware_parameters.at("communication_timeout_ms") : "100");

  if (communication_timeout_ms_ < 1 || communication_timeout_ms_ > 1000) {
    RCLCPP_ERROR(
      logger_,
      "Invalid communication_timeout_ms: %d. Must be between 1 and 1000 ms",
      communication_timeout_ms_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse boolean parameters
  enable_mock_mode_ = (info_.hardware_parameters.count("enable_mock_mode") &&
                       info_.hardware_parameters.at("enable_mock_mode") == "true");

  use_sync_write_ = (info_.hardware_parameters.count("use_sync_write") &&
                     info_.hardware_parameters.at("use_sync_write") == "false") ? false : true;

  // ===== Parse joint-level parameters =====
  size_t num_joints = info_.joints.size();

  if (num_joints == 0) {
    RCLCPP_ERROR(
      logger_,
      "No joints defined in hardware interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_,
    "Configuring %zu joint(s)", num_joints);

  // Resize all per-joint vectors
  joint_names_.resize(num_joints);
  motor_ids_.resize(num_joints);
  operating_modes_.resize(num_joints, MODE_VELOCITY);  // Default to velocity mode

  hw_state_position_.resize(num_joints, 0.0);
  hw_state_velocity_.resize(num_joints, 0.0);
  hw_state_load_.resize(num_joints, 0.0);
  hw_state_voltage_.resize(num_joints, 12.0);  // Mock voltage value
  hw_state_temperature_.resize(num_joints, 25.0);  // Mock temperature value
  hw_state_current_.resize(num_joints, 0.0);
  hw_state_is_moving_.resize(num_joints, 0.0);

  // Initialize state interface tracking (will be populated based on URDF)
  has_position_state_.resize(num_joints, false);
  has_velocity_state_.resize(num_joints, false);
  has_load_state_.resize(num_joints, false);
  has_voltage_state_.resize(num_joints, false);
  has_temperature_state_.resize(num_joints, false);
  has_current_state_.resize(num_joints, false);
  has_is_moving_state_.resize(num_joints, false);

  hw_cmd_position_.resize(num_joints, 0.0);
  hw_cmd_velocity_.resize(num_joints, 0.0);
  hw_cmd_acceleration_.resize(num_joints, 0.0);
  hw_cmd_effort_.resize(num_joints, 0.0);

  // Initialize broadcast emergency stop
  hw_cmd_emergency_stop_ = 0.0;
  emergency_stop_active_ = false;

  position_min_.resize(num_joints, 0.0);  // Default: 0 radians (0 steps)
  position_max_.resize(num_joints, 2.0 * M_PI);  // Default: 2π radians (4096 steps)
  velocity_max_.resize(num_joints, static_cast<double>(STS_MAX_VELOCITY_STEPS) * STEPS_TO_RAD);  // Default: 3400 steps/s ≈ 5.22 rad/s
  effort_max_.resize(num_joints, 1.0);
  has_position_limits_.resize(num_joints, false);
  has_velocity_limits_.resize(num_joints, false);
  has_effort_limits_.resize(num_joints, false);

  // Parse each joint
  for (size_t i = 0; i < num_joints; ++i) {
    const auto & joint = info_.joints[i];
    joint_names_[i] = joint.name;
    joint_name_to_index_[joint.name] = i;

    // Parse motor_id (required per joint)
    try {
      motor_ids_[i] = std::stoi(joint.parameters.at("motor_id"));
    } catch (const std::out_of_range &) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s': Missing required parameter 'motor_id'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate motor ID range
    if (motor_ids_[i] < STS_MIN_MOTOR_ID || motor_ids_[i] > STS_MAX_MOTOR_ID) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s': Invalid motor_id %d (must be between %d and %d)",
        joint.name.c_str(), motor_ids_[i], STS_MIN_MOTOR_ID, STS_MAX_MOTOR_ID);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Parse operating_mode (per joint)
    if (joint.parameters.count("operating_mode")) {
      operating_modes_[i] = std::stoi(joint.parameters.at("operating_mode"));
    } else {
      // Default to velocity mode if not specified
      operating_modes_[i] = MODE_VELOCITY;
    }

    // Validate operating mode
    if (operating_modes_[i] < MODE_SERVO || operating_modes_[i] > MODE_PWM) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s': Invalid operating_mode: %d (must be 0, 1, or 2)",
        joint.name.c_str(), operating_modes_[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Parse and validate optional joint limits
    if (joint.parameters.count("min_position")) {
      position_min_[i] = std::stod(joint.parameters.at("min_position"));
      has_position_limits_[i] = true;
    }
    if (joint.parameters.count("max_position")) {
      position_max_[i] = std::stod(joint.parameters.at("max_position"));
      has_position_limits_[i] = true;
    }

    // Validate position limits are sensible
    if (has_position_limits_[i] && position_min_[i] >= position_max_[i]) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s': min_position (%.3f) must be less than max_position (%.3f)",
        joint.name.c_str(), position_min_[i], position_max_[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.parameters.count("max_velocity")) {
      velocity_max_[i] = std::stod(joint.parameters.at("max_velocity"));

      // Validate velocity limit is positive and reasonable
      if (velocity_max_[i] <= 0.0) {
        RCLCPP_ERROR(
          logger_,
          "Joint '%s': max_velocity must be positive (got %.3f rad/s)",
          joint.name.c_str(), velocity_max_[i]);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Warn if velocity exceeds hardware maximum (~20 rad/s based on 3400 steps/s)
      const double hw_max_velocity = (STS_MAX_VELOCITY_STEPS * STEPS_TO_RAD);
      if (velocity_max_[i] > hw_max_velocity) {
        RCLCPP_WARN(
          logger_,
          "Joint '%s': max_velocity (%.3f rad/s) exceeds hardware maximum (~%.3f rad/s)",
          joint.name.c_str(), velocity_max_[i], hw_max_velocity);
      }

      has_velocity_limits_[i] = true;
    }

    if (joint.parameters.count("max_effort")) {
      effort_max_[i] = std::stod(joint.parameters.at("max_effort"));

      // Validate effort limit for PWM mode (should be 0-1)
      if (operating_modes_[i] == MODE_PWM && (effort_max_[i] <= 0.0 || effort_max_[i] > 1.0)) {
        RCLCPP_ERROR(
          logger_,
          "Joint '%s': max_effort must be between 0 and 1 for PWM mode (got %.3f)",
          joint.name.c_str(), effort_max_[i]);
        return hardware_interface::CallbackReturn::ERROR;
      }

      has_effort_limits_[i] = true;
    }


    // Validate command interfaces match operating mode
    std::vector<std::string> required_cmd_interfaces;
    switch (operating_modes_[i]) {
      case MODE_SERVO:
        required_cmd_interfaces = {"position", "velocity", "acceleration"};
        break;
      case MODE_VELOCITY:
        required_cmd_interfaces = {"velocity", "acceleration"};
        break;
      case MODE_PWM:
        required_cmd_interfaces = {"effort"};
        break;
    }

    for (const auto & required_interface : required_cmd_interfaces) {
      bool found = false;
      for (const auto & cmd_interface : joint.command_interfaces) {
        if (cmd_interface.name == required_interface) {
          found = true;
          break;
        }
      }
      if (!found) {
        RCLCPP_ERROR(
          logger_,
          "Joint '%s': Missing required command interface '%s' for operating mode %d",
          joint.name.c_str(), required_interface.c_str(), operating_modes_[i]);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Populate state interface tracking based on URDF declarations
    for (const auto & state_interface : joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        has_position_state_[i] = true;
      } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_state_[i] = true;
      } else if (state_interface.name == "load") {
        has_load_state_[i] = true;
      } else if (state_interface.name == "voltage") {
        has_voltage_state_[i] = true;
      } else if (state_interface.name == "temperature") {
        has_temperature_state_[i] = true;
      } else if (state_interface.name == "current") {
        has_current_state_[i] = true;
      } else if (state_interface.name == "is_moving") {
        has_is_moving_state_[i] = true;
      }
    }

    RCLCPP_INFO(
      logger_,
      "Joint '%s': motor_id=%d, mode=%d, limits: pos[%.2f, %.2f] vel[%.2f] eff[%.2f]",
      joint.name.c_str(), motor_ids_[i], operating_modes_[i],
      position_min_[i], position_max_[i], velocity_max_[i], effort_max_[i]);
  }

  // Check for duplicate motor IDs
  for (size_t i = 0; i < num_joints; ++i) {
    for (size_t j = i + 1; j < num_joints; ++j) {
      if (motor_ids_[i] == motor_ids_[j]) {
        RCLCPP_ERROR(
          logger_,
          "Duplicate motor_id %d found in joints '%s' and '%s'",
          motor_ids_[i], joint_names_[i].c_str(), joint_names_[j].c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // Initialize error tracking
  consecutive_read_errors_ = 0;
  consecutive_write_errors_ = 0;

  RCLCPP_INFO(
    logger_,
    "Initialization complete: %zu joints, port=%s, baud=%d, sync_write=%s, mock=%s",
    num_joints, serial_port_.c_str(), baud_rate_,
    use_sync_write_ ? "true" : "false",
    enable_mock_mode_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Initialize serial communication and verify motors */
hardware_interface::CallbackReturn STSHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    logger_,
    "Configuring STS hardware interface...");

  // Skip serial port initialization in mock mode
  if (enable_mock_mode_) {
    RCLCPP_INFO(
      logger_,
      "Mock mode enabled - skipping serial port initialization");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Create serial connection
  servo_ = std::make_shared<SMS_STS>();

  if (!servo_->begin(baud_rate_, serial_port_.c_str())) {
    RCLCPP_ERROR(
      logger_,
      "Failed to open serial port %s at baud rate %d",
      serial_port_.c_str(), baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_,
    "Opened serial port %s at baud rate %d",
    serial_port_.c_str(), baud_rate_);

  // Verify communication with all motors
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int ping_result = servo_->Ping(motor_ids_[i]);
    if (ping_result == -1) {
      int servo_error = servo_->getErr();
      RCLCPP_ERROR(
        logger_,
        "Failed to ping motor %d (joint '%s') - servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(), servo_error);
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      logger_,
      "Motor %d (joint '%s') responded to ping",
      motor_ids_[i], joint_names_[i].c_str());
  }

  RCLCPP_INFO(
    logger_,
    "Configuration complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Set operating modes, enable torque, and read initial states */
hardware_interface::CallbackReturn STSHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    logger_,
    "Activating STS hardware interface...");

  // Skip hardware activation in mock mode
  if (enable_mock_mode_) {
    RCLCPP_INFO(
      logger_,
      "Mock mode: Motors initialized (simulated)");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Initialize all motors
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int result = servo_->InitMotor(motor_ids_[i], operating_modes_[i], 1);  // 1 = enable torque
    if (result != 1) {
      int servo_error = servo_->getErr();
      RCLCPP_ERROR(
        logger_,
        "Failed to initialize motor %d (joint '%s') in mode %d - servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i], servo_error);
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      logger_,
      "Motor %d (joint '%s') initialized in mode %d with torque enabled",
      motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i]);
  }

  RCLCPP_INFO(
    logger_,
    "All motors activated and ready");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Export state interfaces for all joints */
std::vector<hardware_interface::StateInterface>
STSHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string & joint_name = joint_names_[i];

    // Only export state interfaces declared in URDF
    if (has_position_state_[i]) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint_name, hardware_interface::HW_IF_POSITION, &hw_state_position_[i]));
    }

    if (has_velocity_state_[i]) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint_name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_[i]));
    }

    if (has_load_state_[i]) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint_name, "load", &hw_state_load_[i]));
    }

    if (has_voltage_state_[i]) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint_name, "voltage", &hw_state_voltage_[i]));
    }

    if (has_temperature_state_[i]) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint_name, "temperature", &hw_state_temperature_[i]));
    }

    if (has_current_state_[i]) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint_name, "current", &hw_state_current_[i]));
    }

    if (has_is_moving_state_[i]) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint_name, "is_moving", &hw_state_is_moving_[i]));
    }
  }

  RCLCPP_INFO(
    logger_,
    "Exported %zu state interfaces for %zu joints",
    state_interfaces.size(), joint_names_.size());

  return state_interfaces;
}

/** @brief Export command interfaces for all joints */
std::vector<hardware_interface::CommandInterface>
STSHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string & joint_name = joint_names_[i];

    // Mode-dependent command interfaces (per joint)
    switch (operating_modes_[i]) {
      case MODE_SERVO:  // Position control
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &hw_cmd_position_[i]));
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocity_[i]));
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            joint_name, "acceleration", &hw_cmd_acceleration_[i]));
        break;

      case MODE_VELOCITY:  // Velocity control
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocity_[i]));
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            joint_name, "acceleration", &hw_cmd_acceleration_[i]));
        break;

      case MODE_PWM:  // PWM/effort control
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_EFFORT, &hw_cmd_effort_[i]));
        break;
    }

  }

  // Broadcast emergency stop (stops ALL motors at once)
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.name, "emergency_stop", &hw_cmd_emergency_stop_));

  RCLCPP_INFO(
    logger_,
    "Exported %zu command interfaces for %zu joints (per-joint modes)",
    command_interfaces.size(), joint_names_.size());

  return command_interfaces;
}

/** @brief Read motor states from hardware */
hardware_interface::return_type STSHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Mock mode: simulate hardware behavior using state variables directly
  if (enable_mock_mode_) {
    double dt = period.seconds();

    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      // Always simulate position and velocity for control logic
      double prev_position = hw_state_position_[i];

      if (operating_modes_[i] == MODE_SERVO) {
        // Simulate position control with first-order response
        double position_error = hw_cmd_position_[i] - hw_state_position_[i];
        double max_step = hw_cmd_velocity_[i] * dt;
        if (std::abs(position_error) > max_step && max_step > 0) {
          hw_state_position_[i] += (position_error > 0 ? max_step : -max_step);
        } else {
          hw_state_position_[i] = hw_cmd_position_[i];
        }
        hw_state_velocity_[i] = (hw_state_position_[i] - prev_position) / (dt > 0 ? dt : 0.001);
      } else if (operating_modes_[i] == MODE_VELOCITY) {
        // Simulate velocity control
        hw_state_velocity_[i] = hw_cmd_velocity_[i];
        hw_state_position_[i] += hw_state_velocity_[i] * dt;
      } else {  // MODE_PWM
        // Simulate PWM as torque control
        hw_state_velocity_[i] = hw_cmd_effort_[i] * 10.0;
        hw_state_position_[i] += hw_state_velocity_[i] * dt;
      }

      // Only simulate dynamic state interfaces
      if (has_load_state_[i]) {
        // Simulate load based on velocity
        hw_state_load_[i] = hw_state_velocity_[i] / (STS_MAX_VELOCITY_STEPS * STEPS_TO_RAD) * 100.0;
        hw_state_load_[i] = std::clamp(hw_state_load_[i], -100.0, 100.0);
      }

      // Voltage, temperature, and current remain at constant mock values (12.0V, 25.0°C, 0.0A)

      if (has_is_moving_state_[i]) {
        // Update is_moving state
        hw_state_is_moving_[i] = (std::abs(hw_state_velocity_[i]) > 0.01) ? 1.0 : 0.0;
      }
    }

    return hardware_interface::return_type::OK;
  }

  // Real hardware mode: read feedback from all motors
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int result = servo_->FeedBack(motor_ids_[i]);
    if (result != 1) {
      consecutive_read_errors_++;
      int servo_error = servo_->getErr();
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(), 1000,
        "Failed to read feedback from motor %d (joint '%s') - error count: %d/%d, servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(),
        consecutive_read_errors_, MAX_CONSECUTIVE_ERRORS, servo_error);

      if (consecutive_read_errors_ >= MAX_CONSECUTIVE_ERRORS) {
        RCLCPP_ERROR(
          logger_,
          "Too many consecutive read errors (%d), attempting recovery...",
          consecutive_read_errors_);

        if (attempt_error_recovery()) {
          RCLCPP_INFO(
            logger_,
            "Error recovery successful");
          consecutive_read_errors_ = 0;
        } else {
          RCLCPP_ERROR(
            logger_,
            "Error recovery failed");
          return hardware_interface::return_type::ERROR;
        }
      }
      return hardware_interface::return_type::ERROR;
    }

    // Reset error counter on successful read
    consecutive_read_errors_ = 0;

    // Read and update only enabled state interfaces for efficiency
    if (has_position_state_[i]) {
      int raw_position = servo_->ReadPos(-1);
      hw_state_position_[i] = raw_position_to_radians(raw_position);
    }

    if (has_velocity_state_[i]) {
      int raw_velocity = servo_->ReadSpeed(-1);
      hw_state_velocity_[i] = raw_velocity_to_rad_s(raw_velocity);
    }

    if (has_load_state_[i]) {
      int raw_load = servo_->ReadLoad(-1);
      hw_state_load_[i] = raw_load_to_percentage(raw_load);
    }

    if (has_voltage_state_[i]) {
      int raw_voltage = servo_->ReadVoltage(-1);
      hw_state_voltage_[i] = raw_voltage_to_volts(raw_voltage);
    }

    if (has_temperature_state_[i]) {
      int raw_temperature = servo_->ReadTemper(-1);
      hw_state_temperature_[i] = static_cast<double>(raw_temperature);
    }

    if (has_current_state_[i]) {
      int raw_current = servo_->ReadCurrent(-1);
      hw_state_current_[i] = raw_current_to_amperes(raw_current);
    }

    if (has_is_moving_state_[i]) {
      int raw_is_moving = servo_->ReadMove(-1);
      hw_state_is_moving_[i] = (raw_is_moving > 0) ? 1.0 : 0.0;
    }
  }

  return hardware_interface::return_type::OK;
}

/** @brief Send commands to motors based on operating mode */
hardware_interface::return_type STSHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Mock mode: skip hardware writes
  if (enable_mock_mode_) {
    // Handle broadcast emergency stop in mock mode
    if (hw_cmd_emergency_stop_ > 0.5 && !emergency_stop_active_) {
      for (size_t i = 0; i < motor_ids_.size(); ++i) {
        hw_state_velocity_[i] = 0.0;
      }
      emergency_stop_active_ = true;
      RCLCPP_WARN(logger_, "Emergency stop activated - ALL motors stopped");
    } else if (hw_cmd_emergency_stop_ <= 0.5 && emergency_stop_active_) {
      emergency_stop_active_ = false;
      RCLCPP_INFO(logger_, "Emergency stop released");
    }

    return hardware_interface::return_type::OK;
  }

  // Check for broadcast emergency stop (stops ALL motors)
  if (hw_cmd_emergency_stop_ > 0.5 && !emergency_stop_active_) {
    RCLCPP_WARN(logger_, "Emergency stop activated - stopping ALL motors");
    servo_->WriteSpe(STS_BROADCAST_ID, 0, STS_MAX_ACCELERATION);
    emergency_stop_active_ = true;
    return hardware_interface::return_type::OK;
  }

  if (hw_cmd_emergency_stop_ <= 0.5 && emergency_stop_active_) {
    RCLCPP_INFO(logger_, "Emergency stop released");
    emergency_stop_active_ = false;
  }

  // Skip all commands if emergency stop is active
  if (emergency_stop_active_) {
    return hardware_interface::return_type::OK;
  }

  // Group motors by operating mode for efficient SyncWrite
  std::vector<size_t> servo_motors;     // MODE_SERVO (position control)
  std::vector<size_t> velocity_motors;  // MODE_VELOCITY
  std::vector<size_t> pwm_motors;       // MODE_PWM

  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    switch (operating_modes_[i]) {
      case MODE_SERVO:
        servo_motors.push_back(i);
        break;
      case MODE_VELOCITY:
        velocity_motors.push_back(i);
        break;
      case MODE_PWM:
        pwm_motors.push_back(i);
        break;
    }
  }

  // ===== WRITE COMMANDS FOR SERVO MODE MOTORS =====
  if (!servo_motors.empty()) {
    if (use_sync_write_ && servo_motors.size() > 1) {
      // Use SyncWrite for multiple servo motors
      std::vector<u8> ids;
      std::vector<s16> positions;
      std::vector<u16> speeds;
      std::vector<u8> accelerations;

      for (size_t idx : servo_motors) {
        // Apply position limits
        double target_position = apply_position_limits(hw_cmd_position_[idx], idx);

        // Apply velocity limits
        double max_speed = hw_cmd_velocity_[idx];
        if (has_velocity_limits_[idx]) {
          max_speed = std::min(max_speed, velocity_max_[idx]);
        }

        ids.push_back(motor_ids_[idx]);
        positions.push_back(radians_to_raw_position(target_position));
        speeds.push_back(static_cast<u16>(std::clamp(
          rad_s_to_raw_velocity(max_speed), 0, STS_MAX_VELOCITY_STEPS)));
        accelerations.push_back(static_cast<u8>(std::clamp(
          hw_cmd_acceleration_[idx], 0.0, static_cast<double>(STS_MAX_ACCELERATION))));
      }

      // SyncWrite to all servo mode motors
      servo_->SyncWritePosEx(
        ids.data(), ids.size(),
        positions.data(), speeds.data(), accelerations.data());

      // No result to check for SyncWritePosEx (void return)

    } else {
      // Individual writes for single servo motor or when SyncWrite disabled
      for (size_t idx : servo_motors) {
        double target_position = apply_position_limits(hw_cmd_position_[idx], idx);

        double max_speed = hw_cmd_velocity_[idx];
        if (has_velocity_limits_[idx]) {
          max_speed = std::min(max_speed, velocity_max_[idx]);
        }

        int raw_position = radians_to_raw_position(target_position);
        int raw_max_speed = std::clamp(
          rad_s_to_raw_velocity(max_speed), 0, STS_MAX_VELOCITY_STEPS);
        int acceleration = static_cast<int>(std::clamp(
          hw_cmd_acceleration_[idx], 0.0, static_cast<double>(STS_MAX_ACCELERATION)));

        int result = servo_->WritePosEx(motor_ids_[idx], raw_position, raw_max_speed, acceleration);

        if (result != 1) {
          consecutive_write_errors_++;
          int servo_error = servo_->getErr();
          RCLCPP_WARN_THROTTLE(
            logger_,
            *rclcpp::Clock::make_shared(), 1000,
            "Failed to write position to motor %d (joint '%s') - error count: %d/%d, servo error: %d",
            motor_ids_[idx], joint_names_[idx].c_str(),
            consecutive_write_errors_, MAX_CONSECUTIVE_ERRORS, servo_error);

          if (consecutive_write_errors_ >= MAX_CONSECUTIVE_ERRORS && attempt_error_recovery()) {
            consecutive_write_errors_ = 0;
          }
          return hardware_interface::return_type::ERROR;
        }
      }
    }
  }

  // ===== WRITE COMMANDS FOR VELOCITY MODE MOTORS =====
  if (!velocity_motors.empty()) {
    if (use_sync_write_ && velocity_motors.size() > 1) {
      // Use SyncWrite for multiple velocity motors
      std::vector<u8> ids;
      std::vector<s16> velocities;
      std::vector<u8> accelerations;

      for (size_t idx : velocity_motors) {
        // Apply velocity limits
        double target_velocity = apply_velocity_limits(hw_cmd_velocity_[idx], idx);

        ids.push_back(motor_ids_[idx]);
        velocities.push_back(rad_s_to_raw_velocity(target_velocity));
        accelerations.push_back(static_cast<u8>(std::clamp(
          hw_cmd_acceleration_[idx], 0.0, static_cast<double>(STS_MAX_ACCELERATION))));
      }

      // SyncWrite to all velocity mode motors
      servo_->SyncWriteSpe(
        ids.data(), ids.size(),
        velocities.data(), accelerations.data());

      // No result to check for SyncWriteSpe (void return)

    } else {
      // Individual writes for single velocity motor or when SyncWrite disabled
      for (size_t idx : velocity_motors) {
        double target_velocity = apply_velocity_limits(hw_cmd_velocity_[idx], idx);

        int raw_velocity = rad_s_to_raw_velocity(target_velocity);
        int acceleration = static_cast<int>(std::clamp(
          hw_cmd_acceleration_[idx], 0.0, static_cast<double>(STS_MAX_ACCELERATION)));

        int result = servo_->WriteSpe(motor_ids_[idx], raw_velocity, acceleration);

        if (result != 1) {
          consecutive_write_errors_++;
          int servo_error = servo_->getErr();
          RCLCPP_WARN_THROTTLE(
            logger_,
            *rclcpp::Clock::make_shared(), 1000,
            "Failed to write velocity to motor %d (joint '%s') - error count: %d/%d, servo error: %d",
            motor_ids_[idx], joint_names_[idx].c_str(),
            consecutive_write_errors_, MAX_CONSECUTIVE_ERRORS, servo_error);

          if (consecutive_write_errors_ >= MAX_CONSECUTIVE_ERRORS && attempt_error_recovery()) {
            consecutive_write_errors_ = 0;
          }
          return hardware_interface::return_type::ERROR;
        }
      }
    }
  }

  // ===== WRITE COMMANDS FOR PWM MODE MOTORS =====
  if (!pwm_motors.empty()) {
    if (use_sync_write_ && pwm_motors.size() > 1) {
      // Use SyncWrite for multiple PWM motors
      std::vector<u8> ids;
      std::vector<s16> pwm_values;

      for (size_t idx : pwm_motors) {
        // Apply effort limits
        double target_effort = hw_cmd_effort_[idx];
        if (has_effort_limits_[idx]) {
          double normalized_limit = effort_max_[idx];
          target_effort = std::clamp(target_effort, -normalized_limit, normalized_limit);
        }

        ids.push_back(motor_ids_[idx]);
        pwm_values.push_back(effort_to_raw_pwm(target_effort));
      }

      // SyncWrite to all PWM mode motors
      servo_->SyncWritePwm(ids.data(), ids.size(), pwm_values.data());

    } else {
      // Individual writes for single PWM motor or when SyncWrite disabled
      for (size_t idx : pwm_motors) {
        double target_effort = hw_cmd_effort_[idx];
        if (has_effort_limits_[idx]) {
          double normalized_limit = effort_max_[idx];
          target_effort = std::clamp(target_effort, -normalized_limit, normalized_limit);
        }

        int raw_pwm = effort_to_raw_pwm(target_effort);
        int result = servo_->WritePwm(motor_ids_[idx], raw_pwm);

        if (result != 1) {
          consecutive_write_errors_++;
          int servo_error = servo_->getErr();
          RCLCPP_WARN_THROTTLE(
            logger_,
            *rclcpp::Clock::make_shared(), 1000,
            "Failed to write PWM to motor %d (joint '%s') - error count: %d/%d, servo error: %d",
            motor_ids_[idx], joint_names_[idx].c_str(),
            consecutive_write_errors_, MAX_CONSECUTIVE_ERRORS, servo_error);

          if (consecutive_write_errors_ >= MAX_CONSECUTIVE_ERRORS && attempt_error_recovery()) {
            consecutive_write_errors_ = 0;
          }
          return hardware_interface::return_type::ERROR;
        }
      }
    }
  }

  consecutive_write_errors_ = 0;

  return hardware_interface::return_type::OK;
}

/** @brief Stop motors and disable torque */
hardware_interface::CallbackReturn STSHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    logger_,
    "Deactivating STS hardware interface...");

  // Skip hardware deactivation in mock mode
  if (enable_mock_mode_) {
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      hw_state_velocity_[i] = 0.0;
      hw_state_load_[i] = 0.0;
    }
    RCLCPP_INFO(
      logger_,
      "Mock mode: All motors stopped (simulated)");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Stop all motors based on current operating mode
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int result = -1;
    switch (operating_modes_[i]) {
      case MODE_SERVO: {
        int current_pos = servo_->ReadPos(motor_ids_[i]);
        if (current_pos != -1) {
          result = servo_->WritePosEx(motor_ids_[i], current_pos, 0, 0);
        }
        break;
      }
      case MODE_VELOCITY:
        result = servo_->WriteSpe(motor_ids_[i], 0, 0);
        break;
      case MODE_PWM:
        result = servo_->WritePwm(motor_ids_[i], 0);
        break;
    }

    if (result != 1) {
      RCLCPP_WARN(
        logger_,
        "Failed to stop motor %d (joint '%s') during deactivation (mode: %d)",
        motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i]);
    }

    // Disable torque
    result = servo_->EnableTorque(motor_ids_[i], 0);
    if (result != 1) {
      int servo_error = servo_->getErr();
      RCLCPP_WARN(
        logger_,
        "Failed to disable torque on motor %d (joint '%s') - servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(), servo_error);
    }

    RCLCPP_INFO(
      logger_,
      "Motor %d (joint '%s') stopped and torque disabled",
      motor_ids_[i], joint_names_[i].c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Close serial connection and release resources */
hardware_interface::CallbackReturn STSHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up STS hardware interface...");

  // Close serial connection
  if (servo_) {
    servo_->end();
    RCLCPP_INFO(
      logger_,
      "Closed serial connection to %s", serial_port_.c_str());
  }

  servo_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Emergency shutdown - stop all motors and close connection */
hardware_interface::CallbackReturn STSHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    logger_,
    "Shutting down STS hardware interface...");

  // Ensure all motors are stopped and torque disabled
  if (servo_) {
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      // Stop motor based on current operating mode
      switch (operating_modes_[i]) {
        case MODE_SERVO: {
          int current_pos = servo_->ReadPos(motor_ids_[i]);
          if (current_pos != -1) {
            servo_->WritePosEx(motor_ids_[i], current_pos, 0, 0);
          }
          break;
        }
        case MODE_VELOCITY:
          servo_->WriteSpe(motor_ids_[i], 0, 0);
          break;
        case MODE_PWM:
          servo_->WritePwm(motor_ids_[i], 0);
          break;
      }

      // Disable torque for safety
      servo_->EnableTorque(motor_ids_[i], 0);

      RCLCPP_INFO(
        logger_,
        "Motor %d (joint '%s') shutdown complete",
        motor_ids_[i], joint_names_[i].c_str());
    }

    // Close serial connection
    servo_->end();
    RCLCPP_INFO(
      logger_,
      "Closed serial connection to %s during shutdown", serial_port_.c_str());

    servo_.reset();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Handle error state - emergency stop all motors */
hardware_interface::CallbackReturn STSHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(
    logger_,
    "Error state detected, attempting emergency stop...");

  if (!servo_) {
    RCLCPP_ERROR(
      logger_,
      "Servo interface not available for emergency stop");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Emergency stop: set all motors to safe state
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int result = -1;
    switch (operating_modes_[i]) {
      case MODE_SERVO: {
        int current_pos = servo_->ReadPos(motor_ids_[i]);
        if (current_pos != -1) {
          result = servo_->WritePosEx(motor_ids_[i], current_pos, 0, 0);
        }
        break;
      }
      case MODE_VELOCITY:
        result = servo_->WriteSpe(motor_ids_[i], 0, 254);  // Max deceleration
        break;
      case MODE_PWM:
        result = servo_->WritePwm(motor_ids_[i], 0);
        break;
    }

    if (result != 1) {
      RCLCPP_ERROR(
        logger_,
        "Failed to send emergency stop command to motor %d (joint '%s')",
        motor_ids_[i], joint_names_[i].c_str());
    }

    // Disable torque for safety
    result = servo_->EnableTorque(motor_ids_[i], 0);
    if (result != 1) {
      RCLCPP_ERROR(
        logger_,
        "Failed to disable torque on motor %d (joint '%s') during error handling",
        motor_ids_[i], joint_names_[i].c_str());
    } else {
      RCLCPP_INFO(
        logger_,
        "Motor %d (joint '%s') emergency stopped and torque disabled",
        motor_ids_[i], joint_names_[i].c_str());
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Convert motor steps (0-4095) to radians (0-2π) */
double STSHardwareInterface::raw_position_to_radians(int raw_position) const
{
  // Convert raw position (0-4095) to radians, then invert direction
  // We want: raw 0 → 0, raw increases → position decreases
  // Invert by computing (4095 - raw_position) which maps:
  //   raw 0 → 4095 → ~2π, raw 4095 → 0 → 0
  // This makes increasing raw position correspond to decreasing angle
  int inverted_raw = STS_MAX_POSITION - raw_position;
  return static_cast<double>(inverted_raw) * STEPS_TO_RAD;
}

/** @brief Convert motor velocity (steps/s) to rad/s */
double STSHardwareInterface::raw_velocity_to_rad_s(int raw_velocity) const
{
  return static_cast<double>(-raw_velocity) * STEPS_TO_RAD;
}

/** @brief Convert rad/s to motor velocity (steps/s) */
int STSHardwareInterface::rad_s_to_raw_velocity(double velocity_rad_s) const
{
  double raw = -velocity_rad_s * RAD_TO_STEPS;
  int clamped = static_cast<int>(std::clamp(
    raw,
    static_cast<double>(-STS_MAX_VELOCITY_STEPS),
    static_cast<double>(STS_MAX_VELOCITY_STEPS)));
  return clamped;
}

/** @brief Convert motor load (-1000 to +1000) to percentage */
double STSHardwareInterface::raw_load_to_percentage(int raw_load) const
{
  return static_cast<double>(raw_load) * LOAD_SCALE;
}

/** @brief Convert motor voltage (decivolts) to volts */
double STSHardwareInterface::raw_voltage_to_volts(int raw_voltage) const
{
  return static_cast<double>(raw_voltage) * VOLTAGE_SCALE;
}

/** @brief Convert motor current (milliamps) to amperes */
double STSHardwareInterface::raw_current_to_amperes(int raw_current) const
{
  return static_cast<double>(raw_current) * CURRENT_SCALE;
}

/** @brief Convert radians to motor steps (0-4095) */
int STSHardwareInterface::radians_to_raw_position(double position_rad) const
{
  double normalized = std::fmod(-position_rad, 2.0 * M_PI);
  if (normalized < 0.0)
    normalized += 2.0 * M_PI;
  int raw = static_cast<int>(normalized * RAD_TO_STEPS);
  return std::clamp(raw, 0, 4095);
}

/** @brief Convert effort (-1.0 to +1.0) to motor PWM (-1000 to +1000) */
int STSHardwareInterface::effort_to_raw_pwm(double effort) const
{
  // Effort is normalized [-1.0, 1.0] -> raw PWM [-1000, 1000]
  double clamped = std::clamp(effort, -1.0, 1.0);
  return static_cast<int>(clamped * STS_MAX_PWM);
}

/** @brief Apply position limits to a target position */
double STSHardwareInterface::apply_position_limits(double target_position, size_t joint_idx) const
{
  if (has_position_limits_[joint_idx]) {
    return std::clamp(target_position, position_min_[joint_idx], position_max_[joint_idx]);
  }
  return target_position;
}

/** @brief Apply velocity limits to a target velocity */
double STSHardwareInterface::apply_velocity_limits(double target_velocity, size_t joint_idx) const
{
  if (has_velocity_limits_[joint_idx]) {
    return std::clamp(target_velocity, -velocity_max_[joint_idx], velocity_max_[joint_idx]);
  }
  return target_velocity;
}

/** @brief Attempt to recover from communication errors by pinging motors */
bool STSHardwareInterface::attempt_error_recovery()
{
  RCLCPP_WARN(
    logger_,
    "Attempting error recovery for all motors...");

  if (!servo_) {
    RCLCPP_ERROR(
      logger_,
      "Servo interface not available for recovery");
    return false;
  }

  // Try to ping all motors
  bool all_motors_responsive = true;
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int ping_result = servo_->Ping(motor_ids_[i]);
    if (ping_result == -1) {
      RCLCPP_WARN(
        logger_,
        "Motor %d (joint '%s') not responding to ping",
        motor_ids_[i], joint_names_[i].c_str());
      all_motors_responsive = false;
    } else {
      RCLCPP_INFO(
        logger_,
        "Motor %d (joint '%s') responded to ping",
        motor_ids_[i], joint_names_[i].c_str());
    }
  }

  if (all_motors_responsive) {
    RCLCPP_INFO(
      logger_,
      "All motors responded to ping, communication restored");
    return true;
  }

  // Try to re-establish serial connection
  RCLCPP_WARN(
    logger_,
    "Some motors not responding, attempting to reinitialize serial connection to %s",
    serial_port_.c_str());

  // Close and reopen serial port
  servo_->end();

  if (!servo_->begin(baud_rate_, serial_port_.c_str())) {
    RCLCPP_ERROR(
      logger_,
      "Failed to reopen serial port %s during recovery", serial_port_.c_str());
    return false;
  }

  // Verify all motors communication
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int ping_result = servo_->Ping(motor_ids_[i]);
    if (ping_result == -1) {
      RCLCPP_ERROR(
        logger_,
        "Motor %d (joint '%s') still not responding after serial reinit",
        motor_ids_[i], joint_names_[i].c_str());
      return false;
    }

    // Re-activate motor
    int init_result = servo_->InitMotor(motor_ids_[i], operating_modes_[i], 1);
    if (init_result != 1) {
      RCLCPP_ERROR(
        logger_,
        "Failed to reinitialize motor %d (joint '%s') after recovery",
        motor_ids_[i], joint_names_[i].c_str());
      return false;
    }

    RCLCPP_INFO(
      logger_,
      "Motor %d (joint '%s') reinitialized successfully",
      motor_ids_[i], joint_names_[i].c_str());
  }

  RCLCPP_INFO(
    logger_,
    "Successfully recovered communication with all motors");
  return true;
}

}  // namespace sts_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  sts_hardware_interface::STSHardwareInterface,
  hardware_interface::SystemInterface)

