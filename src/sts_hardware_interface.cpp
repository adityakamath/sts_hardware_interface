// Copyright (c) 2025 Aditya Kamath
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "sts_hardware_interface/sts_hardware_interface.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

namespace sts_hardware_interface
{

// Static member initialization
std::map<std::string, std::shared_ptr<SMS_STS>> STSHardwareInterface::serial_port_connections_;
std::mutex STSHardwareInterface::serial_port_mutex_;

// ============================================================================
// LIFECYCLE: on_init
// ============================================================================

hardware_interface::CallbackReturn STSHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // Store hardware info
  info_ = params.hardware_info;

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Initializing STS Hardware Interface: %s", info_.name.c_str());

  // ===== Parse hardware-level parameters =====
  try {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  } catch (const std::out_of_range &) {
    RCLCPP_ERROR(
      rclcpp::get_logger("STSHardwareInterface"),
      "Missing required parameter: serial_port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse optional hardware parameters with defaults
  baud_rate_ = std::stoi(
    info_.hardware_parameters.count("baud_rate") ?
    info_.hardware_parameters.at("baud_rate") : "1000000");

  communication_timeout_ms_ = std::stoi(
    info_.hardware_parameters.count("communication_timeout_ms") ?
    info_.hardware_parameters.at("communication_timeout_ms") : "100");

  enable_multi_turn_ = (info_.hardware_parameters.count("enable_multi_turn") &&
                        info_.hardware_parameters.at("enable_multi_turn") == "true");

  enable_mock_mode_ = (info_.hardware_parameters.count("enable_mock_mode") &&
                       info_.hardware_parameters.at("enable_mock_mode") == "true");

  use_sync_write_ = (info_.hardware_parameters.count("use_sync_write") &&
                     info_.hardware_parameters.at("use_sync_write") == "false") ? false : true;

  // ===== Parse joint-level parameters =====
  size_t num_joints = info_.joints.size();

  if (num_joints == 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("STSHardwareInterface"),
      "No joints defined in hardware interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Configuring %zu joint(s)", num_joints);

  // Resize all per-joint vectors
  joint_names_.resize(num_joints);
  motor_ids_.resize(num_joints);
  operating_modes_.resize(num_joints, MODE_VELOCITY);  // Default to velocity mode
  reverse_direction_.resize(num_joints, false);  // Default to no reversal

  hw_state_position_.resize(num_joints, 0.0);
  hw_state_velocity_.resize(num_joints, 0.0);
  hw_state_load_.resize(num_joints, 0.0);
  hw_state_voltage_.resize(num_joints, 0.0);
  hw_state_temperature_.resize(num_joints, 0.0);
  hw_state_current_.resize(num_joints, 0.0);
  hw_state_is_moving_.resize(num_joints, 0.0);

  hw_cmd_position_.resize(num_joints, 0.0);
  hw_cmd_velocity_.resize(num_joints, 0.0);
  hw_cmd_acceleration_.resize(num_joints, 0.0);
  hw_cmd_effort_.resize(num_joints, 0.0);
  hw_cmd_emergency_stop_.resize(num_joints, 0.0);
  emergency_stop_active_.resize(num_joints, false);

  // Initialize broadcast emergency stop
  hw_cmd_broadcast_emergency_stop_ = 0.0;
  broadcast_emergency_stop_active_ = false;

  position_min_.resize(num_joints, -std::numeric_limits<double>::infinity());
  position_max_.resize(num_joints, std::numeric_limits<double>::infinity());
  velocity_max_.resize(num_joints, std::numeric_limits<double>::infinity());
  effort_max_.resize(num_joints, 1.0);
  has_position_limits_.resize(num_joints, false);
  has_velocity_limits_.resize(num_joints, false);
  has_effort_limits_.resize(num_joints, false);

  last_raw_position_.resize(num_joints, 0);
  revolution_count_.resize(num_joints, 0);
  continuous_position_.resize(num_joints, 0.0);

  initial_position_offset_.resize(num_joints, 0.0);

  mock_position_.resize(num_joints, 0.0);
  mock_velocity_.resize(num_joints, 0.0);
  mock_load_.resize(num_joints, 0.0);
  mock_temperature_.resize(num_joints, 25.0);
  mock_voltage_.resize(num_joints, 12.0);
  mock_current_.resize(num_joints, 0.0);

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
        rclcpp::get_logger("STSHardwareInterface"),
        "Joint '%s': Missing required parameter 'motor_id'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate motor ID range
    if (motor_ids_[i] < STS_MIN_MOTOR_ID || motor_ids_[i] > STS_MAX_MOTOR_ID) {
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
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
        rclcpp::get_logger("STSHardwareInterface"),
        "Joint '%s': Invalid operating_mode: %d (must be 0, 1, or 2)",
        joint.name.c_str(), operating_modes_[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Parse optional joint limits
    if (joint.parameters.count("min_position")) {
      position_min_[i] = std::stod(joint.parameters.at("min_position"));
      has_position_limits_[i] = true;
    }
    if (joint.parameters.count("max_position")) {
      position_max_[i] = std::stod(joint.parameters.at("max_position"));
      has_position_limits_[i] = true;
    }
    if (joint.parameters.count("max_velocity")) {
      velocity_max_[i] = std::stod(joint.parameters.at("max_velocity"));
      has_velocity_limits_[i] = true;
    }
    if (joint.parameters.count("max_effort")) {
      effort_max_[i] = std::stod(joint.parameters.at("max_effort"));
      has_effort_limits_[i] = true;
    }

    // Parse optional reverse_direction parameter
    if (joint.parameters.count("reverse_direction")) {
      reverse_direction_[i] = (joint.parameters.at("reverse_direction") == "true");
    } else {
      reverse_direction_[i] = false;
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
          rclcpp::get_logger("STSHardwareInterface"),
          "Joint '%s': Missing required command interface '%s' for operating mode %d",
          joint.name.c_str(), required_interface.c_str(), operating_modes_[i]);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Joint '%s': motor_id=%d, mode=%d, limits: pos[%.2f, %.2f] vel[%.2f] eff[%.2f]",
      joint.name.c_str(), motor_ids_[i], operating_modes_[i],
      position_min_[i], position_max_[i], velocity_max_[i], effort_max_[i]);
  }

  // Check for duplicate motor IDs
  for (size_t i = 0; i < num_joints; ++i) {
    for (size_t j = i + 1; j < num_joints; ++j) {
      if (motor_ids_[i] == motor_ids_[j]) {
        RCLCPP_ERROR(
          rclcpp::get_logger("STSHardwareInterface"),
          "Duplicate motor_id %d found in joints '%s' and '%s'",
          motor_ids_[i], joint_names_[i].c_str(), joint_names_[j].c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // Initialize performance monitoring
  consecutive_read_errors_ = 0;
  consecutive_write_errors_ = 0;
  cycle_count_ = 0;
  read_duration_ms_ = 0.0;
  write_duration_ms_ = 0.0;
  max_read_duration_ms_ = 0.0;
  max_write_duration_ms_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Initialization complete: %zu joints, port=%s, baud=%d, sync_write=%s, mock=%s",
    num_joints, serial_port_.c_str(), baud_rate_,
    use_sync_write_ ? "true" : "false",
    enable_mock_mode_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// LIFECYCLE: on_configure
// ============================================================================

hardware_interface::CallbackReturn STSHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Configuring STS hardware interface...");

  // Skip serial port initialization in mock mode
  if (enable_mock_mode_) {
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Mock mode enabled - skipping serial port initialization");

    // Create diagnostic updater
    diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this->get_node());

    diagnostic_updater_->setHardwareID(info_.name);
    diagnostic_updater_->add(
      "Motor Status", this, &STSHardwareInterface::diagnostics_callback);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Thread-safe serial port management
  std::lock_guard<std::mutex> lock(serial_port_mutex_);

  // Check if serial port is already open
  auto it = serial_port_connections_.find(serial_port_);
  if (it != serial_port_connections_.end()) {
    // Reuse existing connection
    servo_ = it->second;
    owns_serial_connection_ = false;
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Reusing existing serial connection to %s", serial_port_.c_str());
  } else {
    // Create new serial connection
    servo_ = std::make_shared<SMS_STS>();

    if (!servo_->begin(baud_rate_, serial_port_.c_str())) {
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
        "Failed to open serial port %s at baud rate %d",
        serial_port_.c_str(), baud_rate_);
      return hardware_interface::CallbackReturn::ERROR;
    }

    serial_port_connections_[serial_port_] = servo_;
    owns_serial_connection_ = true;

    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Opened serial port %s at baud rate %d",
      serial_port_.c_str(), baud_rate_);
  }

  // Verify communication with all motors
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int ping_result = servo_->Ping(motor_ids_[i]);
    if (ping_result == -1) {
      int servo_error = servo_->getErr();
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
        "Failed to ping motor %d (joint '%s') - servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(), servo_error);
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Motor %d (joint '%s') responded to ping",
      motor_ids_[i], joint_names_[i].c_str());
  }

  // Create diagnostic updater
  diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this->get_node());

  diagnostic_updater_->setHardwareID(info_.name);
  diagnostic_updater_->add(
    "Motor Chain Status", this, &STSHardwareInterface::diagnostics_callback);

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Configuration complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// TO BE CONTINUED - This file is getting very long. Let me save this and continue in the next section.

// ============================================================================
// LIFECYCLE: on_activate
// ============================================================================

hardware_interface::CallbackReturn STSHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Activating STS hardware interface...");

  // Skip hardware activation in mock mode
  if (enable_mock_mode_) {
    // In mock mode, initialize position offset to 0 (mock motors start at 0)
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      initial_position_offset_[i] = 0.0;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Mock mode: Motors initialized (simulated)");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Initialize all motors
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int result = servo_->InitMotor(motor_ids_[i], operating_modes_[i], 1);  // 1 = enable torque
    if (result != 1) {
      int servo_error = servo_->getErr();
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
        "Failed to initialize motor %d (joint '%s') in mode %d - servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i], servo_error);
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Motor %d (joint '%s') initialized in mode %d with torque enabled",
      motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i]);
  }

  // Read initial state from all motors and capture position offset
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int result = servo_->FeedBack(motor_ids_[i]);
    if (result == 1) {
      int raw_position = servo_->ReadPos(-1);
      double raw_position_rad = raw_position_to_radians(raw_position);
      last_raw_position_[i] = raw_position;

      // Capture initial position as zero reference
      // This allows position to start at 0.0 regardless of motor's absolute encoder position
      initial_position_offset_[i] = raw_position_rad;
      hw_state_position_[i] = 0.0;

      RCLCPP_INFO(
        rclcpp::get_logger("STSHardwareInterface"),
        "Motor %d (joint '%s') initial raw position: %.3f rad (zeroed to 0.0)",
        motor_ids_[i], joint_names_[i].c_str(), raw_position_rad);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "All motors activated and ready");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// INTERFACE EXPORT: export_state_interfaces
// ============================================================================

std::vector<hardware_interface::StateInterface>
STSHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string & joint_name = joint_names_[i];

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_POSITION, &hw_state_position_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, "load", &hw_state_load_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, "voltage", &hw_state_voltage_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, "temperature", &hw_state_temperature_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, "current", &hw_state_current_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, "is_moving", &hw_state_is_moving_[i]));
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Exported %zu state interfaces for %zu joints",
    state_interfaces.size(), joint_names_.size());

  return state_interfaces;
}

// ============================================================================
// INTERFACE EXPORT: export_command_interfaces
// ============================================================================

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

    // Per-joint emergency stop (available in all modes)
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_name, "emergency_stop", &hw_cmd_emergency_stop_[i]));
  }

  // Broadcast emergency stop (stops ALL motors at once)
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.name, "broadcast_emergency_stop", &hw_cmd_broadcast_emergency_stop_));

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Exported %zu command interfaces for %zu joints (per-joint modes)",
    command_interfaces.size(), joint_names_.size());

  return command_interfaces;
}

// ============================================================================
// CONTROL LOOP: read (with multi-motor support)
// ============================================================================

hardware_interface::return_type STSHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto start_time = std::chrono::steady_clock::now();

  // Mock mode: simulate hardware behavior for all motors
  if (enable_mock_mode_) {
    double dt = period.seconds();

    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      if (operating_modes_[i] == MODE_SERVO) {
        // Simulate position control with first-order response
        double position_error = hw_cmd_position_[i] - mock_position_[i];
        double max_step = hw_cmd_velocity_[i] * dt;
        if (std::abs(position_error) > max_step && max_step > 0) {
          mock_position_[i] += (position_error > 0 ? max_step : -max_step);
        } else {
          mock_position_[i] = hw_cmd_position_[i];
        }
        mock_velocity_[i] = (mock_position_[i] - hw_state_position_[i]) / (dt > 0 ? dt : 0.001);
      } else if (operating_modes_[i] == MODE_VELOCITY) {
        // Simulate velocity control
        mock_velocity_[i] = hw_cmd_velocity_[i];
        mock_position_[i] += mock_velocity_[i] * dt;
      } else {  // MODE_PWM
        // Simulate PWM as torque control
        mock_velocity_[i] = hw_cmd_effort_[i] * 10.0;
        mock_position_[i] += mock_velocity_[i] * dt;
      }

      // Simulate load based on velocity
      mock_load_[i] = mock_velocity_[i] / (STS_MAX_VELOCITY_STEPS * STEPS_TO_RAD) * 100.0;
      mock_load_[i] = std::clamp(mock_load_[i], -100.0, 100.0);

      // Simulate temperature increase with load
      double temp_increase = std::abs(mock_load_[i]) * 0.01 * dt;
      mock_temperature_[i] += temp_increase;
      mock_temperature_[i] -= (mock_temperature_[i] - 25.0) * 0.1 * dt;
      mock_temperature_[i] = std::clamp(mock_temperature_[i], 20.0, 80.0);

      // Simulate current based on load
      mock_current_[i] = std::abs(mock_load_[i]) * 0.01;

      // Update state interfaces
      hw_state_position_[i] = mock_position_[i];
      hw_state_velocity_[i] = mock_velocity_[i];
      hw_state_load_[i] = mock_load_[i];
      hw_state_temperature_[i] = mock_temperature_[i];
      hw_state_voltage_[i] = mock_voltage_[i];
      hw_state_current_[i] = mock_current_[i];
      hw_state_is_moving_[i] = (std::abs(mock_velocity_[i]) > 0.01) ? 1.0 : 0.0;

      // Apply direction reversal to feedback if configured
      // This ensures controller sees consistent joint-space motion
      if (reverse_direction_[i]) {
        hw_state_position_[i] = -hw_state_position_[i];
        hw_state_velocity_[i] = -hw_state_velocity_[i];
      }
    }

    // Update diagnostics
    if (diagnostic_updater_) {
      diagnostic_updater_->force_update();
    }

    auto end_time = std::chrono::steady_clock::now();
    read_duration_ms_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    max_read_duration_ms_ = std::max(max_read_duration_ms_, read_duration_ms_);
    cycle_count_++;

    return hardware_interface::return_type::OK;
  }

  // Real hardware mode: read feedback from all motors
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int result = servo_->FeedBack(motor_ids_[i]);
    if (result != 1) {
      consecutive_read_errors_++;
      int servo_error = servo_->getErr();
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("STSHardwareInterface"),
        *rclcpp::Clock::make_shared(), 1000,
        "Failed to read feedback from motor %d (joint '%s') - error count: %d/%d, servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(),
        consecutive_read_errors_, MAX_CONSECUTIVE_ERRORS, servo_error);

      if (consecutive_read_errors_ >= MAX_CONSECUTIVE_ERRORS) {
        RCLCPP_ERROR(
          rclcpp::get_logger("STSHardwareInterface"),
          "Too many consecutive read errors (%d), attempting recovery...",
          consecutive_read_errors_);

        if (attempt_error_recovery()) {
          RCLCPP_INFO(
            rclcpp::get_logger("STSHardwareInterface"),
            "Error recovery successful");
          consecutive_read_errors_ = 0;
        } else {
          RCLCPP_ERROR(
            rclcpp::get_logger("STSHardwareInterface"),
            "Error recovery failed");
          return hardware_interface::return_type::ERROR;
        }
      }
      return hardware_interface::return_type::ERROR;
    }

    // Reset error counter on successful read
    consecutive_read_errors_ = 0;

    // Read all state data using cached values (ID = -1)
    int raw_position = servo_->ReadPos(-1);
    int raw_velocity = servo_->ReadSpeed(-1);
    int raw_load = servo_->ReadLoad(-1);
    int raw_voltage = servo_->ReadVoltage(-1);
    int raw_temperature = servo_->ReadTemper(-1);
    int raw_current = servo_->ReadCurrent(-1);
    int raw_is_moving = servo_->ReadMove(-1);

    // Multi-turn position tracking
    if (enable_multi_turn_) {
      int position_diff = raw_position - last_raw_position_[i];

      if (position_diff > STS_MAX_POSITION / 2) {
        revolution_count_[i]--;
      } else if (position_diff < -STS_MAX_POSITION / 2) {
        revolution_count_[i]++;
      }

      last_raw_position_[i] = raw_position;
      continuous_position_[i] = (revolution_count_[i] * 2.0 * M_PI) +
                                 raw_position_to_radians(raw_position);
      hw_state_position_[i] = continuous_position_[i];
    } else {
      hw_state_position_[i] = raw_position_to_radians(raw_position);
    }

    // Apply initial position offset (zeroing)
    // This makes the first reading become the reference zero point
    hw_state_position_[i] -= initial_position_offset_[i];

    // Convert other state data to SI units
    hw_state_velocity_[i] = raw_velocity_to_rad_s(raw_velocity);
    hw_state_load_[i] = raw_load_to_percentage(raw_load);
    hw_state_voltage_[i] = raw_voltage_to_volts(raw_voltage);
    hw_state_temperature_[i] = static_cast<double>(raw_temperature);
    hw_state_current_[i] = raw_current_to_amperes(raw_current);
    hw_state_is_moving_[i] = (raw_is_moving > 0) ? 1.0 : 0.0;

    // Apply direction reversal to feedback if configured
    // This ensures controller sees consistent joint-space motion
    if (reverse_direction_[i]) {
      hw_state_position_[i] = -hw_state_position_[i];
      hw_state_velocity_[i] = -hw_state_velocity_[i];
    }
  }

  // Performance monitoring
  auto end_time = std::chrono::steady_clock::now();
  read_duration_ms_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  max_read_duration_ms_ = std::max(max_read_duration_ms_, read_duration_ms_);
  last_read_time_ = end_time;

  // Update diagnostics
  if (diagnostic_updater_) {
    diagnostic_updater_->force_update();
  }

  // Log performance periodically
  cycle_count_++;
  if (cycle_count_ % PERFORMANCE_LOG_INTERVAL == 0) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("STSHardwareInterface"),
      "Performance: read=%.2fms (max=%.2fms), write=%.2fms (max=%.2fms)",
      read_duration_ms_, max_read_duration_ms_,
      write_duration_ms_, max_write_duration_ms_);
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// CONTROL LOOP: write (with SyncWrite support for motor chains)
// ============================================================================

hardware_interface::return_type STSHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto start_time = std::chrono::steady_clock::now();

  // Mock mode: skip hardware writes
  if (enable_mock_mode_) {
    // Handle broadcast emergency stop in mock mode
    if (hw_cmd_broadcast_emergency_stop_ > 0.5 && !broadcast_emergency_stop_active_) {
      for (size_t i = 0; i < motor_ids_.size(); ++i) {
        mock_velocity_[i] = 0.0;
      }
      broadcast_emergency_stop_active_ = true;
      RCLCPP_WARN(
        rclcpp::get_logger("STSHardwareInterface"),
        "Broadcast emergency stop activated for ALL motors");
    } else if (hw_cmd_broadcast_emergency_stop_ <= 0.5 && broadcast_emergency_stop_active_) {
      broadcast_emergency_stop_active_ = false;
      RCLCPP_INFO(
        rclcpp::get_logger("STSHardwareInterface"),
        "Broadcast emergency stop released");
    }

    // Handle per-joint emergency stop in mock mode
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      if (hw_cmd_emergency_stop_[i] > 0.5) {
        mock_velocity_[i] = 0.0;
        emergency_stop_active_[i] = true;
      } else if (emergency_stop_active_[i]) {
        emergency_stop_active_[i] = false;
      }
    }

    auto end_time = std::chrono::steady_clock::now();
    write_duration_ms_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    max_write_duration_ms_ = std::max(max_write_duration_ms_, write_duration_ms_);

    return hardware_interface::return_type::OK;
  }

  // ===== BROADCAST EMERGENCY STOP (Real Hardware) =====
  // Check for broadcast emergency stop (stops ALL motors)
  if (hw_cmd_broadcast_emergency_stop_ > 0.5 && !broadcast_emergency_stop_active_) {
    RCLCPP_WARN(
      rclcpp::get_logger("STSHardwareInterface"),
      "Broadcast emergency stop activated - stopping ALL motors");

    // Use broadcast ID to stop all motors at once
    servo_->WriteSpe(STS_BROADCAST_ID, 0, STS_MAX_ACCELERATION);
    broadcast_emergency_stop_active_ = true;

    return hardware_interface::return_type::OK;
  }

  if (hw_cmd_broadcast_emergency_stop_ <= 0.5 && broadcast_emergency_stop_active_) {
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Broadcast emergency stop released");
    broadcast_emergency_stop_active_ = false;
  }

  // Skip all commands if broadcast emergency stop is active
  if (broadcast_emergency_stop_active_) {
    return hardware_interface::return_type::OK;
  }

  // ===== PER-JOINT EMERGENCY STOP =====
  // Check for emergency stop on any motor
  bool any_emergency_stop = false;
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    bool emergency_stop_requested = (hw_cmd_emergency_stop_[i] > 0.5);

    if (emergency_stop_requested && !emergency_stop_active_[i]) {
      // Emergency stop activated for this motor
      RCLCPP_WARN(
        rclcpp::get_logger("STSHardwareInterface"),
        "Emergency stop activated for motor %d (joint '%s', mode %d)",
        motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i]);

      // Stop motor immediately based on its operating mode
      switch (operating_modes_[i]) {
        case MODE_SERVO: {
          int current_pos = servo_->ReadPos(motor_ids_[i]);
          if (current_pos != -1) {
            servo_->WritePosEx(motor_ids_[i], current_pos, 0, 0);
          }
          break;
        }
        case MODE_VELOCITY:
          servo_->WriteSpe(motor_ids_[i], 0, STS_MAX_ACCELERATION);
          break;
        case MODE_PWM:
          servo_->WritePwm(motor_ids_[i], 0);
          break;
      }

      emergency_stop_active_[i] = true;
      any_emergency_stop = true;
    }

    if (!emergency_stop_requested && emergency_stop_active_[i]) {
      // Emergency stop released
      RCLCPP_INFO(
        rclcpp::get_logger("STSHardwareInterface"),
        "Emergency stop released for motor %d (joint '%s')",
        motor_ids_[i], joint_names_[i].c_str());
      emergency_stop_active_[i] = false;
    }

    if (emergency_stop_active_[i]) {
      any_emergency_stop = true;
    }
  }

  // Skip normal commands if any emergency stop is active
  if (any_emergency_stop) {
    return hardware_interface::return_type::OK;
  }

  // ===== MODE-SPECIFIC WRITE LOGIC WITH MIXED-MODE SYNCWRITE SUPPORT =====
  // Group motors by operating mode for efficient SyncWrite

  std::vector<size_t> servo_motors;    // MODE_SERVO (position control)
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
        double target_position = hw_cmd_position_[idx];
        if (has_position_limits_[idx]) {
          target_position = std::clamp(target_position, position_min_[idx], position_max_[idx]);
        }

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
        double target_position = hw_cmd_position_[idx];
        if (has_position_limits_[idx]) {
          target_position = std::clamp(target_position, position_min_[idx], position_max_[idx]);
        }

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
            rclcpp::get_logger("STSHardwareInterface"),
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
        double target_velocity = hw_cmd_velocity_[idx];
        if (has_velocity_limits_[idx]) {
          target_velocity = std::clamp(target_velocity, -velocity_max_[idx], velocity_max_[idx]);
        }

        // Apply direction reversal if configured
        if (reverse_direction_[idx]) {
          target_velocity = -target_velocity;
        }

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
        double target_velocity = hw_cmd_velocity_[idx];
        if (has_velocity_limits_[idx]) {
          target_velocity = std::clamp(target_velocity, -velocity_max_[idx], velocity_max_[idx]);
        }

        // Apply direction reversal if configured
        if (reverse_direction_[idx]) {
          target_velocity = -target_velocity;
        }

        int raw_velocity = rad_s_to_raw_velocity(target_velocity);
        int acceleration = static_cast<int>(std::clamp(
          hw_cmd_acceleration_[idx], 0.0, static_cast<double>(STS_MAX_ACCELERATION)));

        int result = servo_->WriteSpe(motor_ids_[idx], raw_velocity, acceleration);

        if (result != 1) {
          consecutive_write_errors_++;
          int servo_error = servo_->getErr();
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("STSHardwareInterface"),
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

      // No result to check for SyncWritePwm (void return)

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
            rclcpp::get_logger("STSHardwareInterface"),
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

  // Performance monitoring
  auto end_time = std::chrono::steady_clock::now();
  write_duration_ms_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  max_write_duration_ms_ = std::max(max_write_duration_ms_, write_duration_ms_);
  last_write_time_ = end_time;

  return hardware_interface::return_type::OK;
}

// ============================================================================
// LIFECYCLE: on_deactivate
// ============================================================================

hardware_interface::CallbackReturn STSHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Deactivating STS hardware interface...");

  // Skip hardware deactivation in mock mode
  if (enable_mock_mode_) {
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      mock_velocity_[i] = 0.0;
      mock_load_[i] = 0.0;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
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
        rclcpp::get_logger("STSHardwareInterface"),
        "Failed to stop motor %d (joint '%s') during deactivation (mode: %d)",
        motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i]);
    }

    // Disable torque
    result = servo_->EnableTorque(motor_ids_[i], 0);
    if (result != 1) {
      int servo_error = servo_->getErr();
      RCLCPP_WARN(
        rclcpp::get_logger("STSHardwareInterface"),
        "Failed to disable torque on motor %d (joint '%s') - servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(), servo_error);
    }

    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Motor %d (joint '%s') stopped and torque disabled",
      motor_ids_[i], joint_names_[i].c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// LIFECYCLE: on_cleanup
// ============================================================================

hardware_interface::CallbackReturn STSHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Cleaning up STS hardware interface...");

  // Close serial connection only if we own it (thread-safe)
  if (owns_serial_connection_ && servo_) {
    std::lock_guard<std::mutex> lock(serial_port_mutex_);
    servo_->end();
    serial_port_connections_.erase(serial_port_);
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Closed serial connection to %s", serial_port_.c_str());
  }

  servo_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// LIFECYCLE: on_shutdown
// ============================================================================

hardware_interface::CallbackReturn STSHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
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
        rclcpp::get_logger("STSHardwareInterface"),
        "Motor %d (joint '%s') shutdown complete",
        motor_ids_[i], joint_names_[i].c_str());
    }

    // Close serial connection if we own it
    if (owns_serial_connection_) {
      std::lock_guard<std::mutex> lock(serial_port_mutex_);
      servo_->end();
      serial_port_connections_.erase(serial_port_);
      RCLCPP_INFO(
        rclcpp::get_logger("STSHardwareInterface"),
        "Closed serial connection to %s during shutdown", serial_port_.c_str());
    }

    servo_.reset();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// LIFECYCLE: on_error
// ============================================================================

hardware_interface::CallbackReturn STSHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(
    rclcpp::get_logger("STSHardwareInterface"),
    "Error state detected, attempting emergency stop...");

  if (!servo_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("STSHardwareInterface"),
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
        rclcpp::get_logger("STSHardwareInterface"),
        "Failed to send emergency stop command to motor %d (joint '%s')",
        motor_ids_[i], joint_names_[i].c_str());
    }

    // Disable torque for safety
    result = servo_->EnableTorque(motor_ids_[i], 0);
    if (result != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
        "Failed to disable torque on motor %d (joint '%s') during error handling",
        motor_ids_[i], joint_names_[i].c_str());
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("STSHardwareInterface"),
        "Motor %d (joint '%s') emergency stopped and torque disabled",
        motor_ids_[i], joint_names_[i].c_str());
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// UNIT CONVERSION HELPER FUNCTIONS
// ============================================================================

double STSHardwareInterface::raw_position_to_radians(int raw_position) const
{
  return static_cast<double>(raw_position) * STEPS_TO_RAD;
}

double STSHardwareInterface::raw_velocity_to_rad_s(int raw_velocity) const
{
  return static_cast<double>(raw_velocity) * STEPS_TO_RAD;
}

int STSHardwareInterface::rad_s_to_raw_velocity(double velocity_rad_s) const
{
  double raw = velocity_rad_s * RAD_TO_STEPS;
  int clamped = static_cast<int>(std::clamp(
    raw,
    static_cast<double>(-STS_MAX_VELOCITY_STEPS),
    static_cast<double>(STS_MAX_VELOCITY_STEPS)));
  return clamped;
}

double STSHardwareInterface::raw_load_to_percentage(int raw_load) const
{
  return static_cast<double>(raw_load) * LOAD_SCALE;
}

double STSHardwareInterface::raw_voltage_to_volts(int raw_voltage) const
{
  return static_cast<double>(raw_voltage) * VOLTAGE_SCALE;
}

double STSHardwareInterface::raw_current_to_amperes(int raw_current) const
{
  return static_cast<double>(raw_current) * CURRENT_SCALE;
}

int STSHardwareInterface::radians_to_raw_position(double position_rad) const
{
  // Normalize to [0, 2π) range first
  double normalized = std::fmod(position_rad, 2.0 * M_PI);
  if (normalized < 0.0) {
    normalized += 2.0 * M_PI;
  }

  // Convert to raw position
  int raw = static_cast<int>(normalized * RAD_TO_STEPS);

  // Ensure within valid range
  return std::clamp(raw, 0, STS_MAX_POSITION);
}

int STSHardwareInterface::effort_to_raw_pwm(double effort) const
{
  // Effort is normalized [-1.0, 1.0] -> raw PWM [-1000, 1000]
  double clamped = std::clamp(effort, -1.0, 1.0);
  return static_cast<int>(clamped * STS_MAX_PWM);
}

// ============================================================================
// ERROR RECOVERY
// ============================================================================

bool STSHardwareInterface::attempt_error_recovery()
{
  RCLCPP_WARN(
    rclcpp::get_logger("STSHardwareInterface"),
    "Attempting error recovery for all motors...");

  if (!servo_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("STSHardwareInterface"),
      "Servo interface not available for recovery");
    return false;
  }

  // Try to ping all motors
  bool all_motors_responsive = true;
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int ping_result = servo_->Ping(motor_ids_[i]);
    if (ping_result == -1) {
      RCLCPP_WARN(
        rclcpp::get_logger("STSHardwareInterface"),
        "Motor %d (joint '%s') not responding to ping",
        motor_ids_[i], joint_names_[i].c_str());
      all_motors_responsive = false;
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("STSHardwareInterface"),
        "Motor %d (joint '%s') responded to ping",
        motor_ids_[i], joint_names_[i].c_str());
    }
  }

  if (all_motors_responsive) {
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "All motors responded to ping, communication restored");
    return true;
  }

  // If we own the connection, try to re-establish it
  if (owns_serial_connection_) {
    RCLCPP_WARN(
      rclcpp::get_logger("STSHardwareInterface"),
      "Some motors not responding, attempting to reinitialize serial connection to %s",
      serial_port_.c_str());

    std::lock_guard<std::mutex> lock(serial_port_mutex_);

    // Close and reopen serial port
    servo_->end();

    if (!servo_->begin(baud_rate_, serial_port_.c_str())) {
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
        "Failed to reopen serial port %s during recovery", serial_port_.c_str());
      return false;
    }

    // Verify all motors communication
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      int ping_result = servo_->Ping(motor_ids_[i]);
      if (ping_result == -1) {
        RCLCPP_ERROR(
          rclcpp::get_logger("STSHardwareInterface"),
          "Motor %d (joint '%s') still not responding after serial reinit",
          motor_ids_[i], joint_names_[i].c_str());
        return false;
      }

      // Re-activate motor
      int init_result = servo_->InitMotor(motor_ids_[i], operating_modes_[i], 1);
      if (init_result != 1) {
        RCLCPP_ERROR(
          rclcpp::get_logger("STSHardwareInterface"),
          "Failed to reinitialize motor %d (joint '%s') after recovery",
          motor_ids_[i], joint_names_[i].c_str());
        return false;
      }

      RCLCPP_INFO(
        rclcpp::get_logger("STSHardwareInterface"),
        "Motor %d (joint '%s') reinitialized successfully",
        motor_ids_[i], joint_names_[i].c_str());
    }

    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Successfully recovered communication with all motors");
    return true;
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("STSHardwareInterface"),
    "Recovery failed: motors not responding and we don't own the serial connection");
  return false;
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

void STSHardwareInterface::diagnostics_callback(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Determine overall status
  if (consecutive_read_errors_ > 0 || consecutive_write_errors_ > 0) {
    if (consecutive_read_errors_ >= MAX_CONSECUTIVE_ERRORS ||
        consecutive_write_errors_ >= MAX_CONSECUTIVE_ERRORS) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                   "Communication errors detected");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "Intermittent communication errors");
    }
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "All motors operating normally");
  }

  // Hardware configuration
  // Check if all motors have the same mode
  bool mixed_mode = false;
  if (operating_modes_.size() > 1) {
    for (size_t i = 1; i < operating_modes_.size(); ++i) {
      if (operating_modes_[i] != operating_modes_[0]) {
        mixed_mode = true;
        break;
      }
    }
  }

  if (mixed_mode) {
    stat.add("Operating Mode", "Mixed (per-motor modes)");
  } else if (!operating_modes_.empty()) {
    const char* mode_name = (operating_modes_[0] == MODE_SERVO) ? "Servo" :
                           (operating_modes_[0] == MODE_VELOCITY) ? "Velocity" : "PWM";
    stat.add("Operating Mode", mode_name);
  }
  stat.add("Serial Port", serial_port_);
  stat.add("Baud Rate", baud_rate_);
  stat.add("Number of Motors", static_cast<int>(motor_ids_.size()));
  stat.add("SyncWrite Enabled", use_sync_write_ ? "Yes" : "No");
  stat.add("Mock Mode", enable_mock_mode_ ? "Yes" : "No");
  stat.add("Multi-turn Tracking", enable_multi_turn_ ? "Yes" : "No");

  // Per-motor status (show first motor or summary)
  for (size_t i = 0; i < std::min(motor_ids_.size(), size_t(3)); ++i) {
    std::string prefix = "Motor " + std::to_string(motor_ids_[i]) +
                        " (" + joint_names_[i] + ")";

    const char* mode_name = (operating_modes_[i] == MODE_SERVO) ? "Servo" :
                           (operating_modes_[i] == MODE_VELOCITY) ? "Velocity" : "PWM";
    stat.add(prefix + " Mode", mode_name);
    stat.add(prefix + " Position (rad)", hw_state_position_[i]);
    stat.add(prefix + " Velocity (rad/s)", hw_state_velocity_[i]);
    stat.add(prefix + " Load (%)", hw_state_load_[i]);
    stat.add(prefix + " Voltage (V)", hw_state_voltage_[i]);
    stat.add(prefix + " Temperature (°C)", hw_state_temperature_[i]);
    stat.add(prefix + " Current (A)", hw_state_current_[i]);
    stat.add(prefix + " Is Moving", hw_state_is_moving_[i] > 0.5 ? "Yes" : "No");
  }

  if (motor_ids_.size() > 3) {
    stat.add("Note", "Showing first 3 motors only");
  }

  // Error counters
  stat.add("Read Errors", consecutive_read_errors_);
  stat.add("Write Errors", consecutive_write_errors_);

  // Performance metrics
  stat.add("Read Duration (ms)", read_duration_ms_);
  stat.add("Write Duration (ms)", write_duration_ms_);
  stat.add("Max Read Duration (ms)", max_read_duration_ms_);
  stat.add("Max Write Duration (ms)", max_write_duration_ms_);
  stat.add("Cycle Count", static_cast<int>(cycle_count_));
}

}  // namespace sts_hardware_interface

// ============================================================================
// PLUGIN EXPORT
// ============================================================================

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  sts_hardware_interface::STSHardwareInterface,
  hardware_interface::SystemInterface)

