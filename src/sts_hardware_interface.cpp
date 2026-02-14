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
  const hardware_interface::HardwareInfo & hardware_info)
{
  // Store hardware info
  info_ = hardware_info;

  // Initialize logger
  logger_ = rclcpp::get_logger("STSHardwareInterface");

  RCLCPP_INFO(logger_, "Initializing STS Hardware Interface: %s", info_.name.c_str());

  // ===== Parse hardware-level parameters =====
  try {
    serial_port_ = info_.hardware_parameters.at("serial_port");
  } catch (const std::out_of_range &) {
    RCLCPP_ERROR(logger_, "Missing required parameter: serial_port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse and validate baud_rate (default: 1000000)
  // Supported baud rates: 9600, 19200, 38400, 57600, 115200, 500000, 1000000
  static const std::vector<int> valid_baud_rates = {9600, 19200, 38400, 57600, 115200, 500000, 1000000};
  try {
    baud_rate_ = std::stoi(
      info_.hardware_parameters.count("baud_rate") ?
      info_.hardware_parameters.at("baud_rate") : "1000000");
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger_, "Invalid baud_rate value: '%s' (must be a valid integer)",
      info_.hardware_parameters.at("baud_rate").c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (std::find(valid_baud_rates.begin(), valid_baud_rates.end(), baud_rate_) == valid_baud_rates.end()) {
    RCLCPP_ERROR(logger_, "Invalid baud_rate: %d. Supported rates: 9600, 19200, 38400, 57600, 115200, 500000, 1000000", baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse and validate communication_timeout_ms (default: 100, range: 1-1000)
  try {
    communication_timeout_ms_ = std::stoi(
      info_.hardware_parameters.count("communication_timeout_ms") ?
      info_.hardware_parameters.at("communication_timeout_ms") : "100");
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger_, "Invalid communication_timeout_ms value: '%s' (must be a valid integer)",
      info_.hardware_parameters.at("communication_timeout_ms").c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (communication_timeout_ms_ < 1 || communication_timeout_ms_ > 1000) {
    RCLCPP_ERROR(logger_, "Invalid communication_timeout_ms: %d. Must be between 1 and 1000 ms", communication_timeout_ms_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse boolean parameters
  enable_mock_mode_ = parse_bool_param("enable_mock_mode", false);
  use_sync_write_ = parse_bool_param("use_sync_write", true);
  reset_states_on_activate_ = parse_bool_param("reset_states_on_activate", true);

  // Parse motor-specific parameter (model-dependent)
  try {
    max_velocity_steps_ = std::stoi(
      info_.hardware_parameters.count("max_velocity_steps") ?
      info_.hardware_parameters.at("max_velocity_steps") : "3400");  // STS3215 default
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger_, "Invalid max_velocity_steps value: '%s' (must be a valid integer)",
      info_.hardware_parameters.at("max_velocity_steps").c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (max_velocity_steps_ <= 0) {
    RCLCPP_ERROR(logger_, "Invalid max_velocity_steps: %d. Must be a positive integer", max_velocity_steps_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Motor model parameter: max_velocity=%d steps/s", max_velocity_steps_);

  // ===== Parse joint-level parameters =====
  size_t num_joints = info_.joints.size();

  if (num_joints == 0) {
    RCLCPP_ERROR(logger_, "No joints defined in hardware interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring %zu joint(s)", num_joints);

  // Resize all per-joint vectors
  joint_names_.resize(num_joints);
  motor_ids_.resize(num_joints);
  operating_modes_.resize(num_joints, MODE_VELOCITY);  // Default to velocity mode

  hw_state_position_.resize(num_joints, 0.0);
  hw_state_velocity_.resize(num_joints, 0.0);
  hw_state_effort_.resize(num_joints, 0.0);
  hw_state_voltage_.resize(num_joints, 0.0);
  hw_state_temperature_.resize(num_joints, 0.0);
  hw_state_current_.resize(num_joints, 0.0);
  hw_state_is_moving_.resize(num_joints, 0.0);

  hw_cmd_position_.resize(num_joints, 0.0);
  hw_cmd_velocity_.resize(num_joints, 0.0);
  hw_cmd_acceleration_.resize(num_joints, 0.0);
  hw_cmd_effort_.resize(num_joints, 0.0);

  // Initialize broadcast emergency stop
  hw_cmd_emergency_stop_ = 0.0;
  emergency_stop_active_ = false;

  position_min_.resize(num_joints, 0.0);  // Default: 0 radians (0 steps)
  position_max_.resize(num_joints, 2.0 * M_PI);  // Default: 2π radians (4096 steps)
  velocity_max_.resize(num_joints, static_cast<double>(max_velocity_steps_) * STEPS_TO_RAD);  // Use configured max velocity
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
    if (!joint.parameters.count("motor_id")) {
      RCLCPP_ERROR(logger_, "Joint '%s': Missing required parameter 'motor_id'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    try {
      motor_ids_[i] = std::stoi(joint.parameters.at("motor_id"));
    } catch (const std::exception &) {
      RCLCPP_ERROR(logger_, "Joint '%s': Invalid motor_id value: '%s' (must be a valid integer)",
        joint.name.c_str(), joint.parameters.at("motor_id").c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate motor ID range
    if (motor_ids_[i] < STS_MIN_MOTOR_ID || motor_ids_[i] > STS_MAX_MOTOR_ID) {
      RCLCPP_ERROR(logger_, "Joint '%s': Invalid motor_id %d (must be between %d and %d)",
        joint.name.c_str(), motor_ids_[i], STS_MIN_MOTOR_ID, STS_MAX_MOTOR_ID);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Parse operating_mode (per joint)
    if (joint.parameters.count("operating_mode")) {
      try {
        operating_modes_[i] = std::stoi(joint.parameters.at("operating_mode"));
      } catch (const std::exception &) {
        RCLCPP_ERROR(logger_, "Joint '%s': Invalid operating_mode value: '%s' (must be 0, 1, or 2)",
          joint.name.c_str(), joint.parameters.at("operating_mode").c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else {
      // Default to velocity mode if not specified
      operating_modes_[i] = MODE_VELOCITY;
    }

    // Validate operating mode
    if (operating_modes_[i] < MODE_SERVO || operating_modes_[i] > MODE_PWM) {
      RCLCPP_ERROR(logger_, "Joint '%s': Invalid operating_mode: %d (must be 0, 1, or 2)",
        joint.name.c_str(), operating_modes_[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Parse and validate optional joint limits
    if (joint.parameters.count("min_position")) {
      try {
        position_min_[i] = std::stod(joint.parameters.at("min_position"));
      } catch (const std::exception &) {
        RCLCPP_ERROR(logger_, "Joint '%s': Invalid min_position value: '%s' (must be a valid number)",
          joint.name.c_str(), joint.parameters.at("min_position").c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      has_position_limits_[i] = true;
    }
    if (joint.parameters.count("max_position")) {
      try {
        position_max_[i] = std::stod(joint.parameters.at("max_position"));
      } catch (const std::exception &) {
        RCLCPP_ERROR(logger_, "Joint '%s': Invalid max_position value: '%s' (must be a valid number)",
          joint.name.c_str(), joint.parameters.at("max_position").c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      has_position_limits_[i] = true;
    }
    if (has_position_limits_[i] && position_min_[i] >= position_max_[i]) {
      RCLCPP_ERROR(logger_, "Joint '%s': min_position (%.3f) must be less than max_position (%.3f)",
        joint.name.c_str(), position_min_[i], position_max_[i]);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.parameters.count("max_velocity")) {
      try {
        velocity_max_[i] = std::stod(joint.parameters.at("max_velocity"));
      } catch (const std::exception &) {
        RCLCPP_ERROR(logger_, "Joint '%s': Invalid max_velocity value: '%s' (must be a valid number)",
          joint.name.c_str(), joint.parameters.at("max_velocity").c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (velocity_max_[i] <= 0.0) {
        RCLCPP_ERROR(logger_, "Joint '%s': max_velocity must be greater than 0 (got %.3f)",
          joint.name.c_str(), velocity_max_[i]);
        return hardware_interface::CallbackReturn::ERROR;
      }
      has_velocity_limits_[i] = true;
    }

    if (joint.parameters.count("max_effort")) {
      try {
        effort_max_[i] = std::stod(joint.parameters.at("max_effort"));
      } catch (const std::exception &) {
        RCLCPP_ERROR(logger_, "Joint '%s': Invalid max_effort value: '%s' (must be a valid number)",
          joint.name.c_str(), joint.parameters.at("max_effort").c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (effort_max_[i] <= 0.0 || effort_max_[i] > 1.0) {
        RCLCPP_ERROR(logger_, "Joint '%s': max_effort must be in range (0.0, 1.0] (got %.3f)",
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

    RCLCPP_INFO(logger_, "Joint '%s': motor_id=%d, mode=%d, limits: pos[%.2f, %.2f] vel[%.2f] eff[%.2f]",
      joint.name.c_str(), motor_ids_[i], operating_modes_[i],
      position_min_[i], position_max_[i], velocity_max_[i], effort_max_[i]);
  }

  // Check for duplicate motor IDs
  for (size_t i = 0; i < num_joints; ++i) {
    for (size_t j = i + 1; j < num_joints; ++j) {
      if (motor_ids_[i] == motor_ids_[j]) {
        RCLCPP_ERROR(logger_, "Duplicate motor_id %d found in joints '%s' and '%s'",
          motor_ids_[i], joint_names_[i].c_str(), joint_names_[j].c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // Pre-compute motor groupings by operating mode (static after init)
  for (size_t i = 0; i < num_joints; ++i) {
    switch (operating_modes_[i]) {
      case MODE_SERVO:
        servo_motor_indices_.push_back(i);
        servo_sync_ids_.push_back(static_cast<u8>(motor_ids_[i]));
        break;
      case MODE_VELOCITY:
        velocity_motor_indices_.push_back(i);
        velocity_sync_ids_.push_back(static_cast<u8>(motor_ids_[i]));
        break;
      case MODE_PWM:
        pwm_motor_indices_.push_back(i);
        pwm_sync_ids_.push_back(static_cast<u8>(motor_ids_[i]));
        break;
    }
  }
  // Pre-allocate SyncWrite data buffers (IDs are fixed, data overwritten each cycle)
  servo_sync_positions_.resize(servo_motor_indices_.size());
  servo_sync_speeds_.resize(servo_motor_indices_.size());
  servo_sync_accelerations_.resize(servo_motor_indices_.size());
  velocity_sync_velocities_.resize(velocity_motor_indices_.size());
  velocity_sync_accelerations_.resize(velocity_motor_indices_.size());
  pwm_sync_pwm_values_.resize(pwm_motor_indices_.size());

  RCLCPP_INFO(logger_, "Motor groupings: %zu servo, %zu velocity, %zu PWM",
    servo_motor_indices_.size(), velocity_motor_indices_.size(), pwm_motor_indices_.size());

  // Initialize error tracking
  consecutive_read_errors_ = 0;
  consecutive_write_errors_ = 0;

  RCLCPP_INFO(logger_, "Initialization complete: %zu joints, port=%s, baud=%d, sync_write=%s, mock=%s",
    num_joints, serial_port_.c_str(), baud_rate_,
    use_sync_write_ ? "true" : "false",
    enable_mock_mode_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Initialize serial communication and verify motors */
hardware_interface::CallbackReturn STSHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring STS hardware interface...");

  // Create ROS 2 node for emergency stop service (both mock and real hardware)
  if (!node_) {
    node_ = std::make_shared<rclcpp::Node>("sts_hardware_interface_node");
    RCLCPP_INFO(logger_, "Created ROS 2 node for emergency stop service");
  }

  // Create emergency stop service server (both mock and real hardware)
  emergency_stop_service_ = node_->create_service<std_srvs::srv::SetBool>(
    "/emergency_stop",
    std::bind(&STSHardwareInterface::emergency_stop_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(logger_, "Created /emergency_stop service");

  // Skip serial port initialization in mock mode
  if (enable_mock_mode_) {
    RCLCPP_INFO(logger_, "Mock mode enabled - skipping serial port initialization");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Create serial connection
  servo_ = std::make_shared<SMS_STS>();

  if (!servo_->begin(baud_rate_, serial_port_.c_str())) {
    RCLCPP_ERROR(logger_, "Failed to open serial port %s at baud rate %d",
      serial_port_.c_str(), baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set communication timeout
  servo_->IOTimeOut = communication_timeout_ms_;

  RCLCPP_INFO(logger_, "Opened serial port %s at baud rate %d with %d ms timeout",
    serial_port_.c_str(), baud_rate_, communication_timeout_ms_);

  // Verify communication with all motors
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int ping_result = servo_->Ping(motor_ids_[i]);
    if (ping_result == -1) {
      int servo_error = servo_->getErr();
      RCLCPP_ERROR(logger_, "Failed to ping motor %d (joint '%s') - servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(), servo_error);
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "Motor %d (joint '%s') responded to ping",
      motor_ids_[i], joint_names_[i].c_str());
  }

  RCLCPP_INFO(logger_, "Configuration complete");

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

    // Zero position and velocity for all joints (if enabled)
    if (reset_states_on_activate_) {
      for (size_t i = 0; i < motor_ids_.size(); ++i) {
        hw_state_position_[i] = 0.0;
        hw_state_velocity_[i] = 0.0;
      }
      RCLCPP_INFO(logger_, "Mock mode: Odometry initialized to zero (position and velocity)");
    } else {
      RCLCPP_INFO(logger_, "Mock mode: State reset disabled - preserving existing odometry");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Initialize all motors
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int result = servo_->InitMotor(motor_ids_[i], operating_modes_[i], 1);  // 1 = enable torque
    if (result != 1) {
      int servo_error = servo_->getErr();
      RCLCPP_ERROR(logger_, "Failed to initialize motor %d (joint '%s') in mode %d - servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i], servo_error);
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Motor %d (joint '%s') initialized in mode %d with torque enabled",
      motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i]);
  }

  // Zero position and velocity for all joints to initialize odometry (if enabled)
  if (reset_states_on_activate_) {
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      hw_state_position_[i] = 0.0;
      hw_state_velocity_[i] = 0.0;
    }
    RCLCPP_INFO(logger_, "All motors activated and ready - odometry initialized to zero (position and velocity)");
  } else {
    RCLCPP_INFO(logger_, "All motors activated and ready - state reset disabled, preserving existing odometry");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Export state interfaces for all joints */
std::vector<hardware_interface::StateInterface>
STSHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string & joint_name = joint_names_[i];

    // Export all state interfaces
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_POSITION, &hw_state_position_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_EFFORT, &hw_state_effort_[i]));

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

  RCLCPP_INFO(logger_, "Exported %zu command interfaces for %zu joints (per-joint modes)",
    command_interfaces.size(), joint_names_.size());

  return command_interfaces;
}

/** @brief Read motor states from hardware */
hardware_interface::return_type STSHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Process ROS 2 callbacks (e.g., emergency stop subscriber)
  if (node_) {
    rclcpp::spin_some(node_);
  }

  // Mock mode: simulate hardware behavior using state variables directly
  if (enable_mock_mode_) {
    double dt = period.seconds();

    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      double prev_position = hw_state_position_[i];

      switch (operating_modes_[i]) {
        case MODE_SERVO: {
          // Simulate position control with first-order response
          double position_error = hw_cmd_position_[i] - hw_state_position_[i];
          double max_step = hw_cmd_velocity_[i] * dt;
          if (std::abs(position_error) > max_step && max_step > 0) {
            hw_state_position_[i] += (position_error > 0 ? max_step : -max_step);
          } else {
            hw_state_position_[i] = hw_cmd_position_[i];
          }
          hw_state_velocity_[i] = (hw_state_position_[i] - prev_position) / (dt > 0 ? dt : 0.001);
          break;
        }
        case MODE_VELOCITY:
          // Simulate velocity control
          hw_state_velocity_[i] = hw_cmd_velocity_[i];
          hw_state_position_[i] += hw_state_velocity_[i] * dt;
          break;
        case MODE_PWM:
          // Simulate PWM as torque control
          hw_state_velocity_[i] = hw_cmd_effort_[i] * 10.0;
          hw_state_position_[i] += hw_state_velocity_[i] * dt;
          break;
      }

      // Simulate load based on velocity (convert simulated load percentage to effort units)
      double simulated_load_percentage = hw_state_velocity_[i] / (max_velocity_steps_ * STEPS_TO_RAD) * 100.0;
      simulated_load_percentage = std::clamp(simulated_load_percentage, -100.0, 100.0);
      // Motor load is absolute - doesn't depend on max_effort limit
      hw_state_effort_[i] = simulated_load_percentage / 100.0;

      // Simulate voltage (STS3215 nominal: 12V, range: 10-14V under load)
      double voltage_drop = std::abs(hw_state_effort_[i]) * 0.5;  // Up to 0.5V drop at max load
      hw_state_voltage_[i] = 12.0 - voltage_drop;

      // Simulate temperature (ambient ~25°C + heating from load/velocity)
      double thermal_load = std::abs(hw_state_velocity_[i]) * 2.0 + std::abs(hw_state_effort_[i]) * 5.0;
      hw_state_temperature_[i] = 25.0 + thermal_load;

      // Simulate current (proportional to effort: ~0-1000mA at full load)
      hw_state_current_[i] = std::abs(hw_state_effort_[i]) * 1.0;  // ~1A at max effort

      // Update is_moving state
      hw_state_is_moving_[i] = (std::abs(hw_state_velocity_[i]) > 0.01) ? 1.0 : 0.0;
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
        throttle_clock_, 1000,
        "Failed to read feedback from motor %d (joint '%s') - error count: %d/%d, servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(),
        consecutive_read_errors_, MAX_CONSECUTIVE_ERRORS, servo_error);

      if (consecutive_read_errors_ >= MAX_CONSECUTIVE_ERRORS) {
        RCLCPP_ERROR(
          logger_,
          "Too many consecutive read errors (%d) - last failure on motor %d (joint '%s'), attempting recovery...",
          consecutive_read_errors_, motor_ids_[i], joint_names_[i].c_str());

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

    // Read all state interfaces
    int raw_position = servo_->ReadPos(-1);
    hw_state_position_[i] = conversions::raw_position_to_radians(raw_position);

    int raw_velocity = servo_->ReadSpeed(-1);
    hw_state_velocity_[i] = conversions::raw_velocity_to_rad_s(raw_velocity);

    int raw_load = servo_->ReadLoad(-1);
    // Convert load percentage to effort: (-100% to 100%) -> (-1.0 to 1.0)
    // Motor load is absolute - doesn't depend on max_effort limit
    double load_percentage = raw_load * LOAD_SCALE;  // 0.1 units per percent
    hw_state_effort_[i] = load_percentage / 100.0;

    int raw_voltage = servo_->ReadVoltage(-1);
    hw_state_voltage_[i] = static_cast<double>(raw_voltage) * VOLTAGE_SCALE;

    int raw_temperature = servo_->ReadTemper(-1);
    hw_state_temperature_[i] = static_cast<double>(raw_temperature);

    int raw_current = servo_->ReadCurrent(-1);
    hw_state_current_[i] = static_cast<double>(raw_current) * CURRENT_SCALE;

    int raw_is_moving = servo_->ReadMove(-1);
    hw_state_is_moving_[i] = (raw_is_moving > 0) ? 1.0 : 0.0;
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
      emergency_stop_active_ = true;
      RCLCPP_WARN(logger_, "Emergency stop activated - ALL motors stopped, torque disabled (mock mode)");
    } else if (hw_cmd_emergency_stop_ <= 0.5 && emergency_stop_active_) {
      emergency_stop_active_ = false;
      RCLCPP_INFO(logger_, "Emergency stop released - torque enabled (mock mode)");
    }

    // Continuously clear all commands while emergency stop is active
    if (emergency_stop_active_) {
      for (size_t i = 0; i < motor_ids_.size(); ++i) {
        hw_cmd_velocity_[i] = 0.0;
        hw_cmd_position_[i] = hw_state_position_[i];  // Hold current position
        hw_cmd_effort_[i] = 0.0;
        hw_cmd_acceleration_[i] = 0.0;
      }
    }

    return hardware_interface::return_type::OK;
  }

  // Check for broadcast emergency stop (stops ALL motors)
  if (hw_cmd_emergency_stop_ > 0.5 && !emergency_stop_active_) {
    RCLCPP_WARN(logger_, "Emergency stop activated - stopping ALL motors and disabling torque");
    servo_->WriteSpe(STS_BROADCAST_ID, 0, STS_MAX_ACCELERATION);

    // Disable torque on all motors (makes them freely movable by hand)
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      int result = servo_->EnableTorque(motor_ids_[i], 0);
      if (result != 1) {
        RCLCPP_WARN(logger_, "Failed to disable torque on motor %d (joint '%s') during emergency stop",
          motor_ids_[i], joint_names_[i].c_str());
      }
    }

    emergency_stop_active_ = true;
    return hardware_interface::return_type::OK;
  }

  if (hw_cmd_emergency_stop_ <= 0.5 && emergency_stop_active_) {
    RCLCPP_INFO(logger_, "Emergency stop released - re-enabling torque");

    // Re-enable torque on all motors
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      int result = servo_->EnableTorque(motor_ids_[i], 1);
      if (result != 1) {
        RCLCPP_WARN(logger_, "Failed to enable torque on motor %d (joint '%s') after emergency stop release",
          motor_ids_[i], joint_names_[i].c_str());
      }
    }

    emergency_stop_active_ = false;
  }

  // Skip all commands if emergency stop is active
  if (emergency_stop_active_) {
    return hardware_interface::return_type::OK;
  }

  // ===== WRITE COMMANDS FOR SERVO MODE MOTORS =====
  if (!servo_motor_indices_.empty()) {
    if (use_sync_write_ && servo_motor_indices_.size() > 1) {
      // Update pre-allocated SyncWrite buffers
      for (size_t j = 0; j < servo_motor_indices_.size(); ++j) {
        size_t idx = servo_motor_indices_[j];
        double target_position = conversions::apply_limit(hw_cmd_position_[idx], position_min_[idx], position_max_[idx], has_position_limits_[idx]);
        double max_speed = conversions::apply_limit(hw_cmd_velocity_[idx], 0.0, velocity_max_[idx], has_velocity_limits_[idx]);

        servo_sync_positions_[j] = conversions::radians_to_raw_position(target_position);
        servo_sync_speeds_[j] = static_cast<u16>(std::clamp(
          conversions::rad_s_to_raw_velocity(max_speed, max_velocity_steps_), 0, max_velocity_steps_));
        servo_sync_accelerations_[j] = static_cast<u8>(conversions::clamp_acceleration(hw_cmd_acceleration_[idx]));
      }

      servo_->SyncWritePosEx(
        servo_sync_ids_.data(), servo_sync_ids_.size(),
        servo_sync_positions_.data(), servo_sync_speeds_.data(), servo_sync_accelerations_.data());

    } else {
      // Individual writes for single servo motor or when SyncWrite disabled
      for (size_t idx : servo_motor_indices_) {
        double target_position = conversions::apply_limit(hw_cmd_position_[idx], position_min_[idx], position_max_[idx], has_position_limits_[idx]);
        double max_speed = conversions::apply_limit(hw_cmd_velocity_[idx], 0.0, velocity_max_[idx], has_velocity_limits_[idx]);

        int raw_position = conversions::radians_to_raw_position(target_position);
        int raw_max_speed = std::clamp(
          conversions::rad_s_to_raw_velocity(max_speed, max_velocity_steps_), 0, max_velocity_steps_);
        int acceleration = conversions::clamp_acceleration(hw_cmd_acceleration_[idx]);

        int result = servo_->WritePosEx(motor_ids_[idx], raw_position, raw_max_speed, acceleration);

        if (handle_write_error(result, idx, "position")) {
          return hardware_interface::return_type::ERROR;
        }
      }
    }
  }

  // ===== WRITE COMMANDS FOR VELOCITY MODE MOTORS =====
  if (!velocity_motor_indices_.empty()) {
    if (use_sync_write_ && velocity_motor_indices_.size() > 1) {
      // Update pre-allocated SyncWrite buffers
      for (size_t j = 0; j < velocity_motor_indices_.size(); ++j) {
        size_t idx = velocity_motor_indices_[j];
        double target_velocity = conversions::apply_limit(hw_cmd_velocity_[idx], -velocity_max_[idx], velocity_max_[idx], has_velocity_limits_[idx]);
        velocity_sync_velocities_[j] = conversions::rad_s_to_raw_velocity(target_velocity, max_velocity_steps_);
        velocity_sync_accelerations_[j] = static_cast<u8>(conversions::clamp_acceleration(hw_cmd_acceleration_[idx]));
      }

      servo_->SyncWriteSpe(
        velocity_sync_ids_.data(), velocity_sync_ids_.size(),
        velocity_sync_velocities_.data(), velocity_sync_accelerations_.data());

    } else {
      // Individual writes for single velocity motor or when SyncWrite disabled
      for (size_t idx : velocity_motor_indices_) {
        double target_velocity = conversions::apply_limit(hw_cmd_velocity_[idx], -velocity_max_[idx], velocity_max_[idx], has_velocity_limits_[idx]);
        int raw_velocity = conversions::rad_s_to_raw_velocity(target_velocity, max_velocity_steps_);
        int acceleration = conversions::clamp_acceleration(hw_cmd_acceleration_[idx]);

        int result = servo_->WriteSpe(motor_ids_[idx], raw_velocity, acceleration);

        if (handle_write_error(result, idx, "velocity")) {
          return hardware_interface::return_type::ERROR;
        }
      }
    }
  }

  // ===== WRITE COMMANDS FOR PWM MODE MOTORS =====
  if (!pwm_motor_indices_.empty()) {
    if (use_sync_write_ && pwm_motor_indices_.size() > 1) {
      // Update pre-allocated SyncWrite buffers
      for (size_t j = 0; j < pwm_motor_indices_.size(); ++j) {
        size_t idx = pwm_motor_indices_[j];
        pwm_sync_pwm_values_[j] = conversions::effort_to_raw_pwm(
          conversions::normalize_effort(hw_cmd_effort_[idx], effort_max_[idx], has_effort_limits_[idx]));
      }

      servo_->SyncWritePwm(pwm_sync_ids_.data(), pwm_sync_ids_.size(), pwm_sync_pwm_values_.data());

    } else {
      // Individual writes for single PWM motor or when SyncWrite disabled
      for (size_t idx : pwm_motor_indices_) {
        int raw_pwm = conversions::effort_to_raw_pwm(
          conversions::normalize_effort(hw_cmd_effort_[idx], effort_max_[idx], has_effort_limits_[idx]));
        int result = servo_->WritePwm(motor_ids_[idx], raw_pwm);

        if (handle_write_error(result, idx, "PWM")) {
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
      hw_state_effort_[i] = 0.0;
    }
    RCLCPP_INFO(
      logger_,
      "Mock mode: All motors stopped and torque disabled (simulated)");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Stop all motors based on current operating mode
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int result = stop_motor(i, 0);
    if (result != 1) {
      RCLCPP_WARN(logger_, "Failed to stop motor %d (joint '%s') during deactivation (mode: %d)",
        motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i]);
    }

    // Disable torque
    result = servo_->EnableTorque(motor_ids_[i], 0);
    if (result != 1) {
      int servo_error = servo_->getErr();
      RCLCPP_WARN(logger_, "Failed to disable torque on motor %d (joint '%s') - servo error: %d",
        motor_ids_[i], joint_names_[i].c_str(), servo_error);
    }

    RCLCPP_INFO(logger_, "Motor %d (joint '%s') stopped and torque disabled",
      motor_ids_[i], joint_names_[i].c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Close serial connection and release resources */
hardware_interface::CallbackReturn STSHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up STS hardware interface...");

  // Disable torque on all motors before closing connection (skip in mock mode)
  if (servo_ && !enable_mock_mode_) {
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      int result = servo_->EnableTorque(motor_ids_[i], 0);
      if (result != 1) {
        RCLCPP_WARN(logger_, "Failed to disable torque on motor %d (joint '%s') during cleanup",
          motor_ids_[i], joint_names_[i].c_str());
      }
    }
    RCLCPP_INFO(logger_, "All motor torques disabled");
  }

  // Close serial connection
  if (servo_) {
    servo_->end();
    RCLCPP_INFO(logger_, "Closed serial connection to %s", serial_port_.c_str());
  }

  servo_.reset();

  // Cleanup ROS 2 node and service
  emergency_stop_service_.reset();
  node_.reset();
  RCLCPP_INFO(logger_, "Released ROS 2 node and service resources");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Emergency shutdown - stop all motors and close connection */
hardware_interface::CallbackReturn STSHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Shutting down STS hardware interface...");

  // Skip hardware shutdown in mock mode
  if (enable_mock_mode_) {
    RCLCPP_INFO(logger_, "Mock mode: Shutdown complete (simulated)");
  } else if (servo_) {
    // Real hardware: stop all motors and disable torque
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      stop_motor(i, 0);
      servo_->EnableTorque(motor_ids_[i], 0);
      RCLCPP_INFO(logger_, "Motor %d (joint '%s') shutdown complete - torque disabled",
        motor_ids_[i], joint_names_[i].c_str());
    }

    // Close serial connection
    servo_->end();
    RCLCPP_INFO(logger_, "Closed serial connection to %s during shutdown", serial_port_.c_str());

    servo_.reset();
  }

  // Cleanup ROS 2 node and service
  emergency_stop_service_.reset();
  node_.reset();
  RCLCPP_INFO(logger_, "Released ROS 2 node and service resources");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Handle error state - emergency stop all motors */
hardware_interface::CallbackReturn STSHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(logger_, "Error state detected, attempting emergency stop...");

  // Skip hardware error handling in mock mode
  if (enable_mock_mode_) {
    RCLCPP_ERROR(logger_, "Mock mode: Emergency stop activated (simulated)");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (!servo_) {
    RCLCPP_ERROR(logger_, "Servo interface not available for emergency stop");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Emergency stop: set all motors to safe state and disable torque
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    int result = stop_motor(i, 254);  // Max deceleration for velocity mode
    if (result != 1) {
      RCLCPP_ERROR(logger_, "Failed to send emergency stop command to motor %d (joint '%s')",
        motor_ids_[i], joint_names_[i].c_str());
    }

    // Disable torque for safety
    result = servo_->EnableTorque(motor_ids_[i], 0);
    if (result != 1) {
      RCLCPP_ERROR(logger_, "Failed to disable torque on motor %d (joint '%s') during error handling",
        motor_ids_[i], joint_names_[i].c_str());
    } else {
      RCLCPP_INFO(logger_, "Motor %d (joint '%s') emergency stopped and torque disabled",
        motor_ids_[i], joint_names_[i].c_str());
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief Parse a boolean hardware parameter with default value */
bool STSHardwareInterface::parse_bool_param(const std::string& key, bool default_value) const
{
  auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end()) return default_value;
  return it->second == "true";
}

/** @brief Handle write operation errors with logging and recovery */
bool STSHardwareInterface::handle_write_error(int result, size_t idx, const char* operation)
{
  if (result != 1) {
    consecutive_write_errors_++;
    int servo_error = servo_->getErr();
    RCLCPP_WARN_THROTTLE(
      logger_,
      throttle_clock_, 1000,
      "Failed to write %s to motor %d (joint '%s') - error count: %d/%d, servo error: %d",
      operation, motor_ids_[idx], joint_names_[idx].c_str(),
      consecutive_write_errors_, MAX_CONSECUTIVE_ERRORS, servo_error);

    if (consecutive_write_errors_ >= MAX_CONSECUTIVE_ERRORS) {
      RCLCPP_ERROR(logger_,
        "Too many consecutive write errors (%d) - last failure on motor %d (joint '%s'), attempting recovery...",
        consecutive_write_errors_, motor_ids_[idx], joint_names_[idx].c_str());
      if (attempt_error_recovery()) {
        consecutive_write_errors_ = 0;
      }
    }
    return true;  // Error occurred
  }
  return false;  // No error
}

/** @brief Stop a motor based on its operating mode */
int STSHardwareInterface::stop_motor(size_t idx, int acceleration)
{
  switch (operating_modes_[idx]) {
    case MODE_SERVO: {
      int current_pos = servo_->ReadPos(motor_ids_[idx]);
      if (current_pos != -1) {
        return servo_->WritePosEx(motor_ids_[idx], current_pos, 0, acceleration);
      }
      return -1;
    }
    case MODE_VELOCITY:
      return servo_->WriteSpe(motor_ids_[idx], 0, acceleration);
    case MODE_PWM:
      return servo_->WritePwm(motor_ids_[idx], 0);
    default:
      return -1;
  }
}

/** @brief Attempt to recover from communication errors by pinging motors */
bool STSHardwareInterface::attempt_error_recovery()
{
  RCLCPP_WARN(logger_, "Attempting error recovery - reinitializing serial connection to %s", serial_port_.c_str());

  if (!servo_) {
    RCLCPP_ERROR(logger_, "Servo interface not available for recovery");
    return false;
  }

  // Close and reopen serial port
  servo_->end();
  if (!servo_->begin(baud_rate_, serial_port_.c_str())) {
    RCLCPP_ERROR(logger_, "Failed to reopen serial port %s during recovery", serial_port_.c_str());
    return false;
  }

  // Restore communication timeout
  servo_->IOTimeOut = communication_timeout_ms_;

  // Verify and reinitialize all motors
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    if (servo_->Ping(motor_ids_[i]) == -1) {
      RCLCPP_ERROR(logger_, "Motor %d (joint '%s') not responding after serial reinit",
        motor_ids_[i], joint_names_[i].c_str());
      return false;
    }

    if (servo_->InitMotor(motor_ids_[i], operating_modes_[i], 1) != 1) {
      RCLCPP_ERROR(logger_, "Failed to reinitialize motor %d (joint '%s') after recovery",
        motor_ids_[i], joint_names_[i].c_str());
      return false;
    }
  }

  RCLCPP_INFO(logger_, "Successfully recovered communication with all motors");
  return true;
}

/** @brief Emergency stop service callback */
void STSHardwareInterface::emergency_stop_callback(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  hw_cmd_emergency_stop_ = req->data ? 1.0 : 0.0;

  if (req->data) {
    RCLCPP_WARN(logger_, "Emergency stop ACTIVATE received via /emergency_stop service");
    res->message = "Emergency stop activated";
  } else {
    RCLCPP_INFO(logger_, "Emergency stop RELEASE received via /emergency_stop service");
    res->message = "Emergency stop released";
  }
  res->success = true;
}

}  // namespace sts_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  sts_hardware_interface::STSHardwareInterface,
  hardware_interface::SystemInterface)

