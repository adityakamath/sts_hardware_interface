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

namespace {
/// Wraps an SDK integer return code into a Result.
/// SDK convention: 1 = success, 0 or negative = failure.
[[nodiscard]] Result check(int ret, std::string_view ctx) {
  if (ret != 1) {
    return std::string(ctx) + " failed (code " + std::to_string(ret) + ")";
  }
  return std::nullopt;
}
}  // namespace


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

  // Parse proportional acceleration parameters
  try {
    proportional_acc_max_ = std::stoi(
      info_.hardware_parameters.count("proportional_acc_max") ?
      info_.hardware_parameters.at("proportional_acc_max") : "100");
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger_, "Invalid proportional_acc_max value: '%s' (must be a valid integer)",
      info_.hardware_parameters.at("proportional_acc_max").c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (proportional_acc_max_ < 0 || proportional_acc_max_ > STS_MAX_ACCELERATION) {
    RCLCPP_ERROR(logger_, "Invalid proportional_acc_max: %d. Must be in [0, %d]",
      proportional_acc_max_, STS_MAX_ACCELERATION);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Steady-state cruise deadband for velocity mode proportional acceleration.
  // Below this max Δv (rad/s), ACC=0 is sent to all wheels (hardware-native slew).
  // Seldom needs tuning; the default 0.05 rad/s is safe for most drive geometries.
  try {
    proportional_acc_deadband_rad_s_ = std::stod(
      info_.hardware_parameters.count("proportional_acc_deadband") ?
      info_.hardware_parameters.at("proportional_acc_deadband") : "0.05");
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger_, "Invalid proportional_acc_deadband value: '%s' (must be a valid number)",
      info_.hardware_parameters.at("proportional_acc_deadband").c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (proportional_acc_deadband_rad_s_ < 0.0) {
    RCLCPP_ERROR(logger_, "Invalid proportional_acc_deadband: %.4f. Must be >= 0",
      proportional_acc_deadband_rad_s_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (proportional_acc_deadband_rad_s_ == 0.0) {
    RCLCPP_WARN(logger_, "proportional_acc_deadband is 0.0; division by zero will be avoided but proportional scaling will be disabled at zero delta.");
  }

  // Steady-state hold deadband for servo/position mode proportional velocity.
  // Below this max Δpos (rad), all joints revert to their commanded velocity.
  // ~0.01 rad ≈ 0.57° — too small a delta for synchronized arrival to be meaningful.
  // Seldom needs tuning; the default is safe for most arm/pan-tilt geometries.
  try {
    proportional_vel_deadband_rad_ = std::stod(
      info_.hardware_parameters.count("proportional_vel_deadband") ?
      info_.hardware_parameters.at("proportional_vel_deadband") : "0.01");
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger_, "Invalid proportional_vel_deadband value: '%s' (must be a valid number)",
      info_.hardware_parameters.at("proportional_vel_deadband").c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (proportional_vel_deadband_rad_ < 0.0) {
    RCLCPP_ERROR(logger_, "Invalid proportional_vel_deadband: %.4f. Must be >= 0",
      proportional_vel_deadband_rad_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (proportional_vel_deadband_rad_ == 0.0) {
    RCLCPP_WARN(logger_, "proportional_vel_deadband is 0.0; division by zero will be avoided but proportional scaling will be disabled at zero delta.");
  }

  // Parse proportional velocity parameter (servo/position mode path)
  try {
    proportional_vel_max_ = std::stoi(
      info_.hardware_parameters.count("proportional_vel_max") ?
      info_.hardware_parameters.at("proportional_vel_max") : "0");
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger_, "Invalid proportional_vel_max value: '%s' (must be a valid integer)",
      info_.hardware_parameters.at("proportional_vel_max").c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (proportional_vel_max_ < 0 || proportional_vel_max_ > max_velocity_steps_) {
    RCLCPP_ERROR(logger_, "Invalid proportional_vel_max: %d. Must be in [0, %d]",
      proportional_vel_max_, max_velocity_steps_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Motor model parameter: max_velocity=%d steps/s", max_velocity_steps_);
  RCLCPP_INFO(logger_, "Proportional ACC: max=%d, deadband=%.4f rad/s",
    proportional_acc_max_, proportional_acc_deadband_rad_s_);
  RCLCPP_INFO(logger_, "Proportional VEL: max=%d steps/s (%s), deadband=%.4f rad",
    proportional_vel_max_, proportional_vel_max_ > 0 ? "enabled" : "disabled",
    proportional_vel_deadband_rad_);

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
  position_center_.resize(num_joints, conversions::STS_DEFAULT_CENTER);  // 4095 = legacy default
  is_readonly_.resize(num_joints, false);
  p_coefficient_.resize(num_joints, std::nullopt);
  d_coefficient_.resize(num_joints, std::nullopt);
  i_coefficient_.resize(num_joints, std::nullopt);
  protection_current_.resize(num_joints, std::nullopt);
  overload_torque_.resize(num_joints, std::nullopt);
  return_delay_.resize(num_joints, std::nullopt);

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

    if (joint.parameters.count("position_center_steps")) {
      if (operating_modes_[i] != MODE_SERVO) {
        RCLCPP_WARN(logger_, "Joint '%s': position_center_steps is ignored in operating mode %d (only used in position mode %d)",
          joint.name.c_str(), operating_modes_[i], MODE_SERVO);
      } else {
        try {
          int center = std::stoi(joint.parameters.at("position_center_steps"));
          if (center < 0 || center > conversions::STS_MAX_POSITION) {
            RCLCPP_ERROR(logger_, "Joint '%s': position_center_steps %d out of range [0, 4095]",
              joint.name.c_str(), center);
            return hardware_interface::CallbackReturn::ERROR;
          }
          position_center_[i] = center;
        } catch (const std::exception &) {
          RCLCPP_ERROR(logger_, "Joint '%s': Invalid position_center_steps value: '%s'",
            joint.name.c_str(), joint.parameters.at("position_center_steps").c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
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

    // A joint with no command interfaces is readonly: torque disabled at activation,
    // excluded from all write loops, state still read every cycle.
    is_readonly_[i] = joint.command_interfaces.empty();

    // Validate command interfaces match operating mode (skipped for readonly joints).
    // For MODE_SERVO: only position is required. velocity (max speed for the move) and
    //   acceleration are optional — when omitted, speed 0 (= hardware max) and ACC 0
    //   (= hardware default) are used. proportional_vel_max drives velocity internally
    //   during SyncWrite when != 0.
    // For MODE_VELOCITY: only velocity is required. acceleration is optional — when omitted,
    //   proportional_acc_max drives ACC internally during SyncWrite, or ACC 0 otherwise.
    if (!is_readonly_[i]) {
      std::vector<std::string> required_cmd_interfaces;
      switch (operating_modes_[i]) {
        case MODE_SERVO:
          required_cmd_interfaces = {"position"};
          break;
        case MODE_VELOCITY:
          required_cmd_interfaces = {"velocity"};
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
    }

    RCLCPP_INFO(logger_, "Joint '%s': motor_id=%d, mode=%d, read-only=%s, limits: pos[%.2f, %.2f] vel[%.2f] eff[%.2f]",
      joint.name.c_str(), motor_ids_[i], operating_modes_[i],
      is_readonly_[i] ? "true" : "false",
      position_min_[i], position_max_[i], velocity_max_[i], effort_max_[i]);

    // Parse optional PID coefficients (0-255; absent = do not write, preserve EEPROM value).
    // Routing depends on operating mode:
    //   Mode 0 (position): p, d, i stored → written to EEPROM addrs 21, 22, 23
    //   Mode 1 (velocity): p, i stored   → written to EEPROM addrs 37, 39; d has no register, ignored
    //   Mode 2 (PWM):      no PID registers, all three ignored
    auto parse_pid_coef = [&](const char * param_name, std::vector<std::optional<int>> & target) {
      if (!joint.parameters.count(param_name)) return true;
      int val = 0;
      try {
        val = std::stoi(joint.parameters.at(param_name));
      } catch (const std::exception &) {
        RCLCPP_ERROR(logger_, "Joint '%s': Invalid %s value: '%s' (must be an integer 0-255)",
          joint.name.c_str(), param_name, joint.parameters.at(param_name).c_str());
        return false;
      }
      if (val < 0 || val > 255) {
        RCLCPP_ERROR(logger_, "Joint '%s': %s=%d out of range [0, 255]",
          joint.name.c_str(), param_name, val);
        return false;
      }
      target[i] = val;
      return true;
    };
    if (operating_modes_[i] == MODE_PWM) {
      for (const char * name : {"p_coefficient", "d_coefficient", "i_coefficient"}) {
        if (joint.parameters.count(name)) {
          RCLCPP_WARN(logger_, "Joint '%s': %s is ignored in Mode 2 (PWM has no PID)",
            joint.name.c_str(), name);
        }
      }
    } else {
      if (!parse_pid_coef("p_coefficient", p_coefficient_)) return hardware_interface::CallbackReturn::ERROR;
      if (!parse_pid_coef("i_coefficient", i_coefficient_)) return hardware_interface::CallbackReturn::ERROR;
      if (operating_modes_[i] == MODE_SERVO) {
        if (!parse_pid_coef("d_coefficient", d_coefficient_)) return hardware_interface::CallbackReturn::ERROR;
      } else {
        if (joint.parameters.count("d_coefficient")) {
          RCLCPP_WARN(logger_, "Joint '%s': d_coefficient is ignored in Mode 1 (velocity PI controller has no D term)",
            joint.name.c_str());
        }
      }
    }

    // Parse optional protection parameters (all modes; absent = do not write, preserve EEPROM value).
    // protection_current: uint16 raw value in 6.5mA units (e.g. 462 ≈ 3.0A); addr 28/29.
    // overload_torque: load percentage threshold for overload cutoff; addr 36.
    // return_delay: response delay in 2µs units; addr 7.
    auto parse_eeprom_param = [&](const char* param_name, std::vector<std::optional<int>>& target, int min_val, int max_val) {
      if (!joint.parameters.count(param_name)) return true;
      int val = 0;
      try {
        val = std::stoi(joint.parameters.at(param_name));
      } catch (const std::exception &) {
        RCLCPP_ERROR(logger_, "Joint '%s': Invalid %s value: '%s' (must be a valid integer)",
          joint.name.c_str(), param_name, joint.parameters.at(param_name).c_str());
        return false;
      }
      if (val < min_val || val > max_val) {
        RCLCPP_ERROR(logger_, "Joint '%s': %s=%d out of range [%d, %d]",
          joint.name.c_str(), param_name, val, min_val, max_val);
        return false;
      }
      target[i] = val;
      return true;
    };
    if (!parse_eeprom_param("protection_current", protection_current_, 0, 65535)) return hardware_interface::CallbackReturn::ERROR;
    if (!parse_eeprom_param("overload_torque",    overload_torque_,    0, 254))   return hardware_interface::CallbackReturn::ERROR;
    if (!parse_eeprom_param("return_delay",       return_delay_,       0, 254))   return hardware_interface::CallbackReturn::ERROR;
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

  // Pre-compute motor groupings by operating mode (static after init).
  // Readonly joints are excluded — they are never written in the control loop.
  for (size_t i = 0; i < num_joints; ++i) {
    if (is_readonly_[i]) continue;
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
  servo_sync_deltas_.resize(servo_motor_indices_.size());
  velocity_sync_velocities_.resize(velocity_motor_indices_.size());
  velocity_sync_accelerations_.resize(velocity_motor_indices_.size());
  velocity_sync_deltas_.resize(velocity_motor_indices_.size());
  pwm_sync_pwm_values_.resize(pwm_motor_indices_.size());

  RCLCPP_INFO(logger_, "Motor groupings: %zu servo, %zu velocity, %zu PWM",
    servo_motor_indices_.size(), velocity_motor_indices_.size(), pwm_motor_indices_.size());

  // Warn when proportional coordination parameters are configured but cannot take effect.
  // SyncWrite (and therefore proportional scaling) requires >1 joint in the same mode.
  if (proportional_vel_max_ > 0 && servo_motor_indices_.size() <= 1) {
    RCLCPP_WARN(logger_,
      "proportional_vel_max=%d is set but only %zu servo joint(s) configured. "
      "SyncWrite requires >1 joints in the same mode; proportional_vel_max has no effect.",
      proportional_vel_max_, servo_motor_indices_.size());
  }
  if (proportional_acc_max_ != 0 && velocity_motor_indices_.size() == 0) {
    // No velocity joints at all — proportional_acc_max is meaningless; silence it automatically
    // so the URDF does not need to explicitly set proportional_acc_max=0 for position-only configs.
    RCLCPP_DEBUG(logger_,
      "proportional_acc_max=%d has no effect (no velocity joints configured); auto-disabling.",
      proportional_acc_max_);
    proportional_acc_max_ = 0;
  } else if (proportional_acc_max_ != 0 && velocity_motor_indices_.size() == 1) {
    RCLCPP_WARN(logger_,
      "proportional_acc_max=%d is set but only %zu velocity joint(s) configured. "
      "SyncWrite requires >1 joints in the same mode; proportional_acc_max has no effect.",
      proportional_acc_max_, velocity_motor_indices_.size());
  }

  // Initialize error tracking
  consecutive_read_errors_ = 0;
  consecutive_write_errors_ = 0;

  size_t readonly_count = std::count(is_readonly_.begin(), is_readonly_.end(), true);
  RCLCPP_INFO(logger_, "Initialization complete: %zu joints (%zu read-only), port=%s, baud=%d, sync_write=%s, mock=%s",
    num_joints, readonly_count, serial_port_.c_str(), baud_rate_,
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

  // Enable service introspection: publishes full request/response content to
  // /emergency_stop/_service_event for monitoring activations and releases.
  //
  // /emergency_stop reflects ongoing safety state, not a one-shot action, so this uses
  // transient-local durability (depth 2, enough to cache the last request+response pair)
  // instead of the volatile system default: a late-joining subscriber - e.g. lekiwi_audio's
  // indicator_node, or bool_toggle_node syncing its own belief about current state - gets
  // the current e-stop state replayed on connect rather than waiting for the next toggle.
  // The liveliness lease lets a subscriber notice if this node dies without a clean exit.
  const auto safety_qos = rclcpp::QoS(2)
    .reliable()
    .transient_local()
    .liveliness(rclcpp::LivelinessPolicy::Automatic)
    .liveliness_lease_duration(rclcpp::Duration(std::chrono::seconds(1)));

  emergency_stop_service_->configure_introspection(
    node_->get_clock(),
    safety_qos,
    RCL_SERVICE_INTROSPECTION_CONTENTS);

  RCLCPP_INFO(logger_, "Created /emergency_stop service with introspection enabled");

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

  // Write per-joint PID coefficients to EEPROM (only for joints that have them set).
  // By on_init() time, d_coefficient is only populated for Mode 0, and no PID
  // coefficients are stored for Mode 2, so no mode checks are needed here.
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    bool has_p = p_coefficient_[i].has_value();
    bool has_d = d_coefficient_[i].has_value();
    bool has_i = i_coefficient_[i].has_value();
    if (!has_p && !has_d && !has_i) continue;

    int id = motor_ids_[i];
    if (auto r = check(servo_->unLockEeprom(id), "unLockEeprom"); r.has_value()) {
      RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): %s", id, joint_names_[i].c_str(), r->c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (has_p) {
      int addr = (operating_modes_[i] == MODE_SERVO) ? SMS_STS_MODE0_P_COEF : SMS_STS_MODE1_P_COEF;
      if (auto r = check(servo_->writeByte(id, addr, static_cast<u8>(p_coefficient_[i].value())), "writeByte P_COEF"); r.has_value()) {
        RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): %s", id, joint_names_[i].c_str(), r->c_str());
        servo_->LockEeprom(id);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (has_d) {  // only set for MODE_SERVO (guaranteed by on_init)
      if (auto r = check(servo_->writeByte(id, SMS_STS_MODE0_D_COEF, static_cast<u8>(d_coefficient_[i].value())), "writeByte D_COEF"); r.has_value()) {
        RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): %s", id, joint_names_[i].c_str(), r->c_str());
        servo_->LockEeprom(id);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (has_i) {
      int addr = (operating_modes_[i] == MODE_SERVO) ? SMS_STS_MODE0_I_COEF : SMS_STS_MODE1_I_COEF;
      if (auto r = check(servo_->writeByte(id, addr, static_cast<u8>(i_coefficient_[i].value())), "writeByte I_COEF"); r.has_value()) {
        RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): %s", id, joint_names_[i].c_str(), r->c_str());
        servo_->LockEeprom(id);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (auto r = check(servo_->LockEeprom(id), "LockEeprom"); r.has_value()) {
      RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): %s", id, joint_names_[i].c_str(), r->c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "Motor %d (joint '%s'): wrote PID coefficients P=%s D=%s I=%s",
      id, joint_names_[i].c_str(),
      has_p ? std::to_string(p_coefficient_[i].value()).c_str() : "(unchanged)",
      has_d ? std::to_string(d_coefficient_[i].value()).c_str() : "(unchanged)",
      has_i ? std::to_string(i_coefficient_[i].value()).c_str() : "(unchanged)");
  }

  // Write per-joint hardware protection parameters to EEPROM (only for joints that have them set).
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    bool has_pc = protection_current_[i].has_value();
    bool has_ot = overload_torque_[i].has_value();
    bool has_rd = return_delay_[i].has_value();
    if (!has_pc && !has_ot && !has_rd) continue;

    int id = motor_ids_[i];
    if (auto r = check(servo_->unLockEeprom(id), "unLockEeprom"); r.has_value()) {
      RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): %s", id, joint_names_[i].c_str(), r->c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (has_pc) {
      if (auto r = check(servo_->writeWord(id, SMS_STS_PROTECTION_CURRENT_L, static_cast<u16>(protection_current_[i].value())), "writeWord PROTECTION_CURRENT"); r.has_value()) {
        RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): %s", id, joint_names_[i].c_str(), r->c_str());
        servo_->LockEeprom(id);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (has_ot) {
      if (auto r = check(servo_->writeByte(id, SMS_STS_OVERLOAD_TORQUE, static_cast<u8>(overload_torque_[i].value())), "writeByte OVERLOAD_TORQUE"); r.has_value()) {
        RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): %s", id, joint_names_[i].c_str(), r->c_str());
        servo_->LockEeprom(id);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (has_rd) {
      if (auto r = check(servo_->writeByte(id, SMS_STS_RETURN_DELAY, static_cast<u8>(return_delay_[i].value())), "writeByte RETURN_DELAY"); r.has_value()) {
        RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): %s", id, joint_names_[i].c_str(), r->c_str());
        servo_->LockEeprom(id);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (auto r = check(servo_->LockEeprom(id), "LockEeprom"); r.has_value()) {
      RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): %s", id, joint_names_[i].c_str(), r->c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "Motor %d (joint '%s'): wrote protection params PC=%s OT=%s RD=%s",
      id, joint_names_[i].c_str(),
      has_pc ? std::to_string(protection_current_[i].value()).c_str() : "(unchanged)",
      has_ot ? std::to_string(overload_torque_[i].value()).c_str() : "(unchanged)",
      has_rd ? std::to_string(return_delay_[i].value()).c_str() : "(unchanged)");
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

  // Initialize all motors. Readonly joints get torque disabled and are never commanded.
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    u8 enable_torque = is_readonly_[i] ? 0 : 1;
    if (auto r = check(servo_->InitMotor(motor_ids_[i], operating_modes_[i], enable_torque), "InitMotor"); r.has_value()) {
      RCLCPP_ERROR(logger_, "Motor %d (joint '%s') in mode %d: %s",
        motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i], r->c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Motor %d (joint '%s') initialized in mode %d, torque %s",
      motor_ids_[i], joint_names_[i].c_str(), operating_modes_[i],
      is_readonly_[i] ? "disabled (read-only)" : "enabled");
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
    if (is_readonly_[i]) continue;  // readonly joint — no command interfaces exported
    const std::string & joint_name = joint_names_[i];

    // Mode-dependent command interfaces (per joint)
    switch (operating_modes_[i]) {
      case MODE_SERVO:  // Position control
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &hw_cmd_position_[i]));
        // velocity and acceleration are optional: export only if declared in the URDF.
        // When velocity is omitted, raw speed 0 (= hardware max speed) is used.
        // When acceleration is omitted, ACC 0 (= hardware default ramp) is used.
        // proportional_vel_max drives velocity internally in SyncWrite when != 0.
        for (const auto & cmd : info_.joints[i].command_interfaces) {
          if (cmd.name == "velocity") {
            command_interfaces.emplace_back(
              hardware_interface::CommandInterface(
                joint_name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocity_[i]));
          } else if (cmd.name == "acceleration") {
            command_interfaces.emplace_back(
              hardware_interface::CommandInterface(
                joint_name, "acceleration", &hw_cmd_acceleration_[i]));
          }
        }
        break;

      case MODE_VELOCITY:  // Velocity control
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocity_[i]));
        // acceleration is optional: export only if declared in the URDF.
        // When omitted, proportional_acc_max drives ACC internally.
        for (const auto & cmd : info_.joints[i].command_interfaces) {
          if (cmd.name == "acceleration") {
            command_interfaces.emplace_back(
              hardware_interface::CommandInterface(
                joint_name, "acceleration", &hw_cmd_acceleration_[i]));
            break;
          }
        }
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
    hw_state_position_[i] = conversions::raw_position_to_radians(raw_position, position_center_[i]);

    // Always clamp the reported position to URDF limits. A joint can drift past software limits
    // under gravity during estop, or be physically moved outside limits before startup. Clamping
    // here prevents JointSaturationLimiter from deactivating the controller; write() already
    // clamps commands, so the motor will be commanded to the nearest limit and return to range.
    if (has_position_limits_[i]) {
      hw_state_position_[i] = std::clamp(hw_state_position_[i], position_min_[i], position_max_[i]);
    }

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
      RCLCPP_INFO(logger_, "Emergency stop released - torque re-enabled (mock mode)");
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
    if (auto r = check(servo_->WriteSpe(STS_BROADCAST_ID, 0, STS_MAX_ACCELERATION), "broadcast stop"); r.has_value()) {
      RCLCPP_WARN(logger_, "Broadcast emergency stop: %s", r->c_str());
    }

    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      if (auto r = check(servo_->EnableTorque(motor_ids_[i], 0), "EnableTorque"); r.has_value()) {
        RCLCPP_WARN(logger_, "Motor %d (joint '%s'): disable torque during emergency stop: %s",
          motor_ids_[i], joint_names_[i].c_str(), r->c_str());
      }
    }

    emergency_stop_active_ = true;
    return hardware_interface::return_type::OK;
  }

  if (hw_cmd_emergency_stop_ <= 0.5 && emergency_stop_active_) {
    RCLCPP_INFO(logger_, "Emergency stop released - re-enabling torque");

    // Re-enable torque on commanded joints only; readonly joints keep torque disabled
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
      if (is_readonly_[i]) continue;
      if (auto r = check(servo_->EnableTorque(motor_ids_[i], 1), "EnableTorque"); r.has_value()) {
        RCLCPP_WARN(logger_, "Motor %d (joint '%s'): enable torque after emergency stop release: %s",
          motor_ids_[i], joint_names_[i].c_str(), r->c_str());
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
      // Pass 1: compute target positions, per-joint deltas, and max delta.
      // Delta = |target - current| in radians; used by proportional velocity scaling.
      double max_delta_rad = 0.0;
      for (size_t j = 0; j < servo_motor_indices_.size(); ++j) {
        size_t idx = servo_motor_indices_[j];
        double target_position = conversions::apply_limit(hw_cmd_position_[idx], position_min_[idx], position_max_[idx], has_position_limits_[idx]);
        servo_sync_positions_[j] = conversions::radians_to_raw_position(target_position, position_center_[idx]);
        servo_sync_deltas_[j] = std::abs(target_position - hw_state_position_[idx]);
        max_delta_rad = std::max(max_delta_rad, servo_sync_deltas_[j]);
        servo_sync_accelerations_[j] = static_cast<u8>(conversions::clamp_acceleration(hw_cmd_acceleration_[idx]));
      }

      // Pass 2: assign speed proportional to each joint's displacement so all joints
      // finish moving at the same time. The master joint (largest delta) gets
      // proportional_vel_max_; others are scaled down accordingly.
      // Below 0.01 rad max-delta (steady-state hold) or when disabled, use commanded speed.
      for (size_t j = 0; j < servo_motor_indices_.size(); ++j) {
        size_t idx = servo_motor_indices_[j];
        if (proportional_vel_max_ == 0 || max_delta_rad < proportional_vel_deadband_rad_ || max_delta_rad == 0.0) {
          // Proportional velocity disabled, below deadband, or all joints at target: use each joint's commanded max speed.
          double max_speed = std::max(0.0,
            conversions::apply_limit(hw_cmd_velocity_[idx], 0.0, velocity_max_[idx], has_velocity_limits_[idx]));
          servo_sync_speeds_[j] = static_cast<u16>(
            conversions::rad_s_to_raw_speed(max_speed, max_velocity_steps_));
        } else if (servo_sync_deltas_[j] == 0.0) {
          // This joint is already at its target: set speed to 0 (no move)
          servo_sync_speeds_[j] = 0;
        } else {
          // Proportional scaling, but respect per-joint velocity_max
          double scaled = (servo_sync_deltas_[j] / max_delta_rad) * proportional_vel_max_;
          double limited = std::min(scaled, velocity_max_[idx]);
          servo_sync_speeds_[j] = static_cast<u16>(std::clamp(static_cast<int>(std::round(limited)), 1, max_velocity_steps_));
        }
      }

      servo_->SyncWritePosEx(
        servo_sync_ids_.data(), servo_sync_ids_.size(),
        servo_sync_positions_.data(), servo_sync_speeds_.data(), servo_sync_accelerations_.data());

    } else {
      // Individual writes for single servo motor or when SyncWrite disabled
      for (size_t idx : servo_motor_indices_) {
        double target_position = conversions::apply_limit(hw_cmd_position_[idx], position_min_[idx], position_max_[idx], has_position_limits_[idx]);
        double max_speed = conversions::apply_limit(hw_cmd_velocity_[idx], 0.0, velocity_max_[idx], has_velocity_limits_[idx]);

        int raw_position = conversions::radians_to_raw_position(target_position, position_center_[idx]);
        // rad_s_to_raw_speed: unsigned magnitude conversion for position mode — no sign inversion.
        // hw_cmd_velocity_ == 0.0 (not declared / not written) → raw 0 = STS protocol max speed.
        int raw_max_speed = conversions::rad_s_to_raw_speed(
          std::max(0.0, max_speed), max_velocity_steps_);
        int acceleration = conversions::clamp_acceleration(hw_cmd_acceleration_[idx]);

        if (auto r = check_write(servo_->WritePosEx(motor_ids_[idx], raw_position, raw_max_speed, acceleration), idx, "position"); r.has_value()) {
          return hardware_interface::return_type::ERROR;
        }
      }
    }
  }

  // ===== WRITE COMMANDS FOR VELOCITY MODE MOTORS =====
  if (!velocity_motor_indices_.empty()) {
    if (use_sync_write_ && velocity_motor_indices_.size() > 1) {
      // Pass 1: compute clamped target velocities, per-wheel deltas, and the maximum delta.
      // Delta = |target - current| in rad/s, matching exactly what the servo will ramp through.
      // ACC units: 1 ACC unit = 100 steps/s². Ramp time T = delta_steps / (ACC * 100).
      // Since steps/s = rad/s * (4096 / 2π), the rad/s ratio equals the steps/s ratio, so
      // proportional scaling in rad/s is equivalent to scaling in steps/s.
      double max_delta_rad_s = 0.0;
      for (size_t j = 0; j < velocity_motor_indices_.size(); ++j) {
        size_t idx = velocity_motor_indices_[j];
        double target_velocity = conversions::apply_limit(hw_cmd_velocity_[idx], -velocity_max_[idx], velocity_max_[idx], has_velocity_limits_[idx]);
        velocity_sync_velocities_[j] = conversions::rad_s_to_raw_velocity(target_velocity, max_velocity_steps_);
        velocity_sync_deltas_[j] = std::abs(target_velocity - hw_state_velocity_[idx]);
        max_delta_rad_s = std::max(max_delta_rad_s, velocity_sync_deltas_[j]);
      }

      // Pass 2: assign ACC for each wheel.
      // Priority:
      //   proportional_acc_max != 0 → proportional scaling wins; controller-set ACC is ignored.
      //     Below the deadband (steady-state cruise), ACC=0 lets the hardware follow naturally.
      //   proportional_acc_max == 0 → use per-joint hw_cmd_acceleration_ set by the controller
      //     (or ACC=0 if the acceleration command interface is not declared / not written).
      for (size_t j = 0; j < velocity_motor_indices_.size(); ++j) {
        size_t idx = velocity_motor_indices_[j];
        if (proportional_acc_max_ == 0 || max_delta_rad_s == 0.0) {
          // Proportional ACC disabled or all wheels at target: pass through controller-commanded ACC per joint.
          velocity_sync_accelerations_[j] = static_cast<u8>(
            conversions::clamp_acceleration(hw_cmd_acceleration_[idx]));
        } else if (max_delta_rad_s < proportional_acc_deadband_rad_s_) {
          // Steady-state cruise: send ACC=0 so hardware follows naturally.
          velocity_sync_accelerations_[j] = 0;
        } else if (velocity_sync_deltas_[j] == 0.0) {
          // This wheel is already at its target: set ACC to 0
          velocity_sync_accelerations_[j] = 0;
        } else {
          int acc = static_cast<int>(std::round((velocity_sync_deltas_[j] / max_delta_rad_s) * proportional_acc_max_));
          velocity_sync_accelerations_[j] = static_cast<u8>(std::clamp(acc, 1, STS_MAX_ACCELERATION));
        }
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

        if (auto r = check_write(servo_->WriteSpe(motor_ids_[idx], raw_velocity, acceleration), idx, "velocity"); r.has_value()) {
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
        if (auto r = check_write(servo_->WritePwm(motor_ids_[idx], raw_pwm), idx, "PWM"); r.has_value()) {
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
    if (auto r = check(stop_motor(i, 0), "stop_motor"); r.has_value()) {
      RCLCPP_WARN(logger_, "Motor %d (joint '%s'): stop during deactivation: %s",
        motor_ids_[i], joint_names_[i].c_str(), r->c_str());
    }

    // Disable torque
    if (auto r = check(servo_->EnableTorque(motor_ids_[i], 0), "EnableTorque"); r.has_value()) {
      RCLCPP_WARN(logger_, "Motor %d (joint '%s'): disable torque during deactivation: %s",
        motor_ids_[i], joint_names_[i].c_str(), r->c_str());
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
      if (auto r = check(servo_->EnableTorque(motor_ids_[i], 0), "EnableTorque"); r.has_value()) {
        RCLCPP_WARN(logger_, "Motor %d (joint '%s'): disable torque during cleanup: %s",
          motor_ids_[i], joint_names_[i].c_str(), r->c_str());
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
      if (auto r = check(stop_motor(i, 0), "stop_motor"); r.has_value()) {
        RCLCPP_WARN(logger_, "Motor %d (joint '%s'): stop during shutdown: %s",
          motor_ids_[i], joint_names_[i].c_str(), r->c_str());
      }
      if (auto r = check(servo_->EnableTorque(motor_ids_[i], 0), "EnableTorque"); r.has_value()) {
        RCLCPP_WARN(logger_, "Motor %d (joint '%s'): disable torque during shutdown: %s",
          motor_ids_[i], joint_names_[i].c_str(), r->c_str());
      }
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
    if (auto r = check(stop_motor(i, 254), "stop_motor"); r.has_value()) {
      RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): emergency stop command: %s",
        motor_ids_[i], joint_names_[i].c_str(), r->c_str());
    }

    // Disable torque for safety
    if (auto r = check(servo_->EnableTorque(motor_ids_[i], 0), "EnableTorque"); r.has_value()) {
      RCLCPP_ERROR(logger_, "Motor %d (joint '%s'): disable torque during error: %s",
        motor_ids_[i], joint_names_[i].c_str(), r->c_str());
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

/** @brief Check a write SDK return code; logs, tracks recovery, returns error on failure */
Result STSHardwareInterface::check_write(int result, size_t idx, const char* operation)
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
    return std::string(operation) + " failed on motor " + std::to_string(motor_ids_[idx]);
  }
  return std::nullopt;
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

    u8 enable_torque = is_readonly_[i] ? 0 : 1;
    if (auto r = check(servo_->InitMotor(motor_ids_[i], operating_modes_[i], enable_torque), "InitMotor"); r.has_value()) {
      RCLCPP_ERROR(logger_, "Failed to reinitialize motor %d (joint '%s') after recovery: %s",
        motor_ids_[i], joint_names_[i].c_str(), r->c_str());
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
