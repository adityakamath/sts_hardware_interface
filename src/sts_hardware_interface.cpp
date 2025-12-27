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

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sts_hardware_interface
{

// Static map to share serial port connections across multiple hardware interfaces
static std::map<std::string, std::shared_ptr<SMS_STS>> serial_port_connections;

hardware_interface::CallbackReturn STSHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::ActuatorInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Extract required parameters
  try {
    motor_id_ = std::stoi(info_.hardware_parameters.at("motor_id"));
    serial_port_ = info_.hardware_parameters.at("serial_port");
  } catch (const std::out_of_range & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("STSHardwareInterface"),
      "Missing required parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Extract optional parameters
  baud_rate_ = 1000000;  // Default baud rate
  if (info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end()) {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  }
  
  operating_mode_ = 1;  // Default to velocity mode
  if (info_.hardware_parameters.find("operating_mode") != info_.hardware_parameters.end()) {
    operating_mode_ = std::stoi(info_.hardware_parameters.at("operating_mode"));
  }
  
  // Validate operating mode
  if (operating_mode_ < 0 || operating_mode_ > 2) {
    RCLCPP_FATAL(
      rclcpp::get_logger("STSHardwareInterface"),
      "Invalid operating_mode: %d (must be 0=servo, 1=velocity, 2=PWM)", operating_mode_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate motor ID
  if (motor_id_ < 1 || motor_id_ > 253) {
    RCLCPP_FATAL(
      rclcpp::get_logger("STSHardwareInterface"),
      "Invalid motor_id: %d (must be 1-253)", motor_id_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state and command values
  hw_state_position_ = std::numeric_limits<double>::quiet_NaN();
  hw_state_velocity_ = std::numeric_limits<double>::quiet_NaN();
  hw_state_load_ = std::numeric_limits<double>::quiet_NaN();
  hw_state_voltage_ = std::numeric_limits<double>::quiet_NaN();
  hw_state_temperature_ = std::numeric_limits<double>::quiet_NaN();
  hw_state_current_ = std::numeric_limits<double>::quiet_NaN();
  hw_state_is_moving_ = 0.0;
  
  hw_cmd_position_ = 0.0;
  hw_cmd_velocity_ = 0.0;
  hw_cmd_acceleration_ = 0.0;
  hw_cmd_effort_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Initialized STS hardware interface: motor_id=%d, port=%s, baud=%d, mode=%d",
    motor_id_, serial_port_.c_str(), baud_rate_, operating_mode_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STSHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Configuring STS hardware interface motor_id=%d...", motor_id_);

  // Check if serial port is already open (shared connection)
  if (serial_port_connections.find(serial_port_) != serial_port_connections.end()) {
    servo_ = serial_port_connections[serial_port_];
    owns_serial_connection_ = false;
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Reusing existing serial connection to %s", serial_port_.c_str());
  } else {
    // Create new serial connection
    servo_ = std::make_shared<SMS_STS>();
    if (!servo_->begin(baud_rate_, serial_port_.c_str())) {
      RCLCPP_FATAL(
        rclcpp::get_logger("STSHardwareInterface"),
        "Failed to open serial port %s at baud rate %d",
        serial_port_.c_str(), baud_rate_);
      return hardware_interface::CallbackReturn::ERROR;
    }
    serial_port_connections[serial_port_] = servo_;
    owns_serial_connection_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Opened new serial connection to %s", serial_port_.c_str());
  }

  // Verify motor communication with ping
  int result = servo_->Ping(motor_id_);
  if (result == -1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("STSHardwareInterface"),
      "Failed to ping motor ID %d - check motor ID and connections", motor_id_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Successfully pinged motor ID %d", motor_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
STSHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Standard interfaces
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_state_position_));
  
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_));

  // Custom diagnostic interfaces
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, "load", &hw_state_load_));
  
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, "voltage", &hw_state_voltage_));
  
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, "temperature", &hw_state_temperature_));
  
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, "current", &hw_state_current_));
  
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, "is_moving", &hw_state_is_moving_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
STSHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export mode-specific command interfaces
  switch (operating_mode_) {
    case 0:  // Servo mode (position control)
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_cmd_position_));
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocity_));  // max speed
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[0].name, "acceleration", &hw_cmd_acceleration_));
      break;
      
    case 1:  // Velocity mode (closed-loop speed control)
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocity_));
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[0].name, "acceleration", &hw_cmd_acceleration_));
      break;
      
    case 2:  // PWM mode (open-loop effort control)
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_cmd_effort_));
      break;
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn STSHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Activating STS hardware interface motor_id=%d in mode %d...", motor_id_, operating_mode_);

  // Initialize motor: set operating mode and enable torque
  // Mode 0 = servo (position), 1 = wheel closed-loop (velocity), 2 = wheel open-loop (PWM)
  int result = servo_->InitMotor(motor_id_, operating_mode_, 1);
  if (result != 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("STSHardwareInterface"),
      "Failed to initialize motor %d to mode %d", motor_id_, operating_mode_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  const char* mode_name = (operating_mode_ == 0) ? "servo (position)" :
                          (operating_mode_ == 1) ? "velocity (closed-loop)" : 
                          "PWM (open-loop)";
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Motor %d activated in %s mode with torque enabled", motor_id_, mode_name);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STSHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Deactivating STS hardware interface motor_id=%d (mode %d)...", motor_id_, operating_mode_);

  // Stop motor based on current operating mode
  int result = -1;
  switch (operating_mode_) {
    case 0:  // Servo mode - send current position with zero speed
      result = servo_->WritePosEx(motor_id_, servo_->ReadPos(motor_id_), 0, 0);
      break;
    case 1:  // Velocity mode - set velocity to 0
      result = servo_->WriteSpe(motor_id_, 0, 0);
      break;
    case 2:  // PWM mode - set PWM to 0
      result = servo_->WritePwm(motor_id_, 0);
      break;
  }
  
  if (result != 1) {
    RCLCPP_WARN(
      rclcpp::get_logger("STSHardwareInterface"),
      "Failed to stop motor %d during deactivation", motor_id_);
  }
  
  // Disable torque
  result = servo_->EnableTorque(motor_id_, 0);
  if (result != 1) {
    RCLCPP_WARN(
      rclcpp::get_logger("STSHardwareInterface"),
      "Failed to disable torque on motor %d", motor_id_);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Motor %d stopped and torque disabled", motor_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STSHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Cleaning up STS hardware interface motor_id=%d...", motor_id_);

  // Close serial connection only if we own it
  if (owns_serial_connection_ && servo_) {
    servo_->end();
    serial_port_connections.erase(serial_port_);
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Closed serial connection to %s", serial_port_.c_str());
  }

  servo_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STSHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(
    rclcpp::get_logger("STSHardwareInterface"),
    "Error state detected for motor_id=%d, attempting emergency stop...", motor_id_);

  if (!servo_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("STSHardwareInterface"),
      "Servo interface not available for emergency stop");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Emergency stop: set motor to safe state regardless of mode
  int result = -1;
  switch (operating_mode_) {
    case 0:  // Servo mode - hold current position
      result = servo_->WritePosEx(motor_id_, servo_->ReadPos(motor_id_), 0, 0);
      break;
    case 1:  // Velocity mode - stop immediately
      result = servo_->WriteSpe(motor_id_, 0, 254);  // Max deceleration
      break;
    case 2:  // PWM mode - cut power
      result = servo_->WritePwm(motor_id_, 0);
      break;
  }
  
  if (result != 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("STSHardwareInterface"),
      "Failed to send emergency stop command to motor %d", motor_id_);
  }
  
  // Disable torque for safety
  result = servo_->EnableTorque(motor_id_, 0);
  if (result != 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("STSHardwareInterface"),
      "Failed to disable torque on motor %d during error handling", motor_id_);
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Motor %d emergency stopped and torque disabled", motor_id_);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type STSHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Request all feedback data from motor in one transaction
  int result = servo_->FeedBack(motor_id_);
  if (result != 1) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("STSHardwareInterface"),
      *rclcpp::Clock::make_shared(), 1000,
      "Failed to read feedback from motor %d", motor_id_);
    return hardware_interface::return_type::ERROR;
  }

  // Read all state data using cached values (ID = -1)
  int raw_position = servo_->ReadPos(-1);
  int raw_velocity = servo_->ReadSpeed(-1);
  int raw_load = servo_->ReadLoad(-1);
  int raw_voltage = servo_->ReadVoltage(-1);
  int raw_temperature = servo_->ReadTemper(-1);
  int raw_current = servo_->ReadCurrent(-1);
  int raw_is_moving = servo_->ReadMove(-1);

  // Convert to SI units
  hw_state_position_ = raw_position_to_radians(raw_position);
  hw_state_velocity_ = raw_velocity_to_rad_s(raw_velocity);
  hw_state_load_ = raw_load_to_percentage(raw_load);
  hw_state_voltage_ = raw_voltage_to_volts(raw_voltage);
  hw_state_temperature_ = static_cast<double>(raw_temperature);
  hw_state_current_ = raw_current_to_amperes(raw_current);
  hw_state_is_moving_ = (raw_is_moving > 0) ? 1.0 : 0.0;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STSHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  int result = -1;
  
  switch (operating_mode_) {
    case 0: {  // Servo mode (position control)
      int raw_position = radians_to_raw_position(hw_cmd_position_);
      int max_speed = static_cast<int>(std::clamp(hw_cmd_velocity_, 0.0, 3400.0));  // 0-3400 steps/s
      int acceleration = static_cast<int>(std::clamp(hw_cmd_acceleration_, 0.0, 254.0));  // 0-254
      
      // Use WritePosEx for position command (single motor)
      // Protocol: WritePosEx(ID, Position, Speed, Acceleration)
      result = servo_->WritePosEx(motor_id_, raw_position, max_speed, acceleration);
      
      if (result != 1) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("STSHardwareInterface"),
          *rclcpp::Clock::make_shared(), 1000,
          "Failed to write position to motor %d", motor_id_);
        return hardware_interface::return_type::ERROR;
      }
      break;
    }
    
    case 1: {  // Velocity mode (closed-loop speed control)
      int raw_velocity = rad_s_to_raw_velocity(hw_cmd_velocity_);
      int acceleration = static_cast<int>(std::clamp(hw_cmd_acceleration_, 0.0, 254.0));
      
      // Use WriteSpe for velocity command (single motor)
      // Protocol: WriteSpe(ID, Speed, Acceleration)
      result = servo_->WriteSpe(motor_id_, raw_velocity, acceleration);
      
      if (result != 1) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("STSHardwareInterface"),
          *rclcpp::Clock::make_shared(), 1000,
          "Failed to write velocity to motor %d", motor_id_);
        return hardware_interface::return_type::ERROR;
      }
      break;
    }
    
    case 2: {  // PWM mode (open-loop effort control)
      int raw_pwm = effort_to_raw_pwm(hw_cmd_effort_);
      
      // Use WritePwm for effort command
      result = servo_->WritePwm(motor_id_, raw_pwm);
      
      if (result != 1) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("STSHardwareInterface"),
          *rclcpp::Clock::make_shared(), 1000,
          "Failed to write PWM to motor %d", motor_id_);
        return hardware_interface::return_type::ERROR;
      }
      break;
    }
    
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
        "Invalid operating mode: %d", operating_mode_);
      return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// Unit conversion helper functions

double STSHardwareInterface::raw_position_to_radians(int raw_position) const
{
  // Position wraps around at 4096 steps
  // Convert to radians: 0-4095 steps -> 0-2π radians
  return static_cast<double>(raw_position) * STEPS_TO_RAD;
}

double STSHardwareInterface::raw_velocity_to_rad_s(int raw_velocity) const
{
  // Velocity is in steps/second, can be negative
  // Convert to rad/s
  return static_cast<double>(raw_velocity) * STEPS_TO_RAD;
}

int STSHardwareInterface::rad_s_to_raw_velocity(double velocity_rad_s) const
{
  // Convert rad/s to steps/second
  double raw = velocity_rad_s * RAD_TO_STEPS;
  
  // Clamp to motor limits: ±3400 steps/s (STS series limit)
  int clamped = static_cast<int>(std::clamp(raw, -3400.0, 3400.0));
  
  return clamped;
}

double STSHardwareInterface::raw_load_to_percentage(int raw_load) const
{
  // Load is -1000 to +1000 representing ±100% PWM
  // Convert to percentage: -100.0 to +100.0
  return static_cast<double>(raw_load) * LOAD_SCALE;
}

double STSHardwareInterface::raw_voltage_to_volts(int raw_voltage) const
{
  // Voltage is in decivolts (0.1V units)
  // e.g., 120 = 12.0V
  return static_cast<double>(raw_voltage) * VOLTAGE_SCALE;
}

double STSHardwareInterface::raw_current_to_amperes(int raw_current) const
{
  // Current is in milliamps
  // Convert to amps
  return static_cast<double>(raw_current) * CURRENT_SCALE;
}

int STSHardwareInterface::radians_to_raw_position(double position_rad) const
{
  // Convert radians to steps (0-4095)
  // Normalize to [0, 2π) range first
  double normalized = std::fmod(position_rad, 2.0 * M_PI);
  if (normalized < 0.0) {
    normalized += 2.0 * M_PI;
  }
  
  // Convert to raw position
  int raw = static_cast<int>(normalized * RAD_TO_STEPS);
  
  // Ensure within valid range
  return std::clamp(raw, 0, 4095);
}

int STSHardwareInterface::effort_to_raw_pwm(double effort) const
{
  // Effort is normalized [-1.0, 1.0] -> raw PWM [-1000, 1000]
  // Clamp to safe range
  double clamped = std::clamp(effort, -1.0, 1.0);
  
  // Convert to raw PWM value
  return static_cast<int>(clamped * 1000.0);
}

}  // namespace sts_hardware_interface

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  sts_hardware_interface::STSHardwareInterface, hardware_interface::ActuatorInterface)
