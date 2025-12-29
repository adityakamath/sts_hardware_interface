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
#include <mutex>


  // Skip hardware deactivation in mock mode
  if (enable_mock_mode_) {
    mock_velocity_ = 0.0;
    mock_load_ = 0.0;
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Mock mode: Motor stopped (simulated)");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Stop motor based on current operating mode
  int result = -1;
  switch (operating_mode_) {
    case MODE_SERVO:  // Servo mode - send current position with zero speed
      result = servo_->WritePosEx(motor_id_, servo_->ReadPos(motor_id_), 0, 0);
      break;
    case MODE_VELOCITY:  // Velocity mode - set velocity to 0
      result = servo_->WriteSpe(motor_id_, 0, 0);
      break;
    case MODE_PWM:  // PWM mode - set PWM to 0
      result = servo_->WritePwm(motor_id_, 0);
      break;
  }
  
  if (result != 1) {
    RCLCPP_WARN(
      rclcpp::get_logger("STSHardwareInterface"),
      "Failed to stop motor %d during deactivation (mode: %d)", motor_id_, operating_mode_);
  }
  
  // Disable torque
  result = servo_->EnableTorque(motor_id_, 0);
  if (result != 1) {
    int servo_error = servo_->getErr();
    RCLCPP_WARN(
      rclcpp::get_logger("STSHardwareInterface"),
      "Failed to disable torque on motor %d (servo error: %d)", motor_id_, servo_error);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Motor %d on %s stopped and torque disabled", motor_id_, serial_port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STSHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Cleaning up STS hardware interface motor_id=%d...", motor_id_);

  // Close serial connection only if we own it (thread-safe)
  if (owns_serial_connection_ && servo_) {
    std::lock_guard<std::mutex> lock(serial_port_mutex);
    servo_->end();
    serial_port_connections.erase(serial_port_);
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Closed serial connection to %s", serial_port_.c_str());
  }

  servo_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STSHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Shutting down STS hardware interface motor_id=%d...", motor_id_);

  // Ensure motor is stopped and torque disabled
  if (servo_) {
    // Stop motor based on current operating mode
    switch (operating_mode_) {
      case MODE_SERVO:  // Servo mode - send current position with zero speed
        {
          int current_pos = servo_->ReadPos(motor_id_);
          if (current_pos != -1) {
            servo_->WritePosEx(motor_id_, current_pos, 0, 0);
          }
        }
        break;
      case MODE_VELOCITY:  // Velocity mode - set velocity to 0
        servo_->WriteSpe(motor_id_, 0, 0);
        break;
      case MODE_PWM:  // PWM mode - set PWM to 0
        servo_->WritePwm(motor_id_, 0);
        break;
    }
    
    // Disable torque for safety
    servo_->EnableTorque(motor_id_, 0);
    
    // Close serial connection if we own it
    if (owns_serial_connection_) {
      std::lock_guard<std::mutex> lock(serial_port_mutex);
      servo_->end();
      serial_port_connections.erase(serial_port_);
      RCLCPP_INFO(
        rclcpp::get_logger("STSHardwareInterface"),
        "Closed serial connection to %s during shutdown", serial_port_.c_str());
    }
    
    servo_.reset();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STSHardwareInterface"),
    "Motor %d shutdown complete", motor_id_);

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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto start_time = std::chrono::steady_clock::now();
  
  // Mock mode: simulate hardware behavior
  if (enable_mock_mode_) {
    // Simple physics simulation
    double dt = period.seconds();
    
    if (operating_mode_ == MODE_SERVO) {
      // Simulate position control with simple first-order response
      double position_error = hw_cmd_position_ - mock_position_;
      double max_step = hw_cmd_velocity_ * dt;  // Use commanded velocity as max speed
      if (std::abs(position_error) > max_step && max_step > 0) {
        mock_position_ += (position_error > 0 ? max_step : -max_step);
      } else {
        mock_position_ = hw_cmd_position_;
      }
      mock_velocity_ = (mock_position_ - hw_state_position_) / (dt > 0 ? dt : 0.001);
    } else if (operating_mode_ == MODE_VELOCITY) {
      // Simulate velocity control
      mock_velocity_ = hw_cmd_velocity_;
      mock_position_ += mock_velocity_ * dt;
    } else {  // MODE_PWM
      // Simulate PWM as torque control with simplified dynamics
      mock_velocity_ = hw_cmd_effort_ * 10.0;  // Simplified: effort -> velocity
      mock_position_ += mock_velocity_ * dt;
    }
    
    // Simulate load based on acceleration
    mock_load_ = mock_velocity_ / (STS_MAX_VELOCITY_STEPS * STEPS_TO_RAD) * 100.0;
    mock_load_ = std::clamp(mock_load_, -100.0, 100.0);
    
    // Simulate temperature increase with load
    double temp_increase = std::abs(mock_load_) * 0.01 * dt;
    mock_temperature_ += temp_increase;
    // Cool down towards ambient
    mock_temperature_ -= (mock_temperature_ - 25.0) * 0.1 * dt;
    mock_temperature_ = std::clamp(mock_temperature_, 20.0, 80.0);
    
    // Simulate current based on load
    mock_current_ = std::abs(mock_load_) * 0.01;  // Simplified
    
    // Update state interfaces with simulated values
    hw_state_position_ = mock_position_;
    hw_state_velocity_ = mock_velocity_;
    hw_state_load_ = mock_load_;
    hw_state_temperature_ = mock_temperature_;
    hw_state_voltage_ = mock_voltage_;
    hw_state_current_ = mock_current_;
    hw_state_is_moving_ = (std::abs(mock_velocity_) > 0.01) ? 1.0 : 0.0;
    
    // Update diagnostic updater
    if (diagnostic_updater_) {
      diagnostic_updater_->force_update();
    }
    
    // Track performance
    auto end_time = std::chrono::steady_clock::now();
    read_duration_ms_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    max_read_duration_ms_ = std::max(max_read_duration_ms_, read_duration_ms_);
    cycle_count_++;
    
    return hardware_interface::return_type::OK;
  }
  
  // Real hardware mode: request all feedback data from motor in one transaction
  int result = servo_->FeedBack(motor_id_);
  if (result != 1) {
    consecutive_read_errors_++;
    
    // Get detailed error information from servo SDK
    int servo_error = servo_->getErr();
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("STSHardwareInterface"),
      *rclcpp::Clock::make_shared(), 1000,
      "Failed to read feedback from motor %d (error count: %d/%d, servo error code: %d)", 
      motor_id_, consecutive_read_errors_, MAX_CONSECUTIVE_ERRORS, servo_error);
    
    // Attempt recovery if too many consecutive errors
    if (consecutive_read_errors_ >= MAX_CONSECUTIVE_ERRORS) {
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
        "Too many consecutive read errors (%d), attempting recovery...", consecutive_read_errors_);
      
      if (attempt_error_recovery()) {
        RCLCPP_INFO(
          rclcpp::get_logger("STSHardwareInterface"),
          "Error recovery successful for motor %d", motor_id_);
        consecutive_read_errors_ = 0;
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger("STSHardwareInterface"),
          "Error recovery failed for motor %d", motor_id_);
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
    // Detect wrap-around (crossing 0/4095 boundary)
    int position_diff = raw_position - last_raw_position_;
    
    // If position jumped more than half range, we wrapped around
    if (position_diff > STS_MAX_POSITION / 2) {
      // Wrapped backward (4095 -> 0)
      revolution_count_--;
    } else if (position_diff < -STS_MAX_POSITION / 2) {
      // Wrapped forward (0 -> 4095)
      revolution_count_++;
    }
    
    last_raw_position_ = raw_position;
    
    // Calculate continuous position
    continuous_position_ = (revolution_count_ * 2.0 * M_PI) + raw_position_to_radians(raw_position);
    hw_state_position_ = continuous_position_;
  } else {
    // Single-turn mode: position wraps at 2π
    hw_state_position_ = raw_position_to_radians(raw_position);
  }

  // Convert other state data to SI units
  hw_state_velocity_ = raw_velocity_to_rad_s(raw_velocity);
  hw_state_load_ = raw_load_to_percentage(raw_load);
  hw_state_voltage_ = raw_voltage_to_volts(raw_voltage);
  hw_state_temperature_ = static_cast<double>(raw_temperature);
  hw_state_current_ = raw_current_to_amperes(raw_current);
  hw_state_is_moving_ = (raw_is_moving > 0) ? 1.0 : 0.0;
  
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
      "Motor %d performance: read=%.2fms (max=%.2fms), write=%.2fms (max=%.2fms)",
      motor_id_, read_duration_ms_, max_read_duration_ms_, 
      write_duration_ms_, max_write_duration_ms_);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STSHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto start_time = std::chrono::steady_clock::now();
  
  // Mock mode: skip hardware writes
  if (enable_mock_mode_) {
    // Just track performance and return
    auto end_time = std::chrono::steady_clock::now();
    write_duration_ms_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    max_write_duration_ms_ = std::max(max_write_duration_ms_, write_duration_ms_);
    
    // Handle emergency stop in mock mode
    if (hw_cmd_emergency_stop_ > 0.5) {
      mock_velocity_ = 0.0;
      emergency_stop_active_ = true;
    } else if (emergency_stop_active_) {
      emergency_stop_active_ = false;
    }
    
    return hardware_interface::return_type::OK;
  }
  
  // Check for emergency stop command
  bool emergency_stop_requested = (hw_cmd_emergency_stop_ > 0.5);
  
  if (emergency_stop_requested && !emergency_stop_active_) {
    // Emergency stop activated
    RCLCPP_WARN(
      rclcpp::get_logger("STSHardwareInterface"),
      "Emergency stop activated for motor %d", motor_id_);
    
    // Stop motor immediately
    switch (operating_mode_) {
      case MODE_SERVO:  // Servo mode - hold current position
        {
          int current_pos = servo_->ReadPos(motor_id_);
          if (current_pos != -1) {
            servo_->WritePosEx(motor_id_, current_pos, 0, 0);
          }
        }
        break;
      case MODE_VELOCITY:  // Velocity mode - stop with max deceleration
        servo_->WriteSpe(motor_id_, 0, STS_MAX_ACCELERATION);
        break;
      case MODE_PWM:  // PWM mode - cut power
        servo_->WritePwm(motor_id_, 0);
        break;
    }
    
    emergency_stop_active_ = true;
    return hardware_interface::return_type::OK;
  }
  
  if (!emergency_stop_requested && emergency_stop_active_) {
    // Emergency stop released
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Emergency stop released for motor %d", motor_id_);
    emergency_stop_active_ = false;
    // Continue to normal operation below
  }
  
  // Skip normal commands if emergency stop is active
  if (emergency_stop_active_) {
    return hardware_interface::return_type::OK;
  }
  
  int result = -1;
  
  switch (operating_mode_) {
    case MODE_SERVO: {  // Servo mode (position control)
      // Apply position limits if defined
      double target_position = hw_cmd_position_;
      if (has_position_limits_) {
        if (target_position < position_min_ || target_position > position_max_) {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("STSHardwareInterface"),
            *rclcpp::Clock::make_shared(), 1000,
            "Position command %.3f rad exceeds limits [%.3f, %.3f], clamping",
            target_position, position_min_, position_max_);
          target_position = std::clamp(target_position, position_min_, position_max_);
        }
      }
      
      int raw_position = radians_to_raw_position(target_position);
      
      // Apply velocity limits if defined
      double max_speed = hw_cmd_velocity_;
      if (has_velocity_limits_) {
        max_speed = std::min(max_speed, velocity_max_);
      }
      int raw_max_speed = static_cast<int>(std::clamp(max_speed, 0.0, static_cast<double>(STS_MAX_VELOCITY_STEPS)));
      
      int acceleration = static_cast<int>(std::clamp(hw_cmd_acceleration_, 0.0, static_cast<double>(STS_MAX_ACCELERATION)));
      
      // Use WritePosEx for position command (single motor)
      // Protocol: WritePosEx(ID, Position, Speed, Acceleration)
      result = servo_->WritePosEx(motor_id_, raw_position, raw_max_speed, acceleration);
      
      if (result != 1) {
        consecutive_write_errors_++;
        int servo_error = servo_->getErr();
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("STSHardwareInterface"),
          *rclcpp::Clock::make_shared(), 1000,
          "Failed to write position to motor %d (error count: %d/%d, servo error: %d)", 
          motor_id_, consecutive_write_errors_, MAX_CONSECUTIVE_ERRORS, servo_error);
        
        if (consecutive_write_errors_ >= MAX_CONSECUTIVE_ERRORS && attempt_error_recovery()) {
          consecutive_write_errors_ = 0;
        }
        
        return hardware_interface::return_type::ERROR;
      }
      consecutive_write_errors_ = 0;
      break;
    }
    
    case MODE_VELOCITY: {  // Velocity mode (closed-loop speed control)
      // Apply velocity limits if defined
      double target_velocity = hw_cmd_velocity_;
      if (has_velocity_limits_) {
        if (std::abs(target_velocity) > velocity_max_) {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("STSHardwareInterface"),
            *rclcpp::Clock::make_shared(), 1000,
            "Velocity command %.3f rad/s exceeds limit %.3f, clamping",
            target_velocity, velocity_max_);
          target_velocity = std::clamp(target_velocity, -velocity_max_, velocity_max_);
        }
      }
      
      int raw_velocity = rad_s_to_raw_velocity(target_velocity);
      int acceleration = static_cast<int>(std::clamp(hw_cmd_acceleration_, 0.0, static_cast<double>(STS_MAX_ACCELERATION)));
      
      // Use WriteSpe for velocity command (single motor)
      // Protocol: WriteSpe(ID, Speed, Acceleration)
      result = servo_->WriteSpe(motor_id_, raw_velocity, acceleration);
      
      if (result != 1) {
        consecutive_write_errors_++;
        int servo_error = servo_->getErr();
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("STSHardwareInterface"),
          *rclcpp::Clock::make_shared(), 1000,
          "Failed to write velocity to motor %d (error count: %d/%d, servo error: %d)", 
          motor_id_, consecutive_write_errors_, MAX_CONSECUTIVE_ERRORS, servo_error);
        
        if (consecutive_write_errors_ >= MAX_CONSECUTIVE_ERRORS && attempt_error_recovery()) {
          consecutive_write_errors_ = 0;
        }
        
        return hardware_interface::return_type::ERROR;
      }
      consecutive_write_errors_ = 0;
      break;
    }
    
    case MODE_PWM: {  // PWM mode (open-loop effort control)
      // Apply effort limits if defined
      double target_effort = hw_cmd_effort_;
      if (has_effort_limits_) {
        double normalized_limit = effort_max_;  // Assumes effort_max in range [0, 1.0]
        if (std::abs(target_effort) > normalized_limit) {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("STSHardwareInterface"),
            *rclcpp::Clock::make_shared(), 1000,
            "Effort command %.3f exceeds limit %.3f, clamping",
            target_effort, normalized_limit);
          target_effort = std::clamp(target_effort, -normalized_limit, normalized_limit);
        }
      }
      
      int raw_pwm = effort_to_raw_pwm(target_effort);
      
      // Use WritePwm for effort command
      result = servo_->WritePwm(motor_id_, raw_pwm);
      
      if (result != 1) {
        consecutive_write_errors_++;
        int servo_error = servo_->getErr();
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("STSHardwareInterface"),
          *rclcpp::Clock::make_shared(), 1000,
          "Failed to write PWM to motor %d (error count: %d/%d, servo error: %d)", 
          motor_id_, consecutive_write_errors_, MAX_CONSECUTIVE_ERRORS, servo_error);
        
        if (consecutive_write_errors_ >= MAX_CONSECUTIVE_ERRORS && attempt_error_recovery()) {
          consecutive_write_errors_ = 0;
        }
        
        return hardware_interface::return_type::ERROR;
      }
      consecutive_write_errors_ = 0;
      break;
    }
    
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
        "Invalid operating mode: %d", operating_mode_);
      return hardware_interface::return_type::ERROR;
  }
  
  // Performance monitoring
  auto end_time = std::chrono::steady_clock::now();
  write_duration_ms_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  max_write_duration_ms_ = std::max(max_write_duration_ms_, write_duration_ms_);
  last_write_time_ = end_time;

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
  
  // Clamp to motor limits
  int clamped = static_cast<int>(std::clamp(raw, static_cast<double>(-STS_MAX_VELOCITY_STEPS), static_cast<double>(STS_MAX_VELOCITY_STEPS)));
  
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
  return std::clamp(raw, 0, STS_MAX_POSITION);
}

int STSHardwareInterface::effort_to_raw_pwm(double effort) const
{
  // Effort is normalized [-1.0, 1.0] -> raw PWM [-1000, 1000]
  // Clamp to safe range
  double clamped = std::clamp(effort, -1.0, 1.0);
  
  // Convert to raw PWM value
  return static_cast<int>(clamped * STS_MAX_PWM);
}

bool STSHardwareInterface::attempt_error_recovery()
{
  RCLCPP_WARN(
    rclcpp::get_logger("STSHardwareInterface"),
    "Attempting error recovery for motor %d...", motor_id_);
  
  if (!servo_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("STSHardwareInterface"),
      "Servo interface not available for recovery");
    return false;
  }
  
  // Step 1: Try to ping the motor
  int ping_result = servo_->Ping(motor_id_);
  if (ping_result != -1) {
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Motor %d responded to ping, communication restored", motor_id_);
    return true;
  }
  
  // Step 2: If we own the connection, try to re-establish it
  if (owns_serial_connection_) {
    RCLCPP_WARN(
      rclcpp::get_logger("STSHardwareInterface"),
      "Ping failed, attempting to reinitialize serial connection to %s", serial_port_.c_str());
    
    std::lock_guard<std::mutex> lock(serial_port_mutex);
    
    // Close and reopen serial port
    servo_->end();
    
    if (!servo_->begin(baud_rate_, serial_port_.c_str())) {
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
        "Failed to reopen serial port %s during recovery", serial_port_.c_str());
      return false;
    }
    
    // Verify motor communication
    ping_result = servo_->Ping(motor_id_);
    if (ping_result == -1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
        "Motor %d still not responding after serial reinit", motor_id_);
      return false;
    }
    
    // Re-activate motor
    int init_result = servo_->InitMotor(motor_id_, operating_mode_, 1);
    if (init_result != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("STSHardwareInterface"),
        "Failed to reinitialize motor %d after recovery", motor_id_);
      return false;
    }
    
    RCLCPP_INFO(
      rclcpp::get_logger("STSHardwareInterface"),
      "Successfully recovered communication with motor %d", motor_id_);
    return true;
  }
  
  RCLCPP_ERROR(
    rclcpp::get_logger("STSHardwareInterface"),
    "Recovery failed: motor %d not responding and we don't own the serial connection", motor_id_);
  return false;
}

void STSHardwareInterface::diagnostics_callback(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Determine overall status
  if (consecutive_read_errors_ > 0 || consecutive_write_errors_ > 0) {
    if (consecutive_read_errors_ >= MAX_CONSECUTIVE_ERRORS || 
        consecutive_write_errors_ >= MAX_CONSECUTIVE_ERRORS) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Communication errors detected");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Intermittent communication errors");
    }
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Motor operating normally");
  }
  
  // Add diagnostic data
  const char* mode_name = (operating_mode_ == MODE_SERVO) ? "Servo" :
                          (operating_mode_ == MODE_VELOCITY) ? "Velocity" : "PWM";
  stat.add("Operating Mode", mode_name);
  stat.add("Motor ID", motor_id_);
  stat.add("Serial Port", serial_port_);
  stat.add("Baud Rate", baud_rate_);
  
  // Current state
  stat.add("Position (rad)", hw_state_position_);
  stat.add("Velocity (rad/s)", hw_state_velocity_);
  stat.add("Load (%)", hw_state_load_);
  stat.add("Voltage (V)", hw_state_voltage_);
  stat.add("Temperature (°C)", hw_state_temperature_);
  stat.add("Current (A)", hw_state_current_);
  stat.add("Is Moving", hw_state_is_moving_ > 0.5 ? "Yes" : "No");
  
  // Multi-turn tracking
  if (enable_multi_turn_) {
    stat.add("Multi-turn Enabled", "Yes");
    stat.add("Revolution Count", revolution_count_);
    stat.add("Continuous Position (rad)", continuous_position_);
  } else {
    stat.add("Multi-turn Enabled", "No");
  }
  
  // Error counters
  stat.add("Read Errors", consecutive_read_errors_);
  stat.add("Write Errors", consecutive_write_errors_);
  
  // Performance metrics
  stat.add("Read Duration (ms)", read_duration_ms_);
  stat.add("Write Duration (ms)", write_duration_ms_);
  stat.add("Max Read Duration (ms)", max_read_duration_ms_);
  stat.add("Max Write Duration (ms)", max_write_duration_ms_);
  stat.add("Cycle Count", cycle_count_);
  
  // Limits status
  if (has_position_limits_) {
    stat.add("Position Limits (rad)", 
             std::to_string(position_min_) + " to " + std::to_string(position_max_));
  }
  if (has_velocity_limits_) {
    stat.add("Velocity Limit (rad/s)", velocity_max_);
  }
  if (has_effort_limits_) {
    stat.add("Effort Limit", effort_max_);
  }
}

}  // namespace sts_hardware_interface

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  sts_hardware_interface::STSHardwareInterface, hardware_interface::ActuatorInterface)
