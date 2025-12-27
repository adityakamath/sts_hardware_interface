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

#ifndef STS_HARDWARE_INTERFACE__STS_HARDWARE_INTERFACE_HPP_
#define STS_HARDWARE_INTERFACE__STS_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// SCServo SDK
#include "SMS_STS.h"

namespace sts_hardware_interface
{

/**
 * @brief ros2_control ActuatorInterface for Feetech STS series servo motors
 * 
 * Supports all three STS operating modes with complete diagnostics access.
 * Each actuator instance represents one motor on the shared serial bus.
 * Uses SyncWrite functions for optimal serial bus efficiency.
 * 
 * OPERATING MODES:
 * - Mode 0 (Servo): Position control with speed and acceleration limits
 * - Mode 1 (Velocity): Closed-loop velocity control (default)
 * - Mode 2 (PWM): Open-loop PWM/effort control
 * 
 * STATE INTERFACES (Read from hardware - all modes):
 * - position: Current position in radians
 * - velocity: Current velocity in rad/s
 * - load: Motor load/torque as percentage (-100.0 to +100.0)
 * - voltage: Supply voltage in volts
 * - temperature: Internal temperature in degrees Celsius
 * - current: Motor current draw in amperes
 * - is_moving: Motion status (1.0=moving, 0.0=stopped)
 * 
 * COMMAND INTERFACES (Write to hardware - mode dependent):
 * Mode 0: position (rad), velocity (max speed, rad/s), acceleration (0-254)
 * Mode 1: velocity (rad/s), acceleration (0-254)
 * Mode 2: effort (PWM duty cycle, -1.0 to +1.0)
 * 
 * PARAMETERS (from ros2_control URDF):
 * - serial_port: Serial port path (e.g., "/dev/ttyACM0") [required]
 * - motor_id: Motor ID on the serial bus (1-253) [required]
 * - operating_mode: 0=servo, 1=velocity, 2=PWM (default: 1)
 * - baud_rate: Communication baud rate (default: 1000000)
 * 
 * USAGE IN URDF:
 * @code{.xml}
 * <ros2_control name="motor_left" type="actuator">
 *   <hardware>
 *     <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
 *     <param name="serial_port">/dev/ttyACM0</param>
 *     <param name="motor_id">7</param>
 *   </hardware>
 *   <joint name="left_wheel_joint">
 *     <command_interface name="velocity"/>
 *     <command_interface name="acceleration"/>
 *     <state_interface name="position"/>
 *     <state_interface name="velocity"/>
 *     <state_interface name="load"/>
 *     <state_interface name="voltage"/>
 *     <state_interface name="temperature"/>
 *     <state_interface name="current"/>
 *     <state_interface name="is_moving"/>
 *   </joint>
 * </ros2_control>
 * @endcode
 */
class STSHardwareInterface : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(STSHardwareInterface)

  /**
   * @brief Initialize the hardware interface from URDF hardware info
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  /**
   * @brief Configure the hardware (establish serial connection)
   * 
   * Opens serial port and verifies communication with the motor.
   * 
   * @param previous_state Previous lifecycle state
   * @return CallbackReturn::SUCCESS if configuration successful
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate the hardware (enable motor torque)
   * 
   * Sets motor to velocity mode and enables torque output.
   * 
   * @param previous_state Previous lifecycle state
   * @return CallbackReturn::SUCCESS if activation successful
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the hardware (disable motor torque)
   * 
   * Stops motor and disables torque output.
   * 
   * @param previous_state Previous lifecycle state
   * @return CallbackReturn::SUCCESS if deactivation successful
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Cleanup the hardware (close serial connection)
   * 
   * @param previous_state Previous lifecycle state
   * @return CallbackReturn::SUCCESS if cleanup successful
   */
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Handle error state (emergency stop motor)
   * 
   * Stops motor immediately and disables torque for safety.
   * 
   * @param previous_state Previous lifecycle state
   * @return CallbackReturn::SUCCESS if error handling successful
   */
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Export state interfaces (position, velocity, etc.)
   * 
   * @return Vector of state interface descriptions
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Export command interfaces (velocity)
   * 
   * @return Vector of command interface descriptions
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Read current state from the motor
   * 
   * Queries motor for current position, velocity, and optional
   * current/temperature values.
   * 
   * @param time Current time
   * @param period Time since last read
   * @return return_type::OK if read successful
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief Write velocity command to the motor
   * 
   * Sends the commanded velocity to the motor in velocity control mode.
   * 
   * @param time Current time
   * @param period Time since last write
   * @return return_type::OK if write successful
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief Get shared servo communication object
   * 
   * Provides access to the shared SMS_STS instance for system-level operations
   * like broadcast commands.
   * 
   * @return Shared pointer to SMS_STS communication object
   */
  std::shared_ptr<SMS_STS> get_servo() const { return servo_; }

  /**
   * @brief Get motor ID
   * @return Motor ID on the serial bus
   */
  int get_motor_id() const { return motor_id_; }

  /**
   * @brief Get current velocity command in rad/s
   * @return Commanded velocity
   */
  double get_cmd_velocity() const { return hw_cmd_velocity_; }

  /**
   * @brief Get current acceleration command (0-254)
   * @return Commanded acceleration
   */
  double get_cmd_acceleration() const { return hw_cmd_acceleration_; }

private:
  // Configuration parameters (from URDF)
  int motor_id_;
  std::string serial_port_;
  int baud_rate_;
  int operating_mode_;  // 0=servo, 1=velocity, 2=PWM
  
  // SCServo communication object (shared among motors on same bus)
  std::shared_ptr<SMS_STS> servo_;
  bool owns_serial_connection_;  // True if this instance opened the serial port
  
  // State interfaces (read from hardware) - all in SI units
  double hw_state_position_;      // radians
  double hw_state_velocity_;      // rad/s  
  double hw_state_load_;          // Percentage: -100.0 to +100.0
  double hw_state_voltage_;       // Volts
  double hw_state_temperature_;   // Degrees Celsius
  double hw_state_current_;       // Amperes
  double hw_state_is_moving_;     // Boolean: 1.0=moving, 0.0=stopped
  
  // Command interfaces (write to hardware) - all in SI units
  // Mode 0 (Servo): position, velocity (max speed), acceleration
  double hw_cmd_position_;        // rad (Mode 0 only)
  double hw_cmd_velocity_;        // rad/s (Mode 0: max speed, Mode 1: target velocity)
  double hw_cmd_acceleration_;    // Acceleration value (0-254, 0=no limit) (Mode 0, 1)
  // Mode 2 (PWM): effort
  double hw_cmd_effort_;          // PWM duty cycle -1.0 to +1.0 (Mode 2 only)
  
  // Unit conversion constants
  static constexpr double STEPS_PER_REVOLUTION = 4096.0;
  static constexpr double STEPS_TO_RAD = (2.0 * M_PI) / STEPS_PER_REVOLUTION;
  static constexpr double RAD_TO_STEPS = STEPS_PER_REVOLUTION / (2.0 * M_PI);
  static constexpr double VOLTAGE_SCALE = 0.1;      // Decivolts to volts
  static constexpr double CURRENT_SCALE = 0.001;    // Milliamps to amps
  static constexpr double LOAD_SCALE = 0.1;         // Raw load to percentage
  
  /**
   * @brief Convert raw motor position (0-4095 steps) to radians
   */
  double raw_position_to_radians(int raw_position) const;
  
  /**
   * @brief Convert raw motor velocity (steps/s) to rad/s
   */
  double raw_velocity_to_rad_s(int raw_velocity) const;
  
  /**
   * @brief Convert velocity in rad/s to raw motor velocity (steps/s)
   */
  int rad_s_to_raw_velocity(double velocity_rad_s) const;
  
  /**
   * @brief Convert raw load value (-1000 to +1000) to percentage
   */
  double raw_load_to_percentage(int raw_load) const;
  
  /**
   * @brief Convert raw voltage (decivolts) to volts
   */
  double raw_voltage_to_volts(int raw_voltage) const;
  
  /**
   * @brief Convert raw current (milliamps) to amperes
   */
  double raw_current_to_amperes(int raw_current) const;
  
  /**
   * @brief Convert position in radians to raw motor position (steps)
   */
  int radians_to_raw_position(double position_rad) const;
  
  /**
   * @brief Convert effort (-1.0 to +1.0) to raw PWM (-1000 to +1000)
   */
  int effort_to_raw_pwm(double effort) const;
};

}  // namespace sts_hardware_interface

#endif  // STS_HARDWARE_INTERFACE__STS_HARDWARE_INTERFACE_HPP_
