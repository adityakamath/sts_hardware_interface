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
#include <chrono>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"

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

class STSHardwareInterface : public hardware_interface::ActuatorInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(STSHardwareInterface)

  /**
   * @brief Initialize the hardware interface from URDF hardware info (multi-motor)
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief Write all velocity commands using syncWrite or individual write.
   *
   * If sync_write_mode_ is true, uses syncWrite to send all commands in one packet.
   * Otherwise, writes each motor individually.
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief Write speed to a single motor (individual write).
   * @param motor_id Motor ID
   * @param speed Target speed (rad/s)
   */
  void write_motor(int motor_id, double speed);

  /**
   * @brief SyncWrite speeds to all motors (atomic bus transaction).
   * @param speeds Vector of target speeds (rad/s), one per motor
   */
  void sync_write_speeds(const std::vector<double>& speeds);

  /**
   * @brief Set syncWrite mode (true = syncWrite, false = individual write)
   */
  void set_sync_write_mode(bool enable) { sync_write_mode_ = enable; }

  /**
   * @brief Get syncWrite mode
   */
  bool get_sync_write_mode() const { return sync_write_mode_; }

  /**
   * @brief Get list of motor IDs
   */
  const std::vector<int>& get_motor_ids() const { return motor_ids_; }

  // ...existing conversion and diagnostic methods remain unchanged...

private:
  // List of all motor IDs managed by this interface
  std::vector<int> motor_ids_;

  // Command/state buffers for each motor
  std::vector<double> hw_cmd_velocity_;
  std::vector<double> hw_state_velocity_;
  std::vector<double> hw_state_position_;
  std::vector<double> hw_state_load_;
  std::vector<double> hw_state_voltage_;
  std::vector<double> hw_state_temperature_;
  std::vector<double> hw_state_current_;
  std::vector<double> hw_state_is_moving_;

  // SyncWrite mode flag
  bool sync_write_mode_ = true;

  // ...existing private members remain unchanged...

private:
  // Configuration parameters (from URDF)
  int motor_id_;
  std::string serial_port_;
  int baud_rate_;
  int operating_mode_;  // 0=servo, 1=velocity, 2=PWM
  int communication_timeout_ms_;  // Timeout for serial operations (milliseconds)
  bool enable_multi_turn_;  // Enable multi-turn position tracking
  bool enable_mock_mode_;  // Enable simulation mode for testing without hardware
  
  // Hardware limits (from URDF joint limits)
  double position_min_;
  double position_max_;
  double velocity_max_;
  double effort_max_;
  bool has_position_limits_;
  bool has_velocity_limits_;
  bool has_effort_limits_;
  
  // SCServo communication object (shared among motors on same bus)
  std::shared_ptr<SMS_STS> servo_;
  bool owns_serial_connection_;  // True if this instance opened the serial port
  
  // Diagnostic updater
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  
  // Error tracking and recovery
  int consecutive_read_errors_;
  int consecutive_write_errors_;
  static constexpr int MAX_CONSECUTIVE_ERRORS = 5;  // Trigger recovery after this many errors
  
  // Performance monitoring
  std::chrono::steady_clock::time_point last_read_time_;
  std::chrono::steady_clock::time_point last_write_time_;
  double read_duration_ms_;
  double write_duration_ms_;
  double max_read_duration_ms_;
  double max_write_duration_ms_;
  unsigned int cycle_count_;
  static constexpr unsigned int PERFORMANCE_LOG_INTERVAL = 1000;  // Log every N cycles
  
  // State interfaces (read from hardware) - all in SI units
  double hw_state_position_;      // radians
  double hw_state_velocity_;      // rad/s  
  double hw_state_load_;          // Percentage: -100.0 to +100.0
  double hw_state_voltage_;       // Volts
  double hw_state_temperature_;   // Degrees Celsius
  double hw_state_current_;       // Amperes
  double hw_state_is_moving_;     // Boolean: 1.0=moving, 0.0=stopped
  
  // Multi-turn position tracking
  int last_raw_position_;         // Last raw position reading (0-4095)
  int revolution_count_;          // Number of complete revolutions
  double continuous_position_;    // Continuous position in radians (unbounded)
  
  // Command interfaces (write to hardware) - all in SI units
  // Mode 0 (Servo): position, velocity (max speed), acceleration
  double hw_cmd_position_;        // rad (Mode 0 only)
  double hw_cmd_velocity_;        // rad/s (Mode 0: max speed, Mode 1: target velocity)
  double hw_cmd_acceleration_;    // Acceleration value (0-254, 0=no limit) (Mode 0, 1)
  // Mode 2 (PWM): effort
  double hw_cmd_effort_;          // PWM duty cycle -1.0 to +1.0 (Mode 2 only)
  // Emergency stop (all modes)
  double hw_cmd_emergency_stop_;  // 1.0 = emergency stop active, 0.0 = normal operation
  bool emergency_stop_active_;    // Internal flag tracking emergency stop state
  
  // Mock/simulation mode state variables
  double mock_position_;          // Simulated position (rad)
  double mock_velocity_;          // Simulated velocity (rad/s)
  double mock_load_;              // Simulated load (%)
  double mock_temperature_;       // Simulated temperature (°C)
  double mock_voltage_;           // Simulated voltage (V)
  double mock_current_;           // Simulated current (A)
  
  // Unit conversion constants
  static constexpr double STEPS_PER_REVOLUTION = 4096.0;
  static constexpr double STEPS_TO_RAD = (2.0 * M_PI) / STEPS_PER_REVOLUTION;
  static constexpr double RAD_TO_STEPS = STEPS_PER_REVOLUTION / (2.0 * M_PI);
  static constexpr double VOLTAGE_SCALE = 0.1;      // Decivolts to volts
  static constexpr double CURRENT_SCALE = 0.001;    // Milliamps to amps
  static constexpr double LOAD_SCALE = 0.1;         // Raw load to percentage
  
  // STS servo motor limits
  static constexpr int STS_MAX_VELOCITY_STEPS = 3400;  // Maximum velocity in steps/s
  static constexpr int STS_MAX_ACCELERATION = 254;     // Maximum acceleration value
  static constexpr int STS_MAX_POSITION = 4095;        // Maximum position in steps
  static constexpr int STS_MAX_PWM = 1000;             // Maximum PWM value
  static constexpr int STS_MIN_MOTOR_ID = 1;           // Minimum valid motor ID
  static constexpr int STS_MAX_MOTOR_ID = 253;         // Maximum valid motor ID
  
  // Operating mode constants
  static constexpr int MODE_SERVO = 0;      // Position control mode
  static constexpr int MODE_VELOCITY = 1;   // Velocity control mode
  static constexpr int MODE_PWM = 2;        // PWM/effort control mode
  
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
  
  /**
   * @brief Attempt to recover from communication errors
   * 
   * Tries to re-establish communication with the motor by pinging it.
   * If ping fails, attempts to reinitialize the serial connection.
   * 
   * @return true if recovery successful, false otherwise
   */
  bool attempt_error_recovery();
  
  /**
   * @brief Diagnostic callback for hardware status
   * 
   * Updates diagnostic status with motor health information.
   * 
   * @param stat Diagnostic status wrapper to update
   */
  void diagnostics_callback(diagnostic_updater::DiagnosticStatusWrapper & stat);
};

}  // namespace sts_hardware_interface

#endif  // STS_HARDWARE_INTERFACE__STS_HARDWARE_INTERFACE_HPP_
