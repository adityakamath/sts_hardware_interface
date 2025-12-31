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

#include <map>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
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
 * @brief ros2_control SystemInterface for Feetech STS series servo motors (single or chain)
 *
 * Supports controlling a single motor or chain of motors on the same serial bus.
 * Uses SyncWrite functions for efficient multi-motor control.
 *
 * OPERATING MODES:
 * - Mode 0 (Servo): Position control with speed and acceleration limits
 * - Mode 1 (Velocity): Closed-loop velocity control (default)
 * - Mode 2 (PWM): Open-loop PWM/effort control
 *
 * STATE INTERFACES (Read from hardware - all modes, per joint):
 * - position: Current position in radians
 * - velocity: Current velocity in rad/s
 * - load: Motor load/torque as percentage (-100.0 to +100.0)
 * - voltage: Supply voltage in volts
 * - temperature: Internal temperature in degrees Celsius
 * - current: Motor current draw in amperes
 * - is_moving: Motion status (1.0=moving, 0.0=stopped)
 *
 * COMMAND INTERFACES (Write to hardware - mode dependent, per joint):
 * Mode 0: position (rad), velocity (max speed, rad/s), acceleration (0-254)
 * Mode 1: velocity (rad/s), acceleration (0-254)
 * Mode 2: effort (PWM duty cycle, -1.0 to +1.0)
 * All modes: emergency_stop (boolean)
 *
 * PARAMETERS (from ros2_control URDF):
 * - serial_port: Serial port path (e.g., "/dev/ttyACM0") [required]
 * - motor_id: Motor ID on the serial bus (1-253) [required for each joint]
 * - operating_mode: 0=servo, 1=velocity, 2=PWM (default: 1)
 * - baud_rate: Communication baud rate (default: 1000000)
 * - use_sync_write: Enable SyncWrite for multi-motor commands (default: true)
 *
 * SINGLE MOTOR USAGE:
 * @code{.xml}
 * <ros2_control name="my_motor" type="system">
 *   <hardware>
 *     <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
 *     <param name="serial_port">/dev/ttyACM0</param>
 *     <param name="operating_mode">1</param>
 *     <param name="baud_rate">1000000</param>
 *   </hardware>
 *   <joint name="wheel_joint">
 *     <param name="motor_id">1</param>
 *     <command_interface name="velocity"/>
 *     <command_interface name="acceleration"/>
 *     <state_interface name="position"/>
 *     <state_interface name="velocity"/>
 *     ...
 *   </joint>
 * </ros2_control>
 * @endcode
 *
 * MOTOR CHAIN USAGE:
 * @code{.xml}
 * <ros2_control name="motor_chain" type="system">
 *   <hardware>
 *     <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
 *     <param name="serial_port">/dev/ttyACM0</param>
 *     <param name="operating_mode">1</param>
 *     <param name="use_sync_write">true</param>
 *   </hardware>
 *   <joint name="joint1">
 *     <param name="motor_id">1</param>
 *     <command_interface name="velocity"/>
 *     ...
 *   </joint>
 *   <joint name="joint2">
 *     <param name="motor_id">2</param>
 *     <command_interface name="velocity"/>
 *     ...
 *   </joint>
 *   <joint name="joint3">
 *     <param name="motor_id">3</param>
 *     <command_interface name="velocity"/>
 *     ...
 *   </joint>
 * </ros2_control>
 * @endcode
 */

class STSHardwareInterface : public hardware_interface::SystemInterface {
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

private:
  // ===== SHARED STATIC RESOURCES (for serial port sharing) =====
  static std::map<std::string, std::shared_ptr<SMS_STS>> serial_port_connections_;
  static std::mutex serial_port_mutex_;

  // ===== CONFIGURATION PARAMETERS (from URDF) =====
  std::string serial_port_;
  int baud_rate_;
  int communication_timeout_ms_;
  bool enable_multi_turn_;
  bool enable_mock_mode_;
  bool use_sync_write_;  // Use SyncWrite for multi-motor commands

  // ===== HARDWARE COMMUNICATION =====
  std::shared_ptr<SMS_STS> servo_;
  bool owns_serial_connection_;

  // ===== JOINT/MOTOR CONFIGURATION =====
  std::vector<std::string> joint_names_;  // Joint names from URDF
  std::vector<int> motor_ids_;            // Corresponding motor IDs (1-253)
  std::vector<int> operating_modes_;      // Per-joint operating mode (0=servo, 1=velocity, 2=PWM)
  std::vector<bool> reverse_direction_;   // Per-joint direction reversal (applies to both commands and feedback)
  std::map<std::string, size_t> joint_name_to_index_;  // Quick lookup

  // ===== PER-JOINT STATE INTERFACES (indexed by joint) =====
  std::vector<double> hw_state_position_;
  std::vector<double> hw_state_velocity_;
  std::vector<double> hw_state_load_;
  std::vector<double> hw_state_voltage_;
  std::vector<double> hw_state_temperature_;
  std::vector<double> hw_state_current_;
  std::vector<double> hw_state_is_moving_;

  // ===== PER-JOINT COMMAND INTERFACES (indexed by joint) =====
  std::vector<double> hw_cmd_position_;      // Mode 0
  std::vector<double> hw_cmd_velocity_;      // Mode 0, 1
  std::vector<double> hw_cmd_acceleration_;  // Mode 0, 1
  std::vector<double> hw_cmd_effort_;        // Mode 2
  std::vector<double> hw_cmd_emergency_stop_;
  std::vector<bool> emergency_stop_active_;

  // ===== BROADCAST EMERGENCY STOP =====
  double hw_cmd_broadcast_emergency_stop_;  // Stops ALL motors when > 0.5
  bool broadcast_emergency_stop_active_;

  // ===== PER-JOINT HARDWARE LIMITS =====
  std::vector<double> position_min_;
  std::vector<double> position_max_;
  std::vector<double> velocity_max_;
  std::vector<double> effort_max_;
  std::vector<bool> has_position_limits_;
  std::vector<bool> has_velocity_limits_;
  std::vector<bool> has_effort_limits_;

  // ===== PER-JOINT MULTI-TURN TRACKING =====
  std::vector<int> last_raw_position_;
  std::vector<int> revolution_count_;
  std::vector<double> continuous_position_;

  // ===== PER-JOINT POSITION ZEROING =====
  std::vector<double> initial_position_offset_;  // Position offset captured on activation

  // ===== PER-JOINT MOCK MODE STATE =====
  std::vector<double> mock_position_;
  std::vector<double> mock_velocity_;
  std::vector<double> mock_load_;
  std::vector<double> mock_temperature_;
  std::vector<double> mock_voltage_;
  std::vector<double> mock_current_;

  // ===== ERROR TRACKING =====
  int consecutive_read_errors_;
  int consecutive_write_errors_;
  static constexpr int MAX_CONSECUTIVE_ERRORS = 5;

  // ===== PERFORMANCE MONITORING =====
  std::chrono::steady_clock::time_point last_read_time_;
  std::chrono::steady_clock::time_point last_write_time_;
  double read_duration_ms_;
  double write_duration_ms_;
  double max_read_duration_ms_;
  double max_write_duration_ms_;
  unsigned int cycle_count_;
  static constexpr unsigned int PERFORMANCE_LOG_INTERVAL = 1000;

  // ===== DIAGNOSTICS =====
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  
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
  static constexpr int STS_BROADCAST_ID = 0xFE;        // Broadcast ID (254) for all motors
  
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
