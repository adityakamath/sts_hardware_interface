#ifndef STS_HARDWARE_INTERFACE_STS_HARDWARE_INTERFACE_HPP_
#define STS_HARDWARE_INTERFACE_STS_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

// SCServo SDK
#include "SMS_STS.h"

namespace sts_hardware_interface
{

/**
 * @brief ros2_control SystemInterface for Feetech STS series servo motors (single or chain)
 *
 * Designed and tested with STS3215 servo motors. Compatible with other STS series motors
 * (STS3032, STS3235, etc.) but may require adjusting motor-specific parameters like
 * max_velocity to match the maximum speed in steps/s for your specific motor model.
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
 * - effort: Normalized motor load (-1.0 to +1.0, scaled by effort_max)
 * - voltage: Supply voltage in volts
 * - temperature: Internal temperature in degrees Celsius
 * - current: Motor current draw in amperes
 * - is_moving: Motion status (1.0=moving, 0.0=stopped)
 *
 * COMMAND INTERFACES (Write to hardware - mode dependent, per joint):
 * Mode 0: position (rad), velocity (max speed, rad/s), acceleration (0-254)
 * Mode 1: velocity (rad/s), acceleration (0-254)
 * Mode 2: effort (PWM duty cycle, -1.0 to +1.0)
 * Broadcast: emergency_stop (boolean, stops ALL motors)
 *
 * EMERGENCY STOP:
 * The emergency_stop command interface can be triggered via:
 * 1. Command interface: Set emergency_stop > 0.5 programmatically
 * 2. ROS 2 topic: Publish to /emergency_stop (std_msgs/Bool)
 *    - Example: ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true"
 * When activated, ALL motors stop immediately in both real and mock modes.
 * The hardware interface creates an internal node and subscriber during on_configure().
 *
 * HARDWARE PARAMETERS (from ros2_control URDF):
 * - serial_port: Serial port path (e.g., "/dev/ttyACM0") [required]
 * - baud_rate: Communication baud rate, 9600-1000000 (default: 1000000)
 * - communication_timeout_ms: Serial communication timeout, 1-1000 ms (default: 100)
 * - use_sync_write: Enable SyncWrite for multi-motor commands (default: true)
 * - enable_mock_mode: Enable simulation mode without hardware (default: false)
 * - max_velocity_steps: Maximum motor velocity in steps/s (default: 3400, STS3215 spec)
 *                       Adjust for other models: STS3032=2900, STS3235=3400
 *
 * JOINT PARAMETERS (from ros2_control URDF, per joint):
 * - motor_id: Motor ID on the serial bus (1-253) [required]
 * - operating_mode: 0=servo, 1=velocity, 2=PWM (default: 1)
 * - min_position: Minimum position limit in radians (default: 0.0)
 * - max_position: Maximum position limit in radians (default: 6.283, 2π)
 * - max_velocity: Maximum velocity limit in rad/s (default: 5.22, STS3215 max: 3400 steps/s)
 *                 NOTE: Adjust this for other STS motors (e.g., STS3032 max: 2900 steps/s)
 * - max_effort: Maximum effort limit, 0-1 for PWM mode (default: 1.0)
**/
class STSHardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(STSHardwareInterface)

  STSHardwareInterface() : logger_(rclcpp::get_logger("STSHardwareInterface")) {}

  /** @brief Parse URDF parameters and validate configuration */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  /** @brief Initialize serial communication and verify motors */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Set operating modes, enable torque, and read initial states */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Disable motor torque and stop all motion */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Close serial port and cleanup resources */
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Emergency shutdown and resource cleanup */
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Handle error state with recovery attempts */
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Export state interfaces for all joints */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /** @brief Export command interfaces for all joints */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /** @brief Read motor states from hardware */
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /** @brief Write motor commands to hardware */
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ===== CONFIGURATION PARAMETERS (from URDF) =====
  std::string serial_port_;
  int baud_rate_;
  int communication_timeout_ms_;
  bool enable_mock_mode_;
  bool use_sync_write_;  // Use SyncWrite for multi-motor commands

  // Motor-specific parameter (model-dependent)
  int max_velocity_steps_;   // Maximum velocity in steps/s (default: 3400 for STS3215)

  // ===== LOGGING =====
  rclcpp::Logger logger_;
  rclcpp::Clock throttle_clock_{RCL_SYSTEM_TIME};  // Reusable clock for RCLCPP_*_THROTTLE macros

  // ===== ROS 2 NODE AND COMMUNICATION =====
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscriber_;

  // ===== HARDWARE COMMUNICATION =====
  std::shared_ptr<SMS_STS> servo_;

  // ===== JOINT/MOTOR CONFIGURATION =====
  std::vector<std::string> joint_names_;  // Joint names from URDF
  std::vector<int> motor_ids_;            // Corresponding motor IDs (1-253)
  std::vector<int> operating_modes_;      // Per-joint operating mode (0=servo, 1=velocity, 2=PWM)
  std::map<std::string, size_t> joint_name_to_index_;  // Quick lookup

  // ===== PRE-COMPUTED MOTOR GROUPINGS (set once in on_init) =====
  std::vector<size_t> servo_motor_indices_;     // MODE_SERVO joint indices
  std::vector<size_t> velocity_motor_indices_;  // MODE_VELOCITY joint indices
  std::vector<size_t> pwm_motor_indices_;       // MODE_PWM joint indices

  // ===== PRE-ALLOCATED SYNCWRITE BUFFERS (sized once, values updated each cycle) =====
  std::vector<u8> servo_sync_ids_;
  std::vector<s16> servo_sync_positions_;
  std::vector<u16> servo_sync_speeds_;
  std::vector<u8> servo_sync_accelerations_;
  std::vector<u8> velocity_sync_ids_;
  std::vector<s16> velocity_sync_velocities_;
  std::vector<u8> velocity_sync_accelerations_;
  std::vector<u8> pwm_sync_ids_;
  std::vector<s16> pwm_sync_pwm_values_;

  // ===== PER-JOINT STATE INTERFACES (indexed by joint) =====
  std::vector<double> hw_state_position_;
  std::vector<double> hw_state_velocity_;
  std::vector<double> hw_state_effort_;
  std::vector<double> hw_state_voltage_;
  std::vector<double> hw_state_temperature_;
  std::vector<double> hw_state_current_;
  std::vector<double> hw_state_is_moving_;

  // ===== PER-JOINT COMMAND INTERFACES (indexed by joint) =====
  std::vector<double> hw_cmd_position_;      // Mode 0
  std::vector<double> hw_cmd_velocity_;      // Mode 0, 1
  std::vector<double> hw_cmd_acceleration_;  // Mode 0, 1
  std::vector<double> hw_cmd_effort_;        // Mode 2

  // ===== BROADCAST EMERGENCY STOP =====
  double hw_cmd_emergency_stop_;  // Stops ALL motors when > 0.5
  bool emergency_stop_active_;

  // ===== PER-JOINT HARDWARE LIMITS =====
  std::vector<double> position_min_;
  std::vector<double> position_max_;
  std::vector<double> velocity_max_;
  std::vector<double> effort_max_;
  std::vector<bool> has_position_limits_;
  std::vector<bool> has_velocity_limits_;
  std::vector<bool> has_effort_limits_;

  // ===== ERROR TRACKING =====
  int consecutive_read_errors_;
  int consecutive_write_errors_;
  static constexpr int MAX_CONSECUTIVE_ERRORS = 5;

  // Unit conversion constants
  static constexpr double STEPS_PER_REVOLUTION = 4096.0;
  static constexpr double STEPS_TO_RAD = (2.0 * M_PI) / STEPS_PER_REVOLUTION;
  static constexpr double RAD_TO_STEPS = STEPS_PER_REVOLUTION / (2.0 * M_PI);
  static constexpr double VOLTAGE_SCALE = 0.1;      // 1 unit = 0.1V (Feetech spec)
  static constexpr double CURRENT_SCALE = 0.0065;   // 1 unit = 6.5mA (Feetech spec)
  static constexpr double LOAD_SCALE = 0.1;         // 1 unit = 0.1% (Feetech spec)

  // STS protocol constants (same for all STS motors)
  static constexpr int STS_MAX_ACCELERATION = 254;     // Maximum acceleration value (protocol constant)
  static constexpr int STS_MAX_POSITION = 4095;        // Maximum position in steps (12-bit encoder)
  static constexpr int STS_MAX_PWM = 1000;             // Maximum PWM value
  static constexpr int STS_MIN_MOTOR_ID = 1;           // Minimum valid motor ID
  static constexpr int STS_MAX_MOTOR_ID = 253;         // Maximum valid motor ID
  static constexpr int STS_BROADCAST_ID = 0xFE;        // Broadcast ID (254) for all motors

  // Operating mode constants
  static constexpr int MODE_SERVO = 0;      // Position control mode
  static constexpr int MODE_VELOCITY = 1;   // Velocity control mode
  static constexpr int MODE_PWM = 2;        // PWM/effort control mode

  /** @brief Convert motor steps to radians (0-2π) */
  double raw_position_to_radians(int raw_position) const;

  /** @brief Convert motor velocity (steps/s) to rad/s */
  double raw_velocity_to_rad_s(int raw_velocity) const;

  /** @brief Convert rad/s to motor velocity (steps/s) */
  int rad_s_to_raw_velocity(double velocity_rad_s) const;

  /** @brief Convert radians to motor steps */
  int radians_to_raw_position(double position_rad) const;

  /** @brief Convert effort (-1.0 to +1.0) to motor PWM (-1000 to +1000) */
  int effort_to_raw_pwm(double effort) const;

  /** @brief Attempt to recover from communication errors by pinging motors */
  bool attempt_error_recovery();

  /** @brief Stop a motor based on its operating mode */
  int stop_motor(size_t idx, int acceleration = 0);

  /** @brief Handle write operation errors with logging and recovery */
  bool handle_write_error(int result, size_t idx, const char* operation);

  /** @brief Clamp a value to [min_val, max_val] if limit is enabled */
  template<typename T>
  T apply_limit(T value, T min_val, T max_val, bool has_limit) const {
    if (!has_limit) return value;
    return std::clamp(value, min_val, max_val);
  }

  /** @brief Clamp acceleration command to valid range [0, STS_MAX_ACCELERATION] */
  int clamp_acceleration(size_t idx) const;

  /** @brief Apply effort limit and normalize to [-1.0, 1.0] for PWM conversion */
  double normalize_effort(size_t idx) const;

  /** @brief Parse a boolean hardware parameter with default value */
  bool parse_bool_param(const std::string& key, bool default_value) const;

  /** @brief Emergency stop topic callback */
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace sts_hardware_interface

#endif  // STS_HARDWARE_INTERFACE_STS_HARDWARE_INTERFACE_HPP_
