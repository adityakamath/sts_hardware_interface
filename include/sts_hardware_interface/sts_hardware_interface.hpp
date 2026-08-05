#ifndef STS_HARDWARE_INTERFACE_STS_HARDWARE_INTERFACE_HPP_
#define STS_HARDWARE_INTERFACE_STS_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <optional>
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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sts_hardware_interface/srv/one_key_calibration.hpp"

// SCServo SDK
#include <scservo/SMS_STS.h>

// Conversion utilities
#include "sts_hardware_interface/sts_conversions.hpp"

namespace sts_hardware_interface
{

/// Return type for SDK operations: nullopt on success, error string on failure.
using Result = std::optional<std::string>;

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
 * Mode 0: position (rad) [required], velocity (max speed, rad/s) [optional], acceleration (0-254) [optional]
 * Mode 1: velocity (rad/s) [required], acceleration (0-254) [optional]
 * Mode 2: effort (PWM duty cycle, -1.0 to +1.0) [required]
 *
 * READONLY JOINTS:
 * A joint with no <command_interface> entries in the URDF is treated as readonly.
 * Readonly joints have torque disabled at activation and are excluded from all write
 * loops, but are read every cycle and export all 7 state interfaces normally.
 * Intended for teleoperation leader arms: the arm is moved by hand, its joint positions
 * flow out through /joint_states, and a teleoperation node forwards them as commands
 * to a follower arm. Both arms can coexist in a single ros2_control system / URDF.
 *
 * EMERGENCY STOP:
 * Emergency stop functionality is triggered via ROS 2 service (not a command interface):
 *    Service: /emergency_stop (std_srvs/SetBool)
 *    Activate: ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"
 *    Release:  ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: false}"
 * When activated, ALL motors stop immediately in both real and mock modes.
 * The hardware interface creates an internal node and service server during on_configure().
 *
 * HARDWARE PARAMETERS (from ros2_control URDF):
 * - serial_port: Serial port path (e.g., "/dev/ttyACM0") [required]
 * - baud_rate: Communication baud rate, 9600-1000000 (default: 1000000)
 * - communication_timeout_ms: Serial communication timeout, 1-1000 ms (default: 100)
 * - use_sync_write: Enable SyncWrite for multi-motor commands (default: true)
 * - enable_mock_mode: Enable simulation mode without hardware (default: false)
 * - max_velocity_steps: Maximum motor velocity in steps/s (default: 3400, STS3215 spec)
 *                       Adjust for other models: STS3032=2900, STS3235=3400
 * - proportional_vel_max: SyncWrite only. Velocity [0–max_velocity_steps] assigned to the joint
 *                         with the largest |target_position - current_position| delta; all others
 *                         scaled proportionally. Set to 0 to disable. (default: 0)
 * - proportional_vel_deadband: SyncWrite only. Min max-delta (rad) below which all joints revert
 *                              to their commanded velocity (steady-state hold). (default: 0.01)
 * - proportional_acc_max: SyncWrite only. Acceleration [0–254] assigned to the wheel with the
 *                         largest |target_velocity - current_velocity| delta; others scaled
 *                         proportionally. Set to 0 to disable. (default: 100)
 * - proportional_acc_deadband: SyncWrite only. Min max-delta (rad/s) below which ACC 0 is sent
 *                              to all wheels (steady-state cruise). (default: 0.05)
 * - reset_states_on_activate: Reset position/velocity states to zero on activation (default: true)
 *
 * JOINT PARAMETERS (from ros2_control URDF, per joint):
 * - motor_id: Motor ID on the serial bus (1-253) [required]
 * - operating_mode: 0=servo, 1=velocity, 2=PWM (default: 1)
 * - min_position: Minimum position limit in radians (default: 0.0)
 * - max_position: Maximum position limit in radians (default: 6.283, 2π)
 * - position_center_steps: Raw encoder step that maps to 0 rad, 0-4095
 *                          (default: 4095). Only used in position mode (mode 0). Set to your
 *                          servo's calibrated center; e.g. 2048 for approximately [-π, +π]
 *                          range (one encoder step of asymmetry at the ±π boundary).
 * - max_velocity: Maximum velocity limit in rad/s (default: 5.22, STS3215 max: 3400 steps/s)
 *                 NOTE: Adjust this for other STS motors (e.g., STS3032 max: 2900 steps/s)
 * - max_effort: Maximum effort limit, 0-1 for PWM mode (default: 1.0)
 * - p_coefficient: Proportional gain written to EEPROM (0-255, optional — omit to preserve existing value)
 *                  Mode 0 → addr 21 (SMS_STS_MODE0_P_COEF); Mode 1 → addr 37 (SMS_STS_MODE1_P_COEF); Mode 2: ignored
 * - d_coefficient: Derivative gain written to EEPROM   (0-255, optional — omit to preserve existing value)
 *                  Mode 0 → addr 22 (SMS_STS_MODE0_D_COEF); Mode 1: ignored (PI only); Mode 2: ignored
 * - i_coefficient: Integral gain written to EEPROM     (0-255, optional — omit to preserve existing value)
 *                  Mode 0 → addr 23 (SMS_STS_MODE0_I_COEF); Mode 1 → addr 39 (SMS_STS_MODE1_I_COEF); Mode 2: ignored
 * - protection_current: Hardware current cutoff written to EEPROM (0-65535, 6.5mA units, optional — omit to preserve
 *                       existing value). When the servo exceeds this current for longer than the overload time, the
 *                       servo firmware itself cuts torque. All modes. addr 28/29 (uint16, writeWord).
 * - overload_torque: Load percentage threshold that triggers overload protection (0-254, optional — omit to preserve
 *                   existing value). All modes. addr 36 (uint8, writeByte).
 * - return_delay: Delay before the servo sends a response to each command (0-254, 2µs units, optional — omit to
 *                preserve existing value). All modes. addr 7 (uint8, writeByte).
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

  // Proportional acceleration parameters (SyncWriteSpe velocity path only)
  int proportional_acc_max_;             // ACC given to the wheel with the largest Δv [0-254] (default: 100)
  double proportional_acc_deadband_rad_s_;  // Min Δv below which all ACC → 0 (default: 0.05 rad/s)

  // Proportional velocity parameters (SyncWritePosEx servo path only)
  int proportional_vel_max_;  // max_speed (steps/s) given to the joint with the largest Δpos; others scaled
                              // proportionally so all joints finish moving at the same time.
                              // Set to 0 to disable (each joint uses its commanded velocity). (default: 0)
  double proportional_vel_deadband_rad_;  // Min Δpos below which all joints revert to commanded velocity
                                          // (steady-state hold — avoids noise-driven re-scaling). (default: 0.01 rad)

  // Lifecycle parameter
  bool reset_states_on_activate_;  // Reset position/velocity states on activation (default: true)

  // ===== LOGGING =====
  rclcpp::Logger logger_;
  rclcpp::Clock throttle_clock_{RCL_SYSTEM_TIME};  // Reusable clock for RCLCPP_*_THROTTLE macros

  // ===== ROS 2 NODE AND COMMUNICATION =====
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_service_;

  // ===== HARDWARE COMMUNICATION =====
  std::shared_ptr<SMS_STS> servo_;

  // ===== JOINT/MOTOR CONFIGURATION =====
  std::vector<std::string> joint_names_;  // Joint names from URDF
  std::vector<int> motor_ids_;            // Corresponding motor IDs (1-253)
  std::vector<int> operating_modes_;      // Per-joint operating mode (0=servo, 1=velocity, 2=PWM)
  std::map<std::string, size_t> joint_name_to_index_;  // Quick lookup

  // ===== PRE-COMPUTED MOTOR GROUPINGS (set once in on_init) =====
  std::vector<bool> is_readonly_;               // True if joint has no command interfaces (torque disabled, excluded from write loops)
  std::vector<size_t> servo_motor_indices_;     // MODE_SERVO joint indices (readonly joints excluded)
  std::vector<size_t> velocity_motor_indices_;  // MODE_VELOCITY joint indices (readonly joints excluded)
  std::vector<size_t> pwm_motor_indices_;       // MODE_PWM joint indices (readonly joints excluded)

  // ===== PRE-ALLOCATED SYNCWRITE BUFFERS (sized once, values updated each cycle) =====
  std::vector<u8> servo_sync_ids_;
  std::vector<s16> servo_sync_positions_;
  std::vector<u16> servo_sync_speeds_;
  std::vector<u8> servo_sync_accelerations_;
  std::vector<double> servo_sync_deltas_;   // |target_position - current_position| per joint (rad)
  std::vector<u8> velocity_sync_ids_;
  std::vector<s16> velocity_sync_velocities_;
  std::vector<u8> velocity_sync_accelerations_;
  std::vector<double> velocity_sync_deltas_;  // |target_velocity - current_velocity| per joint (rad/s)
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
  std::vector<int> position_center_;  // Raw step for 0 rad per joint (default: STS_DEFAULT_CENTER=4095)

  // ===== PER-JOINT PID COEFFICIENTS (optional, written to EEPROM in on_configure) =====
  std::vector<std::optional<int>> p_coefficient_;  // Proportional gain (0-255): Mode 0 → SMS_STS_MODE0_P_COEF (addr 21); Mode 1 → SMS_STS_MODE1_P_COEF (addr 37)
  std::vector<std::optional<int>> d_coefficient_;  // Derivative gain   (0-255): Mode 0 → SMS_STS_MODE0_D_COEF (addr 22) only; Mode 1/2: ignored
  std::vector<std::optional<int>> i_coefficient_;  // Integral gain     (0-255): Mode 0 → SMS_STS_MODE0_I_COEF (addr 23); Mode 1 → SMS_STS_MODE1_I_COEF (addr 39)

  // ===== PER-JOINT HARDWARE PROTECTION PARAMETERS (optional, written to EEPROM in on_configure) =====
  std::vector<std::optional<int>> protection_current_;  // Hardware current cutoff (0-65535, 6.5mA/unit): addr 28/29 (uint16, writeWord); all modes
  std::vector<std::optional<int>> overload_torque_;     // Load threshold triggering overload protection (0-254): addr 36 (uint8, writeByte); all modes
  std::vector<std::optional<int>> return_delay_;        // Response delay (0-254, 2µs/unit): addr 7 (uint8, writeByte); all modes
  std::vector<std::optional<int>> deadband_;            // Position insensitive-area (0-255): addr 26/27 (CW_DEAD/CCW_DEAD, both uint8, writeByte); all modes

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

  /** @brief Attempt to recover from communication errors by pinging motors */
  bool attempt_error_recovery();

  /** @brief Stop a motor based on its operating mode */
  int stop_motor(size_t idx, int acceleration = 0);

  /** @brief Check a write SDK return code; logs, tracks recovery, returns error on failure */
  [[nodiscard]] Result check_write(int result, size_t idx, const char* operation);

  /** @brief Parse a boolean hardware parameter with default value */
  bool parse_bool_param(const std::string& key, bool default_value) const;

  /** @brief Emergency stop service callback */
  void emergency_stop_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  /** @brief Queue one-key calibration requests for safe execution in write() thread */
  void one_key_calibration_callback(
    const sts_hardware_interface::srv::OneKeyCalibration::Request::SharedPtr req,
    sts_hardware_interface::srv::OneKeyCalibration::Response::SharedPtr res);

  /** @brief Execute a queued one-key calibration request inside write() */
  void process_pending_one_key_calibration();

  struct PendingCalibrationRequest
  {
    std::vector<size_t> motor_indices;
    std::vector<bool> was_torque_enabled;
  };

  std::mutex calibration_mutex_;
  std::optional<PendingCalibrationRequest> pending_calibration_request_;
  bool calibration_in_progress_{false};

  rclcpp::Service<sts_hardware_interface::srv::OneKeyCalibration>::SharedPtr one_key_calibration_service_;
};

}  // namespace sts_hardware_interface

#endif  // STS_HARDWARE_INTERFACE_STS_HARDWARE_INTERFACE_HPP_
