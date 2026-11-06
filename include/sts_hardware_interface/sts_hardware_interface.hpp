#ifndef SMS_STS_HARDWARE_INTERFACE_SMS_STS_HARDWARE_INTERFACE_HPP_
#define SMS_STS_HARDWARE_INTERFACE_SMS_STS_HARDWARE_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sts_hardware_interface/srv/one_key_calibration.hpp"
#include "sts_hardware_interface/sts_conversions.hpp"

// Feetech SCServo SDK servo class, defined in <scservo/SMS_STS.h> (included by the .cpp only)
class SMS_STS;

namespace sts_hardware_interface
{

/// Return type for SDK operations: nullopt on success, error string on failure.
using Result = std::optional<std::string>;

/**
 * @brief ros2_control SystemInterface for Feetech STS series servo motors (single motor or chain)
 *
 * Designed and tested with STS3215; other STS models work by setting max_velocity_steps per joint.
 * Several motors on one serial bus are driven with SyncWrite/SyncRead.
 *
 * Operating mode per joint (`operating_mode`): 0 = servo (position, optional velocity and
 * acceleration limits), 1 = velocity (default), 2 = PWM (open-loop effort in [-1, 1]).
 * Every joint exports position, velocity, effort, voltage, temperature, current and is_moving
 * states. A joint declaring no command interfaces is read-only: torque is disabled, but it is
 * still read every cycle (e.g. a teleoperation leader arm).
 *
 * Services: /emergency_stop (std_srvs/SetBool) stops all motors; /one_key_calibration
 * (OneKeyCalibration) sets the current position as the encoder midpoint.
 *
 * Hardware and joint parameters: docs/design.md, section "Configuration Parameters".
 */
// NOTE: internal_max_vel/internal_max_acc/internal_acc_coeff/internal_control_period are
// undocumented registers discovered by community testing (@devemin on X, Sep 2026).
class STSHardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(STSHardwareInterface)

  STSHardwareInterface() : logger_(rclcpp::get_logger("STSHardwareInterface")) {}

  /** @brief Parse URDF parameters and validate configuration */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

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

  /** @brief Export the state interfaces missing from the URDF (all 7 are always exported) */
  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override;

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
  // Caps total time read() may spend across all motors' syncReadPacketRx calls in one cycle
  // (default: 15ms). Without this, each unresponsive motor blocks up to communication_timeout_ms_
  // on its own, and those stack per-cycle - one bad motor can already blow a 50Hz/20ms control
  // loop's budget outright. Once exceeded, remaining motors this cycle are treated as failed
  // reads (consecutive_read_errors_ incremented, last known state kept) instead of being polled.
  int read_cycle_budget_ms_;
  bool enable_mock_mode_;
  bool use_sync_write_;  // Use SyncWrite for multi-motor commands

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
  bool hold_position_on_activate_;  // Seed servo position commands from the real position on activation (default: false)

  // ===== LOGGING =====
  rclcpp::Logger logger_;
  rclcpp::Clock throttle_clock_{RCL_SYSTEM_TIME};  // Reusable clock for RCLCPP_*_THROTTLE macros

  // ===== ROS 2 NODE AND COMMUNICATION =====
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_service_;

  // ===== HARDWARE COMMUNICATION =====
  std::shared_ptr<SMS_STS> servo_;

  // ===== PRE-COMPUTED MOTOR GROUPINGS (set once in on_init) =====
  std::vector<size_t> servo_motor_indices_;     // MODE_SERVO joint indices (readonly joints excluded)
  std::vector<size_t> velocity_motor_indices_;  // MODE_VELOCITY joint indices (readonly joints excluded)
  std::vector<size_t> pwm_motor_indices_;       // MODE_PWM joint indices (readonly joints excluded)

  // ===== PRE-ALLOCATED SYNCWRITE BUFFERS (sized once, values updated each cycle) =====
  std::vector<std::uint8_t> servo_sync_ids_;
  std::vector<std::int16_t> servo_sync_positions_;
  std::vector<std::uint16_t> servo_sync_speeds_;
  std::vector<std::uint8_t> servo_sync_accelerations_;
  std::vector<double> servo_sync_deltas_;   // |target_position - current_position| per joint (rad)
  std::vector<std::uint8_t> velocity_sync_ids_;
  std::vector<std::int16_t> velocity_sync_velocities_;
  std::vector<std::uint8_t> velocity_sync_accelerations_;
  std::vector<double> velocity_sync_deltas_;  // |target_velocity - current_velocity| per joint (rad/s)
  std::vector<std::uint8_t> pwm_sync_ids_;
  std::vector<std::int16_t> pwm_sync_pwm_values_;

  // ===== PRE-ALLOCATED SYNCREAD ID LIST (all motors, readonly included - read() polls every
  // motor regardless of write eligibility; buffer itself is allocated once via syncReadBegin
  // in on_configure, freed via syncReadEnd in on_cleanup/on_shutdown) =====
  std::vector<std::uint8_t> read_sync_ids_;

  // ===== PER-JOINT STATE / COMMAND HANDLES (framework-owned, indexed by [kind][joint]) =====
  // State handles always exist (URDF-declared or exported as unlisted); a command handle is null
  // when the interface is not declared in the URDF, in which case cmd_val() returns cmd_default().
  enum StateKind : size_t {
    ST_POSITION, ST_VELOCITY, ST_EFFORT, ST_VOLTAGE, ST_TEMPERATURE, ST_CURRENT, ST_IS_MOVING,
    STATE_KIND_COUNT
  };
  enum CmdKind : size_t {
    CMD_POSITION, CMD_VELOCITY, CMD_ACCELERATION, CMD_EFFORT,
    CMD_KIND_COUNT
  };
  static constexpr std::array<const char *, STATE_KIND_COUNT> kStateNames = {
    "position", "velocity", "effort", "voltage", "temperature", "current", "is_moving"};
  static constexpr std::array<const char *, CMD_KIND_COUNT> kCommandNames = {
    "position", "velocity", "acceleration", "effort"};

  std::array<std::vector<hardware_interface::StateInterface::SharedPtr>, STATE_KIND_COUNT>
  state_handles_;
  std::array<std::vector<hardware_interface::CommandInterface::SharedPtr>, CMD_KIND_COUNT>
  cmd_handles_;
  bool handles_bound_ = false;  // initial values are written on the first bind only

  /** @brief Fetch the framework-owned handles (call in on_configure, after interfaces are exported) */
  void bind_handles();
  /** @brief Value of a command that is undeclared or not yet written */
  double cmd_default(CmdKind kind, size_t i) const;
  /** @brief Read a joint state (0.0 if the handle is absent) */
  double state_val(StateKind kind, size_t i) const;
  /** @brief Write a joint state (no-op if the handle is absent) */
  void put_state(StateKind kind, size_t i, double value);
  /** @brief Read a joint command (cmd_default() if the interface is not declared) */
  double cmd_val(CmdKind kind, size_t i) const;
  /** @brief Write a joint command (no-op if the interface is not declared) */
  void put_cmd(CmdKind kind, size_t i, double value);

  // ===== EMERGENCY STOP =====
  // Both flags are only touched from the control thread: the service callback runs inside
  // read() (rclcpp::spin_some), and write() applies the request. Make them atomic if the
  // services are ever spun from another thread.
  bool emergency_stop_requested_;  // Set by the /emergency_stop service
  bool emergency_stop_active_;     // Applied in write(): motors stopped, commands ignored

  // ===== ERROR TRACKING =====
  int consecutive_read_errors_;
  int consecutive_write_errors_;
  static constexpr int MAX_CONSECUTIVE_ERRORS = 5;
  /// Default per-command ACC (protocol units, 0-254) for position-mode joints with no
  /// "acceleration" command interface (the command would then never be written and would
  /// otherwise stay at its default, 0). Confirmed empirically against a real servo, ROS
  /// uninvolved: ACC=0 does NOT mean "instant" the way Speed=0 means "max" - the servo ignores
  /// the commanded Speed entirely and creeps at a fixed slow internal rate regardless of the
  /// Speed value. The same move with ACC=100 converged in a fraction of the time, and Speed then
  /// actually took effect. 100 is the value tested working (clean, no overshoot); not swept to
  /// find a lower bound.
  static constexpr double DEFAULT_ACCELERATION = 100.0;
  // Ping retry/backoff, configurable via node parameters (see on_configure()).
  int configure_ping_retry_attempts_ = 5;
  int configure_ping_retry_delay_ms_ = 20;
  int recovery_ping_retry_attempts_ = 5;
  int recovery_ping_retry_delay_ms_ = 20;

  // Unit conversion constants (step/rad conversions live in sts_conversions.hpp)
  static constexpr double VOLTAGE_SCALE = 0.1;      // 1 unit = 0.1V (Feetech spec)
  static constexpr double CURRENT_SCALE = 0.0065;   // 1 unit = 6.5mA (Feetech spec)
  static constexpr double LOAD_SCALE = 0.1;         // 1 unit = 0.1% (Feetech spec)

  // STS protocol constants (same for all STS motors)
  static constexpr int SMS_STS_MAX_ACCELERATION = 254;     // Maximum acceleration value (protocol constant)
  static constexpr int SMS_STS_MIN_MOTOR_ID = 1;           // Minimum valid motor ID
  static constexpr int SMS_STS_MAX_MOTOR_ID = 253;         // Maximum valid motor ID
  static constexpr int BROADCAST_ID = 0xFE;                // Broadcast ID (254) for all motors

  // Default max_velocity_steps when a joint does not override it (STS3215 spec; motor-model-dependent)
  static constexpr int DEFAULT_MAX_VELOCITY_STEPS = 3400;

  // Operating mode constants
  static constexpr int MODE_SERVO = 0;      // Position control mode
  static constexpr int MODE_VELOCITY = 1;   // Velocity control mode
  static constexpr int MODE_PWM = 2;        // PWM/effort control mode

  /// Optional servo EEPROM settings, written in on_configure() when present in the URDF.
  /// Order matches eeprom_specs().
  enum EepromReg : size_t {
    EEPROM_P_COEF, EEPROM_D_COEF, EEPROM_I_COEF,
    EEPROM_PROTECTION_CURRENT, EEPROM_OVERLOAD_TORQUE, EEPROM_RETURN_DELAY, EEPROM_DEADBAND,
    EEPROM_INTERNAL_MAX_VEL, EEPROM_INTERNAL_MAX_ACC, EEPROM_INTERNAL_ACC_COEFF,
    EEPROM_INTERNAL_CONTROL_PERIOD,
    EEPROM_REG_COUNT
  };

  /// How one EEPROM setting is parsed, validated and written.
  struct EepromSpec
  {
    EepromReg reg;
    const char * param;      ///< Joint parameter name in the URDF
    int min_value;
    int max_value;
    unsigned mode_mask;      ///< Bit m set: the register applies in operating mode m
    const char * sdk_call;   ///< SDK function name, for error messages
    int (* write)(SMS_STS & servo, int id, int mode, int value);
  };
  static const std::array<EepromSpec, EEPROM_REG_COUNT> & eeprom_specs();

  /// Per-joint configuration parsed from the URDF in on_init().
  struct JointConfig
  {
    std::string name;
    int motor_id = 0;                          // 1-253
    int operating_mode = MODE_VELOCITY;
    bool readonly = false;                     // No command interfaces: torque off, excluded from write loops
    double position_min = 0.0;                 // rad
    double position_max = 2.0 * M_PI;          // rad
    bool has_position_limits = false;
    double velocity_max = DEFAULT_MAX_VELOCITY_STEPS * conversions::STEPS_TO_RAD;  // rad/s
    bool has_velocity_limits = false;
    double effort_max = 1.0;
    bool has_effort_limits = false;
    int position_center = conversions::STS_DEFAULT_CENTER;  // Raw step for 0 rad
    int max_velocity_steps = DEFAULT_MAX_VELOCITY_STEPS;    // Motor max velocity, steps/s (model-dependent)
    std::array<std::optional<int>, EEPROM_REG_COUNT> eeprom;  // Unset = leave the servo's value alone
  };

  std::vector<JointConfig> joints_;  // One entry per URDF joint, in URDF order

  /** @brief Seed the position command of every commanded MODE_SERVO joint from the motor's real
   *  position (retrying each motor). Returns false if any motor cannot be read. */
  bool seed_servo_commands_from_feedback();

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

  /** @brief Mock-mode equivalent: sets simulated position to the encoder midpoint, no real servo I/O */
  void process_pending_one_key_calibration_mock();

  struct PendingCalibrationRequest
  {
    std::vector<size_t> motor_indices;
    std::vector<bool> was_torque_enabled;
  };

  std::mutex calibration_mutex_;  // Guards the two members below; never taken unless calibration_queued_
  std::atomic<bool> calibration_queued_{false};  // Lets write() skip the mutex when nothing is queued
  std::optional<PendingCalibrationRequest> pending_calibration_request_;
  bool calibration_in_progress_{false};

  rclcpp::Service<sts_hardware_interface::srv::OneKeyCalibration>::SharedPtr one_key_calibration_service_;
};

}  // namespace sts_hardware_interface

#endif  // SMS_STS_HARDWARE_INTERFACE_SMS_STS_HARDWARE_INTERFACE_HPP_
