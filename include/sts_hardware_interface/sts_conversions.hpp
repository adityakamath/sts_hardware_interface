#ifndef STS_HARDWARE_INTERFACE_STS_CONVERSIONS_HPP_
#define STS_HARDWARE_INTERFACE_STS_CONVERSIONS_HPP_

/**
 * @file sts_conversions.hpp
 * @brief Pure conversion free functions for STS servo motor unit translation.
 *
 * All functions are stateless and depend only on their arguments and the
 * compile-time protocol constants below. Extracted here so they can be
 * independently unit-tested without instantiating STSHardwareInterface.
 */

#include <algorithm>
#include <cmath>

namespace sts_hardware_interface
{
namespace conversions
{

// ===== STS protocol constants (same for all STS motors) =====
constexpr double STEPS_PER_REVOLUTION = 4096.0;
constexpr double STEPS_TO_RAD = (2.0 * M_PI) / STEPS_PER_REVOLUTION;
constexpr double RAD_TO_STEPS = STEPS_PER_REVOLUTION / (2.0 * M_PI);
constexpr int    STS_MAX_POSITION     = 4095;   // 12-bit encoder
constexpr int    STS_MAX_PWM          = 1000;   // ±100% duty cycle
constexpr int    STS_MAX_ACCELERATION = 254;    // protocol constant

/**
 * @brief Convert motor steps [0, 4095] to radians [0, 2π] with direction inversion.
 *
 * STS motors increment clockwise; ROS 2 uses counter-clockwise positive (REP-103).
 * Direction is inverted by mapping: raw 0 → ~2π, raw 4095 → 0.
 */
inline double raw_position_to_radians(int raw_position)
{
  return static_cast<double>(STS_MAX_POSITION - raw_position) * STEPS_TO_RAD;
}

/**
 * @brief Convert radians [0, 2π] to motor steps [0, 4095] with direction inversion.
 *
 * Wraps the input modulo 2π before converting. The negative sign inside fmod
 * implements the same direction inversion as raw_position_to_radians.
 */
inline int radians_to_raw_position(double position_rad)
{
  double normalized = std::fmod(-position_rad, 2.0 * M_PI);
  if (normalized < 0.0) {
    normalized += 2.0 * M_PI;
  }
  int raw = static_cast<int>(normalized * RAD_TO_STEPS);
  return std::clamp(raw, 0, STS_MAX_POSITION);
}

/**
 * @brief Convert raw velocity (steps/s) to rad/s with sign inversion.
 *
 * Motor positive direction is clockwise; ROS 2 uses counter-clockwise positive.
 */
inline double raw_velocity_to_rad_s(int raw_velocity)
{
  return static_cast<double>(-raw_velocity) * STEPS_TO_RAD;
}

/**
 * @brief Convert rad/s to raw velocity (steps/s), clamped to ±max_velocity_steps.
 *
 * Applies the same sign inversion as raw_velocity_to_rad_s.
 */
inline int rad_s_to_raw_velocity(double velocity_rad_s, int max_velocity_steps)
{
  double raw = -velocity_rad_s * RAD_TO_STEPS;
  return static_cast<int>(std::clamp(
    raw,
    static_cast<double>(-max_velocity_steps),
    static_cast<double>(max_velocity_steps)));
}

/**
 * @brief Convert normalized effort [-1.0, +1.0] to motor PWM [-1000, +1000].
 */
inline int effort_to_raw_pwm(double effort)
{
  return static_cast<int>(std::clamp(effort, -1.0, 1.0) * STS_MAX_PWM);
}

/**
 * @brief Clamp an acceleration command value to the valid protocol range [0, 254].
 */
inline int clamp_acceleration(double accel_cmd)
{
  return static_cast<int>(
    std::clamp(accel_cmd, 0.0, static_cast<double>(STS_MAX_ACCELERATION)));
}

/**
 * @brief Clamp effort to [-max_effort, max_effort] safety limit.
 *
 * Acts as a limiter without scaling: if max_effort=0.5 and command=0.5,
 * the output is 0.5 (not rescaled to 1.0).
 */
inline double normalize_effort(double effort, double max_effort, bool has_limit)
{
  if (!has_limit) return effort;
  return std::clamp(effort, -max_effort, max_effort);
}

/**
 * @brief Clamp value to [min_val, max_val] when limit is enabled; pass through otherwise.
 */
template<typename T>
inline T apply_limit(T value, T min_val, T max_val, bool has_limit)
{
  return has_limit ? std::clamp(value, min_val, max_val) : value;
}

}  // namespace conversions
}  // namespace sts_hardware_interface

#endif  // STS_HARDWARE_INTERFACE_STS_CONVERSIONS_HPP_
