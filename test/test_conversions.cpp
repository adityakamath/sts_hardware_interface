// Copyright 2026 Aditya Kamath
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "sts_hardware_interface/sts_conversions.hpp"

using namespace sts_hardware_interface::conversions;

// Reasonable max_velocity_steps for roundtrip tests (>= max raw value tested)
static constexpr int TEST_MAX_VEL_STEPS = 4095;

// ---- raw_position_to_radians ----

/** @brief raw=0 → maximum angle; raw=4095 → 0; raw=2048 → midpoint */
TEST(ConversionsTest, RawPositionToRadians_KnownValues) {
  EXPECT_NEAR(raw_position_to_radians(0), 4095.0 * (2.0 * M_PI / 4096.0), 1e-6);
  EXPECT_NEAR(raw_position_to_radians(4095), 0.0, 1e-6);
  EXPECT_NEAR(raw_position_to_radians(2048), 2047.0 * (2.0 * M_PI / 4096.0), 1e-6);
}

/** @brief With the unified formula, roundtrip error is ≤1 step (quantization only) */
TEST(ConversionsTest, RawPositionToRadians_Roundtrip) {
  for (int raw = 0; raw <= 4095; raw += 100) {
    double rad = raw_position_to_radians(raw);
    int recovered = radians_to_raw_position(rad);
    EXPECT_LE(std::abs(recovered - raw), 1) << "Roundtrip error exceeds 1 step at raw=" << raw;
  }
}

// ---- radians_to_raw_position ----

/** @brief 0 rad maps to center step (4095 by default) */
TEST(ConversionsTest, RadiansToRawPosition_Zero) {
  EXPECT_EQ(radians_to_raw_position(0.0), 4095);
}

/** @brief 2π, large positive, negative, and ±∞ all clamp to [0, 4095] */
TEST(ConversionsTest, RadiansToRawPosition_StaysInRange) {
  const double inf = std::numeric_limits<double>::infinity();
  for (double rad : {2.0 * M_PI, 100.0, -1.0, inf, -inf}) {
    int result = radians_to_raw_position(rad);
    EXPECT_GE(result, 0) << "out of range for rad=" << rad;
    EXPECT_LE(result, 4095) << "out of range for rad=" << rad;
  }
}

// ---- raw_velocity_to_rad_s ----

/** @brief Raw velocity to rad/s: zero */
TEST(ConversionsTest, RawVelocityToRadS_Zero) {
  EXPECT_NEAR(raw_velocity_to_rad_s(0), 0.0, 1e-9);
}

/** @brief Positive raw → negative rad/s; negative raw → positive rad/s */
TEST(ConversionsTest, RawVelocityToRadS_SignInversion) {
  EXPECT_LT(raw_velocity_to_rad_s(100), 0.0);
  EXPECT_GT(raw_velocity_to_rad_s(-100), 0.0);
}

/** @brief Convert raw → rad/s → raw; should recover original value */
TEST(ConversionsTest, RawVelocityToRadS_Roundtrip) {
  // Pass TEST_MAX_VEL_STEPS so the clamp never truncates within test range.
  for (int raw : {-500, -100, 0, 100, 500}) {
    double rad_s = raw_velocity_to_rad_s(raw);
    int recovered = rad_s_to_raw_velocity(rad_s, TEST_MAX_VEL_STEPS);
    EXPECT_EQ(recovered, raw) << "Failed roundtrip at raw=" << raw;
  }
}

// ---- rad_s_to_raw_velocity ----

/** @brief Rad/sto raw velocity: zero */
TEST(ConversionsTest, RadSToRawVelocity_Zero) {
  EXPECT_EQ(rad_s_to_raw_velocity(0.0, TEST_MAX_VEL_STEPS), 0);
}

/** @brief Positive rad/s → negative raw; negative rad/s → positive raw */
TEST(ConversionsTest, RadSToRawVelocity_SignInversion) {
  EXPECT_LT(rad_s_to_raw_velocity(1.0, TEST_MAX_VEL_STEPS), 0);
  EXPECT_GT(rad_s_to_raw_velocity(-1.0, TEST_MAX_VEL_STEPS), 0);
}

/** @brief Very high values and ±∞ are clamped symmetrically to ±max_velocity_steps */
TEST(ConversionsTest, RadSToRawVelocity_Clamps) {
  EXPECT_EQ(rad_s_to_raw_velocity(-1000.0, 500), 500);
  EXPECT_EQ(rad_s_to_raw_velocity(1000.0, 500), -500);
  const double inf = std::numeric_limits<double>::infinity();
  for (double v : {inf, -inf}) {
    int result = rad_s_to_raw_velocity(v, 500);
    EXPECT_GE(result, -500) << "out of range for v=" << v;
    EXPECT_LE(result, 500) << "out of range for v=" << v;
  }
}

// ---- effort_to_raw_pwm ----

/** @brief Sign-inverted: 0 → 0; +1.0 → -1000; -0.5 → +500 */
TEST(ConversionsTest, EffortToRawPwm_KnownValues) {
  EXPECT_EQ(effort_to_raw_pwm(0.0), 0);
  EXPECT_EQ(effort_to_raw_pwm(1.0), -1000);
  EXPECT_EQ(effort_to_raw_pwm(-1.0), 1000);
  EXPECT_EQ(effort_to_raw_pwm(0.5), -500);
  EXPECT_EQ(effort_to_raw_pwm(-0.5), 500);
}

/** @brief Values outside [-1.0, 1.0] are clamped to ±1000 */
TEST(ConversionsTest, EffortToRawPwm_Clamps) {
  EXPECT_EQ(effort_to_raw_pwm(2.0), -1000);
  EXPECT_EQ(effort_to_raw_pwm(-2.0), 1000);
}

// ---- clamp_acceleration ----

/** @brief 0 → 0; STS_MAX_ACCELERATION (254) → 254 */
TEST(ConversionsTest, ClampAcceleration_KnownValues) {
  EXPECT_EQ(clamp_acceleration(0.0), 0);
  EXPECT_EQ(clamp_acceleration(254.0), 254);
}

/** @brief Clamp acceleration: clamps */
TEST(ConversionsTest, ClampAcceleration_Clamps) {
  EXPECT_EQ(clamp_acceleration(300.0), 254);  // above max → max
  EXPECT_EQ(clamp_acceleration(-10.0), 0);    // negative → 0
}

// ---- normalize_effort ----
// normalize_effort(double effort, double max_effort, bool has_limit)
// When has_limit=true: clamps effort to [-max_effort, +max_effort]
// When has_limit=false: passes effort through unchanged

/** @brief has_limit=false → returned unchanged */
TEST(ConversionsTest, NormalizeEffort_NoLimit_PassThrough) {
  EXPECT_NEAR(normalize_effort(5.0, 1.0, false), 5.0, 1e-9);
}

/** @brief Zero and in-range values are returned unchanged */
TEST(ConversionsTest, NormalizeEffort_WithLimit_InBounds) {
  EXPECT_NEAR(normalize_effort(0.0, 1.0, true), 0.0, 1e-9);
  EXPECT_NEAR(normalize_effort(1.0, 1.0, true), 1.0, 1e-9);
  EXPECT_NEAR(normalize_effort(0.5, 1.0, true), 0.5, 1e-9);
}

/** @brief effort > max_effort → clamped to max_effort */
TEST(ConversionsTest, NormalizeEffort_WithLimit_ClampsPositive) {
  EXPECT_NEAR(normalize_effort(1.5, 1.0, true), 1.0, 1e-9);
}

/** @brief effort < -max_effort → clamped to -max_effort */
TEST(ConversionsTest, NormalizeEffort_WithLimit_ClampsNegative) {
  EXPECT_NEAR(normalize_effort(-1.5, 1.0, true), -1.0, 1e-9);
}

/** @brief max_effort=0.5 limits to [-0.5, 0.5] */
TEST(ConversionsTest, NormalizeEffort_PartialLimit) {
  EXPECT_NEAR(normalize_effort(0.8, 0.5, true), 0.5, 1e-9);
  EXPECT_NEAR(normalize_effort(0.3, 0.5, true), 0.3, 1e-9);
}

// ---- apply_limit ----
// apply_limit(T value, T min_val, T max_val, bool has_limit)
// When has_limit=true: clamps value to [min_val, max_val]
// When has_limit=false: returns value unchanged

/** @brief Interior, minimum, and maximum values returned unchanged */
TEST(ConversionsTest, ApplyLimit_WithinBounds) {
  EXPECT_NEAR(apply_limit(5.0, 0.0, 10.0, true), 5.0, 1e-9);
  EXPECT_NEAR(apply_limit(0.0, 0.0, 10.0, true), 0.0, 1e-9);
  EXPECT_NEAR(apply_limit(10.0, 0.0, 10.0, true), 10.0, 1e-9);
}

/** @brief Apply limit: out of bounds, clamped */
TEST(ConversionsTest, ApplyLimit_OutOfBounds_Clamped) {
  EXPECT_NEAR(apply_limit(-1.0, 0.0, 10.0, true), 0.0, 1e-9);   // below min → min
  EXPECT_NEAR(apply_limit(15.0, 0.0, 10.0, true), 10.0, 1e-9);  // above max → max
}

/** @brief has_limit=false → value passes through even if out of [min, max] */
TEST(ConversionsTest, ApplyLimit_NoLimit_PassThrough) {
  EXPECT_NEAR(apply_limit(100.0, 0.0, 10.0, false), 100.0, 1e-9);
  EXPECT_NEAR(apply_limit(-5.0, 0.0, 10.0, false), -5.0, 1e-9);
}

/** @brief Apply limit: int type */
TEST(ConversionsTest, ApplyLimit_IntType) {
  EXPECT_EQ(apply_limit(5, 0, 10, true), 5);
  EXPECT_EQ(apply_limit(-1, 0, 10, true), 0);
  EXPECT_EQ(apply_limit(15, 0, 10, true), 10);
  EXPECT_EQ(apply_limit(15, 0, 10, false), 15);  // passthrough
}

// ---- rad_s_to_raw_speed ----
//
// Unlike rad_s_to_raw_velocity, this function:
//   - does NOT invert the sign (speed is an unsigned magnitude for WritePosEx)
//   - clamps negative inputs to 0 (not to -max)
// STS protocol: speed==0 sent to WritePosEx → hardware uses its own max speed.

/** @brief 0 rad/s → raw 0.  STS protocol interprets raw 0 as "hardware max speed" */
TEST(ConversionsTest, RadSToRawSpeed_Zero) {
  EXPECT_EQ(rad_s_to_raw_speed(0.0, TEST_MAX_VEL_STEPS), 0);
}

/** @brief Negative magnitude (direction-less: magnitudes are unsigned) → clamped to 0 */
TEST(ConversionsTest, RadSToRawSpeed_NegativeClampsToZero) {
  int result = rad_s_to_raw_speed(-1.0, TEST_MAX_VEL_STEPS);
  EXPECT_EQ(result, 0);
}

/** @brief Very high speed and ±∞ clamp to [0, max_velocity_steps] */
TEST(ConversionsTest, RadSToRawSpeed_ClampsToMax) {
  EXPECT_EQ(rad_s_to_raw_speed(10000.0, 500), 500);
  const double inf = std::numeric_limits<double>::infinity();
  for (double v : {inf, -inf}) {
    int result = rad_s_to_raw_speed(v, 500);
    EXPECT_GE(result, 0) << "out of range for v=" << v;
    EXPECT_LE(result, 500) << "out of range for v=" << v;
  }
}

/** @brief For the same positive rad/s value: */
TEST(ConversionsTest, RadSToRawSpeed_NoSignInversionContrastWithVelocity) {
  //   rad_s_to_raw_velocity → negative raw (CW/CCW inversion)
  //   rad_s_to_raw_speed    → positive raw (unsigned magnitude, no inversion)
  const double speed_rad_s = 1.0;
  int vel_raw = rad_s_to_raw_velocity(speed_rad_s, TEST_MAX_VEL_STEPS);
  int spd_raw = rad_s_to_raw_speed(speed_rad_s, TEST_MAX_VEL_STEPS);
  EXPECT_LT(vel_raw, 0);          // velocity: sign-inverted → negative
  EXPECT_GT(spd_raw, 0);          // speed: unsigned magnitude → positive
  EXPECT_EQ(-vel_raw, spd_raw);   // same magnitude
}

// ---- custom center ----

/** @brief read: center=2048 maps step 2048 → 0 rad */
TEST(ConversionsTest, CustomCenter_KnownValues) {
  EXPECT_NEAR(raw_position_to_radians(2048, 2048), 0.0, 1e-6);
  EXPECT_NEAR(raw_position_to_radians(0,    2048), 2048.0 * (2.0 * M_PI / 4096.0), 1e-6);
  EXPECT_NEAR(raw_position_to_radians(4095, 2048), -2047.0 * (2.0 * M_PI / 4096.0), 1e-6);
  // write: 0 rad → step 2048
  EXPECT_EQ(radians_to_raw_position(0.0, 2048), 2048);
  EXPECT_NEAR(radians_to_raw_position( 2.0, 2048), 744,  1);
  EXPECT_NEAR(radians_to_raw_position(-2.0, 2048), 3352, 1);
  // -0.1 rad → step 2113 (short path from center, NOT wrapping around)
  EXPECT_NEAR(radians_to_raw_position(-0.1, 2048), 2113, 1);
  EXPECT_EQ(  radians_to_raw_position( M_PI, 2048), 0);     // clamped
  EXPECT_EQ(  radians_to_raw_position(-M_PI, 2048), 4095);  // clamped
}

/** @brief roundtrip with center=2048 */
TEST(ConversionsTest, CustomCenter_RoundtripAndClamping) {
  for (double rad : {-2.0, -1.0, -0.1, 0.0, 0.1, 1.0, 2.0}) {
    int raw = radians_to_raw_position(rad, 2048);
    double recovered = raw_position_to_radians(raw, 2048);
    EXPECT_NEAR(recovered, rad, 2.0 * M_PI / 4096.0 + 1e-9)
      << "Roundtrip error too large at rad=" << rad;
  }
  // output always clamped to [0, 4095] for any center
  for (int center : {0, 1000, 2048, 3000, 4095}) {
    for (double rad : {-M_PI, 0.0, M_PI, 100.0, -100.0}) {
      int result = radians_to_raw_position(rad, center);
      EXPECT_GE(result, 0) << "out of range for center=" << center << ", rad=" << rad;
      EXPECT_LE(result, 4095) << "out of range for center=" << center << ", rad=" << rad;
    }
  }
}

// ---- decode_servo_error ----

/** @brief Decode servo error: zero, is ok */
TEST(ConversionsTest, DecodeServoError_Zero_IsOk) {
  EXPECT_EQ(decode_servo_error(0), "OK");
}

/** @brief Decode servo error: single bits */
TEST(ConversionsTest, DecodeServoError_SingleBits) {
  EXPECT_EQ(decode_servo_error(1 << 0), "voltage error");
  EXPECT_EQ(decode_servo_error(1 << 2), "overheat error");
  EXPECT_EQ(decode_servo_error(1 << 5), "overload error");
}

/** @brief Decode servo error: multiple bits */
TEST(ConversionsTest, DecodeServoError_MultipleBits) {
  EXPECT_EQ(decode_servo_error((1 << 0) | (1 << 2)), "voltage error, overheat error");
  EXPECT_EQ(decode_servo_error((1 << 0) | (1 << 2) | (1 << 5)),
    "voltage error, overheat error, overload error");
}

/** @brief Decode servo error: unknown bit, surfaced */
TEST(ConversionsTest, DecodeServoError_UnknownBit_Surfaced) {
  // Bit 1 is reserved/undocumented on STS-series servos - must not be silently dropped.
  EXPECT_EQ(decode_servo_error(1 << 1), "unknown bits 0x02");
  EXPECT_EQ(decode_servo_error((1 << 0) | (1 << 1)), "voltage error, unknown bits 0x02");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
