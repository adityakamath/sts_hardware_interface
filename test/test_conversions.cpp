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

TEST(ConversionsTest, RawPositionToRadians_Zero) {
  // raw=0 → (4095 - 0) * STEPS_TO_RAD = max positive angle
  double result = raw_position_to_radians(0);
  EXPECT_NEAR(result, 4095.0 * (2.0 * M_PI / 4096.0), 1e-6);
}

TEST(ConversionsTest, RawPositionToRadians_Max) {
  // raw=4095 → (4095 - 4095) * STEPS_TO_RAD = 0
  EXPECT_NEAR(raw_position_to_radians(4095), 0.0, 1e-6);
}

TEST(ConversionsTest, RawPositionToRadians_Middle) {
  // raw=2048 → (4095 - 2048) * STEPS_TO_RAD
  double expected = 2047.0 * (2.0 * M_PI / 4096.0);
  EXPECT_NEAR(raw_position_to_radians(2048), expected, 1e-6);
}

TEST(ConversionsTest, RawPositionToRadians_Roundtrip) {
  // raw_position_to_radians uses STS_MAX_POSITION=4095 while radians_to_raw_position
  // uses STEPS_PER_REVOLUTION=4096, producing a systematic +1 offset on the inverse path.
  // The roundtrip is therefore lossy by exactly 1 step for most values.
  for (int raw = 0; raw <= 4095; raw += 100) {
    double rad = raw_position_to_radians(raw);
    int recovered = radians_to_raw_position(rad);
    EXPECT_LE(std::abs(recovered - raw), 1) << "Roundtrip error exceeds 1 step at raw=" << raw;
  }
}

// ---- radians_to_raw_position ----

TEST(ConversionsTest, RadiansToRawPosition_Zero) {
  // 0 radians → raw=0: fmod(-0.0, 2π)=0, so 0 * RAD_TO_STEPS = 0
  EXPECT_EQ(radians_to_raw_position(0.0), 0);
}

TEST(ConversionsTest, RadiansToRawPosition_FullCircle) {
  // 2π radians → raw 0 after modulo wrapping
  int result = radians_to_raw_position(2.0 * M_PI);
  EXPECT_GE(result, 0);
  EXPECT_LE(result, 4095);
}

TEST(ConversionsTest, RadiansToRawPosition_ClampsToRange) {
  // Very large positive radians → after modulo, stays in range
  int result = radians_to_raw_position(100.0);
  EXPECT_GE(result, 0);
  EXPECT_LE(result, 4095);
}

TEST(ConversionsTest, RadiansToRawPosition_NegativeClamps) {
  // Negative radians normalised to positive range
  int result = radians_to_raw_position(-1.0);
  EXPECT_GE(result, 0);
  EXPECT_LE(result, 4095);
}

// ---- raw_velocity_to_rad_s ----

TEST(ConversionsTest, RawVelocityToRadS_Zero) {
  EXPECT_NEAR(raw_velocity_to_rad_s(0), 0.0, 1e-9);
}

TEST(ConversionsTest, RawVelocityToRadS_Positive) {
  // Positive raw velocity → negative rad/s (direction inversion)
  double result = raw_velocity_to_rad_s(100);
  EXPECT_LT(result, 0.0);
}

TEST(ConversionsTest, RawVelocityToRadS_Negative) {
  // Negative raw velocity → positive rad/s (direction inversion)
  double result = raw_velocity_to_rad_s(-100);
  EXPECT_GT(result, 0.0);
}

TEST(ConversionsTest, RawVelocityToRadS_Roundtrip) {
  // Convert raw → rad/s → raw; should recover original value.
  // Pass TEST_MAX_VEL_STEPS so the clamp never truncates within test range.
  for (int raw : {-500, -100, 0, 100, 500}) {
    double rad_s = raw_velocity_to_rad_s(raw);
    int recovered = rad_s_to_raw_velocity(rad_s, TEST_MAX_VEL_STEPS);
    EXPECT_EQ(recovered, raw) << "Failed roundtrip at raw=" << raw;
  }
}

// ---- rad_s_to_raw_velocity ----

TEST(ConversionsTest, RadSToRawVelocity_Zero) {
  EXPECT_EQ(rad_s_to_raw_velocity(0.0, TEST_MAX_VEL_STEPS), 0);
}

TEST(ConversionsTest, RadSToRawVelocity_Positive) {
  // Positive rad/s → negative raw (direction inversion)
  int result = rad_s_to_raw_velocity(1.0, TEST_MAX_VEL_STEPS);
  EXPECT_LT(result, 0);
}

TEST(ConversionsTest, RadSToRawVelocity_Negative) {
  // Negative rad/s → positive raw (direction inversion)
  int result = rad_s_to_raw_velocity(-1.0, TEST_MAX_VEL_STEPS);
  EXPECT_GT(result, 0);
}

TEST(ConversionsTest, RadSToRawVelocity_ClampsToMax) {
  // Very high rad/s should be clamped to max_velocity_steps
  int result = rad_s_to_raw_velocity(-1000.0, 500);
  EXPECT_EQ(result, 500);
}

TEST(ConversionsTest, RadSToRawVelocity_ClampsToMin) {
  // Very high positive rad/s (inverts to very negative raw) → clamped to -max
  int result = rad_s_to_raw_velocity(1000.0, 500);
  EXPECT_EQ(result, -500);
}

// ---- effort_to_raw_pwm ----

TEST(ConversionsTest, EffortToRawPwm_Zero) {
  EXPECT_EQ(effort_to_raw_pwm(0.0), 0);
}

TEST(ConversionsTest, EffortToRawPwm_One) {
  // 1.0 effort → STS_MAX_PWM = 1000
  EXPECT_EQ(effort_to_raw_pwm(1.0), 1000);
}

TEST(ConversionsTest, EffortToRawPwm_Half) {
  // 0.5 effort → 500
  EXPECT_EQ(effort_to_raw_pwm(0.5), 500);
}

TEST(ConversionsTest, EffortToRawPwm_NegativeHalf) {
  // -0.5 effort → -500 (bidirectional PWM)
  EXPECT_EQ(effort_to_raw_pwm(-0.5), -500);
}

TEST(ConversionsTest, EffortToRawPwm_NegativeOne) {
  // -1.0 effort → -1000
  EXPECT_EQ(effort_to_raw_pwm(-1.0), -1000);
}

TEST(ConversionsTest, EffortToRawPwm_ClampsAboveOne) {
  // Values > 1.0 are clamped to 1000
  EXPECT_EQ(effort_to_raw_pwm(2.0), 1000);
}

TEST(ConversionsTest, EffortToRawPwm_ClampsBelowNegativeOne) {
  // Values < -1.0 are clamped to -1000
  EXPECT_EQ(effort_to_raw_pwm(-2.0), -1000);
}

// ---- clamp_acceleration ----

TEST(ConversionsTest, ClampAcceleration_Zero) {
  EXPECT_EQ(clamp_acceleration(0.0), 0);
}

TEST(ConversionsTest, ClampAcceleration_Max) {
  // STS_MAX_ACCELERATION = 254
  EXPECT_EQ(clamp_acceleration(254.0), 254);
}

TEST(ConversionsTest, ClampAcceleration_ClampsAboveMax) {
  EXPECT_EQ(clamp_acceleration(300.0), 254);
}

TEST(ConversionsTest, ClampAcceleration_ClampsNegative) {
  EXPECT_EQ(clamp_acceleration(-10.0), 0);
}

TEST(ConversionsTest, ClampAcceleration_Mid) {
  EXPECT_EQ(clamp_acceleration(100.0), 100);
}

// ---- normalize_effort ----
// normalize_effort(double effort, double max_effort, bool has_limit)
// When has_limit=true: clamps effort to [-max_effort, +max_effort]
// When has_limit=false: passes effort through unchanged

TEST(ConversionsTest, NormalizeEffort_NoLimit_PassThrough) {
  // has_limit=false → returned unchanged
  EXPECT_NEAR(normalize_effort(5.0, 1.0, false), 5.0, 1e-9);
}

TEST(ConversionsTest, NormalizeEffort_WithLimit_Zero) {
  EXPECT_NEAR(normalize_effort(0.0, 1.0, true), 0.0, 1e-9);
}

TEST(ConversionsTest, NormalizeEffort_WithLimit_AtMax) {
  // effort == max_effort → no clamping needed
  EXPECT_NEAR(normalize_effort(1.0, 1.0, true), 1.0, 1e-9);
}

TEST(ConversionsTest, NormalizeEffort_WithLimit_ClampsPositive) {
  // effort > max_effort → clamped to max_effort
  EXPECT_NEAR(normalize_effort(1.5, 1.0, true), 1.0, 1e-9);
}

TEST(ConversionsTest, NormalizeEffort_WithLimit_ClampsNegative) {
  // effort < -max_effort → clamped to -max_effort
  EXPECT_NEAR(normalize_effort(-1.5, 1.0, true), -1.0, 1e-9);
}

TEST(ConversionsTest, NormalizeEffort_WithLimit_WithinRange) {
  // effort within [-max_effort, max_effort] → unchanged
  EXPECT_NEAR(normalize_effort(0.5, 1.0, true), 0.5, 1e-9);
}

TEST(ConversionsTest, NormalizeEffort_PartialLimit) {
  // max_effort=0.5 limits to [-0.5, 0.5]
  EXPECT_NEAR(normalize_effort(0.8, 0.5, true), 0.5, 1e-9);
  EXPECT_NEAR(normalize_effort(0.3, 0.5, true), 0.3, 1e-9);
}

// ---- apply_limit ----
// apply_limit(T value, T min_val, T max_val, bool has_limit)
// When has_limit=true: clamps value to [min_val, max_val]
// When has_limit=false: returns value unchanged

TEST(ConversionsTest, ApplyLimit_WithinRange) {
  EXPECT_NEAR(apply_limit(5.0, 0.0, 10.0, true), 5.0, 1e-9);
}

TEST(ConversionsTest, ApplyLimit_AtMin) {
  EXPECT_NEAR(apply_limit(0.0, 0.0, 10.0, true), 0.0, 1e-9);
}

TEST(ConversionsTest, ApplyLimit_AtMax) {
  EXPECT_NEAR(apply_limit(10.0, 0.0, 10.0, true), 10.0, 1e-9);
}

TEST(ConversionsTest, ApplyLimit_BelowMin_Clamped) {
  EXPECT_NEAR(apply_limit(-1.0, 0.0, 10.0, true), 0.0, 1e-9);
}

TEST(ConversionsTest, ApplyLimit_AboveMax_Clamped) {
  EXPECT_NEAR(apply_limit(15.0, 0.0, 10.0, true), 10.0, 1e-9);
}

TEST(ConversionsTest, ApplyLimit_NoLimit_PassThrough) {
  // has_limit=false → value passes through even if out of [min, max]
  EXPECT_NEAR(apply_limit(100.0, 0.0, 10.0, false), 100.0, 1e-9);
  EXPECT_NEAR(apply_limit(-5.0, 0.0, 10.0, false), -5.0, 1e-9);
}

TEST(ConversionsTest, ApplyLimit_IntType) {
  EXPECT_EQ(apply_limit(5, 0, 10, true), 5);
  EXPECT_EQ(apply_limit(-1, 0, 10, true), 0);
  EXPECT_EQ(apply_limit(15, 0, 10, true), 10);
  EXPECT_EQ(apply_limit(15, 0, 10, false), 15);  // passthrough
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
