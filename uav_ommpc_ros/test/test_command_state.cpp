#include <chrono>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "command_state.hpp"

using namespace std::chrono_literals;
using uav_ommpc_interfaces::msg::ThrustRates;
using uav_ommpc_ros::CommandState;
using uav_ommpc_ros::kGravity;
using uav_ommpc_ros::Limits;

namespace
{
const Limits kLimits{0.7, {3.0, 3.0, 1.5}, 0.9};
const Limits kSuperQuadraticLimits{0.309128, {3.0, 3.0, 1.5}, 0.9, true, 150.0, 1000.0};
const CommandState::Clock::time_point kStart{};

ThrustRates input(double thrust, double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
{
  ThrustRates message;
  message.thrust_acceleration = thrust;
  message.body_rates = {roll, pitch, yaw};
  return message;
}
}  // namespace

TEST(CommandStateTimeout, NotFreshBeforeFirstValidInput)
{
  const CommandState state(kLimits, 500ms);
  EXPECT_FALSE(state.fresh(kStart));
}

TEST(CommandStateTimeout, ExpiresJustAfterTimeout)
{
  CommandState state(kLimits, 500ms);
  ASSERT_TRUE(state.accept(input(kGravity), kStart));
  EXPECT_TRUE(state.fresh(kStart + 500ms));
  EXPECT_FALSE(state.fresh(kStart + 501ms));
}

TEST(CommandStateTimeout, NewValidInputRestartsTimeout)
{
  CommandState state(kLimits, 500ms);
  ASSERT_TRUE(state.accept(input(kGravity), kStart));
  ASSERT_TRUE(state.accept(input(kGravity), kStart + 400ms));
  EXPECT_TRUE(state.fresh(kStart + 900ms));
  EXPECT_FALSE(state.fresh(kStart + 901ms));
}

TEST(CommandStateTimeout, RejectedInputNeitherRefreshesNorReplacesCommand)
{
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  constexpr double inf = std::numeric_limits<double>::infinity();
  CommandState state(kLimits, 500ms);
  ASSERT_TRUE(state.accept(input(kGravity, 0.1, -0.2, 0.3), kStart));
  EXPECT_FALSE(state.accept(input(-1.0), kStart + 400ms));
  EXPECT_FALSE(state.accept(input(nan), kStart + 400ms));
  EXPECT_FALSE(state.accept(input(kGravity, 0.0, inf, 0.0), kStart + 400ms));
  EXPECT_FALSE(state.fresh(kStart + 501ms));
  EXPECT_FLOAT_EQ(state.command().body_rates[1], -0.2f);
  EXPECT_FLOAT_EQ(state.command().throttle, 0.7f);
}

TEST(CommandStateLimits, ScalesThrustAndKeepsSignedRatesWithinLimits)
{
  CommandState state(kLimits, 500ms);
  ASSERT_TRUE(state.accept(input(0.5 * kGravity, 0.1, -0.2, 0.3), kStart));
  EXPECT_FLOAT_EQ(state.command().throttle, 0.35f);
  EXPECT_FLOAT_EQ(state.command().body_rates[0], 0.1f);
  EXPECT_FLOAT_EQ(state.command().body_rates[1], -0.2f);
  EXPECT_FLOAT_EQ(state.command().body_rates[2], 0.3f);
  EXPECT_FALSE(state.command().limited);
}

TEST(CommandStateLimits, ClampsEachRateAxisAndThrottle)
{
  CommandState state(kLimits, 500ms);
  ASSERT_TRUE(state.accept(input(30.0, 4.0, -5.0, 2.0), kStart));
  EXPECT_FLOAT_EQ(state.command().body_rates[0], 3.0f);
  EXPECT_FLOAT_EQ(state.command().body_rates[1], -3.0f);
  EXPECT_FLOAT_EQ(state.command().body_rates[2], 1.5f);
  EXPECT_FLOAT_EQ(state.command().throttle, 0.9f);
  EXPECT_TRUE(state.command().limited);
}

TEST(CommandStateLimits, QuadraticInverseThrustMappingRecoversExactRotorAcceleration)
{
  CommandState state(kSuperQuadraticLimits, 500ms);
  // At 1g hover, quadratic inverse must equal hover_thrust exactly.
  ASSERT_TRUE(state.accept(input(kGravity), kStart));
  EXPECT_NEAR(state.command().throttle, 0.309128f, 1e-6f);
  EXPECT_FALSE(state.command().limited);

  // Across [6.0, 20.0] m/s^2, simulated quadratic rotor thrust g * (omega(u) / omega_hover)^2
  // must match the commanded thrust acceleration to float32 precision.
  const double hover_speed = 150.0 + 850.0 * kSuperQuadraticLimits.hover_thrust;
  for (double desired_a : {6.0, 8.0, 9.80665, 12.0, 14.5, 18.0, 20.0}) {
    ASSERT_TRUE(state.accept(input(desired_a), kStart));
    const double u = state.command().throttle;
    const double rotor_speed = 150.0 + 850.0 * u;
    const double actual_a = kGravity * (rotor_speed / hover_speed) * (rotor_speed / hover_speed);
    EXPECT_NEAR(actual_a, desired_a, 1e-4);
    EXPECT_FALSE(state.command().limited);
  }

  // Below idle rotor thrust, throttle clamps cleanly at 0.0.
  ASSERT_TRUE(state.accept(input(0.0), kStart));
  EXPECT_FLOAT_EQ(state.command().throttle, 0.0f);
  EXPECT_TRUE(state.command().limited);
  ASSERT_TRUE(state.accept(input(1.0), kStart));
  EXPECT_FLOAT_EQ(state.command().throttle, 0.0f);
  EXPECT_TRUE(state.command().limited);

  ASSERT_TRUE(state.accept(input(100.0), kStart));
  EXPECT_FLOAT_EQ(state.command().throttle, 0.9f);
  EXPECT_TRUE(state.command().limited);

  // Rejected input must not renew the lease in quadratic mode either.
  EXPECT_FALSE(state.accept(input(std::numeric_limits<double>::infinity()), kStart + 400ms));
  EXPECT_FALSE(state.fresh(kStart + 501ms));
}

TEST(RateCompensation, DisabledPreservesLinearPathAndEnabledLeavesYawThrustUnchanged)
{
  CommandState original(kSuperQuadraticLimits, 500ms);
  uav_ommpc_ros::RateCompensationSettings settings;
  settings.enabled = true;
  CommandState compensated(kSuperQuadraticLimits, 500ms, settings);
  ASSERT_TRUE(original.accept(input(12.0, 0.0, 0.0, -0.3), kStart));
  ASSERT_TRUE(compensated.accept(input(12.0, 0.0, 0.0, -0.3), kStart));
  ASSERT_TRUE(original.accept(input(12.0, 0.2, -0.2, -0.3), kStart + 20ms));
  ASSERT_TRUE(compensated.accept(input(12.0, 0.2, -0.2, -0.3), kStart + 20ms));
  EXPECT_FLOAT_EQ(original.command().body_rates[0], 0.2f);
  EXPECT_FLOAT_EQ(compensated.command().body_rates[2], original.command().body_rates[2]);
  EXPECT_FLOAT_EQ(compensated.command().throttle, original.command().throttle);
  EXPECT_GT(compensated.command().body_rates[0], 0.2f);
  EXPECT_LT(compensated.command().body_rates[1], -0.2f);
  // A constant target converges to its gain correction, not an accumulating derivative.
  for (int i = 2; i < 100; ++i)
    ASSERT_TRUE(compensated.accept(input(12.0, 0.2, -0.2, -0.3), kStart + i * 20ms));
  EXPECT_NEAR(compensated.command().body_rates[0], 0.2 / settings.gain[0], 1e-6);
}

TEST(RateCompensation, LimitsLeadBoostAndResetsOnExpiredLease)
{
  uav_ommpc_ros::RateCompensationSettings settings;
  settings.enabled = true;
  CommandState state(kLimits, 500ms, settings);
  ASSERT_TRUE(state.accept(input(kGravity, -3.0, 3.0), kStart));
  ASSERT_TRUE(state.accept(input(kGravity, 3.0, -3.0), kStart + 20ms));
  EXPECT_FLOAT_EQ(state.command().body_rates[0], 3.0f);
  EXPECT_FLOAT_EQ(state.command().body_rates[1], -3.0f);
  EXPECT_TRUE(state.command().limited);
  ASSERT_TRUE(state.accept(input(kGravity, -0.5, 0.5), kStart + 1s));
  EXPECT_NEAR(state.command().body_rates[0], -0.5 / settings.gain[0], 1e-6);
  EXPECT_FALSE(state.command().limited);
}

TEST(RateCompensation, RejectedInputDoesNotAffectFilterOrFreshness)
{
  uav_ommpc_ros::RateCompensationSettings settings;
  settings.enabled = true;
  CommandState state(kLimits, 500ms, settings), expected(kLimits, 500ms, settings);
  ASSERT_TRUE(state.accept(input(kGravity, 0.1), kStart));
  ASSERT_TRUE(expected.accept(input(kGravity, 0.1), kStart));
  EXPECT_FALSE(state.accept(input(kGravity, std::numeric_limits<double>::quiet_NaN()), kStart + 10ms));
  ASSERT_TRUE(state.accept(input(kGravity, -0.1), kStart + 20ms));
  ASSERT_TRUE(expected.accept(input(kGravity, -0.1), kStart + 20ms));
  EXPECT_FLOAT_EQ(state.command().body_rates[0], expected.command().body_rates[0]);
  EXPECT_FALSE(state.accept(input(-1.0, 3.0), kStart + 500ms));
  EXPECT_FALSE(state.fresh(kStart + 521ms));
}

TEST(RateCompensation, InvalidSettingsAreRejected)
{
  uav_ommpc_ros::RateCompensationSettings settings;
  settings.filter_time_constant = 0.0;
  EXPECT_THROW((CommandState(kLimits, 500ms, settings)), std::invalid_argument);
  settings.filter_time_constant = 0.012;
  settings.gain[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((CommandState(kLimits, 500ms, settings)), std::invalid_argument);
  settings.gain[0] = std::numeric_limits<double>::denorm_min();
  EXPECT_THROW((CommandState(kLimits, 500ms, settings)), std::invalid_argument);
  settings.gain[0] = 1.056;
  settings.time_constant[1] = -0.1;
  EXPECT_THROW((CommandState(kLimits, 500ms, settings)), std::invalid_argument);
}
