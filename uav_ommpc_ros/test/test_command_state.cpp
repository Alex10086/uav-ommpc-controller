#include <chrono>
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
