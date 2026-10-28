#include <cmath>
#include <cstdint>
#include <limits>
#include <gtest/gtest.h>
#include "state_prediction.hpp"

namespace
{
using uav_ommpc_ros::predict_measured_state;
using uav_ommpc_ros::state_sample_age;
using uav_ommpc_controller::dynamics::State;

TEST(StatePrediction, EpochAgePreservesSmallIntervalsAndRejectsWrongClock)
{
  constexpr std::uint64_t sample = 1793187954722196ULL;
  constexpr std::int64_t now = static_cast<std::int64_t>(sample) * 1000 + 25000000;
  ASSERT_TRUE(state_sample_age(sample, now, 0.08));
  EXPECT_NEAR(*state_sample_age(sample, now, 0.08), 0.025, 1e-15);
  EXPECT_FALSE(state_sample_age(4000000, now, 0.08));  // unsynchronized boot stamp
  EXPECT_FALSE(state_sample_age(sample, now - 26000000, 0.08));
  EXPECT_FALSE(state_sample_age(0, now, 0.08));
  EXPECT_FALSE(state_sample_age(std::numeric_limits<std::uint64_t>::max(), now, 0.08));
}

TEST(StatePrediction, AgeBoundaryIsAcceptedWithoutClampingOlderSamples)
{
  EXPECT_DOUBLE_EQ(*state_sample_age(1000000, 1000000000, 0.08), 0.0);
  EXPECT_NEAR(*state_sample_age(1000000, 1080000000, 0.08), 0.08, 1e-15);
  EXPECT_FALSE(state_sample_age(1000000, 1080000001, 0.08));
  EXPECT_FALSE(state_sample_age(1000000, -1, 0.08));
}

TEST(StatePrediction, ConstantVelocityHasNoGuessedAcceleration)
{
  State state;
  state.p = Eigen::Vector3d(1., -2., -3.);
  state.v = Eigen::Vector3d(3., -1., .5);
  const State predicted = predict_measured_state(state, Eigen::Vector3d::Zero(), .04);
  EXPECT_LT((predicted.p - Eigen::Vector3d(1.12, -2.04, -2.98)).norm(), 1e-14);
  EXPECT_EQ((predicted.v-state.v).norm(), 0.0);
  EXPECT_EQ((predicted.R-state.R).norm(), 0.0);
  EXPECT_EQ((state.p-Eigen::Vector3d(1., -2., -3.)).norm(), 0.0);
}

TEST(StatePrediction, MeasuredBodyRotationIsRightMultiplicative)
{
  State state;
  state.R = Eigen::AngleAxisd(std::acos(-1.)/2., Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const State predicted = predict_measured_state(state, Eigen::Vector3d(2., 0., 0.), .04);
  const Eigen::Matrix3d expected = state.R * Eigen::AngleAxisd(.08, Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_LT((predicted.R-expected).norm(), 1e-14);
  EXPECT_LT((predicted.R.transpose()*predicted.R-Eigen::Matrix3d::Identity()).norm(), 1e-14);
  const State unchanged = predict_measured_state(state, Eigen::Vector3d(2., -1., .5), 0.0);
  EXPECT_EQ((unchanged.R-state.R).norm(), 0.0);
}
}  // namespace
