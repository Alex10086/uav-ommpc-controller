#include <cmath>

#include <gtest/gtest.h>

#include "uav_ommpc_controller/dynamic.hpp"

namespace
{
namespace dynamics = uav_ommpc_controller::dynamics;
using dynamics::ControlInput;
using dynamics::State;
using dynamics::predict_step;
using Eigen::Matrix3d;
using Eigen::Vector3d;
constexpr double kPi = 3.14159265358979323846;
constexpr double kG = dynamics::kGravity;

TEST(Dynamics, ZeroTimePreservesNontrivialState)
{
  const State state{Vector3d(1.0, -2.0, 3.0), Vector3d(-0.5, 0.3, 0.7),
    Eigen::AngleAxisd(0.4, Vector3d(1.0, 2.0, -3.0).normalized()).toRotationMatrix()};
  const ControlInput input{1.5 * kG, Vector3d(0.1, -0.4, 0.8)};
  const State next = predict_step(state, input, 0.0);
  EXPECT_EQ((next.p - state.p).norm(), 0.0);
  EXPECT_EQ((next.v - state.v).norm(), 0.0);
  EXPECT_EQ((next.R - state.R).norm(), 0.0);
}

TEST(Dynamics, LevelHoverBalancesGravityAtAnyYaw)
{
  for (double yaw : {-2.0, 0.0, 1.3})
  {
    const State state{Vector3d(3.0, -1.0, -2.0), Vector3d::Zero(),
      Eigen::AngleAxisd(yaw, Vector3d::UnitZ()).toRotationMatrix()};
    const State next = predict_step(state, ControlInput{kG, Vector3d::Zero()}, 0.02);
    EXPECT_EQ((next.p - state.p).norm(), 0.0);
    EXPECT_LT(next.v.norm(), 1e-15);
    EXPECT_EQ((next.R - state.R).norm(), 0.0);
  }
}

TEST(Dynamics, FreeFallUsesOldVelocityForForwardEulerPosition)
{
  const State state{Vector3d(1.0, 2.0, -3.0), Vector3d(2.0, -1.0, 0.5),
    Matrix3d::Identity()};
  const State next = predict_step(state, ControlInput{0.0, Vector3d::Zero()}, 0.1);
  EXPECT_LT((next.p - Vector3d(1.2, 1.9, -2.95)).norm(), 5e-16);
  EXPECT_LT((next.v - Vector3d(2.0, -1.0, 0.5 + 0.1 * kG)).norm(), 5e-16);
  EXPECT_EQ((next.R - state.R).norm(), 0.0);
}

TEST(Dynamics, ExcessThrustAcceleratesUpWithoutThrottleClamping)
{
  const State state{Vector3d::Zero(), Vector3d::Zero(), Matrix3d::Identity()};
  const State next = predict_step(state, ControlInput{kG + 2.0, Vector3d::Zero()}, 0.1);
  EXPECT_EQ(next.p.norm(), 0.0);
  EXPECT_LT((next.v - Vector3d(0.0, 0.0, -0.2)).norm(), 5e-16);
}

TEST(Dynamics, TiltedThrustUsesCurrentBodyToWorldRotation)
{
  // Positive 30-degree pitch: body +Z has positive world X component.
  const double c = std::sqrt(3.0) / 2.0;
  Matrix3d rotation;
  rotation << c, 0.0, 0.5,
              0.0, 1.0, 0.0,
              -0.5, 0.0, c;
  const State state{Vector3d::Zero(), Vector3d::Zero(), rotation};
  // A substantial rotation within this step must not change Euler acceleration.
  const State next = predict_step(state, ControlInput{8.0, Vector3d(0.0, 2.0, 0.0)}, 0.1);
  EXPECT_EQ(next.p.norm(), 0.0);
  EXPECT_LT((next.v - Vector3d(-0.4, 0.0, 0.1 * (kG - 8.0 * c))).norm(), 5e-16);
  const Matrix3d expected_rotation =
    Eigen::AngleAxisd(kPi / 6.0 + 0.2, Vector3d::UnitY()).toRotationMatrix();
  EXPECT_LT((next.R - expected_rotation).norm(), 5e-16);
  EXPECT_EQ((state.R - rotation).norm(), 0.0);
  EXPECT_EQ(state.v.norm(), 0.0);
}

TEST(Dynamics, BodyRatesRotateOnTheRightAndUseTimeInSeconds)
{
  Matrix3d yaw90;
  yaw90 << 0.0, -1.0, 0.0,
           1.0, 0.0, 0.0,
           0.0, 0.0, 1.0;
  const State state{Vector3d::Zero(), Vector3d::Zero(), yaw90};
  // Over 0.2 s, this is a +90-degree body-X rotation, not a world-X rotation.
  const State next = predict_step(state, ControlInput{0.0, Vector3d(kPi / 0.4, 0.0, 0.0)}, 0.2);
  Matrix3d expected;
  expected << 0.0, 0.0, 1.0,
              1.0, 0.0, 0.0,
              0.0, 1.0, 0.0;
  EXPECT_LT((next.R - expected).norm(), 5e-16);
  EXPECT_LT((next.R.transpose() * next.R - Matrix3d::Identity()).norm(), 5e-16);
  EXPECT_NEAR(next.R.determinant(), 1.0, 5e-16);
}

TEST(Dynamics, SuppliedGravityChangesOnlyTranslationalAcceleration)
{
  const State state{Vector3d(1.0, 2.0, 3.0), Vector3d(0.5, -0.5, 1.0), Matrix3d::Identity()};
  const State next = predict_step(state, ControlInput{4.0, Vector3d::Zero()}, 0.2, 10.0);
  EXPECT_LT((next.p - Vector3d(1.1, 1.9, 3.2)).norm(), 5e-16);
  EXPECT_LT((next.v - Vector3d(0.5, -0.5, 2.2)).norm(), 5e-16);
  EXPECT_EQ((next.R - state.R).norm(), 0.0);
}

TEST(Dynamics, RepeatedStepsMatchDiscreteBallisticsAndConstantBodyRates)
{
  const State initial{Vector3d(1.0, -2.0, -4.0), Vector3d(0.4, 0.2, -0.3),
    Eigen::AngleAxisd(0.7, Vector3d::UnitY()).toRotationMatrix()};
  const ControlInput input{0.0, Vector3d(0.5, -0.3, 0.7)};
  constexpr int steps = 1000;
  constexpr double dt = 0.001;
  const double duration = steps * dt;
  State state = initial;
  for (int step = 0; step < steps; ++step)
  {
    state = predict_step(state, input, dt);
  }
  const Vector3d expected_position = initial.p + duration * initial.v +
    Vector3d(0.0, 0.0, 0.5 * kG * steps * (steps - 1) * dt * dt);
  const Vector3d expected_velocity = initial.v + Vector3d(0.0, 0.0, kG * duration);
  const Matrix3d expected_rotation = initial.R * Eigen::AngleAxisd(
    duration * input.body_rates.norm(), input.body_rates.normalized()).toRotationMatrix();
  EXPECT_LT((state.p - expected_position).norm(), 3e-13);
  EXPECT_LT((state.v - expected_velocity).norm(), 3e-13);
  EXPECT_LT((state.R - expected_rotation).norm(), 3e-13);
  EXPECT_LT((state.R.transpose() * state.R - Matrix3d::Identity()).norm(), 5e-13);
  EXPECT_NEAR(state.R.determinant(), 1.0, 5e-13);
}
}  // namespace
