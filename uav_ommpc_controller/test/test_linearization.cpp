#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "uav_ommpc_controller/linearization.hpp"

namespace
{
namespace dynamics = uav_ommpc_controller::dynamics;
namespace lin = uav_ommpc_controller::linearization;
using dynamics::ControlInput;
using dynamics::State;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using lin::InputError;
using lin::StateError;
constexpr double kPi = 3.14159265358979323846;

// Independent chart extraction for checking the library's packing and signs.
StateError measure_error(const State &actual, const State &reference)
{
  const Eigen::AngleAxisd rotation(reference.R.transpose() * actual.R);
  StateError error;
  error << actual.p - reference.p, actual.v - reference.v, rotation.angle() * rotation.axis();
  return error;
}

ControlInput perturb_input(const ControlInput &reference, const InputError &error)
{
  return {reference.thrust_acceleration + error[0], reference.body_rates + error.tail<3>()};
}

TEST(Linearization, StateChartUsesActualMinusReferenceAndRightLocalRotation)
{
  const State reference{Vector3d(1.0, 2.0, -3.0), Vector3d(-0.5, 1.0, 2.0),
    Eigen::AngleAxisd(0.7, Vector3d::UnitX()).toRotationMatrix()};
  const State actual{Vector3d(2.0, 0.0, -2.5), Vector3d(-0.7, 1.4, 2.7),
    reference.R * Eigen::AngleAxisd(0.2, Vector3d::UnitY()).toRotationMatrix()};
  StateError expected;
  expected << 1.0, -2.0, 0.5, -0.2, 0.4, 0.7, 0.0, 0.2, 0.0;
  EXPECT_LT((lin::state_error(actual, reference) - expected).norm(), 1e-15);
  const State restored = lin::apply_state_error(reference, expected);
  EXPECT_LT((restored.p - actual.p).norm(), 1e-15);
  EXPECT_LT((restored.v - actual.v).norm(), 1e-15);
  EXPECT_LT((restored.R - actual.R).norm(), 1e-15);
}

TEST(Linearization, ZeroTimePreservesErrorsRegardlessOfInputPerturbation)
{
  const State reference{Vector3d(1.0, -2.0, 3.0), Vector3d(0.1, 0.2, -0.3),
    Eigen::AngleAxisd(0.6, Vector3d::UnitY()).toRotationMatrix()};
  const ControlInput input{12.0, Vector3d(0.3, -0.4, 0.5)};
  StateError dx;
  dx << 0.1, -0.2, 0.3, -0.4, 0.5, 0.6, 0.02, -0.03, 0.04;
  const InputError du(0.5, -0.1, 0.2, 0.3);
  const StateError predicted = lin::linearize(reference, input, reference, 0.0).predict(dx, du);
  EXPECT_EQ((predicted - dx).norm(), 0.0);
  const State next = dynamics::predict_step(
    lin::apply_state_error(reference, dx), perturb_input(input, du), 0.0);
  EXPECT_LT((measure_error(next, reference) - predicted).norm(), 5e-16);
}

TEST(Linearization, HoverPredictionHasNedThrustAndTiltSigns)
{
  const State reference{Vector3d::Zero(), Vector3d::Zero(), Matrix3d::Identity()};
  const ControlInput input{dynamics::kGravity, Vector3d::Zero()};
  StateError dx;
  dx << 1.0, 2.0, 3.0, 0.4, -0.2, 0.1, 0.01, -0.02, 0.03;
  const InputError du(0.4, 0.2, -0.3, 0.1);
  const StateError predicted = lin::linearize(reference, input, reference, 0.1).predict(dx, du);
  StateError expected;
  // Positive roll drives +East; positive pitch drives -North; more thrust drives -Down.
  expected << 1.04, 1.98, 3.01,
    0.4 + 0.002 * dynamics::kGravity, -0.2 + 0.001 * dynamics::kGravity, 0.06,
    0.03, -0.05, 0.04;
  EXPECT_LT((predicted - expected).norm(), 1e-15);
}

TEST(Linearization, RotatingReferenceTransportsErrorIntoNextTangentFrame)
{
  const State reference{Vector3d::Zero(), Vector3d::Zero(),
    Eigen::AngleAxisd(0.4, Vector3d::UnitY()).toRotationMatrix()};
  const ControlInput input{0.0, Vector3d(0.0, 0.0, kPi / 2.0)};
  StateError dx = StateError::Zero();
  dx[6] = 0.01;
  const StateError predicted = lin::linearize(
    reference, input, dynamics::predict_step(reference, input, 1.0), 1.0).predict(dx, InputError::Zero());
  StateError expected = StateError::Zero();
  expected[7] = -0.01;
  EXPECT_LT((predicted - expected).norm(), 1e-15);
  const State next = dynamics::predict_step(lin::apply_state_error(reference, dx), input, 1.0);
  const State reference_next = dynamics::predict_step(reference, input, 1.0);
  EXPECT_LT((measure_error(next, reference_next) - predicted).norm(), 1e-15);
}

TEST(Linearization, AllJacobianColumnsMatchNonlinearCentralDifferences)
{
  const Vector3d rate_axis = Vector3d(1.0, -2.0, 3.0).normalized();
  const std::array<double, 6> angles{0.0, 1e-12, 0.000999999, 0.001000001, 1.2, kPi - 1e-7};
  constexpr double h = 1e-6;
  for (std::size_t sample = 0; sample < angles.size(); ++sample)
  {
    SCOPED_TRACE(sample);
    const double dt = 0.02 + 0.03 * sample;
    const double gravity = sample % 2 == 0 ? dynamics::kGravity : 3.7;
    const State reference{Vector3d(1.0, -2.0, 0.5), Vector3d(-0.4, 0.7, -0.2),
      Eigen::AngleAxisd(sample == 5 ? kPi - 1e-8 : 0.6,
        Vector3d(-2.0, 1.0, 3.0).normalized()).toRotationMatrix()};
    const ControlInput input{8.0 + sample, (angles[sample] / dt) * rate_axis};
    State reference_next = dynamics::predict_step(reference, input, dt, gravity);
    // Nonzero defect in every chart, including a noncommuting rotation.
    reference_next.p += Vector3d(0.01, -0.02, 0.005);
    reference_next.v += Vector3d(-0.03, 0.02, 0.01);
    reference_next.R *= Eigen::AngleAxisd(0.07,
      Vector3d(-1.0, 3.0, 2.0).normalized()).toRotationMatrix();
    const auto model = lin::linearize(reference, input, reference_next, dt, gravity);
    EXPECT_LT((model.d - measure_error(
      dynamics::predict_step(reference, input, dt, gravity), reference_next)).norm(), 1e-15);
    EXPECT_LT((model.predict(StateError::Zero(), InputError::Zero()) - model.d).norm(), 1e-15);
    Eigen::Matrix<double, 9, 9> numeric_a;
    for (int column = 0; column < 9; ++column)
    {
      State plus = reference;
      State minus = reference;
      if (column < 3)
      {
        plus.p[column] += h;
        minus.p[column] -= h;
      }
      else if (column < 6)
      {
        plus.v[column - 3] += h;
        minus.v[column - 3] -= h;
      }
      else
      {
        const Vector3d axis = Vector3d::Unit(column - 6);
        plus.R = reference.R * Eigen::AngleAxisd(h, axis).toRotationMatrix();
        minus.R = reference.R * Eigen::AngleAxisd(-h, axis).toRotationMatrix();
      }
      numeric_a.col(column) =
        (measure_error(dynamics::predict_step(plus, input, dt, gravity), reference_next) -
        measure_error(dynamics::predict_step(minus, input, dt, gravity), reference_next)) / (2.0 * h);
    }
    Eigen::Matrix<double, 9, 4> numeric_b;
    for (int column = 0; column < 4; ++column)
    {
      ControlInput plus = input;
      ControlInput minus = input;
      if (column == 0)
      {
        plus.thrust_acceleration += h;
        minus.thrust_acceleration -= h;
      }
      else
      {
        plus.body_rates[column - 1] += h;
        minus.body_rates[column - 1] -= h;
      }
      numeric_b.col(column) =
        (measure_error(dynamics::predict_step(reference, plus, dt, gravity), reference_next) -
        measure_error(dynamics::predict_step(reference, minus, dt, gravity), reference_next)) / (2.0 * h);
    }
    EXPECT_LT((model.A - numeric_a).cwiseAbs().maxCoeff(), 3e-9);
    EXPECT_LT((model.B - numeric_b).cwiseAbs().maxCoeff(), 3e-9);
  }
}

TEST(Linearization, ZeroThrustRemovesAttitudeCouplingFromVelocity)
{
  const State reference{Vector3d(1.0, -2.0, 3.0), Vector3d(-0.3, 0.2, 0.1),
    Eigen::AngleAxisd(0.6, Vector3d::UnitX()).toRotationMatrix()};
  const ControlInput input{0.0, Vector3d(0.2, -0.5, 0.7)};
  StateError dx = StateError::Zero();
  dx.tail<3>() << 0.3, -0.2, 0.1;
  const auto model = lin::linearize(
    reference, input, dynamics::predict_step(reference, input, 0.1), 0.1);
  const StateError predicted = model.predict(dx, InputError::Zero());
  const State next = dynamics::predict_step(lin::apply_state_error(reference, dx), input, 0.1);
  const State reference_next = dynamics::predict_step(reference, input, 0.1);
  EXPECT_EQ(predicted.head<6>().norm(), 0.0);
  EXPECT_LT((measure_error(next, reference_next) - predicted).norm(), 1e-15);
  // A positive one-sided thrust perturbation is valid at the zero-thrust boundary.
  const InputError du(1e-5, 0.0, 0.0, 0.0);
  const State thrust_next = dynamics::predict_step(reference, perturb_input(input, du), 0.1);
  EXPECT_LT((measure_error(thrust_next, reference_next) -
    model.predict(StateError::Zero(), du)).norm(), 1e-15);
}

TEST(Linearization, JointStateAndInputResidualIsSecondOrder)
{
  const State reference{Vector3d(0.2, -0.3, 0.4), Vector3d(0.5, -0.2, 0.1),
    Eigen::AngleAxisd(0.8, Vector3d(1.0, -2.0, 3.0).normalized()).toRotationMatrix()};
  const ControlInput input{11.0, Vector3d(0.8, -0.5, 0.6)};
  constexpr double dt = 0.08;
  State reference_next = dynamics::predict_step(reference, input, dt);
  reference_next.p += Vector3d(0.01, -0.02, 0.005);
  reference_next.v += Vector3d(-0.03, 0.02, 0.01);
  reference_next.R *= Eigen::AngleAxisd(0.08,
    Vector3d(-1.0, 3.0, 2.0).normalized()).toRotationMatrix();
  const auto model = lin::linearize(reference, input, reference_next, dt);
  StateError direction;
  direction << 0.3, -0.2, 0.1, 0.4, 0.2, -0.3, 0.6, -0.4, 0.3;
  const InputError input_direction(1.1, -0.5, 0.7, 0.2);
  double previous_velocity = 0.0;
  double previous_rotation = 0.0;
  for (double scale : {0.02, 0.01, 0.005})
  {
    const StateError dx = scale * direction;
    const InputError du = scale * input_direction;
    const State actual_next = dynamics::predict_step(
      lin::apply_state_error(reference, dx), perturb_input(input, du), dt);
    const StateError residual = measure_error(actual_next, reference_next) - model.predict(dx, du);
    EXPECT_LT(residual.head<3>().norm(), 1e-15);  // Position equation is exactly linear.
    const double velocity = residual.segment<3>(3).norm();
    const double rotation = residual.tail<3>().norm();
    EXPECT_LT(velocity, scale * scale);
    EXPECT_LT(rotation, scale * scale);
    if (previous_velocity > 0.0)
    {
      EXPECT_NEAR(previous_velocity / velocity, 4.0, 0.1);
      EXPECT_NEAR(previous_rotation / rotation, 4.0, 0.1);
    }
    previous_velocity = velocity;
    previous_rotation = rotation;
  }
}

TEST(Linearization, ChainedModelsTrackMovingReferenceWithSecondOrderResidual)
{
  StateError direction;
  direction << 0.3, -0.2, 0.1, 0.4, 0.2, -0.3, 0.6, -0.4, 0.3;
  double previous_residual = 0.0;
  for (double scale : {0.01, 0.005})
  {
    State reference{Vector3d(0.0, 0.0, -2.0), Vector3d(0.1, -0.2, 0.3),
      Eigen::AngleAxisd(0.5, Vector3d::UnitY()).toRotationMatrix()};
    StateError predicted = scale * direction;
    State actual = lin::apply_state_error(reference, predicted);
    for (int step = 0; step < 20; ++step)
    {
      const double phase = 0.2 * step;
      const ControlInput input{10.0 + 0.5 * std::sin(phase),
        Vector3d(0.5 * std::cos(phase), 0.2 * std::sin(phase), -0.3)};
      const InputError du = scale * InputError(0.7, 0.2, -0.4, 0.3);
      predicted = lin::linearize(reference, input,
        dynamics::predict_step(reference, input, 0.02), 0.02).predict(predicted, du);
      actual = dynamics::predict_step(actual, perturb_input(input, du), 0.02);
      reference = dynamics::predict_step(reference, input, 0.02);
    }
    const State reconstructed = lin::apply_state_error(reference, predicted);
    const double residual = measure_error(actual, reconstructed).norm();
    EXPECT_LT(residual, 5.0 * scale * scale);
    if (previous_residual > 0.0)
    {
      EXPECT_NEAR(previous_residual / residual, 4.0, 0.1);
    }
    previous_residual = residual;
  }
}
}  // namespace
