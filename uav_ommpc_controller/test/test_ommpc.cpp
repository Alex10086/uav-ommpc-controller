#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>
#include <gtest/gtest.h>

#include "uav_ommpc_controller/ommpc.hpp"

namespace
{
namespace dynamics = uav_ommpc_controller::dynamics;
namespace lin = uav_ommpc_controller::linearization;
namespace mpc = uav_ommpc_controller::ommpc;
using dynamics::ControlInput;
using dynamics::State;
using Eigen::Vector3d;
using lin::InputError;
using lin::StateError;
constexpr int kOsqpSolved = 1;  // OSQP 0.6.3 native status.

struct Reference
{
  std::vector<State> states;
  std::vector<ControlInput> inputs;
};

Reference make_reference(const mpc::Settings &settings, bool moving = false, double phase = 0.0)
{
  Reference reference;
  State initial{Vector3d(0.0, 0.0, -2.0), Vector3d::Zero(), Eigen::Matrix3d::Identity()};
  if (moving)
  {
    initial.v = Vector3d(0.2, -0.1, 0.05);
    initial.R = Eigen::AngleAxisd(0.12 + phase * 0.1,
      Vector3d(1.0, -2.0, 3.0).normalized()).toRotationMatrix();
  }
  reference.states.push_back(initial);
  for (int k = 0; k < settings.horizon; ++k)
  {
    const double t = phase + 0.37 * k;
    const ControlInput input = moving ?
      ControlInput{9.4 + 0.4 * std::sin(t),
        Vector3d(0.25 * std::cos(t), -0.3 * std::sin(t), 0.15 + 0.1 * std::cos(t))} :
      ControlInput{settings.gravity, Vector3d::Zero()};
    reference.inputs.push_back(input);
    reference.states.push_back(dynamics::predict_step(
      reference.states.back(), input, settings.dt, settings.gravity));
  }
  return reference;
}

InputError pack(const ControlInput &input)
{
  InputError result;
  result << input.thrust_acceleration, input.body_rates;
  return result;
}

Eigen::VectorXd recursive_prediction(const mpc::Settings &settings, const Reference &reference,
  StateError error, const Eigen::VectorXd &inputs)
{
  Eigen::VectorXd states(9 * settings.horizon);
  for (int k = 0; k < settings.horizon; ++k)
  {
    error = lin::linearize(reference.states[k], reference.inputs[k], settings.dt)
      .predict(error, inputs.segment<4>(4 * k));
    states.segment<9>(9 * k) = error;
  }
  return states;
}

double recursive_cost(const mpc::Settings &settings, const Reference &reference,
  const StateError &initial_error, const Eigen::VectorXd &inputs)
{
  const Eigen::VectorXd states = recursive_prediction(settings, reference, initial_error, inputs);
  double cost = 0.0;
  for (int k = 0; k < settings.horizon; ++k)
  {
    const StateError &weights = k + 1 == settings.horizon ?
      settings.terminal_weights : settings.state_weights;
    cost += (states.segment<9>(9 * k).array().square() * weights.array()).sum();
    cost += (inputs.segment<4>(4 * k).array().square() * settings.input_weights.array()).sum();
  }
  return cost;
}

void expect_failure(const mpc::Controller &controller, const mpc::Result &result,
  mpc::Status expected, int horizon)
{
  EXPECT_EQ(result.status, expected);
  EXPECT_FALSE(result.command.has_value());
  ASSERT_EQ(controller.input_errors().size(), 4 * horizon);
  ASSERT_EQ(controller.predicted_errors().size(), 9 * horizon);
  EXPECT_TRUE(controller.input_errors().array().isNaN().all());
  EXPECT_TRUE(controller.predicted_errors().array().isNaN().all());
}

TEST(Ommpc, TimeVaryingCondensationAndObjectiveMatchRecursiveModels)
{
  mpc::Settings settings;
  settings.horizon = 4;
  settings.dt = 0.08;
  settings.state_weights << 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0;
  settings.terminal_weights << 31.0, 29.0, 23.0, 19.0, 17.0, 13.0, 11.0, 7.0, 5.0;
  settings.input_weights << 0.7, 1.3, 1.9, 2.3;
  const auto reference = make_reference(settings, true);
  StateError initial;
  initial << 0.03, -0.02, 0.01, -0.04, 0.02, 0.03, 0.01, -0.02, 0.015;
  mpc::Controller controller(settings);
  ASSERT_EQ(controller.solve(lin::apply_state_error(reference.states.front(), initial),
    reference.states, reference.inputs).status, mpc::Status::Solved);
  const auto &qp = controller.problem();
  const int dimension = 4 * settings.horizon;
  ASSERT_EQ(qp.M.rows(), 9 * settings.horizon);
  ASSERT_EQ(qp.M.cols(), dimension);
  ASSERT_EQ(qp.H.rows(), 9 * settings.horizon);
  ASSERT_EQ(qp.H.cols(), 9);
  ASSERT_EQ(qp.P.rows(), dimension);
  ASSERT_EQ(qp.P.cols(), dimension);
  ASSERT_EQ(qp.q.size(), dimension);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(dimension);
  for (int column = 0; column < 9; ++column)
  {
    const StateError basis = StateError::Unit(column);
    EXPECT_LT((qp.H.col(column) - recursive_prediction(settings, reference, basis, zero)).norm(),
      1e-12);
  }
  const double baseline = recursive_cost(settings, reference, initial, zero);
  for (int column = 0; column < dimension; ++column)
  {
    const Eigen::VectorXd basis = Eigen::VectorXd::Unit(dimension, column);
    EXPECT_LT((qp.M.col(column) - recursive_prediction(
      settings, reference, StateError::Zero(), basis)).norm(), 1e-12);
    const double plus = recursive_cost(settings, reference, initial, basis);
    const double minus = recursive_cost(settings, reference, initial, -basis);
    EXPECT_NEAR(qp.q[column], (plus - minus) / 2.0, 1e-10);
    EXPECT_NEAR(qp.P(column, column), plus + minus - 2.0 * baseline, 1e-10);
    for (int other = 0; other < column; ++other)
    {
      const Eigen::VectorXd other_basis = Eigen::VectorXd::Unit(dimension, other);
      const double mixed = recursive_cost(settings, reference, initial, basis + other_basis) -
        plus - recursive_cost(settings, reference, initial, other_basis) + baseline;
      EXPECT_NEAR(qp.P(column, other), mixed, 1e-10);
      EXPECT_NEAR(qp.P(other, column), mixed, 1e-10);
    }
  }
  const Eigen::VectorXd trial = Eigen::VectorXd::LinSpaced(dimension, -0.2, 0.3);
  EXPECT_NEAR(0.5 * trial.dot(qp.P * trial) + qp.q.dot(trial),
    recursive_cost(settings, reference, initial, trial) - baseline, 1e-10);
  EXPECT_LT((controller.predicted_errors() - recursive_prediction(
    settings, reference, initial, controller.input_errors())).norm(), 1e-10);
}

TEST(Ommpc, WideBoxSolutionMatchesUnconstrainedLdltMinimum)
{
  mpc::Settings settings;
  settings.horizon = 8;
  settings.input_min << 0.0, -100.0, -100.0, -100.0;
  settings.input_max << 100.0, 100.0, 100.0, 100.0;
  const auto reference = make_reference(settings, true);
  StateError initial;
  initial << 0.015, -0.01, 0.02, 0.01, -0.02, 0.015, 0.01, -0.015, 0.02;
  mpc::Controller controller(settings);
  const auto result = controller.solve(lin::apply_state_error(reference.states.front(), initial),
    reference.states, reference.inputs);
  ASSERT_EQ(result.status, mpc::Status::Solved);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(result.solver_status, kOsqpSolved);
  const auto &qp = controller.problem();
  Eigen::LDLT<Eigen::MatrixXd> factor(qp.P);
  ASSERT_EQ(factor.info(), Eigen::Success);
  const Eigen::VectorXd expected = factor.solve(-qp.q);
  ASSERT_GT((expected - qp.lower).minCoeff(), 1.0);
  ASSERT_GT((qp.upper - expected).minCoeff(), 1.0);
  EXPECT_LT((controller.input_errors() - expected).lpNorm<Eigen::Infinity>(), 1e-4);
  EXPECT_LT((pack(*result.command) - pack(reference.inputs.front()) - expected.head<4>()).norm(),
    2e-4);
}

TEST(Ommpc, ZeroHoverErrorReturnsExactFeedforward)
{
  mpc::Settings settings;
  const auto reference = make_reference(settings);
  mpc::Controller controller(settings);
  const auto result = controller.solve(reference.states.front(), reference.states, reference.inputs);
  ASSERT_EQ(result.status, mpc::Status::Solved);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(result.solver_status, kOsqpSolved);
  EXPECT_LT((pack(*result.command) - pack(reference.inputs.front())).norm(), 1e-10);
  EXPECT_LT(controller.input_errors().norm(), 1e-10);
  EXPECT_LT(controller.predicted_errors().norm(), 1e-10);
  EXPECT_LE(result.max_constraint_violation, 1e-4);
}

TEST(Ommpc, ShiftedNonzeroFeedforwardBoxIsEnforcedWithActiveConstraints)
{
  mpc::Settings settings;
  settings.max_iterations = 4000;
  const auto reference = make_reference(settings, true, 0.7);
  State actual = reference.states.front();
  actual.p += Vector3d(3.0, -2.0, 2.0);
  actual.v += Vector3d(0.4, -0.3, 0.5);
  mpc::Controller controller(settings);
  const auto result = controller.solve(actual, reference.states, reference.inputs);
  ASSERT_EQ(result.status, mpc::Status::Solved);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(result.solver_status, kOsqpSolved);
  const auto &qp = controller.problem();
  ASSERT_EQ(qp.lower.size(), 4 * settings.horizon);
  ASSERT_EQ(qp.upper.size(), 4 * settings.horizon);
  double closest_boundary = std::numeric_limits<double>::infinity();
  double violation = 0.0;
  for (int k = 0; k < settings.horizon; ++k)
  {
    const InputError feedforward = pack(reference.inputs[k]);
    EXPECT_LT((qp.lower.segment<4>(4 * k) - (settings.input_min - feedforward)).norm(), 1e-12);
    EXPECT_LT((qp.upper.segment<4>(4 * k) - (settings.input_max - feedforward)).norm(), 1e-12);
    const InputError input = feedforward + controller.input_errors().segment<4>(4 * k);
    for (int j = 0; j < 4; ++j)
    {
      EXPECT_GE(input[j], settings.input_min[j] - 1e-4);
      EXPECT_LE(input[j], settings.input_max[j] + 1e-4);
      closest_boundary = std::min(closest_boundary, std::min(
        std::abs(input[j] - settings.input_min[j]), std::abs(input[j] - settings.input_max[j])));
      violation = std::max(violation,
        std::max(settings.input_min[j] - input[j], input[j] - settings.input_max[j]));
    }
  }
  EXPECT_LT(closest_boundary, 1e-4);
  EXPECT_NEAR(result.max_constraint_violation, violation, 1e-10);
  EXPECT_LE(result.max_constraint_violation, 1e-4);
  EXPECT_LT((pack(*result.command) - pack(reference.inputs.front()) -
    controller.input_errors().head<4>()).norm(), 1e-12);
}

TEST(Ommpc, ReusedWorkspaceUpdatesReferencesMatricesAndGradientWithoutStaleSolution)
{
  mpc::Settings settings;
  settings.horizon = 7;
  const auto first = make_reference(settings);
  const auto second = make_reference(settings, true, 1.3);
  mpc::Controller reused(settings);
  State actual_first = first.states.front();
  actual_first.p.x() += 0.2;
  ASSERT_EQ(reused.solve(actual_first, first.states, first.inputs).status, mpc::Status::Solved);
  const Eigen::VectorXd old_solution = reused.input_errors();
  State actual_second = second.states.front();
  actual_second.p += Vector3d(-0.1, 0.07, -0.08);
  actual_second.v += Vector3d(0.03, -0.04, 0.02);
  const auto reused_result = reused.solve(actual_second, second.states, second.inputs);
  mpc::Controller fresh(settings);
  const auto fresh_result = fresh.solve(actual_second, second.states, second.inputs);
  ASSERT_EQ(reused_result.status, mpc::Status::Solved);
  ASSERT_EQ(fresh_result.status, mpc::Status::Solved);
  ASSERT_TRUE(reused_result.command.has_value());
  ASSERT_TRUE(fresh_result.command.has_value());
  EXPECT_GT((old_solution - reused.input_errors()).norm(), 1e-2);
  EXPECT_LT((reused.input_errors() - fresh.input_errors()).lpNorm<Eigen::Infinity>(), 2e-4);
  EXPECT_LT((pack(*reused_result.command) - pack(*fresh_result.command)).norm(), 3e-4);
  EXPECT_LT((reused.predicted_errors() - fresh.predicted_errors()).norm(), 3e-4);
  EXPECT_LT((reused.problem().P - fresh.problem().P).norm(), 1e-12);
  EXPECT_LT((reused.problem().q - fresh.problem().q).norm(), 1e-12);
  EXPECT_LT((reused.problem().lower - fresh.problem().lower).norm(), 1e-12);
  EXPECT_LT((reused.problem().upper - fresh.problem().upper).norm(), 1e-12);
}

TEST(Ommpc, SingleStepUsesOnlyTerminalStateWeight)
{
  mpc::Settings settings;
  settings.horizon = 1;
  settings.state_weights.setConstant(1e7);
  settings.terminal_weights << 2.0, 3.0, 5.0, 7.0, 11.0, 13.0, 17.0, 19.0, 23.0;
  const auto reference = make_reference(settings, true);
  StateError initial = StateError::Constant(0.01);
  mpc::Controller controller(settings);
  ASSERT_EQ(controller.solve(lin::apply_state_error(reference.states.front(), initial),
    reference.states, reference.inputs).status, mpc::Status::Solved);
  const auto model = lin::linearize(reference.states.front(), reference.inputs.front(), settings.dt);
  Eigen::Matrix4d expected = model.B.transpose() * settings.terminal_weights.asDiagonal() * model.B;
  expected.diagonal() += settings.input_weights;
  expected *= 2.0;
  const InputError expected_q = 2.0 * model.B.transpose() *
    settings.terminal_weights.asDiagonal() * model.A * initial;
  EXPECT_LT((controller.problem().M - model.B).norm(), 1e-12);
  EXPECT_LT((controller.problem().H - model.A).norm(), 1e-12);
  EXPECT_LT((controller.problem().P - expected).norm(), 1e-12);
  EXPECT_LT((controller.problem().q - expected_q).norm(), 1e-12);
  EXPECT_LT((controller.input_errors() - expected.ldlt().solve(-expected_q)).norm(), 1e-4);
}

TEST(Ommpc, ReferenceMismatchClearsPreviousCommandAndSolutionForEveryStateComponent)
{
  mpc::Settings settings;
  settings.horizon = 4;
  const auto reference = make_reference(settings, true);
  mpc::Controller controller(settings);
  for (int component = 0; component < 3; ++component)
  {
    SCOPED_TRACE(component);
    ASSERT_EQ(controller.solve(reference.states.front(), reference.states, reference.inputs).status,
      mpc::Status::Solved);
    auto inconsistent = reference.states;
    if (component == 0) inconsistent[2].p.x() += 1e-4;
    if (component == 1) inconsistent[2].v.y() += 1e-4;
    if (component == 2)
      inconsistent[2].R *= Eigen::AngleAxisd(1e-4, Vector3d::UnitZ()).toRotationMatrix();
    expect_failure(controller, controller.solve(reference.states.front(), inconsistent,
      reference.inputs), mpc::Status::ReferenceMismatch, settings.horizon);
  }
}

TEST(Ommpc, InvalidLengthsNonfiniteStatesBadRotationsAndInputsClearPriorSolution)
{
  mpc::Settings settings;
  settings.horizon = 3;
  const auto reference = make_reference(settings);
  mpc::Controller controller(settings);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (int scenario = 0; scenario < 13; ++scenario)
  {
    SCOPED_TRACE(scenario);
    ASSERT_EQ(controller.solve(reference.states.front(), reference.states, reference.inputs).status,
      mpc::Status::Solved);
    auto states = reference.states;
    auto inputs = reference.inputs;
    State actual = reference.states.front();
    switch (scenario)
    {
      case 0: states.pop_back(); break;
      case 1: states.push_back(states.back()); break;
      case 2: inputs.pop_back(); break;
      case 3: inputs.push_back(inputs.back()); break;
      case 4: actual.p.x() = nan; break;
      case 5: actual.v.y() = inf; break;
      case 6: actual.R(0, 0) = 2.0; break;
      case 7: actual.R(0, 0) = -1.0; break;  // Reflection is not SO(3).
      case 8: states.back().p.z() = nan; break;
      case 9: states.back().R(1, 1) = nan; break;
      case 10: inputs.back().thrust_acceleration = nan; break;
      case 11: inputs.back().body_rates.x() = inf; break;
      case 12: inputs.back().thrust_acceleration = settings.input_max[0] + 1.0; break;
    }
    expect_failure(controller, controller.solve(actual, states, inputs),
      mpc::Status::InvalidInput, settings.horizon);
  }
  EXPECT_EQ(controller.solve(reference.states.front(), reference.states, reference.inputs).status,
    mpc::Status::Solved);  // A rejected call does not poison subsequent valid solves.
}

TEST(Ommpc, IterationLimitNeverReturnsCommandOrPreviousFiniteSolution)
{
  mpc::Settings settings;
  settings.max_iterations = 1;
  const auto reference = make_reference(settings);
  mpc::Controller controller(settings);
  ASSERT_EQ(controller.solve(reference.states.front(), reference.states, reference.inputs).status,
    mpc::Status::Solved);
  State actual = reference.states.front();
  actual.p += Vector3d(2.0, -1.5, 1.0);
  actual.v += Vector3d(0.5, -0.3, 0.4);
  const auto result = controller.solve(actual, reference.states, reference.inputs);
  expect_failure(controller, result, mpc::Status::SolverFailure, settings.horizon);
  EXPECT_NE(result.solver_status, kOsqpSolved);
  EXPECT_EQ(result.iterations, 1);
}

TEST(Ommpc, InvalidSettingsThrowBeforeCreatingAController)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<std::function<void(mpc::Settings &)>> invalid{
    [](auto &s) { s.horizon = 0; },
    [](auto &s) { s.horizon = 101; },
    [](auto &s) { s.dt = 0.0; },
    [](auto &s) { s.dt = -0.02; },
    [nan](auto &s) { s.dt = nan; },
    [inf](auto &s) { s.dt = inf; },
    [](auto &s) { s.gravity = 0.0; },
    [nan](auto &s) { s.gravity = nan; },
    [inf](auto &s) { s.gravity = inf; },
    [](auto &s) { s.state_weights[0] = -1.0; },
    [nan](auto &s) { s.state_weights[4] = nan; },
    [](auto &s) { s.terminal_weights[8] = -1.0; },
    [inf](auto &s) { s.terminal_weights[2] = inf; },
    [](auto &s) { s.input_weights[0] = 0.0; },
    [](auto &s) { s.input_weights[1] = -1.0; },
    [nan](auto &s) { s.input_weights[2] = nan; },
    [inf](auto &s) { s.input_weights[3] = inf; },
    [](auto &s) { s.input_min[0] = -0.1; },
    [](auto &s) { s.input_max[1] = s.input_min[1]; },
    [](auto &s) { s.input_min[2] = s.input_max[2] + 1.0; },
    [nan](auto &s) { s.input_min[3] = nan; },
    [inf](auto &s) { s.input_max[3] = inf; },
    [](auto &s) { s.max_iterations = 0; },
    [](auto &s) { s.absolute_tolerance = 0.0; },
    [](auto &s) { s.relative_tolerance = -1.0; },
    [nan](auto &s) { s.absolute_tolerance = nan; },
    [inf](auto &s) { s.relative_tolerance = inf; }
  };
  for (std::size_t i = 0; i < invalid.size(); ++i)
  {
    SCOPED_TRACE(i);
    mpc::Settings settings;
    invalid[i](settings);
    EXPECT_THROW({ mpc::Controller controller(settings); }, std::invalid_argument);
  }
}
}  // namespace
