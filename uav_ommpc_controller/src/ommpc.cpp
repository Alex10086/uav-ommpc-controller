#include "uav_ommpc_controller/ommpc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include <osqp.h>

namespace uav_ommpc_controller::ommpc
{
namespace
{
using linearization::InputError;
using linearization::StateError;
constexpr double kReferenceTolerance = 1e-6;
constexpr double kConstraintTolerance = 1e-4;

Settings checked_settings(const Settings &s)
{
  if (s.horizon < 1 || s.horizon > 100 || !std::isfinite(s.dt) || s.dt <= 0.0 ||
    !std::isfinite(s.gravity) || s.gravity <= 0.0 ||
    !s.state_weights.allFinite() || (s.state_weights.array() < 0.0).any() ||
    !s.terminal_weights.allFinite() || (s.terminal_weights.array() < 0.0).any() ||
    !s.input_weights.allFinite() || (s.input_weights.array() <= 0.0).any() ||
    !s.input_min.allFinite() || !s.input_max.allFinite() || s.input_min[0] < 0.0 ||
    (s.input_min.array() >= s.input_max.array()).any() || s.max_iterations < 1 ||
    !std::isfinite(s.absolute_tolerance) || s.absolute_tolerance <= 0.0 ||
    !std::isfinite(s.relative_tolerance) || s.relative_tolerance <= 0.0)
  {
    throw std::invalid_argument("Invalid OMMPC horizon, time step, weights, limits or solver settings");
  }
  return s;
}

bool valid_state(const dynamics::State &state)
{
  return state.p.allFinite() && state.v.allFinite() && state.R.allFinite() &&
    (state.R.transpose() * state.R - Eigen::Matrix3d::Identity()).norm() <= 1e-6 &&
    std::abs(state.R.determinant() - 1.0) <= 1e-6;
}

InputError input_vector(const dynamics::ControlInput &input)
{
  InputError vector;
  vector << input.thrust_acceleration, input.body_rates;
  return vector;
}
}  // namespace

struct Controller::Impl
{
  const Settings settings;
  const int n;
  CondensedQp qp;
  Eigen::VectorXd state_weights, input_weights, free_response, weighted_free;
  Eigen::VectorXd weighted_column;
  Eigen::VectorXd delta_u, delta_x, previous_controls, warm_primal, warm_dual;
  bool have_previous = false;
  std::vector<c_int> p_columns, p_rows, identity_columns, identity_rows;
  std::vector<c_float> p_values, identity_values;
  std::unique_ptr<OSQPWorkspace, decltype(&osqp_cleanup)> workspace{nullptr, &osqp_cleanup};

  explicit Impl(const Settings &config)
  : settings(checked_settings(config)), n(4 * settings.horizon),
    qp{Eigen::MatrixXd::Zero(9 * settings.horizon, n),
      Eigen::MatrixXd::Zero(9 * settings.horizon, 9), Eigen::MatrixXd::Zero(n, n),
      Eigen::VectorXd::Zero(n), Eigen::VectorXd(n), Eigen::VectorXd(n)},
    state_weights(9 * settings.horizon), input_weights(n),
    free_response(9 * settings.horizon), weighted_free(9 * settings.horizon),
    weighted_column(9 * settings.horizon), delta_u(n), delta_x(9 * settings.horizon),
    previous_controls(n), warm_primal(n), warm_dual(Eigen::VectorXd::Zero(n)),
    p_columns(n + 1), p_rows(n * (n + 1) / 2), identity_columns(n + 1), identity_rows(n),
    p_values(n * (n + 1) / 2, 0.0), identity_values(n, 1.0)
  {
    static_assert(std::is_same<c_float, double>::value, "OMMPC requires double-precision OSQP");
    for (int k = 0; k < settings.horizon; ++k)
    {
      state_weights.segment<9>(9 * k) =
        k == settings.horizon - 1 ? settings.terminal_weights : settings.state_weights;
      input_weights.segment<4>(4 * k) = settings.input_weights;
      qp.lower.segment<4>(4 * k) = settings.input_min;
      qp.upper.segment<4>(4 * k) = settings.input_max;
    }
    // Dense upper-triangular P pattern and identity constraints stay fixed forever.
    c_int entry = 0;
    for (int column = 0; column < n; ++column)
    {
      p_columns[column] = entry;
      for (int row = 0; row <= column; ++row)
      {
        p_rows[entry] = row;
        p_values[entry++] = row == column ? 2.0 * input_weights[column] : 0.0;
      }
      identity_columns[column] = column;
      identity_rows[column] = column;
    }
    p_columns[n] = entry;
    identity_columns[n] = n;
    csc p{entry, n, n, p_columns.data(), p_rows.data(), p_values.data(), -1};
    csc identity{n, n, n, identity_columns.data(), identity_rows.data(), identity_values.data(), -1};
    OSQPData data{n, n, &p, &identity, qp.q.data(), qp.lower.data(), qp.upper.data()};
    OSQPSettings solver_settings;
    osqp_set_default_settings(&solver_settings);
    solver_settings.verbose = false;
    solver_settings.polish = false;  // Avoid polish's extra factorization/allocations.
    solver_settings.warm_start = true;
    solver_settings.max_iter = settings.max_iterations;
    solver_settings.eps_abs = settings.absolute_tolerance;
    solver_settings.eps_rel = settings.relative_tolerance;
    solver_settings.check_termination = 10;
    solver_settings.adaptive_rho_interval = 25;  // Deterministic, not timing-derived.
    OSQPWorkspace *raw = nullptr;
    const c_int status = osqp_setup(&raw, &data, &solver_settings);
    workspace.reset(raw);
    if (status != 0)
    {
      throw std::runtime_error("OSQP workspace initialization failed");
    }
    invalidate_solution();
  }

  void invalidate_solution()
  {
    delta_u.setConstant(std::numeric_limits<double>::quiet_NaN());
    delta_x.setConstant(std::numeric_limits<double>::quiet_NaN());
    have_previous = false;
  }

  Status validate(const dynamics::State &actual, const std::vector<dynamics::State> &reference,
    const std::vector<dynamics::ControlInput> &feedforward) const
  {
    if (reference.size() != static_cast<std::size_t>(settings.horizon + 1) ||
      feedforward.size() != static_cast<std::size_t>(settings.horizon) || !valid_state(actual))
    {
      return Status::InvalidInput;
    }
    for (const auto &state : reference)
    {
      if (!valid_state(state)) return Status::InvalidInput;
    }
    for (int k = 0; k < settings.horizon; ++k)
    {
      const InputError input = input_vector(feedforward[k]);
      if (!input.allFinite() || (input.array() < settings.input_min.array()).any() ||
        (input.array() > settings.input_max.array()).any()) return Status::InvalidInput;
      const StateError defect = linearization::state_error(
        dynamics::predict_step(reference[k], feedforward[k], settings.dt, settings.gravity), reference[k + 1]);
      if (!defect.allFinite() || defect.head<3>().norm() > kReferenceTolerance ||
        defect.segment<3>(3).norm() > kReferenceTolerance || defect.tail<3>().norm() > kReferenceTolerance)
      {
        return Status::ReferenceMismatch;
      }
    }
    return Status::Solved;
  }

  void condense(const StateError &initial_error, const std::vector<dynamics::State> &reference,
    const std::vector<dynamics::ControlInput> &feedforward)
  {
    // Rows are x1..xN. Only causal blocks are written; future-input blocks stay zero.
    for (int k = 0; k < settings.horizon; ++k)
    {
      const auto model = linearization::linearize(reference[k], feedforward[k], settings.dt);
      if (k == 0)
      {
        qp.H.topRows<9>() = model.A;
      }
      else
      {
        qp.H.middleRows<9>(9 * k).noalias() = model.A * qp.H.middleRows<9>(9 * (k - 1));
        qp.M.block(9 * k, 0, 9, 4 * k).noalias() = model.A * qp.M.block(9 * (k - 1), 0, 9, 4 * k);
      }
      qp.M.block<9, 4>(9 * k, 4 * k) = model.B;
      const InputError nominal = input_vector(feedforward[k]);
      qp.lower.segment<4>(4 * k) = settings.input_min - nominal;
      qp.upper.segment<4>(4 * k) = settings.input_max - nominal;
      // Shift absolute controls, then express them relative to the NEW reference.
      warm_primal.segment<4>(4 * k) = have_previous ?
        InputError(previous_controls.segment<4>(4 * std::min(k + 1, settings.horizon - 1)) - nominal) :
        InputError::Zero();
      warm_primal.segment<4>(4 * k) = warm_primal.segment<4>(4 * k)
        .cwiseMax(qp.lower.segment<4>(4 * k)).cwiseMin(qp.upper.segment<4>(4 * k));
    }
    free_response.noalias() = qp.H * initial_error;
    weighted_free = state_weights.cwiseProduct(free_response);
    c_int entry = 0;
    for (int column = 0; column < n; ++column)
    {
      // Column j affects only x_{j/4+1}..xN. Dot products avoid Eigen GEMM's
      // heap scratch space and skip the structurally zero prefix.
      const int rows = 9 * (settings.horizon - column / 4);
      weighted_column.tail(rows) =
        state_weights.tail(rows).cwiseProduct(qp.M.col(column).tail(rows));
      qp.q[column] = 2.0 * qp.M.col(column).tail(rows).dot(weighted_free.tail(rows));
      for (int row = 0; row <= column; ++row)
      {
        double value = 2.0 * qp.M.col(row).tail(rows).dot(weighted_column.tail(rows));
        if (row == column) value += 2.0 * input_weights[column];
        qp.P(row, column) = value;
        qp.P(column, row) = value;
        p_values[entry++] = value;
      }
    }
  }
};

Controller::Controller(const Settings &settings) : impl_(std::make_unique<Impl>(settings)) {}
Controller::~Controller() = default;

Result Controller::solve(const dynamics::State &actual, const std::vector<dynamics::State> &reference,
  const std::vector<dynamics::ControlInput> &feedforward)
{
  auto &w = *impl_;
  Result result;
  result.status = w.validate(actual, reference, feedforward);
  if (result.status != Status::Solved)
  {
    w.invalidate_solution();
    return result;
  }
  w.condense(linearization::state_error(actual, reference.front()), reference, feedforward);
  result.status = Status::SolverFailure;
  if (!w.qp.P.allFinite() || !w.qp.q.allFinite() || !w.qp.lower.allFinite() || !w.qp.upper.allFinite() ||
    osqp_update_P(w.workspace.get(), w.p_values.data(), nullptr, static_cast<c_int>(w.p_values.size())) != 0 ||
    osqp_update_lin_cost(w.workspace.get(), w.qp.q.data()) != 0 ||
    osqp_update_bounds(w.workspace.get(), w.qp.lower.data(), w.qp.upper.data()) != 0 ||
    osqp_warm_start(w.workspace.get(), w.warm_primal.data(), w.warm_dual.data()) != 0)
  {
    w.invalidate_solution();
    return result;
  }
  const c_int exit_code = osqp_solve(w.workspace.get());
  result.solver_status = static_cast<int>(w.workspace->info->status_val);
  result.iterations = static_cast<int>(w.workspace->info->iter);
  if (exit_code != 0 || result.solver_status != OSQP_SOLVED)
  {
    w.invalidate_solution();
    return result;
  }
  w.delta_u = Eigen::Map<const Eigen::VectorXd>(w.workspace->solution->x, w.n);
  if (!w.delta_u.allFinite())
  {
    w.invalidate_solution();
    return result;
  }
  result.max_constraint_violation = std::max({0.0,
    (w.qp.lower - w.delta_u).maxCoeff(), (w.delta_u - w.qp.upper).maxCoeff()});
  if (result.max_constraint_violation > kConstraintTolerance)
  {
    w.invalidate_solution();
    return result;
  }
  w.delta_x.noalias() = w.qp.M * w.delta_u;
  w.delta_x += w.free_response;
  if (!w.delta_x.allFinite())
  {
    w.invalidate_solution();
    return result;
  }
  for (int k = 0; k < w.settings.horizon; ++k)
  {
    w.previous_controls.segment<4>(4 * k) = w.delta_u.segment<4>(4 * k) + input_vector(feedforward[k]);
  }
  const InputError first = w.previous_controls.head<4>();
  result.command = dynamics::ControlInput{first[0], first.tail<3>()};
  result.status = Status::Solved;
  w.have_previous = true;
  return result;
}

const CondensedQp &Controller::problem() const { return impl_->qp; }
const Eigen::VectorXd &Controller::input_errors() const { return impl_->delta_u; }
const Eigen::VectorXd &Controller::predicted_errors() const { return impl_->delta_x; }
}  // namespace uav_ommpc_controller::ommpc
