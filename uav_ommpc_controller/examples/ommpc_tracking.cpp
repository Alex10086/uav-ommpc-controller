#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "uav_ommpc_controller/ommpc.hpp"

namespace
{
namespace dynamics = uav_ommpc_controller::dynamics;
namespace ommpc = uav_ommpc_controller::ommpc;
namespace so3 = uav_ommpc_controller::so3;

bool finite(const dynamics::State &state)
{
  return state.p.allFinite() && state.v.allFinite() && state.R.allFinite();
}

bool finite(const dynamics::ControlInput &input)
{
  return std::isfinite(input.thrust_acceleration) && input.body_rates.allFinite();
}

double input_violation(const dynamics::ControlInput &input, const ommpc::Settings &settings)
{
  const Eigen::Vector4d value{input.thrust_acceleration, input.body_rates.x(),
    input.body_rates.y(), input.body_rates.z()};
  return std::max(0.0, std::max((settings.input_min - value).maxCoeff(),
    (value - settings.input_max).maxCoeff()));
}

struct ErrorStatistics
{
  std::size_t count = 0;
  double squared_sum = 0.0;
  double maximum = 0.0;

  void add(double error)
  {
    ++count;
    squared_sum += error * error;
    maximum = std::max(maximum, error);
  }

  double rmse() const
  {
    return count == 0 ? std::numeric_limits<double>::quiet_NaN() :
      std::sqrt(squared_sum / static_cast<double>(count));
  }
};

int run()
{
  const ommpc::Settings settings;
  constexpr double radius = 5.0;
  constexpr double speed = 5.0;
  constexpr double altitude = 2.0;
  constexpr double duration = 12.0;
  constexpr double settling_time = 2.0;
  constexpr double violation_tolerance = 1e-4;
  constexpr double steady_rmse_limit = 0.15;
  constexpr double deadline_ms = 20.0;
  const auto steps = static_cast<std::size_t>(std::ceil(duration / settings.dt));
  const auto horizon = static_cast<std::size_t>(settings.horizon);
  const std::size_t state_count = steps + horizon + 1;
  const double angular_speed = speed / radius;

  std::cout << std::fixed << std::setprecision(9)
            << "scope=nonlinear_discrete_model_only_no_ROS_no_PX4_no_hardware\n"
            << "timing_scope=steady_clock_solve_calls_not_a_hard_real_time_guarantee\n"
            << "trajectory=circle_fixed_altitude_fixed_yaw\n"
            << "frame=NED_FRD\n"
            << "radius_m=" << radius << '\n'
            << "analytic_speed_m_s=" << speed << '\n'
            << "altitude_m=" << altitude << '\n'
            << "yaw_rad=0\n"
            << "dt_s=" << settings.dt << '\n'
            << "horizon=" << settings.horizon << '\n'
            << "duration_s=" << steps * settings.dt << '\n'
            << "steady_state_start_s=" << settling_time << '\n'
            << "steady_state_rmse_limit_m=" << steady_rmse_limit << '\n'
            << "deadline_ms=" << deadline_ms << '\n';

  // Two extra positions supply forward differences through the final rotation.
  std::vector<Eigen::Vector3d> positions(state_count + 2);
  for (std::size_t k = 0; k < positions.size(); ++k)
  {
    const double phase = angular_speed * static_cast<double>(k) * settings.dt;
    positions[k] = Eigen::Vector3d(radius * std::cos(phase),
      radius * std::sin(phase), -altitude);
  }

  std::vector<dynamics::State> states(state_count);
  std::vector<dynamics::ControlInput> inputs(state_count - 1);
  for (std::size_t k = 0; k < states.size(); ++k)
  {
    auto &state = states[k];
    state.p = positions[k];
    state.v = (positions[k + 1] - positions[k]) / settings.dt;
    const Eigen::Vector3d next_velocity = (positions[k + 2] - positions[k + 1]) / settings.dt;
    const Eigen::Vector3d acceleration = (next_velocity - state.v) / settings.dt;
    const Eigen::Vector3d thrust_vector =
      settings.gravity * Eigen::Vector3d::UnitZ() - acceleration;
    const double thrust = thrust_vector.norm();
    state.R.col(2) = thrust_vector / thrust;
    // The body X axis lies in the world XZ plane with positive X: yaw is zero.
    state.R.col(0) = Eigen::Vector3d::UnitY().cross(state.R.col(2)).normalized();
    state.R.col(1) = state.R.col(2).cross(state.R.col(0));
    if (!finite(state) || !std::isfinite(thrust))
      throw std::runtime_error("nonfinite reference state");
    if (k < inputs.size())
      inputs[k].thrust_acceleration = thrust;
  }

  double reference_violation = 0.0;
  double reference_residual = 0.0;
  for (std::size_t k = 0; k < inputs.size(); ++k)
  {
    inputs[k].body_rates = so3::log(states[k].R.transpose() * states[k + 1].R) / settings.dt;
    if (!finite(inputs[k]))
      throw std::runtime_error("nonfinite reference input");
    reference_violation = std::max(reference_violation, input_violation(inputs[k], settings));
    const auto next = dynamics::predict_step(states[k], inputs[k], settings.dt, settings.gravity);
    const double residual = std::max({(next.p - states[k + 1].p).norm(),
      (next.v - states[k + 1].v).norm(), so3::boxminus(next.R, states[k + 1].R).norm()});
    if (!finite(next) || !std::isfinite(residual))
      throw std::runtime_error("nonfinite reference dynamics residual");
    reference_residual = std::max(reference_residual, residual);
  }
  std::cout << "reference_max_input_violation=" << reference_violation << '\n'
            << "reference_max_dynamics_residual=" << reference_residual << '\n';
  if (reference_violation > 0.0)
    throw std::runtime_error("circle reference exceeds configured input bounds");
  if (reference_residual > 1e-6)
    throw std::runtime_error("circle reference is not discretely dynamically consistent");

  ommpc::Controller controller(settings);
  std::vector<dynamics::State> reference(horizon + 1);
  std::vector<dynamics::ControlInput> feedforward(horizon);
  std::vector<double> solve_times_ms;
  solve_times_ms.reserve(steps);
  dynamics::State actual = states.front();
  actual.p += Eigen::Vector3d(0.2, -0.15, 0.1);
  actual.v += Eigen::Vector3d(0.1, -0.08, 0.05);
  actual.R = so3::boxplus(actual.R, Eigen::Vector3d(0.025, -0.02, 0.015));

  ErrorStatistics all_errors;
  ErrorStatistics steady_errors;
  double peak_speed = actual.v.norm();
  double max_violation = 0.0;
  std::size_t solver_failures = 0;
  std::size_t deadline_misses = 0;
  std::size_t completed_steps = 0;
  bool nonfinite = false;
  all_errors.add((actual.p - states.front().p).norm());

  // All scenario buffers are allocated above; only the controller may allocate.
  for (std::size_t k = 0; k < steps; ++k)
  {
    std::copy_n(states.begin() + k, horizon + 1, reference.begin());
    std::copy_n(inputs.begin() + k, horizon, feedforward.begin());
    const auto start = std::chrono::steady_clock::now();
    const auto result = controller.solve(actual, reference, feedforward);
    const auto finish = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(finish - start).count();
    solve_times_ms.push_back(elapsed_ms);
    if (elapsed_ms > deadline_ms)
      ++deadline_misses;
    if (result.status != ommpc::Status::Solved || !result.command)
    {
      ++solver_failures;
      std::cerr << "solve_failure_step=" << k << " controller_status="
                << static_cast<int>(result.status) << " solver_status=" << result.solver_status << '\n';
      break;  // No fallback or fabricated command after solver failure.
    }
    if (!finite(*result.command) || !std::isfinite(result.max_constraint_violation) ||
        !std::isfinite(elapsed_ms))
    {
      nonfinite = true;
      break;
    }
    max_violation = std::max(max_violation, input_violation(*result.command, settings));
    max_violation = std::max(max_violation, result.max_constraint_violation);
    if (max_violation > violation_tolerance)
      break;
    actual = dynamics::predict_step(actual, *result.command, settings.dt, settings.gravity);
    const double error = (actual.p - states[k + 1].p).norm();
    const double actual_speed = actual.v.norm();
    if (!finite(actual) || !std::isfinite(error) || !std::isfinite(actual_speed))
    {
      nonfinite = true;
      break;
    }
    ++completed_steps;
    all_errors.add(error);
    if (static_cast<double>(k + 1) * settings.dt >= settling_time)
      steady_errors.add(error);
    peak_speed = std::max(peak_speed, actual_speed);
  }

  std::sort(solve_times_ms.begin(), solve_times_ms.end());
  const auto percentile = [&solve_times_ms](double fraction) {
    if (solve_times_ms.empty())
      return std::numeric_limits<double>::quiet_NaN();
    // Nearest-rank percentile, including failed solve calls.
    const auto rank = static_cast<std::size_t>(std::ceil(fraction * solve_times_ms.size()));
    return solve_times_ms[rank - 1];
  };
  const double all_rmse = all_errors.rmse();
  const double steady_rmse = steady_errors.rmse();
  const bool passed = completed_steps == steps && solver_failures == 0 && !nonfinite &&
    std::isfinite(all_rmse) && std::isfinite(steady_rmse) &&
    max_violation <= violation_tolerance && steady_rmse <= steady_rmse_limit;
  std::cout << "completed_steps=" << completed_steps << '\n'
            << "position_samples_all=" << all_errors.count << '\n'
            << "position_samples_after_2s=" << steady_errors.count << '\n'
            << "position_rmse_all_m=" << all_rmse << '\n'
            << "position_rmse_after_2s_m=" << steady_rmse << '\n'
            << "position_max_error_all_m=" << all_errors.maximum << '\n'
            << "position_max_error_after_2s_m="
            << (steady_errors.count == 0 ? std::numeric_limits<double>::quiet_NaN() : steady_errors.maximum) << '\n'
            << "peak_speed_m_s=" << peak_speed << '\n'
            << "max_input_violation=" << max_violation << '\n'
            << "solver_failures=" << solver_failures << '\n'
            << "nonfinite_detected=" << (nonfinite ? 1 : 0) << '\n'
            << "solve_calls=" << solve_times_ms.size() << '\n'
            << "solve_time_median_ms=" << percentile(0.5) << '\n'
            << "solve_time_p95_ms=" << percentile(0.95) << '\n'
            << "solve_time_max_ms=" << percentile(1.0) << '\n'
            << "deadline_misses_20ms=" << deadline_misses << '\n'
            << "benchmark_pass=" << (passed ? 1 : 0) << '\n';
  return passed ? 0 : 1;
}
}  // namespace

int main()
{
  try
  {
    return run();
  }
  catch (const std::exception &error)
  {
    std::cerr << "benchmark_error=" << error.what() << '\n';
    std::cout << "benchmark_pass=0\n";
    return 1;
  }
}
