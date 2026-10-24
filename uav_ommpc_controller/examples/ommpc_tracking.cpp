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
#include <utility>

#include "uav_ommpc_controller/ommpc.hpp"
#include "uav_ommpc_controller/polynomial_reference.hpp"

namespace
{
namespace dynamics = uav_ommpc_controller::dynamics;
namespace ommpc = uav_ommpc_controller::ommpc;
namespace so3 = uav_ommpc_controller::so3;
namespace polynomial = uav_ommpc_controller::reference;

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

Eigen::Matrix<double, 4, 3> circle_derivatives(double t)
{
  // Radius=5 m, angular speed=1 rad/s: genuine continuous derivatives.
  const double c = std::cos(t), s = std::sin(t);
  Eigen::Matrix<double, 4, 3> values;
  values << 5.0*c, 5.0*s, -2.0,
    -5.0*s, 5.0*c, 0.0,
    -5.0*c, -5.0*s, 0.0,
    5.0*s, -5.0*c, 0.0;
  return values;
}

polynomial::PolynomialReference circle_reference(double duration)
{
  constexpr double piece_duration = 0.25;
  const auto count = static_cast<std::size_t>(std::ceil(duration/piece_duration));
  std::vector<polynomial::Segment> segments;
  segments.reserve(count);
  Eigen::Matrix4d inverse;
  inverse << 35.0, -15.0, 2.5, -1.0/6.0,
    -84.0, 39.0, -7.0, 0.5,
    70.0, -34.0, 6.5, -0.5,
    -20.0, 10.0, -2.0, 1.0/6.0;
  for (std::size_t k = 0; k < count; ++k)
  {
    const auto left = circle_derivatives(k*piece_duration);
    const auto right = circle_derivatives((k+1)*piece_duration);
    Eigen::Matrix<double, 8, 3> coefficients;
    coefficients.row(0) = left.row(0);
    coefficients.row(1) = piece_duration*left.row(1);
    coefficients.row(2) = piece_duration*piece_duration/2.0*left.row(2);
    coefficients.row(3) = piece_duration*piece_duration*piece_duration/6.0*left.row(3);
    Eigen::Matrix<double, 4, 3> residual;
    residual.row(0) = right.row(0)-coefficients.topRows<4>().colwise().sum();
    residual.row(1) = piece_duration*right.row(1)-coefficients.row(1)-
      2.0*coefficients.row(2)-3.0*coefficients.row(3);
    residual.row(2) = piece_duration*piece_duration*right.row(2)-
      2.0*coefficients.row(2)-6.0*coefficients.row(3);
    residual.row(3) = piece_duration*piece_duration*piece_duration*right.row(3)-
      6.0*coefficients.row(3);
    coefficients.bottomRows<4>().noalias() = inverse*residual;
    polynomial::Segment segment;
    segment.duration = piece_duration;
    segment.x.resize(8);
    segment.y.resize(8);
    segment.z.resize(8);
    segment.yaw = {0.0};
    for (int i = 0; i < 8; ++i)
    {
      segment.x[i] = coefficients(i, 0);
      segment.y[i] = coefficients(i, 1);
      segment.z[i] = coefficients(i, 2);
    }
    segments.push_back(std::move(segment));
  }
  return polynomial::PolynomialReference(std::move(segments));
}

dynamics::State continuous_step(
  const dynamics::State &state, const dynamics::ControlInput &input,
  double dt, double gravity)
{
  constexpr int substeps = 20;
  const double h = dt/substeps;
  const auto acceleration = [&](double t) -> Eigen::Vector3d {
    const Eigen::Matrix3d R = state.R*so3::exp(t*input.body_rates);
    return gravity*Eigen::Vector3d::UnitZ()-input.thrust_acceleration*R.col(2);
  };
  dynamics::State next = state;
  Eigen::Vector3d a0 = acceleration(0.0);
  for (int i = 0; i < substeps; ++i)
  {
    const Eigen::Vector3d amid = acceleration((i+0.5)*h);
    const Eigen::Vector3d aend = acceleration((i+1)*h);
    // RK4 for p_dot=v, v_dot=a(t); a is independent of p and v.
    next.p += h*next.v+(h*h/6.0)*(a0+2.0*amid);
    next.v += (h/6.0)*(a0+4.0*amid+aend);
    a0 = aend;
  }
  next.R = state.R*so3::exp(dt*input.body_rates);
  return next;
}

bool run_plant(
  bool continuous, const ommpc::Settings &settings,
  const std::vector<dynamics::State> &states,
  const std::vector<dynamics::ControlInput> &inputs, std::size_t steps)
{
  constexpr double settling_time = 2.0;
  constexpr double violation_tolerance = 1e-4;
  constexpr double steady_rmse_limit = 0.15;
  constexpr double deadline_ms = 20.0;
  const auto horizon = static_cast<std::size_t>(settings.horizon);
  std::cout << "plant=" << (continuous ? "continuous_ZOH_RK4_dt_over_20" : "Euler_discrete") << '\n';
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
  ErrorStatistics all_velocity_errors;
  ErrorStatistics steady_velocity_errors;
  double peak_speed = actual.v.norm();
  double max_violation = 0.0;
  std::size_t solver_failures = 0;
  std::size_t deadline_misses = 0;
  std::size_t completed_steps = 0;
  bool nonfinite = false;
  all_errors.add((actual.p - states.front().p).norm());
  all_velocity_errors.add((actual.v - states.front().v).norm());

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
    actual = continuous ?
      continuous_step(actual, *result.command, settings.dt, settings.gravity) :
      dynamics::predict_step(actual, *result.command, settings.dt, settings.gravity);
    const double error = (actual.p - states[k + 1].p).norm();
    const double velocity_error = (actual.v - states[k + 1].v).norm();
    const double actual_speed = actual.v.norm();
    if (!finite(actual) || !std::isfinite(error) || !std::isfinite(velocity_error) || !std::isfinite(actual_speed))
    {
      nonfinite = true;
      break;
    }
    ++completed_steps;
    all_errors.add(error);
    all_velocity_errors.add(velocity_error);
    if (static_cast<double>(k + 1) * settings.dt >= settling_time)
    {
      steady_errors.add(error);
      steady_velocity_errors.add(velocity_error);
    }
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
    std::isfinite(all_velocity_errors.rmse()) && std::isfinite(steady_velocity_errors.rmse()) &&
    max_violation <= violation_tolerance && steady_rmse <= steady_rmse_limit;
  std::cout << "completed_steps=" << completed_steps << '\n'
            << "position_samples_all=" << all_errors.count << '\n'
            << "position_samples_after_2s=" << steady_errors.count << '\n'
            << "position_rmse_all_m=" << all_rmse << '\n'
            << "position_rmse_after_2s_m=" << steady_rmse << '\n'
            << "position_max_error_all_m=" << all_errors.maximum << '\n'
            << "position_max_error_after_2s_m="
            << (steady_errors.count == 0 ? std::numeric_limits<double>::quiet_NaN() : steady_errors.maximum) << '\n'
            << "velocity_rmse_all_m_s=" << all_velocity_errors.rmse() << '\n'
            << "velocity_rmse_after_2s_m_s=" << steady_velocity_errors.rmse() << '\n'
            << "velocity_max_error_all_m_s=" << all_velocity_errors.maximum << '\n'
            << "velocity_max_error_after_2s_m_s=" << steady_velocity_errors.maximum << '\n'
            << "peak_speed_m_s=" << peak_speed << '\n'
            << "max_input_violation=" << max_violation << '\n'
            << "solver_failures=" << solver_failures << '\n'
            << "nonfinite_detected=" << (nonfinite ? 1 : 0) << '\n'
            << "solve_calls=" << solve_times_ms.size() << '\n'
            << "solve_time_median_ms=" << percentile(0.5) << '\n'
            << "solve_time_p95_ms=" << percentile(0.95) << '\n'
            << "solve_time_max_ms=" << percentile(1.0) << '\n'
            << "deadline_misses_20ms=" << deadline_misses << '\n'
            << "plant_pass=" << (passed ? 1 : 0) << '\n';
  return passed;
}

int run()
{
  const ommpc::Settings settings;
  constexpr double duration = 12.0;
  const auto steps = static_cast<std::size_t>(std::ceil(duration/settings.dt));
  const auto horizon = static_cast<std::size_t>(settings.horizon);
  const std::size_t state_count = steps+horizon+1;
  const auto trajectory = circle_reference((steps+horizon)*settings.dt+0.25);
  std::vector<dynamics::State> states(state_count);
  std::vector<dynamics::ControlInput> inputs(state_count-1);
  for (std::size_t k = 0; k < state_count; ++k)
  {
    dynamics::ControlInput input;
    if (!trajectory.sample(k*settings.dt, states[k], input))
      throw std::runtime_error("analytic polynomial sample failed");
    if (k < inputs.size())
      inputs[k] = input;
  }
  Eigen::Vector3d max_defect = Eigen::Vector3d::Zero();
  double reference_violation = 0.0;
  for (std::size_t k = 0; k < inputs.size(); ++k)
  {
    if (!finite(states[k]) || !finite(inputs[k]))
      throw std::runtime_error("nonfinite polynomial reference");
    reference_violation = std::max(reference_violation, input_violation(inputs[k], settings));
    const auto next = dynamics::predict_step(states[k], inputs[k], settings.dt, settings.gravity);
    const Eigen::Vector3d defect{(next.p-states[k+1].p).norm(),
      (next.v-states[k+1].v).norm(), so3::boxminus(next.R, states[k+1].R).norm()};
    if (!finite(next) || !defect.allFinite())
      throw std::runtime_error("nonfinite reference defect");
    max_defect = max_defect.cwiseMax(defect);
  }
  std::cout << std::fixed << std::setprecision(9)
            << "scope=analytic_polynomial_reference_two_model_plants_no_ROS_no_PX4_no_hardware\n"
            << "timing_scope=steady_clock_solve_calls_not_a_hard_real_time_guarantee\n"
            << "trajectory=degree7_C3_Hermite_circle_fixed_altitude_fixed_yaw\n"
            << "frame=NED_FRD\nradius_m=5\nanalytic_speed_m_s=5\naltitude_m=2\nyaw_rad=0\n"
            << "dt_s=" << settings.dt << '\n'
            << "horizon=" << settings.horizon << '\n'
            << "duration_s=" << steps*settings.dt << '\n'
            << "steady_state_start_s=2\nsteady_state_rmse_limit_m=0.15\ndeadline_ms=20\n"
            << "reference_max_input_violation=" << reference_violation << '\n'
            << "reference_max_position_defect_m=" << max_defect.x() << '\n'
            << "reference_max_velocity_defect_m_s=" << max_defect.y() << '\n'
            << "reference_max_attitude_defect_rad=" << max_defect.z() << '\n';
  if (reference_violation > 0.0)
    throw std::runtime_error("circle reference exceeds configured input bounds");
  if (max_defect.x() > settings.max_position_defect ||
      max_defect.y() > settings.max_velocity_defect ||
      max_defect.z() > settings.max_attitude_defect)
    throw std::runtime_error("analytic circle reference exceeds configured affine model domain");
  const bool discrete_pass = run_plant(false, settings, states, inputs, steps);
  const bool continuous_pass = run_plant(true, settings, states, inputs, steps);
  const bool passed = discrete_pass && continuous_pass;
  std::cout << "benchmark_pass=" << (passed ? 1 : 0) << '\n';
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
