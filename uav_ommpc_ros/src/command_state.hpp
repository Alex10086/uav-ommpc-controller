#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>

#include <uav_ommpc_interfaces/msg/thrust_rates.hpp>

namespace uav_ommpc_ros
{
constexpr double kGravity = 9.80665;

struct Limits
{
  double hover_thrust;                   // normalized throttle at thrust_acceleration == g
  std::array<double, 3> max_body_rates;  // rad/s, symmetric per FRD axis
  double max_throttle;                   // normalized, (0, 1]
  bool quadratic_thrust_model{false};    // true: invert T ~ omega^2 on [min_motor_speed, max_motor_speed]
  double min_motor_speed{150.0};         // rad/s at zero throttle (PX4 SIM_GZ_EC_MIN)
  double max_motor_speed{1000.0};        // rad/s at full throttle (PX4 SIM_GZ_EC_MAX)
};

struct RateCommand
{
  std::array<float, 3> body_rates{};
  float throttle{};
  bool limited{false};
};

// Unclipped throttle for a finite, nonnegative specific thrust; validated motor limits required.
// When quadratic_thrust_model is enabled, inverts omega(u) = omega_min + (omega_max - omega_min) * u
// under rotor thrust T_i = k_f * omega^2 calibrated at hover (a_T == g <=> u == hover_thrust).
inline double compute_throttle(const Limits & limits, double thrust_acceleration)
{
  if (!limits.quadratic_thrust_model) {
    return limits.hover_thrust * thrust_acceleration / kGravity;
  }
  const double speed_span = limits.max_motor_speed - limits.min_motor_speed;
  const double hover_speed = limits.min_motor_speed + speed_span * limits.hover_thrust;
  const double desired_speed = hover_speed * std::sqrt(thrust_acceleration / kGravity);
  return (desired_speed - limits.min_motor_speed) / speed_span;
}

// Latest accepted control input. Rejected input keeps the previous command and its age.
class CommandState
{
public:
  using Clock = std::chrono::steady_clock;

  CommandState(const Limits & limits, Clock::duration timeout)
  : limits_(limits), timeout_(timeout) {}

  bool accept(const uav_ommpc_interfaces::msg::ThrustRates & input, Clock::time_point now)
  {
    if (!std::isfinite(input.thrust_acceleration) || input.thrust_acceleration < 0.0 ||
      !std::all_of(input.body_rates.begin(), input.body_rates.end(),
        [](double rate) {return std::isfinite(rate);}))
    {
      return false;
    }
    RateCommand command;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double limit = limits_.max_body_rates[axis];
      const double rate = std::clamp(input.body_rates[axis], -limit, limit);
      command.limited |= rate != input.body_rates[axis];
      command.body_rates[axis] = static_cast<float>(rate);
    }
    const double throttle = compute_throttle(limits_, input.thrust_acceleration);
    command.limited |= throttle < 0.0 || throttle > limits_.max_throttle;
    command.throttle = static_cast<float>(std::clamp(throttle, 0.0, limits_.max_throttle));
    command_ = command;
    last_accepted_ = now;
    return true;
  }

  bool fresh(Clock::time_point now) const
  {
    return last_accepted_ && now - *last_accepted_ <= timeout_;
  }

  const RateCommand & command() const {return command_;}

private:
  Limits limits_;
  Clock::duration timeout_;
  RateCommand command_{};
  std::optional<Clock::time_point> last_accepted_;
};
}  // namespace uav_ommpc_ros
