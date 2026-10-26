#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace uav_ommpc_ros
{
struct RateCompensationSettings
{
  bool enabled{false};
  std::array<double, 2> gain{1.08, 1.08};  // conservative inverse gain selected by step validation
  std::array<double, 2> time_constant{0.0275, 0.0275};  // identified plant T, seconds
  double filter_time_constant{0.020};    // softer reference filter to limit step overshoot
};

// Proper, causal prefilter F(s) = (1 + T*s) / (K * (1 + Tf*s)).
// Cancels the identified first-order lag approximately; does NOT invert pure delay.
// Fixed-size state, no allocation. The caller clamps desired and resulting rates.
class RollPitchCompensation
{
public:
  explicit RollPitchCompensation(const RateCompensationSettings & settings = {})
  : settings_(settings)
  {
    if (!std::isfinite(settings_.filter_time_constant) || settings_.filter_time_constant <= 0.0)
      throw std::invalid_argument("rate compensation filter time constant must be finite and positive");
    for (std::size_t axis = 0; axis < 2; ++axis) {
      if (!std::isfinite(settings_.gain[axis]) || settings_.gain[axis] <= 0.0 ||
        !std::isfinite(settings_.time_constant[axis]) || settings_.time_constant[axis] < 0.0)
        throw std::invalid_argument("rate compensation requires positive gains and nonnegative time constants");
      inverse_gain_[axis] = 1.0 / settings_.gain[axis];
      if (!std::isfinite(inverse_gain_[axis]))
        throw std::invalid_argument("rate compensation inverse gain must be representable");
    }
  }

  bool enabled() const {return settings_.enabled;}

  // dt <= 0 primes the filter on first input, expired lease or nonadvancing clock.
  double apply(std::size_t axis, double desired, double dt)
  {
    if (dt <= 0.0) {
      filtered_[axis] = desired;
      return desired * inverse_gain_[axis];
    }
    // Backward Euler low-pass: q_dot = (desired - q) / Tf, output = (q + T*q_dot)/K.
    const double derivative = (desired - filtered_[axis]) / (settings_.filter_time_constant + dt);
    filtered_[axis] += dt * derivative;
    return (filtered_[axis] + settings_.time_constant[axis] * derivative) * inverse_gain_[axis];
  }

private:
  RateCompensationSettings settings_;
  std::array<double, 2> inverse_gain_{};
  std::array<double, 2> filtered_{};
};
}  // namespace uav_ommpc_ros
