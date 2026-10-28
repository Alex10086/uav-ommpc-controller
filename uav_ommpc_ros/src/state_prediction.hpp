#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <uav_ommpc_controller/dynamic.hpp>

namespace uav_ommpc_ros
{
// Requires DDS sample stamps synchronized to the node's ROS clock.
// Invalid, future or over-age stamps are rejected, never clamped to a valid age.
inline std::optional<double> state_sample_age(
  std::uint64_t sample_us, std::int64_t now_ns, double maximum_age)
{
  if (sample_us == 0 || now_ns <= 0 ||
    sample_us > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000))
    return std::nullopt;
  const auto sample_ns = static_cast<std::int64_t>(sample_us) * 1000;
  if (sample_ns > now_ns) return std::nullopt;
  const double age = static_cast<double>(now_ns - sample_ns) * 1e-9;
  return age <= maximum_age ? std::optional<double>(age) : std::nullopt;
}

// Short-horizon kinematic extrapolation: measured v and body gyro are held.
// No ideal rate-command substitution or guessed thrust/drag acceleration.
// Caller validates finite state/rates and a bounded nonnegative dt.
inline uav_ommpc_controller::dynamics::State predict_measured_state(
  const uav_ommpc_controller::dynamics::State &state,
  const Eigen::Vector3d &measured_body_rates, double dt)
{
  return {state.p + dt * state.v, state.v,
    uav_ommpc_controller::so3::boxplus(state.R, dt * measured_body_rates)};
}
}  // namespace uav_ommpc_ros
