#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <Eigen/Geometry>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <uav_ommpc_interfaces/msg/reference_state.hpp>

#include "uav_ommpc_controller/dynamic.hpp"

namespace uav_ommpc_ros
{
inline bool decode_odometry(
  const px4_msgs::msg::VehicleOdometry &message,
  uav_ommpc_controller::dynamics::State &out)
{
  using Odometry = px4_msgs::msg::VehicleOdometry;
  if (message.pose_frame != Odometry::POSE_FRAME_NED ||
      (message.velocity_frame != Odometry::VELOCITY_FRAME_NED &&
       message.velocity_frame != Odometry::VELOCITY_FRAME_BODY_FRD)) {
    return false;
  }

  uav_ommpc_controller::dynamics::State state;
  for (std::size_t i = 0; i < 3; ++i) {
    state.p[i] = message.position[i];
    state.v[i] = message.velocity[i];
  }
  Eigen::Quaterniond q(message.q[0], message.q[1], message.q[2], message.q[3]);
  const double norm = q.norm();
  if (!state.p.allFinite() || !state.v.allFinite() ||
      !q.coeffs().allFinite() || !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
    return false;
  }
  q.normalize();
  state.R = q.toRotationMatrix();
  if (message.velocity_frame == Odometry::VELOCITY_FRAME_BODY_FRD) {
    state.v = state.R * state.v;
  }
  // PX4 quality, angular velocity and covariance are not inputs to this model.
  out = state;
  return true;
}

inline bool decode_reference(
  const uav_ommpc_interfaces::msg::ReferenceState &message,
  uav_ommpc_controller::dynamics::State &out)
{
  uav_ommpc_controller::dynamics::State state;
  for (std::size_t i = 0; i < 3; ++i) {
    state.p[i] = message.position[i];
    state.v[i] = message.velocity[i];
  }
  Eigen::Quaterniond q(message.q[0], message.q[1], message.q[2], message.q[3]);
  const double norm = q.norm();
  if (!state.p.allFinite() || !state.v.allFinite() ||
      !q.coeffs().allFinite() || !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-6) {
    return false;
  }
  q.normalize();
  state.R = q.toRotationMatrix();
  out = state;
  return true;
}

inline std::optional<std::size_t> reference_index(
  std::int64_t now_ns, std::int64_t start_ns, std::int64_t dt_ns,
  std::size_t input_count, std::size_t horizon)
{
  if (now_ns <= 0 || start_ns <= 0 || now_ns < start_ns || dt_ns <= 0 ||
      horizon == 0 || input_count < horizon) {
    return std::nullopt;
  }
  // Positive ordered timestamps make subtraction safe. Compare the remaining
  // capacity instead of adding horizon to index, which could overflow.
  const auto index = static_cast<std::uint64_t>((now_ns - start_ns) / dt_ns);
  if (index > input_count - horizon) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(index);
}
}  // namespace uav_ommpc_ros
