#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <uav_ommpc_interfaces/msg/polynomial_trajectory.hpp>

#include "uav_ommpc_controller/dynamic.hpp"
#include "uav_ommpc_controller/polynomial_reference.hpp"

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

// Decode once on receipt; constructor validation is shared with the pure sampler.
// Invalid replacements throw and must invalidate the caller's previous plan.
inline std::unique_ptr<const uav_ommpc_controller::reference::PolynomialReference>
decode_polynomial(
  const uav_ommpc_interfaces::msg::PolynomialTrajectory &message,
  std::int64_t &start_ns, double gravity)
{
  if (message.header.frame_id != "ned" || message.header.stamp.sec < 0 ||
      message.header.stamp.nanosec >= 1000000000u) {
    throw std::invalid_argument("Invalid polynomial frame or timestamp");
  }
  const auto start = static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
    message.header.stamp.nanosec;
  if (start <= 0 || message.segments.empty() || message.segments.size() > 100) {
    throw std::invalid_argument("Invalid polynomial start or segment count");
  }
  std::vector<uav_ommpc_controller::reference::Segment> segments;
  segments.reserve(message.segments.size());
  for (const auto &source : message.segments) {
    segments.push_back({source.duration,
      {source.x.begin(), source.x.end()}, {source.y.begin(), source.y.end()},
      {source.z.begin(), source.z.end()}, {source.yaw.begin(), source.yaw.end()}});
  }
  auto result = std::make_unique<const uav_ommpc_controller::reference::PolynomialReference>(
    std::move(segments), message.hold_final, gravity);
  start_ns = start;
  return result;
}

inline std::optional<double> reference_elapsed(std::int64_t now_ns, std::int64_t start_ns)
{
  if (start_ns <= 0 || now_ns < start_ns) return std::nullopt;
  // Positive ordered timestamps keep the subtraction within int64 range.
  return static_cast<double>(now_ns - start_ns) * 1e-9;
}
}  // namespace uav_ommpc_ros
