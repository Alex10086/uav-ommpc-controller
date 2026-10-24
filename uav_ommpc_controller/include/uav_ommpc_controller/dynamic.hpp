#pragma once

#include "uav_ommpc_controller/so3_math.hpp"

namespace uav_ommpc_controller::dynamics
{
inline constexpr double kGravity = 9.80665;  // m/s^2, consistent with the PX4 bridge

struct State
{
  Eigen::Vector3d p = Eigen::Vector3d::Zero();     // world NED position [m]
  Eigen::Vector3d v = Eigen::Vector3d::Zero();     // world NED velocity [m/s]
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); // body FRD -> world NED
};

struct ControlInput
{
  double thrust_acceleration = 0.0;                         // total thrust / mass [m/s^2], along body -Z
  Eigen::Vector3d body_rates = Eigen::Vector3d::Zero();      // body FRD angular velocity [rad/s]
};

// Paper model: x = (p, v, R), u = (a_T, omega_body).
// Inputs are held constant over dt; body rates are inputs, not propagated states.
// Translation uses forward Euler at the CURRENT v and R; rotation uses SO(3) Exp.
// No drag, torque/rate dynamics, motor model, saturation or frame conversion.
// Preconditions: finite inputs, R in SO(3), dt >= 0, thrust_acceleration >= 0,
// gravity > 0. As with so3_math.hpp, callers validate inputs; nothing is clamped.
inline State predict_step(
  const State &state, const ControlInput &input, double dt, double gravity = kGravity)
{
  const Eigen::Vector3d acceleration =
    Eigen::Vector3d(0.0, 0.0, gravity) - input.thrust_acceleration * state.R.col(2);

  return {
    r3::boxplus(state.p, dt * state.v),
    r3::boxplus(state.v, dt * acceleration),
    so3::boxplus(state.R, dt * input.body_rates)};
}
}  // namespace uav_ommpc_controller::dynamics
