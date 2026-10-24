#pragma once

#include "uav_ommpc_controller/dynamic.hpp"

namespace uav_ommpc_controller::linearization
{
// Actual minus reference: [delta_p_world, delta_v_world, Log(R_ref^T * R)].
using StateError = Eigen::Matrix<double, 9, 1>;
// [delta_thrust_acceleration, delta_omega_body], in m/s^2 and rad/s.
using InputError = Eigen::Matrix<double, 4, 1>;

inline StateError state_error(const dynamics::State &state, const dynamics::State &reference)
{
  StateError error;
  error << r3::boxminus(state.p, reference.p),
    r3::boxminus(state.v, reference.v), so3::boxminus(state.R, reference.R);
  return error;
}

inline dynamics::State apply_state_error(const dynamics::State &reference, const StateError &error)
{
  return {
    r3::boxplus(reference.p, error.head<3>()),
    r3::boxplus(reference.v, error.segment<3>(3)),
    so3::boxplus(reference.R, error.tail<3>())};
}

struct LinearModel
{
  Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
  Eigen::Matrix<double, 9, 4> B = Eigen::Matrix<double, 9, 4>::Zero();

  // Output is local to predict_step(reference, reference_input, dt), NOT reference.
  StateError predict(const StateError &delta_x, const InputError &delta_u) const
  {
    return A * delta_x + B * delta_u;
  }
};

// First-order discrete error dynamics of dynamics::predict_step, not I + dt*Ac.
// Reference state/input must satisfy dynamic.hpp's preconditions; dt >= 0.
// Perturbations must be small; no clipping, matrix projection or validation.
// The same gravity is used for actual/reference propagation, so it cancels from
// the error Jacobians. No affine defect: reference_next must be predict_step(...).
inline LinearModel linearize(
  const dynamics::State &reference, const dynamics::ControlInput &input, double dt)
{
  const Eigen::Vector3d phi = dt * input.body_rates;
  LinearModel model;
  model.A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  model.A.block<3, 3>(3, 6) =
    (dt * input.thrust_acceleration) * reference.R * so3::hat(Eigen::Vector3d::UnitZ());
  model.A.block<3, 3>(6, 6) = so3::exp(-phi);
  model.B.block<3, 1>(3, 0) = -dt * reference.R.col(2);
  model.B.block<3, 3>(6, 1) = dt * so3::right_jacobian(phi);
  return model;
}
}  // namespace uav_ommpc_controller::linearization
