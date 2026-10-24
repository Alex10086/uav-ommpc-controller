#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace uav_ommpc_controller
{
// Euclidean components (position, velocity): ordinary addition/subtraction.
namespace r3
{
inline Eigen::Vector3d boxplus(const Eigen::Vector3d &x, const Eigen::Vector3d &delta)
{
  return x + delta;
}

inline Eigen::Vector3d boxminus(const Eigen::Vector3d &x, const Eigen::Vector3d &reference)
{
  return x - reference;
}
}  // namespace r3

namespace so3
{
// Right-handed, active rotations of column vectors; angles are in radians.
// Inputs must be finite; rotation matrices must already belong to SO(3).
// No validation/projection is performed in these fixed-size math primitives.

// R^3 -> so(3): hat(v) * w == v.cross(w).
inline Eigen::Matrix3d hat(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(),
            v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;
  return result;
}

// so(3) -> R^3. The input must be skew-symmetric, not a rotation matrix.
inline Eigen::Vector3d vee(const Eigen::Matrix3d &skew)
{
  return {skew(2, 1), skew(0, 2), skew(1, 0)};
}

// Rotation vector phi (axis * angle) -> SO(3), using Rodrigues' formula.
inline Eigen::Matrix3d exp(const Eigen::Vector3d &phi)
{
  const double theta2 = phi.squaredNorm();
  double a;  // sin(theta) / theta
  double b;  // (1 - cos(theta)) / theta^2
  if (theta2 < 1e-6)
  {
    a = 1.0 - theta2 / 6.0 + theta2 * theta2 / 120.0;
    b = 0.5 - theta2 / 24.0 + theta2 * theta2 / 720.0;
  }
  else
  {
    const double theta = std::sqrt(theta2);
    const double half_sin = std::sin(0.5 * theta);
    a = std::sin(theta) / theta;
    b = 2.0 * half_sin * half_sin / theta2;
  }
  const Eigen::Matrix3d skew = hat(phi);
  return Eigen::Matrix3d::Identity() + a * skew + b * (skew * skew);
}

// SO(3) -> principal rotation vector, norm in [0, pi]. At pi the axis sign
// is inherently ambiguous; log is not continuous across this branch cut.
inline Eigen::Vector3d log(const Eigen::Matrix3d &rotation)
{
  // Eigen's quaternion-based extraction avoids division by sin(theta) near pi.
  const Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.angle() * angle_axis.axis();
}

// Right/body-local perturbation: R boxplus delta = R * Exp(delta).
inline Eigen::Matrix3d boxplus(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &delta)
{
  return rotation * exp(delta);
}

// Actual relative to reference: R boxminus R_ref = Log(R_ref^T * R).
// Inverse of boxplus for ||delta|| < pi (not a global vector subtraction).
inline Eigen::Vector3d boxminus(
  const Eigen::Matrix3d &rotation, const Eigen::Matrix3d &reference)
{
  return log(reference.transpose() * rotation);
}

// Exp(phi + d) = Exp(J_l(phi) * d) * Exp(phi) + O(||d||^2).
inline Eigen::Matrix3d left_jacobian(const Eigen::Vector3d &phi)
{
  const double theta2 = phi.squaredNorm();
  double b;  // (1 - cos(theta)) / theta^2
  double c;  // (theta - sin(theta)) / theta^3
  if (theta2 < 1e-6)
  {
    b = 0.5 - theta2 / 24.0 + theta2 * theta2 / 720.0;
    c = 1.0 / 6.0 - theta2 / 120.0 + theta2 * theta2 / 5040.0;
  }
  else
  {
    const double theta = std::sqrt(theta2);
    const double half_sin = std::sin(0.5 * theta);
    b = 2.0 * half_sin * half_sin / theta2;
    c = (theta - std::sin(theta)) / (theta2 * theta);
  }
  const Eigen::Matrix3d skew = hat(phi);
  return Eigen::Matrix3d::Identity() + b * skew + c * (skew * skew);
}

// Exp(phi + d) = Exp(phi) * Exp(J_r(phi) * d) + O(||d||^2).
inline Eigen::Matrix3d right_jacobian(const Eigen::Vector3d &phi)
{
  return left_jacobian(-phi);
}

// Derivative of Log(Exp(phi) * Exp(delta)) at delta=0; ||phi|| < pi.
inline Eigen::Matrix3d right_jacobian_inverse(const Eigen::Vector3d &phi)
{
  const double theta2 = phi.squaredNorm();
  double coefficient;
  if (theta2 < 1e-6)
  {
    coefficient = 1.0 / 12.0 + theta2 / 720.0 + theta2 * theta2 / 30240.0;
  }
  else
  {
    const double half_theta = 0.5 * std::sqrt(theta2);
    coefficient = (1.0 - half_theta / std::tan(half_theta)) / theta2;
  }
  const Eigen::Matrix3d skew = hat(phi);
  return Eigen::Matrix3d::Identity() + 0.5 * skew + coefficient * (skew * skew);
}
}  // namespace so3
}  // namespace uav_ommpc_controller
